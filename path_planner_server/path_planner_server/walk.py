from path_planner_server.angle_utils import UnifiedLaserScan
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import queue
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PointStamped, PoseStamped, Pose
from visualization_msgs.msg import MarkerArray
from typing import List
from path_planner_server.marker_utils import getAddMarker

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_point
from tf_transformations import quaternion_about_axis
from geometry_msgs.msg import PointStamped, Quaternion
from path_planner_server.outer_rectangle_utils import OuterRectangle, find_outer_rectangle

import math
from numpy import argmin

two_pi = 2 * math.pi

class Walker(Node):

    def __init__(self):
        super().__init__('walking_node')
        self.laser_scan_queue = queue.Queue()
        self.walking_points: List[PointStamped]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=MutuallyExclusiveCallbackGroup())
        self.marker_publisher = self.create_publisher(MarkerArray, 'marker_topic', 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=MutuallyExclusiveCallbackGroup())

        self.call_timer = self.create_timer(10, self.walk, ReentrantCallbackGroup())
        self.get_logger().info('Walking node init done.')

    def walk(self):
        self.get_logger().info('start walking')
        ''' Timer only is needed for delaying the start'''
        self.destroy_timer(self.call_timer)
        self.get_walking_points()

        index = self.get_min_walking_point_index()
        while True:
            self.get_logger().info('walking index: %d' % (index))
            self.get_walking_points()

            if len(self.walking_points) < 4:
                self.get_logger().info('no valid points')
                continue

            self.go_to_next_point(index)
            '''When navigation successes or fails, it moves to the next corner'''

            index = (index + 1) % 4

    def get_walking_points(self):
        ''' corners in PointStamped in the base_scan coordinate frame '''
        msg: UnifiedLaserScan = self.laser_scan_queue.get()
        rectangle: OuterRectangle = find_outer_rectangle(self, msg.ranges, msg.angle_min, msg.angle_increment, msg.header)
        if rectangle.invalid():
            self.walking_points = []
            return

        '''Get the 0.3 meter inner points of the corners to give the robot enough space'''
        corners = rectangle.corners(0.3)
        self.get_logger().info('got corners: (%s)' % (corners))
        self.add_markers(corners)

        '''Corners are in the frame of the laser scanner. Transform them to the map'''
        map_corners = [self.transform(corner) for corner in corners]

        if any(map(lambda e: e is None, map_corners)):
            self.walking_points = []
            return

        ''' sorting in order of counter-clockwise '''
        map_corners.sort(key= lambda p: math.atan2(p.point.y, p.point.x))

        self.walking_points = map_corners

    def get_min_walking_point_index(self) -> int:
        """
            Compute the closest corner to the robot
            Returns with the corner index
        """
        if len(self.walking_points) < 4:
            return 0

        """
            Robot is at (0,0,0) based on the frame of laser scan
            Laser scan's frame is the frame of the corners
            This line is for getting the robot coordinate on the map
        """
        robot: PointStamped = self.transform(PointStamped(header=self.walking_points[0].header))
        if robot is None:
            return 0

        """
            Lets compute the closest corner to the robot
            No need to use square root for computing distances
                if I am not using square root anywhere
        """
        r_distance = robot.point.x ** 2 + robot.point.y ** 2
        distances = [abs(r_distance - p.point.x ** 2 + p.point.y ** 2) for p in self.walking_points]

        ''' The index of the closest corner'''
        return argmin(distances)

    def go_to_next_point(self, index: int):
        if len(self.walking_points) < 4:
            return

        point = self.walking_points[index]
        next_point = self.walking_points[(index + 1) % 4]

        self.add_marker(point)

        self.navigate_to_coord(point, next_point)

    def transform(self, point: PointStamped, target_frame="map") -> PointStamped:
        """
            Transform point to the map
        """
        try:
            source_frame = point.header.frame_id
            trans = self.tf_buffer.lookup_transform(
                source_frame=source_frame,
                target_frame=target_frame,
                time=point.header.stamp)
        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform {source_frame} to {target_frame}: {ex}')
            return None

        return do_transform_point(point, trans)

    def navigate_to_coord(self, point: PointStamped, next_point: PointStamped):
        """
            Navigate to the point and set the robot pose to the direction of the next_point
        """
        self.get_logger().info('navigate to Point: %s' % (point))

        '''Compute the the direction of the next_point relative to the point'''
        alfa = math.atan2(next_point.point.y - point.point.y, next_point.point.x - point.point.x)
        q = quaternion_about_axis(alfa, (0, 0, 1))
        quaternion = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        goal_msg  = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped(header=point.header, pose=Pose(position=point.point, orientation=quaternion))

        self.get_logger().info('Waiting for the server')
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info('Sending goal')

        '''Synchron call. Hope it will not deadlock'''
        self.nav_to_pose_client.send_goal(goal_msg)

    def add_marker(self, point: PointStamped):
        """
            Sends the goal marker with the second color to RVIZ2
        """
        marker_arr = MarkerArray(
            markers=[
                getAddMarker(point, 1)
            ]
        )

        self.marker_publisher.publish(marker_arr)
        self.get_logger().info('marker added')

    def add_markers(self, points: List[PointStamped]):
        """
            Sends all the markers to RVIZ2 with the first color
        """
        marker_arr = MarkerArray(
            markers=[
                getAddMarker(p)
                for i,p in enumerate(points)
            ]
        )

        self.marker_publisher.publish(marker_arr)
        self.get_logger().info('markers added')

    def laser_callback(self, _msg: LaserScan):
        """
            Stores the latest reading into the queue.
            The queue only contains the latest reading.
        """
        msg = UnifiedLaserScan(_msg)
        ranges = msg.ranges

        self.get_logger().debug('indexes: front: %d, left: %d, back: %d, rigth: %d' % (0, 90, 180, 270))
        self.get_logger().debug('ranges: front: %.2f, left: %.2f, back: %.2f, rigth: %0.2f' % 
                               (ranges[0], ranges[90], ranges[180], ranges[270]))
        self.get_logger().debug('ranges size:%d' % (len(ranges)))

        with self.laser_scan_queue.mutex:
            self.laser_scan_queue.queue.clear()

        self.laser_scan_queue.put(msg)

def main(args=None):
    rclpy.init(args=args)

    walker = Walker()
    executor = MultiThreadedExecutor()
    executor.add_node(walker)

    try:
        walker.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        walker.get_logger().info('Keyboard interrupt, shutting down.\n')

    executor.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()