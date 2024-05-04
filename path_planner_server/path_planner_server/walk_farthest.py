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
import math
from std_msgs.msg import ColorRGBA
from path_planner_interfaces.srv import OuterRectangleFinder
from typing import List
from path_planner_server.marker_utils import getAddMarker

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

import math

class Walker(Node):

    def __init__(self):
        super().__init__('walk_walking')
        self.rate = self.create_rate(0.5)
        self.marker_id = 0
        self.counter = 0
        self.q = queue.Queue()
        self.color1 = ColorRGBA(r=1.0, g=0.984, b=0.0, a=1.0)
        self.color2 = ColorRGBA(r=0.929, g=0.098, b=1.0, a=1.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.outer_r_service = self.create_client(OuterRectangleFinder, 'outer_rectangle_finder')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.marker_publisher = self.create_publisher(MarkerArray, 'marker_topic', 10)
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
        self.destroy_timer(self.call_timer)

        while True:
            self.get_logger().info('start sequence')
            for corner_index in range(4):
                # corners in PointStamped in the base_scan coordinate frame
                corners = self.outer_rectangle(self.q.get())
                self.get_logger().info('got corners: (%s)' % (corners))
                self.add_markers(corners)

                point = self.transform(corners[corner_index], 'map')
                if point is None:
                    self.get_logger().info('transformed point is None!')
                    continue
                self.get_logger().info('transformed to map: %s' % (point))
                self.add_marker(point)

                self.navigate_to_coord(point)
                self.rate.sleep()

   
    def to_point(self, angle, range):
        return [range * math.cos(angle), range * math.sin(angle)]

    def outer_rectangle(self, msg:LaserScan) -> List[PointStamped]:
        while not self.outer_r_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('outer_rectangle service not available, waiting again...')

        request = OuterRectangleFinder.Request(scan=msg, distance=0.3)

        self.get_logger().info('calling rectangle service')
        response = self.outer_r_service.call(request)
        self.get_logger().info('got response %s' % (response.corners))

        return response.corners

    def transform(self, point: PointStamped, target_frame="map") -> PointStamped:
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

    def navigate_to_coord(self, point: PointStamped):
        self.get_logger().info('navigate to Point: %s' % (point))

        goal_msg  = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped(header=point.header, pose=Pose(position=point.point))

        self.get_logger().info('Waiting for the server')
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info('Sending goal')

        self.nav_to_pose_client.send_goal(goal_msg)

    def add_marker(self, point: PointStamped):
        marker_arr = MarkerArray(
            markers=[
                getAddMarker(point, self.color2)
            ]
        )

        self.marker_publisher.publish(marker_arr)
        self.get_logger().info('marker added')


    def add_markers(self, points: List[PointStamped]):
        marker_arr = MarkerArray(
            markers=[
                getAddMarker(p, self.color1)
                for i,p in enumerate(points)
            ]
        )

        self.marker_publisher.publish(marker_arr)
        self.get_logger().info('markers added')

    def laser_callback(self, msg: LaserScan):
        ranges = msg.ranges
        self.get_logger().debug('ranges: 0: %.2f, 90: %.2f, 180: %.2f, 270: %0.2f' % (ranges[0], ranges[90], ranges[180], ranges[270]))

        with self.q.mutex:
            self.q.queue.clear()

        self.q.put(msg)


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    # scanner = Scanner()
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