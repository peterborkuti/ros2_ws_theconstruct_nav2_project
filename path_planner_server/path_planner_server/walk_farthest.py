import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import queue
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.action import ActionClient
from path_planner_interfaces.srv import DistanceAndRadToPoint
from numpy import argmax
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Vector3
from visualization_msgs.msg import Marker
import math
import random
#from builtin_interfaces import Duration
from std_msgs.msg import ColorRGBA

q = queue.Queue()

class Walker(Node):

    def __init__(self):
        super().__init__('walk_walking')
        #self.rate = self.create_rate(4)
        self.marker_id = 0
        self.counter = 0

        self.tr_service = self.create_client(DistanceAndRadToPoint, 'transform_rtp')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.marker_publisher = self.create_publisher(Marker, 'marker_topic', 10)
        self.call_timer = self.create_timer(1, self.walk, ReentrantCallbackGroup())


    def walk(self):
        self.get_logger().info('start walking')
        self.destroy_timer(self.call_timer)

        while True:
            [angle, range] = self.random_angle(q.get())

            self.get_logger().info('got farthest point: (angle %.2f, range %.2f)' % (angle, range))
            range -= 0.3
            point = self.transform(angle, range)
            #[x, y] = self.to_point(angle, range)
            self.get_logger().info('transformed to map: %s' % (point))
            self.navigate_to_coord(point)

    def random_angle(self, msg: LaserScan):
        index = random.randrange(0, len(msg.ranges))
        range = msg.ranges[index]
        if math.isinf(range):
            range = 1.0
        angle = msg.angle_min + msg.angle_increment * index

        return [angle, range]
    
    def max_distance(self, msg: LaserScan):
        index = argmax(msg.ranges)
        range = msg.ranges[index]
        if math.isinf(range):
            range = 1.0
        angle = msg.angle_min + msg.angle_increment * index

        return [angle, range]
    
    def to_point(self, angle, range):
        return [range * math.cos(angle), range * math.sin(angle)]

    def transform(self, angle, range):
        while not self.tr_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('transform_rtp service not available, waiting again...')
        
        request = DistanceAndRadToPoint.Request()
        request.angle = angle
        request.range = range
        response = self.tr_service.call(request)

        return response.point

    def navigate_to_coord(self, point: PointStamped):
        x = point.point.x
        y = point.point.y
        self.get_logger().info('navigate to coord: (%.3f, %.3f)' % (x, y))
        self.remove_marker()
        goal_msg  = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = point.header.frame_id
        goal_msg.pose.header.stamp = point.header.stamp

        goal_msg.pose.pose = Pose()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        self.add_marker(x, y)

        self.get_logger().info('Waiting for the server')
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info('Sending goal')

        self.nav_to_pose_client.send_goal(goal_msg)


    def remove_marker(self):
        if self.marker_id > 0:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.action = 2
            marker.ns = 'walker'
            marker.id = self.marker_id

            self.marker_id = 0
            self.marker_publisher.publish(marker)
            self.get_logger().info('marker removed with id: %d' % (marker.id))

    def add_marker(self, x, y):
        self.marker_id = 100
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'walker'
        marker.action = Marker.ADD
        marker.type = Marker.SPHERE
        marker.id = self.marker_id
        marker.pose = Pose()
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.scale = Vector3()
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color = ColorRGBA()
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

        self.marker_publisher.publish(marker)
        self.get_logger().info('marker added with id: %d, (%f, %f)' % (marker.id, x, y))

class Scanner(Node):

    def __init__(self):
        super().__init__('walk_scanning')
        server_cb_group = MutuallyExclusiveCallbackGroup()
        scanner_cb_group = MutuallyExclusiveCallbackGroup()

        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=scanner_cb_group)

    def laser_callback(self, msg: LaserScan):
        #ranges = msg.ranges
        #self.get_logger().info('ranges: 0: %.2f, 90: %.2f, 180: %.2f, 270: %0.2f' % (ranges[0], ranges[90], ranges[180], ranges[270]))

        with q.mutex:
            q.queue.clear()

        q.put(msg)

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    scanner = Scanner()
    walker = Walker()
    executor = MultiThreadedExecutor()
    executor.add_node(scanner)
    executor.add_node(walker)

    try:
        scanner.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        scanner.get_logger().info('Keyboard interrupt, shutting down.\n')

    executor.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()