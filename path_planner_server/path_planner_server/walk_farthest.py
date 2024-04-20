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
from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point
from visualization_msgs.msg import Marker

q = queue.Queue()

class Walker(Node):

    def __init__(self):
        super().__init__('walk_walking')

        self.tr_service = self.create_client(DistanceAndRadToPoint, 'transform_rtp')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.call_timer = self.create_timer(1, self.walk, ReentrantCallbackGroup())


    def walk(self):
        self.get_logger().info('start walking')
        self.destroy_timer(self.call_timer)

        while True:
            [angle, range] = q.get()
            self.get_logger().info('got farthest point: (%f, %f)' % (angle, range))
            range -= 0.3
            [x, y] = self.transform(angle, range)
            self.get_logger().info('transformed to map: (%f, %f)' % (x, y))
            self.navigate_to_coord(x, y)
    
    def transform(self, angle, range):
        while not self.tr_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('transform_rtp service not available, waiting again...')
        
        request = DistanceAndRadToPoint.Request()
        request.angle = angle
        request.range = range
        response = self.tr_service.call(request)

        return [response.x, response.y]

    def navigate_to_coord(self, x, y):
        self.get_logger().info('navigate to coord: (%f, %f)' % (x, y))
        goal_msg  = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose = Pose()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        self.get_logger().info('Waiting for the server')
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info('Sending goal')

        self.nav_to_pose_client.send_goal(goal_msg)



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
        index = argmax(msg.ranges)
        angle = index * msg.angle_increment
        distance = msg.ranges[index]

        with q.mutex:
            q.queue.clear()

        q.put([angle, distance])

        # print the log info in the terminal
        self.get_logger().debug('Stored: angle: %f, distance: %f' % (angle, distance))


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