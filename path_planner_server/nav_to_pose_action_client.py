import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point



class NavToPoseActionClient(Node):

    def __init__(self):
        super().__init__('nav_to_pose_action_client')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.subscription = self.create_subscription(
            PointStamped,
            'clicked_point',
            self.send_goal,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Init done')


    def send_goal(self, pose: PointStamped):
        self.get_logger().info('Got new pose')
        goal_msg  = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header = pose.header
        goal_msg.pose.pose = Pose()
        goal_msg.pose.pose.position.x = pose.point.x
        goal_msg.pose.pose.position.y = pose.point.y

        self.action_client.wait_for_server()
        self.get_logger().info('Sending goal')

        return self.action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = NavToPoseActionClient()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()