from typing import List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import path_planner_server.outer_rectangle_utils as utils
from path_planner_interfaces.srv import OuterRectangleFinder
from geometry_msgs.msg import PointStamped
import math


class ORectangleFinder(Node):
    def __init__(self):
        super().__init__('outer_rectangle_finder_node')
        self.rect_finder = self.create_service(OuterRectangleFinder, 'outer_rectangle_finder', self.rectangle_finder)
        self.wall_publisher = self.create_publisher(LaserScan, 'wall', 10)
        self.get_logger().info('service started: %s' % (self.rect_finder.srv_name))

    def rectangle_finder(self, request: OuterRectangleFinder.Request, response: OuterRectangleFinder.Response):
        self.get_logger().info('service called')
        msg: LaserScan = request.scan
        if msg.ranges == None:
            return response

        ranges: List[float] = [msg.range_max if math.isinf(r) else r for r in msg.ranges]

        rectangle: utils.OuterRectangle = utils.find_outer_rectangle(self, ranges, msg.angle_increment, msg.header)

        if rectangle.invalid():
            self.get_logger().info('Rectangle area is invalid. Quitting')
            return response

        self.get_logger().info('Beta:' + str(rectangle.beta) + ' Rectangle area:' + str(rectangle.area))

        self.send_wall(msg, ranges, rectangle)

        response.corners: List[PointStamped] = rectangle.corners(request.distance)
        self.get_logger().info('Corners: %s' %(response.corners))

        return response

    def send_wall(self, msg: LaserScan, ranges: LaserScan.ranges, rectangle: utils.OuterRectangle):
        for i,r in enumerate(ranges):
            msg.ranges[i] = r if rectangle.onTheWall(i, 0.3) else 0.0

        self.wall_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ORectangleFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()