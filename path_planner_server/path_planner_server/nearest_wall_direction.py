from wall_follower_interfaces.service import NearestWallDirection
import wall_follower.outer_rectangle_utils as utils

import rclpy
from rclpy.node import Node

from typing import List


class Service(Node):

    def __init__(self):
        super().__init__('nearest_wall_direction_service')
        self.srv = self.create_service(NearestWallDirection, 'nearest_wall_direction', self.get_nearest_wall_direction_callback)

    def get_nearest_wall_direction_callback(self, request, response):
        self.get_logger().info('Incoming request')

        ranges: List[float] = request.ranges
        rectangle = utils.get_outer_rectangle(ranges, request.angle_increment)
        
        min_index = -1
        min_distance = float("inf")
        for index, distance in enumerate(ranges):
            if not rectangle.onTheWall(index, 0.03):
                continue
            if min_distance > distance:
                min_distance = distance
                min_index = index
        
        response.angle = min_index * request.angle_increment
        self.get_logger().info('Nearest wall direction: ' + str(response.angle))

        return response

def main():
    rclpy.init()

    minimal_service = Service()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()