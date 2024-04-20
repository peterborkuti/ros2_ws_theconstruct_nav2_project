from path_planner_interfaces.srv import PointToPoint
from path_planner_interfaces.srv import DistanceAndGradToPoint
from path_planner_interfaces.srv import DistanceAndRadToPoint
# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import math

import tf_transformations

class Service(Node):

    def __init__(self):
        super().__init__('transform_distance_server')
        self.get_logger().info('server starting')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # create the Service Server object
        # defines the type, name, and callback function
        self.srv_ptp = self.create_service(PointToPoint, 'transform_ptp', self.service_ptp)
        self.srv_rtp = self.create_service(DistanceAndRadToPoint, 'transform_rtp', self.service_rtp)
        self.srv_gtp = self.create_service(DistanceAndGradToPoint, 'transform_gtp', self.service_gtp)
        self.get_logger().info('server started')

    def service_ptp(self, request, response):
        self.get_logger().info('Got request x:%f, y:%f' % (request.x, request.y))

        x = request.x
        y = request.y

        range = 0
        angle = 0

        if x != 0.0 or y != 0.0:
            angle = math.atan2(y, x)
            range = math.sqrt(x ** 2 + y ** 2)
        
        [response.x, response.y] = self.transform(angle, range)

        self.get_logger().info('Sending response %f, %f' % (response.x, response.y))

        return response

    def service_rtp(self, request, response):
        self.get_logger().info('Got request angle in radians: %f, range: %f' % (request.angle, request.range))

        [response.x, response.y] = self.transform(request.angle, request.range)

        self.get_logger().info('Sending response %f, %f' % (response.x, response.y))

        return response

    def service_gtp(self, request, response):
        self.get_logger().info('Got request angle in grads: %f, range: %f' % (request.angle, request.range))
        
        [response.x, response.y] = self.transform(request.angle * 0.0174533, request.range)

        self.get_logger().info('Sending response %f, %f' % (response.x, response.y))

        return response

    def transform(self, radians, range, origin_frame="base_scan", dest_frame="map"):
        # Look up for the transformation between dest_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach dest_frame
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                origin_frame,
                dest_frame,
                now)
        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform {origin_frame} to {dest_frame}: {ex}')
            return [None, None]
        
        dx = trans.transform.translation.x
        dy = trans.transform.translation.y

        rot = trans.transform.rotation

        [roll, pitch, yaw] = tf_transformations.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])

        self.get_logger().info('transform %f, %f, %f' % (dx, dy, yaw))

        x = range * math.cos(radians + yaw) + dx
        y = range * math.sin(radians + yaw) + dy

        return [x , y]


def main(args=None):
    rclpy.init(args=args)
    service = Service()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()