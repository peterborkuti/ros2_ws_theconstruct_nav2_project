from path_planner_interfaces.srv import PointToPoint
from path_planner_interfaces.srv import DistanceAndGradToPoint
from path_planner_interfaces.srv import DistanceAndRadToPoint
# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped, Point
from laser_geometry import LaserProjection

import math

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
        
        return do_transform_point(angle, range, response)

    def service_rtp(self, request, response):
        self.get_logger().info('Got request angle in radians: %f, range: %f' % (request.angle, request.range))

        return self.do_transform(request.angle, request.range, response)

    def service_gtp(self, request, response):
        self.get_logger().info('Got request angle in grads: %f, range: %f' % (request.angle, request.range))
        
        return self.do_transform(request.angle * 0.0174533, request.range, response)

    def do_transform(self, angle, range, response):
        response.point = self.transform(angle, range)

        self.get_logger().info('Sending response %s' % (response.point))

        return response


    def transform(self, radians, range, source_frame="base_scan", target_frame="map"):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                source_frame=source_frame,
                target_frame=target_frame,
                time=now)
        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform {source_frame} to {target_frame}: {ex}')
            return [None, None]

        x = range * math.cos(radians)
        y = range * math.sin(radians)
        point_source = PointStamped(point=Point(x=x, y=y, z=0.0))

        p = do_transform_point(point_source, trans)

        return p


def main(args=None):
    rclpy.init(args=args)
    service = Service()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()