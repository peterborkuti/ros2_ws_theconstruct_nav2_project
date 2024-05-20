from typing import List, Dict
from math import isinf, sin, cos, pi, sqrt, atan2
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from rclpy.node import Node


class OuterRectangle:
    xs = []
    ys = []
    x_plus = 0
    x_minus = 0
    y_plus = 0
    y_minus = 0
    beta = 0
    area = float("inf")
    header = Header()

    def corners(self, distance) -> List[PointStamped]:
        points = [
            dict(x=self.x_minus + distance, y=self.y_minus + distance),
            dict(x=self.x_minus + distance, y=self.y_plus - distance),
            dict(x=self.x_plus - distance, y=self.y_plus - distance),
            dict(x=self.x_plus - distance, y=self.y_minus + distance)
        ]

        return [self.rotate(p) for p in points]
    
    def rotate(self, coords: Dict[str, float]) -> PointStamped:
        x = coords['x']
        y = coords['y']
        range = sqrt(x ** 2 + y ** 2)
        alfa = atan2(y, x)

        return PointStamped(
            header=self.header,
            point=Point(
                x=range * cos(alfa - self.beta),
                y=range * sin(alfa - self.beta),
                z=0.0))

    def invalid(self) -> bool:
        return self.area == float("inf")

    def onTheWall(self, index: int, delta: float) -> bool:
        return self.eq(self.xs[index], self.x_plus, self.x_minus, delta) or self.eq(self.ys[index], self.y_plus, self.y_minus, delta)

    def eq(self, v, v_plus, v_minus, delta: float) -> bool:
        return abs(v - v_plus) < delta or abs(v - v_minus) < delta


def find_outer_rectangle(node: Node, ranges: List[float], angle_min: float, angle_increment: float, header: Header) -> OuterRectangle:
    node.get_logger().debug('find outer rectangle started')

    rectangle = OuterRectangle()
    rectangle.header = header

    offset_pi_half = round(pi / 2.0 / angle_increment)
    for beta_index in range(offset_pi_half):
        node.get_logger().debug('betaindex: %d' % beta_index)
        angles: List[float] = [(i + beta_index) * angle_increment + angle_min for i,r in enumerate(ranges)]

        minr = min(ranges)
        xs: List[float] = [(minr if isinf(r) else r) * cos(angles[i]) for i,r in enumerate(ranges)]
        ys: List[float] = [(minr if isinf(r) else r) * sin(angles[i]) for i,r in enumerate(ranges)]

        # rectangle height upward from the robot
        x_plus = max(xs)
        # rectangle height downward from the robot, this is negative
        x_minus = min(xs)
        # rectangle width on the right side of the robot
        y_plus = max(ys)
        # rectangle width on the left side of the robot, this is negative
        y_minus = min(ys)

        node.get_logger().debug('max rectangle: (%.2f, %.2f, %.2f, %.2f)' % (x_plus, x_minus, y_plus, y_minus))

        rectangle_area = (x_plus - x_minus) * (y_plus - y_minus)
        if rectangle.area > rectangle_area:
            rectangle.beta = beta_index * angle_increment + angle_min
            rectangle.area = rectangle_area
            rectangle.xs = xs
            rectangle.ys = ys
            rectangle.x_plus = x_plus
            rectangle.x_minus = x_minus
            rectangle.y_plus = y_plus
            rectangle.y_minus = y_minus

    return rectangle

