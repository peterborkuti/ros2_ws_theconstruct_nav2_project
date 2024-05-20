
from math import pi
from typing import List
from sensor_msgs.msg import LaserScan

two_pi = 2 * pi

def index_from_rad(rad: float, angle_min: float, angle_max: float, angle_increment: float):
    assert rad >= -two_pi and rad <= two_pi, "Invalid radian"

    angle = rad - angle_min

    if angle < 0:
        angle += two_pi
    if angle > two_pi:
        angle -= two_pi

    index = round(angle / angle_increment)

    ranges = round((angle_max - angle_min)/angle_increment) + 1
    return min(max(0, index), ranges - 1)

def index_from_grad(grad: int, rad_angle_min: float, rad_angle_max: float, rad_angle_increment: float):
    rad = grad * two_pi / 360.0
    return index_from_rad(rad, rad_angle_min, rad_angle_max, rad_angle_increment)

class UnifiedLaserScan():
    def __init__(self, msg: LaserScan):
        self._msg = msg
        self.header = msg.header
        self.angle_min = 0
        self.angle_max = two_pi
        self.angle_increment = two_pi/360.0
        self.time_increment = msg.time_increment
        self.scan_time = msg.scan_time
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        self.ranges = msg.ranges if len(msg.ranges) == 360 else self._get_ranges()
        # TODO: intensities are not transformed
        self.intensities = msg.intensities

    def _get_ranges(self) -> List[float]:
        return [
            self._msg.ranges[
                index_from_grad(i,
                    self._msg.angle_min, self._msg.angle_max,
                    self._msg.angle_increment)
            ]
            for i in range(360)
        ]


 