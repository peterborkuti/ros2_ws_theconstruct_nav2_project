from path_planner_server.angle_utils import UnifiedLaserScan, index_from_grad, index_from_rad
from math import pi
import pytest
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

def index_from_rad_simu(rad: float) -> int:
    return index_from_rad(rad, 0.0, 6.28000020980835, 0.01749303564429283)

def index_from_rad_real_robot(rad: float) -> int:
    return index_from_rad(rad, -3.1241390705108643, 3.1415927410125732, 0.008714509196579456)

def index_from_grad_simu(grad: int) -> int:
    return index_from_grad(grad, 0.0, 6.28000020980835, 0.01749303564429283)

def index_from_grad_real_robot(grad: int) -> int:
    return index_from_grad(grad, -3.1241390705108643, 3.1415927410125732, 0.008714509196579456)


def test_index_from_rad_simu():
    assert index_from_rad_simu(0) == 0
    assert index_from_rad_simu(pi/2) == 90
    assert index_from_rad_simu(pi) == 180
    assert index_from_rad_simu(3 * pi/2) == 269
    assert index_from_rad_simu(-pi/2) == 269

def test_index_from_rad_real_robot():
    assert index_from_rad_real_robot(0) == 358
    assert index_from_rad_real_robot(pi/2) == 539
    assert index_from_rad_real_robot(pi) == 719
    assert index_from_rad_real_robot(3 * pi/2) == 178
    assert index_from_rad_real_robot(-pi/2) == 178

def test_index_from_rad_out_of_range():
    assert index_from_rad_simu(-2 * pi) == 0
    with pytest.raises(AssertionError):
        index_from_rad_simu(2 * pi + 0.01)
    with pytest.raises(AssertionError):
        index_from_rad_simu(-2 * pi - 0.01)

def test_index_from_grad_simu():
    assert index_from_grad_simu(0) == 0
    assert index_from_grad_simu(90) == 90
    assert index_from_grad_simu(180) == 180
    assert index_from_grad_simu(270) == 269
    assert index_from_grad_simu(-90) == 269

def test_index_from_grad_real_robot():
    assert index_from_grad_real_robot(0) == 358
    assert index_from_grad_real_robot(90) == 539
    assert index_from_grad_real_robot(180) == 719
    assert index_from_grad_real_robot(270) == 178
    assert index_from_grad_real_robot(-90) == 178

def test_index_from_grad_out_of_range():
    assert index_from_grad_simu(-360) == 0
    with pytest.raises(AssertionError):
        index_from_grad_simu(361)
    with pytest.raises(AssertionError):
        index_from_grad_simu(-361)

def test_unified_laserscan_init_simu():
    scan = LaserScan(angle_min=0.0, angle_max=2*pi, angle_increment=2*pi/360.0)
    scan.ranges = [float(i) for i in range(360)]

    uscan = UnifiedLaserScan(scan)

    assert uscan.ranges[100] == 100.0

def test_unified_laserscan_init_robot():
    scan = LaserScan(angle_min=-3.1241390705108643, angle_max=3.1415927410125732, angle_increment=0.008714509196579456)
    scan.ranges = [float(i) for i in range(720)]

    uscan = UnifiedLaserScan(scan)

    assert uscan.ranges[-180] == 719
    assert uscan.ranges[-90] == 178
    assert uscan.ranges[0] == 358
    assert uscan.ranges[90] == 539
    assert uscan.ranges[180] == 719
    assert uscan.ranges[270] == 178
    assert uscan.ranges[359] == 356