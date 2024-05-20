import rclpy
from rclpy.publisher import Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger
from geometry_msgs.msg import PointStamped, Pose, Vector3
from visualization_msgs.msg import Marker
import random
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Time

MARKER_COLORS = [ColorRGBA(r=1.0, g=0.984, b=0.0, a=1.0), ColorRGBA(r=0.929, g=0.098, b=1.0, a=1.0)]

def getAddMarker(point: PointStamped, color_index = 0) -> Marker:
    """
        color_index: index for MARKER_COLORS
    """
    color = MARKER_COLORS[color_index]
    marker = Marker()
    marker.header = point.header
    marker.ns = 'walker'
    marker.action = Marker.ADD
    marker.type = Marker.SPHERE
    marker.id = random.randint(100, 1000)
    marker.pose = Pose()
    marker.pose.position = point.point
    marker.scale = Vector3(x=0.3, y=0.3, z=0.3)
    marker.color = color
    marker.lifetime = rclpy.duration.Duration(seconds=2).to_msg()

    return marker

def remove_marker(marker_id: int, time: Time, publisher: Publisher, logger: RcutilsLogger):
    """
        How to use time parameter: node.get_clock().now()
    """
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = time.to_msg()
    marker.action = 2
    marker.ns = 'walker'
    marker.id = marker_id

    publisher.publish(marker)
    logger.info('marker removed with id: %d' % (marker.id))