import rclpy
from rclpy.publisher import Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger
from geometry_msgs.msg import PointStamped, Pose, Vector3
from visualization_msgs.msg import Marker
import random
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Time

def getAddMarker(point: PointStamped, color: ColorRGBA) -> Marker:
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

#time: node..get_clock().now()
def remove_marker(marker_id: int, time: Time, publisher: Publisher, logger: RcutilsLogger):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = time.to_msg()
        marker.action = 2
        marker.ns = 'walker'
        marker.id = marker_id

        publisher.publish(marker)
        logger.info('marker removed with id: %d' % (marker.id))