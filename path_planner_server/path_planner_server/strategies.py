from sensor_msgs.msg import LaserScan
from numpy import argmax
import math
import random


def random_angle(msg: LaserScan):
    index = random.randrange(0, len(msg.ranges))
    range = msg.ranges[index]
    if math.isinf(range):
        range = 1.0
    angle = msg.angle_min + msg.angle_increment * index

    return [angle, range]

def max_distance(msg: LaserScan):
    index = argmax(msg.ranges)
    range = msg.ranges[index]
    if math.isinf(range):
        range = 1.0
    angle = msg.angle_min + msg.angle_increment * index

    return [angle, range]