#!/usr/bin/env python

from numpy.core.fromnumeric import _all_dispatcher
from numpy.core.function_base import _linspace_dispatcher
import rospy
from rospy.rostime import Duration, Time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
import math
import numpy as np


class ScanCompleter:
    def __init__(self):

        rospy.init_node('scan_completer', anonymous=True)

        rospy.Subscriber('/scan_raw_1', LaserScan,
                         self.scanCallback1, queue_size=1)
        self.pub_scan1 = rospy.Publisher(
            '/scan_1', LaserScan, queue_size=1)

        rospy.Subscriber('/scan_raw_2', LaserScan,
                         self.scanCallback2, queue_size=1)
        self.pub_scan2 = rospy.Publisher(
            '/scan_2', LaserScan, queue_size=1)

        rospy.Subscriber('/scan_raw_3', LaserScan,
                         self.scanCallback3, queue_size=1)
        self.pub_scan3 = rospy.Publisher(
            '/scan_3', LaserScan, queue_size=1)

        rospy.Subscriber('/scan_raw_4', LaserScan,
                         self.scanCallback4, queue_size=1)
        self.pub_scan4 = rospy.Publisher(
            '/scan_4', LaserScan, queue_size=1)

        rospy.spin()

    def scanCallback1(self, data):

        data_output = data

        data_output.time_increment = 0.0001
        data_output.scan_time = 0.1

        self.pub_scan1.publish(data_output)

    def scanCallback2(self, data):

        data_output = data

        data_output.time_increment = 0.0001
        data_output.scan_time = 0.1

        self.pub_scan2.publish(data_output)

    def scanCallback3(self, data):

        data_output = data

        data_output.time_increment = 0.0001
        data_output.scan_time = 0.1

        self.pub_scan3.publish(data_output)

    def scanCallback4(self, data):

        data_output = data

        data_output.time_increment = 0.0001
        data_output.scan_time = 0.1

        self.pub_scan4.publish(data_output)


if __name__ == '__main__':
    try:
        ScanCompleter()
        sc2 = ScanCompleter('/scan_raw_2', '/scan_2')
        sc3 = ScanCompleter('/scan_raw_3', '/scan_3')
        sc4 = ScanCompleter('/scan_raw_4', '/scan_4')
    except rospy.ROSInterruptException:
        pass
