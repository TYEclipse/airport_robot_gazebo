#!/usr/bin/env python

from numpy.core.fromnumeric import _all_dispatcher
from numpy.core.function_base import _linspace_dispatcher
import rospy
from rospy.rostime import Duration, Time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math
import numpy as np


class trackingABF:
    def __init__(self, x_0=0.0, v_0=0.0, a_0=0.0, alpha=0.05, beta=0.002):
        self.x_k = x_0
        self.v_k = v_0
        self.alpha = alpha
        self.beta = beta
        self.tuning = False

    def predic(self, x, dt=0.005):
        self.x_k = self.x_k + self.v_k * dt
        self.v_k = self.v_k

        x_r = x - self.x_k
        v_r = x_r / dt

        self.x_k = self.x_k + self.alpha * x_r
        self.v_k = self.v_k + self.beta * v_r

        return self.x_k


class ImuFilter:
    def __init__(self):
        rospy.init_node('imu_filter', anonymous=True)

        rospy.Subscriber('/imu', Imu, self.imuCallback, queue_size=1)

        self.pub_imuCorrect = rospy.Publisher('/imu_raw', Imu, queue_size=1)

        self.last_stamp = Time()
        self.stamp = 0

        self.ori_w = trackingABF()
        self.ori_x = trackingABF()
        self.ori_y = trackingABF()
        self.ori_z = trackingABF()

        self.av_x = trackingABF()
        self.av_y = trackingABF()
        self.av_z = trackingABF()

        self.la_x = trackingABF()
        self.la_y = trackingABF()
        self.la_z = trackingABF()

        rospy.spin()

    def imuCallback(self, data):

        self.stamp = data.header.stamp
        dt = self.stamp - self.last_stamp
        self.last_stamp = data.header.stamp
        # print("dt = ", dt.to_sec())

        data_output = Imu()
        data_output.orientation.w = self.ori_w.predic(data.orientation.w,
                                                      dt.to_sec())
        data_output.orientation.x = self.ori_x.predic(data.orientation.x,
                                                      dt.to_sec())
        data_output.orientation.y = self.ori_y.predic(data.orientation.y,
                                                      dt.to_sec())
        data_output.orientation.z = self.ori_z.predic(data.orientation.z,
                                                      dt.to_sec())

        data_output.angular_velocity.x = self.av_x.predic(
            data.angular_velocity.x, dt.to_sec())
        data_output.angular_velocity.y = self.av_y.predic(
            data.angular_velocity.y, dt.to_sec())
        data_output.angular_velocity.z = self.av_z.predic(
            data.angular_velocity.z, dt.to_sec())

        data_output.linear_acceleration.x = self.la_x.predic(
            data.linear_acceleration.x, dt.to_sec())
        data_output.linear_acceleration.y = self.la_y.predic(
            data.linear_acceleration.y, dt.to_sec())
        data_output.linear_acceleration.z = self.la_z.predic(
            data.linear_acceleration.z, dt.to_sec())

        data_output.header = data.header

        self.pub_imuCorrect.publish(data_output)


if __name__ == '__main__':
    try:
        ImuFilter()
    except rospy.ROSInterruptException:
        pass
