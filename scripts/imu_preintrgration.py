#!/usr/bin/env python

from numpy.core.fromnumeric import _all_dispatcher
from numpy.core.function_base import _linspace_dispatcher
from numpy.lib.function_base import quantile
import rospy
from rospy.rostime import Duration, Time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
import numpy as np
import tf


class ImuPreintegration:
    def __init__(self):
        rospy.init_node('imu_filter', anonymous=True)

        rospy.Subscriber('/imu', Imu, self.imuCallback, queue_size=1)

        self.pub_imuOdom = rospy.Publisher('/imu_odom', Odometry, queue_size=1)

        self.tf = tf.Transformer(True, rospy.Duration(10.0))

        # IMU State
        # Orientation (quaternion parts)    N/A     1:4
        # Position (NED or ENU)             m       5:7
        # Velocity (NED or ENU)             m/s     8:10
        # Gyroscope Bias (XYZ)              rad/s   11:13
        # Accelerometer Bias (XYZ)          m/s2    14:16
        # Visual Odometry Scale (XYZ)       N/A     17
        self.imuState = np.zeros((17, 1))
        self.imuState[0] = 1
        self.imuState[16] = 1

        self.last_stamp = Time()
        self.stamp = 0

        self.flag = 0

        rospy.spin()

    def imuCallback(self, data):

        self.stamp = data.header.stamp
        dt = self.stamp - self.last_stamp
        dt = dt.to_sec() if dt.to_sec() < 0.5 else 0.05
        self.last_stamp = data.header.stamp
        # print("dt = ", dt)

        acceleration = np.array([
            data.linear_acceleration.x, data.linear_acceleration.y,
            data.linear_acceleration.z
        ])
        g = np.array([0.0, 0.0, 9.8])

        accelX = acceleration[0]
        accelY = acceleration[1]
        accelZ = acceleration[2]

        angularVelocity = np.array([
            data.angular_velocity.x, data.angular_velocity.y,
            data.angular_velocity.z
        ])

        gyroX = angularVelocity[0]
        gyroY = angularVelocity[1]
        gyroZ = angularVelocity[2]

        # Reference : https://ww2.mathworks.cn/help/fusion/ref/insfiltererrorstate.html
        q0 = self.imuState[0, 0]
        q1 = self.imuState[1, 0]
        q2 = self.imuState[2, 0]
        q3 = self.imuState[3, 0]

        positionN = self.imuState[4, 0]
        positionE = self.imuState[5, 0]
        positionD = self.imuState[6, 0]
        vN = self.imuState[7, 0]
        vE = self.imuState[8, 0]
        vD = self.imuState[9, 0]
        gyrobiasX = self.imuState[10, 0]
        gyrobiasY = self.imuState[11, 0]
        gyrobiasZ = self.imuState[12, 0]
        accelbiasX = self.imuState[13, 0]
        accelbiasY = self.imuState[14, 0]
        accelbiasZ = self.imuState[15, 0]
        # scaleFactor = self.imuState[16]

        d_gyroX = gyrobiasX / 2 - gyroX / 2
        d_gyroY = gyrobiasY / 2 - gyroY / 2
        d_gyroZ = gyrobiasZ / 2 - gyroZ / 2
        d_accelX = accelbiasX - accelX
        d_accelY = accelbiasY - accelY
        d_accelZ = accelbiasZ - accelZ
        q123 = q1 * d_accelX + q2 * d_accelY + q3 * d_accelZ
        q30_1 = q3 * d_accelX + q0 * d_accelY - q1 * d_accelZ
        q_210 = -q2 * d_accelX + q1 * d_accelY + q0 * d_accelZ
        q0_32 = q0 * d_accelX - q3 * d_accelY + q2 * d_accelZ

        q0_next = q0 + \
            (+q1 * d_gyroX + q2 * d_gyroY + q3 * d_gyroZ)*dt
        q1_next = q1 + \
            (-q0 * d_gyroX + q3 * d_gyroY - q2 * d_gyroZ)*dt
        q2_next = q2 + \
            (-q3 * d_gyroX - q0 * d_gyroY + q1 * d_gyroZ)*dt
        q3_next = q3 + \
            (+q2 * d_gyroX - q1 * d_gyroY - q0 * d_gyroZ)*dt
        q_norm = math.sqrt(q0_next*q0_next+q1_next *
                           q1_next+q2_next*q2_next+q3_next*q3_next)
        positionN_next = positionN + dt * vN
        positionE_next = positionE + dt * vE
        positionD_next = positionD + dt * vD
        vN_next = vN - dt * (q0 * q0_32 + q1 * q123 + q2 * q_210 -
                             q3 * q30_1 + g[0])
        vE_next = vE - dt * (q0 * q30_1 - q1 * q_210 + q2 * q123 +
                             q3 * q0_32 + g[1])
        vD_next = vD - dt * (q0 * q_210 + q1 * q30_1 - q2 * q0_32 -
                             q3 * q123 + g[2])

        self.imuState[0, 0] = q0_next/q_norm if False else data.orientation.w
        self.imuState[1, 0] = q1_next/q_norm if False else data.orientation.x
        self.imuState[2, 0] = q2_next/q_norm if False else data.orientation.y
        self.imuState[3, 0] = q3_next/q_norm if False else data.orientation.z
        self.imuState[4, 0] = positionN_next
        self.imuState[5, 0] = positionE_next
        self.imuState[6, 0] = positionD_next
        self.imuState[7, 0] = vN_next
        self.imuState[8, 0] = vE_next
        self.imuState[9, 0] = vD_next

        # self.imuState[10] = gyrobiasX
        # self.imuState[11] = gyrobiasY
        # self.imuState[12] = gyrobiasZ
        # self.imuState[13] = accelbiasX
        # self.imuState[14] = accelbiasY
        # self.imuState[15] = accelbiasZ
        # self.imuState[16] = scaleFactor

        if self.flag % 200 == 0:
            print("acceleration", acceleration)
            print("d_accelX", d_accelX, d_accelY, d_accelZ)
            print("self.imuState", self.imuState[:10])

        self.flag = self.flag+1

        data_output = Odometry()
        data_output.header = data.header
        data_output.header.frame_id = "odom"
        data_output.child_frame_id = "imu_link"
        data_output.pose.pose.orientation.w = self.imuState[0, 0]
        data_output.pose.pose.orientation.x = self.imuState[1, 0]
        data_output.pose.pose.orientation.y = self.imuState[2, 0]
        data_output.pose.pose.orientation.z = self.imuState[3, 0]
        data_output.pose.pose.position.x = self.imuState[4]
        data_output.pose.pose.position.y = self.imuState[5]
        data_output.pose.pose.position.z = self.imuState[6]
        self.pub_imuOdom.publish(data_output)

        m = TransformStamped()
        m.header.frame_id = "odom"
        m.child_frame_id = "imu_link"
        m.transform.rotation = data_output.pose.pose.orientation
        m.transform.translation.x = data_output.pose.pose.position.x
        m.transform.translation.y = data_output.pose.pose.position.y
        m.transform.translation.z = data_output.pose.pose.position.z
        self.tf.setTransform(m)


if __name__ == '__main__':
    try:
        ImuPreintegration()
    except rospy.ROSInterruptException:
        pass
