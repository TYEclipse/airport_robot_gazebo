#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math


class CmdVel2Gazebo:
    def __init__(self):
        rospy.init_node('cmdvel2gazebo', anonymous=True)

        rospy.Subscriber('/cmd_vel', Twist, self.callback, queue_size=1)

        self.pub_steerL = rospy.Publisher(
            '/airport_robot/front_left_velocity_controller/command',
            Float64,
            queue_size=1)
        self.pub_steerR = rospy.Publisher(
            '/airport_robot/front_right_velocity_controller/command',
            Float64,
            queue_size=1)
        self.pub_rearL = rospy.Publisher(
            '/airport_robot/rear_left_velocity_controller/command',
            Float64,
            queue_size=1)
        self.pub_rearR = rospy.Publisher(
            '/airport_robot/rear_right_velocity_controller/command',
            Float64,
            queue_size=1)

        # initial velocity and tire angle are 0
        self.x = 0
        self.z = 0

        # car Wheelbase (in m)
        self.L = 1

        # car Tread
        self.T_front = 0.6
        self.T_rear = 0.6

        # how many seconds delay for the dead man's switch
        self.timeout = rospy.Duration.from_sec(0.2)
        self.lastMsg = rospy.Time.now()

        # maximum steer angle of the "inside" tire
        self.maxsteerInside = 0.6

        # turning radius for maximum steer angle just with the inside tire
        # tan(maxsteerInside) = wheelbase/radius --> solve for max radius at this angle
        rMax = self.L / math.tan(self.maxsteerInside)

        # radius of inside tire is rMax, so radius of the ideal middle tire (rIdeal) is rMax+treadwidth/2
        rIdeal = rMax + (self.T_front / 2.0)

        # maximum steering angle for ideal middle tire
        # tan(angle) = wheelbase/radius
        self.maxsteer = math.atan2(self.L, rIdeal)

        # loop
        rate = rospy.Rate(10)  # run at 10Hz
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def callback(self, data):
        # w = v / r
        self.x = data.linear.x
        # constrain the ideal steering angle such that the ackermann steering is maxed out
        self.z = data.angular.z
        self.lastMsg = rospy.Time.now()

    def publish(self):
        # now that these values are published, we
        # reset the velocity, so that if we don't hear new
        # ones for the next timestep that we time out; note
        # that the tire angle will not change
        # NOTE: we only set self.x to be 0 after 200ms of timeout
        delta_last_msg_time = rospy.Time.now() - self.lastMsg
        msgs_too_old = delta_last_msg_time > self.timeout
        if msgs_too_old:
            self.x = 0
            msgRear = Float64()
            msgRear.data = self.x
            self.pub_rearL.publish(msgRear)
            self.pub_rearR.publish(msgRear)
            msgSteer = Float64()
            msgSteer.data = 0
            self.pub_steerL.publish(msgSteer)
            self.pub_steerR.publish(msgSteer)
            return

        # The self.z is the delta angle in radians of the imaginary front wheel of ackerman model.
        if abs(self.z) >= 0.1 or abs(self.x) >= 0.1:
            T_rear = self.T_rear
            T_front = self.T_front
            L = self.L

            msgRight = Float64()
            msgRight.data = self.x + self.z
            self.pub_rearR.publish(msgRight)
            self.pub_steerR.publish(msgRight)
            msgLeft = Float64()
            msgLeft.data = self.x - self.z
            self.pub_rearL.publish(msgLeft)
            self.pub_steerL.publish(msgLeft)

        else:
            # if we aren't turning
            msgSpeed = Float64()
            msgSpeed.data = 0
            self.pub_rearL.publish(msgSpeed)
            self.pub_rearR.publish(msgSpeed)
            self.pub_steerL.publish(msgSpeed)
            self.pub_steerR.publish(msgSpeed)


if __name__ == '__main__':
    try:
        CmdVel2Gazebo()
    except rospy.ROSInterruptException:
        pass
