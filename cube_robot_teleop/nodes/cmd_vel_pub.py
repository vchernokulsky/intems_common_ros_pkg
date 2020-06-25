#! /usr/bin/env python
from __future__ import division

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class CmdVelPubNode:

    def __init__(self):
        self.maxLinVel = 0
        self.maxAngVel = 0
        self.rate = 0
        self.timeout = 0
        self.name = ''
        self.needLogInfo = False

        self.linearVelocity = 0.0
        self.angularVelocity = 0.0
        self.lastControlTime = 0

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def main(self):
        rospy.init_node('cmd_vel_publisher')
        self.name = rospy.get_name()
        rospy.loginfo("{0} started".format(self.name))

        rospy.Subscriber("lin_vel", Float32, self.lin_callback)
        rospy.Subscriber("ang_vel", Float32, self.ang_callback)

        self.maxLinVel = float(rospy.get_param('~max_lin_vel', 3.82))
        self.maxAngVel = float(rospy.get_param('~max_ang_vel', 3.82))
        self.rate = float(rospy.get_param('~cmd_vel_pub_rate', 2.0))
        self.timeout = float(rospy.get_param('~cmd_vel_pub_timeout', 600.0))
        self.needLogInfo = float(rospy.get_param('~cmd_vel_pub_need_log_info', False))

        rate = rospy.Rate(self.rate)
        self.lastControlTime = rospy.get_time()
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def publish(self):
        if rospy.get_time() - self.lastControlTime < self.timeout:
            msg = self.get_twist()
            self.cmd_vel_pub.publish(msg)
            if self.needLogInfo:
                rospy.loginfo("[%s] velocity: lin = %s; ang = %s", self.name.upper(), msg.linear.x, msg.angular.z)
        else:
            msg = self.get_stop()
            self.cmd_vel_pub.publish(msg)
            if self.needLogInfo:
                rospy.loginfo("[%s] velocity: lin = 0; ang = 0", self.name.upper())

    def get_twist(self):
        twist = Twist()
        twist.linear.x = self.linearVelocity
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.angularVelocity
        return twist

    def get_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        return twist

    def constrain(self, vel, low, high):
        if vel < low:
            vel = low
        elif vel > high:
            vel = high
        return vel

    def check_linear_limit_velocity(self, vel):
        new_vel = self.constrain(vel, -self.maxLinVel, self.maxLinVel)
        rospy.loginfo("[%s] linear_speed_m/s=%s", self.name.upper(), new_vel)
        return new_vel

    def check_angular_limit_velocity(self, vel):
        return self.constrain(vel, -self.maxAngVel, self.maxAngVel)

    def lin_callback(self, lin_vel):
        self.linearVelocity = self.check_linear_limit_velocity(lin_vel.data)
        if self.needLogInfo:
            rospy.loginfo("[%s] linear_speed_m/s=%s", self.name.upper(),  self.linearVelocity)
        self.lastControlTime = rospy.get_time()

    def ang_callback(self, ang_vel):
        self.angularVelocity = self.check_angular_limit_velocity(ang_vel.data)
        if self.needLogInfo:
            rospy.loginfo("[%s] angular_speed_r/s=%s", self.name.upper(), self.angularVelocity)
        self.lastControlTime = rospy.get_time()


if __name__ == '__main__':
    try:
        node = CmdVelPubNode()
        node.main()
    except rospy.ROSInterruptException:
        pass
