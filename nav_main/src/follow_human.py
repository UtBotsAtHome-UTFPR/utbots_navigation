#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Twist
from math import sqrt, atan2, degrees

class FollowHuman():

    def __init__(self):
        rospy.init_node('follow_human', anonymous=True)

        self.msg_targetStatus = String()
        self.msg_targetPoint = PointStamped()
        self.msg_cmdVel = Twist()

        self.sub_targetStatus = rospy.Subscriber("/apollo/vision/lock/target/status", String, self.callback_targetStatus)
        self.sub_targetPoint = rospy.Subscriber("/apollo/vision/lock/target/point", PointStamped, self.callback_targetPoint)
        self.pub_cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.loopRate = rospy.Rate(30)

        self.kp_linear = 0.01
        self.kp_angular = 0.01

        rospy.loginfo("Looping...")
        self.mainLoop()

    def callback_targetStatus(self, msg):
        self.msg_targetStatus = msg

    def callback_targetPoint(self, msg):
        self.msg_targetPoint = msg

    def mainLoop(self):
        while rospy.is_shutdown() == False:
            self.loopRate.sleep()

            if self.msg_targetStatus.data == "Located":
                x = self.msg_targetPoint.point.x
                y = self.msg_targetPoint.point.y

                # Angular
                error_angular = degrees(atan2(y, x))
                if abs(error_angular) > 5:
                    self.msg_cmdVel.angular.z += self.kp_angular * error_angular
                else:
                    self.msg_cmdVel.angular.z = 0

                # Linear
                error_linear = sqrt(x*x + y*y)
                if abs(error_linear) > 2.0:
                    self.msg_cmdVel.linear.x += self.kp_linear * error_linear
                    # self.msg_cmdVel.linear.x = 0
                else:
                    self.msg_cmdVel.linear.x = 0

                rospy.loginfo("error_linear: {}, error_angular: {}".format(error_linear, error_angular))
                rospy.loginfo("v_x: {}, w_z: {}\n".format(self.msg_cmdVel.linear.x, self.msg_cmdVel.angular.z))
                
            else:
                self.msg_cmdVel.linear.x = 0
                self.msg_cmdVel.angular.z = 0
            
            self.pub_cmdVel.publish(self.msg_cmdVel)


if __name__ == "__main__":
    FollowHuman()