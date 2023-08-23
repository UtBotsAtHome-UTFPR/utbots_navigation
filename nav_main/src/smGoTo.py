#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Twist
from math import sqrt, atan2, degrees

class SmGoTo():
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["sent","too_close"],
                             input_keys=["type", "coordinate", "pointStamped"])
        
    self.msg_cmdVel = Twist()
    self.pub_cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    self.kp_linear = 0.01
    self.kp_angular = 0.01
    
    def execute(self, userdata):
        x, y = 0, 0
        if userdata.type == "pointStamped":
            x = userdata.pointStamped.point.x
            y = userdata.pointStamped.point.y
        elif userdata.type == "coordinate":
            x = userdata.coordinate.x
            y = userdata.coordinate.y
            
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
        else:
            self.msg_cmdVel.linear.x = 0

        rospy.loginfo("error_linear: {}, error_angular: {}".format(error_linear, error_angular))
        rospy.loginfo("v_x: {}, w_z: {}\n".format(self.msg_cmdVel.linear.x, self.msg_cmdVel.angular.z))
        
        if self.msg_cmdVel.linear.x == 0 and self.msg_cmdVel.linear.y == 0:
            return "too_close"
        else:
            self.pub_cmdVel.publish(self.msg_cmdVel)
            return "sent"