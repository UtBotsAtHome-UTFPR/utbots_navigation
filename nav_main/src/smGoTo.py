#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Twist, PoseStamped
from math import sqrt, atan2, degrees
import smach

class SmGoTo():
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["adjusted_vel", "stop_vel", "sent"],
                             input_keys=["type", "coordinate"])
        
        self.msg_cmdVel = Twist()
        self.pub_cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_pose = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.kp_linear = 0.01
        self.kp_angular = 0.01
    
    def execute(self, userdata):
        msg_pose = PoseStamped
        pos_x, pos_y = 0, 0
        
        if isinstance(userdata.coordinate, PointStamped):
            pos_x = userdata.coordinate.point.x
            pos_y = userdata.coordinate.point.y
            msg_pose.pose.position = userdata.coordinate
        elif isinstance(userdata.coordinate, PoseStamped):
            pos_x = userdata.coordinate.position.x
            pos_y = userdata.coordinate.position.y
            msg_pose = userdata.coordinate
            
        if userdata.type == "cmd_vel":
            # Angular
            error_angular = degrees(atan2(pos_y, pos_x))
            if abs(error_angular) > 5:
                self.msg_cmdVel.angular.z += self.kp_angular * error_angular
            else:
                self.msg_cmdVel.angular.z = 0

            # Linear
            error_linear = sqrt(pos_x*pos_x + pos_y*pos_y)
            if abs(error_linear) > 2.0:
                self.msg_cmdVel.linear.x += self.kp_linear * error_linear
            else:
                self.msg_cmdVel.linear.x = 0

            rospy.loginfo("error_linear: {}, error_angular: {}".format(error_linear, error_angular))
            rospy.loginfo("v_x: {}, w_z: {}\n".format(self.msg_cmdVel.linear.x, self.msg_cmdVel.angular.z))
            
            if self.msg_cmdVel.linear.x == 0 and self.msg_cmdVel.angular.z == 0:
                return "reached"
            else:
                self.pub_cmdVel.publish(self.msg_cmdVel)
                return "adjusted_vel"
            
        else:
            self.pub_pose.publish(self.msg_pose)
            return "sent"
            
