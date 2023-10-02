#!/usr/bin/env python3

import rospy
import rospkg
import sys
import os
import yaml
from geometry_msgs.msg import PoseStamped

def pose_callback(msg, cb_args):
    dirname = os.path.dirname(os.path.abspath(__file__))
    output_file = os.path.join(dirname, "./maps/" + cb_args[0])
    waypoint_name = cb_args[1]
    # Create a dictionary to store the PoseStamped message
    pose_data = {
        waypoint_name: {
            'header': {
                'frame_id': msg.header.frame_id,
                'stamp': {
                    'secs': msg.header.stamp.secs,
                    'nsecs': msg.header.stamp.nsecs
                }
            },
            'pose': {
                'position': {
                    'x': msg.pose.position.x,
                    'y': msg.pose.position.y,
                    'z': msg.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.orientation.x,
                    'y': msg.pose.orientation.y,
                    'z': msg.pose.orientation.z,
                    'w': msg.pose.orientation.w
                }
            }
        }
    }

    # Save the data to the YAML file
    with open(output_file, 'a') as yaml_file:
        yaml.dump(pose_data, yaml_file, default_flow_style=False)
    
    rospy.loginfo(f'{waypoint_name} PoseStamped message saved to {output_file}')
    
    # Shutdown the ROS node after saving the message
    rospy.signal_shutdown('Message saved, exiting...')

def main():
    if len(sys.argv) != 3:
        print("Usage: rosrun my_package save_pose_to_yaml.py <output_file.yaml> <waypoint name>")
        sys.exit(1)

    output_file = sys.argv[1]
    waypoint_name = sys.argv[2]

    rospy.init_node('pose_saver', anonymous=True)
    rospy.Subscriber('/pose', PoseStamped, pose_callback, (output_file, waypoint_name))

    rospy.spin()

if __name__ == '__main__':
    main()
