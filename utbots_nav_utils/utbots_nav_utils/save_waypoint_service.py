# save_waypoint_service.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from utbots_srvs.srv import SetString  # Update with your actual package name

import yaml
import os
import subprocess

from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from rclpy.action import ActionServer
from std_msgs.msg import String


import ament_index_python
ament_index_python.get_package_share_directory
class WaypointSaver(Node):
    def __init__(self):
        super().__init__('waypoint_saver')
        self.current_pose = None
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.path=(ament_index_python.get_package_share_directory('utbots_nav').rsplit('install/')[0]+
                   "src/utbots_navigation/utbots_nav/map/")
        print(self.path)
        # self.waypoint_file=(os.system("ros2 param get /map_server yaml_filename"))#.rsplit(".yaml")+"_waypoints.yaml"
        map_file = subprocess.check_output(
          ["ros2", "param", "get", "/map_server", "yaml_filename"],
            universal_newlines=True
            ).rsplit("String value is: ")[1]
        self.waypoint_file =map_file.rsplit(".yaml")[0]+"_waypoints.yaml"
        print(self.waypoint_file)
        print(map_file)
        # self.srv = self.create_service(SaveWaypoint, '/utbots/utbots_nav/save_waypoint', self.save_waypoint_callback)
        self.srv = self.create_service(
            SetString,
            '/utbots/utbots_nav/save_waypoint',
            self.save_waypoint_callback)

        self.get_logger().info('Waypoint saver service ready.')

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def save_waypoint_callback(self, request, response):
        name = request.data

        if self.current_pose is None:
            response.success = False
            # response.message = 'No pose received yet.'
            return response

        pose_dict = {
            'position': {
                'x': self.current_pose.position.x,
                'y': self.current_pose.position.y,
                'z': self.current_pose.position.z
            },
            'orientation': {
                'x': self.current_pose.orientation.x,
                'y': self.current_pose.orientation.y,
                'z': self.current_pose.orientation.z,
                'w': self.current_pose.orientation.w
            }
        }

        # Load or create the YAML file
        if os.path.exists(self.waypoint_file):
            with open(self.waypoint_file, 'r') as f:
                try:
                    data = yaml.safe_load(f) or {}
                except yaml.YAMLError:
                    data = {}
        else:
            data = {}

        data.setdefault('waypoints', {})
        data['waypoints'][name] = pose_dict
    
        with open(self.waypoint_file, 'w') as f:
            yaml.dump(data, f)

        self.get_logger().info(f"Saved waypoint '{name}'")
        response.success = True
        # response.message = f"Waypoint '{name}' saved successfully."
        return response

def main():
    rclpy.init()
    node = WaypointSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
