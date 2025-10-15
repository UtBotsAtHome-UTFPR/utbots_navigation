import rclpy
from rclpy.node import Node
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient,ActionServer
from utbots_actions.action import NavigateToWayPoint  # Update with your actual package name
from std_msgs.msg import String, Bool
from rclpy.duration import Duration
import subprocess
import os

class NavToSavedPose(Node):
    def __init__(self):
        super().__init__('nav_to_saved_pose')
        self.action_client = ActionClient(
                        self, 
                        NavigateToPose,
                        'navigate_to_pose'
                        )
        
        self.action_server = ActionServer(
            self,
            NavigateToWayPoint,
            'navigate_to_way_point',
            self.nav_wp_cb
        )

        map_file = subprocess.check_output(
          ["ros2", "param", "get", "/map_server", "yaml_filename"],
            universal_newlines=True
            ).rsplit("String value is: ")[1]
        
        self.waypoint_file =map_file.rsplit(".yaml")[0]+"_waypoints.yaml"
        self.get_logger().info('Node ready')

    async def nav_wp_cb(self, goal_handle):

        self.get_logger().info('Goal received. Waiting for new message...')
        
        result = NavigateToWayPoint.Result()

        request=goal_handle.request

        try:
            wp=request.waypoint.data
            self.send_goal(waypoint_name=wp,
                           yaml_path=self.waypoint_file)
            goal_handle.succeed()
            data=Bool()
            data.data=True
            result.result= data
            return result
        except Exception as e:
                self.get_logger().error(f"Error Sending Waypoint: {str(e)}")
                goal_handle.abort()
                data=Bool()
                data.data=False
                result.result= data
                return result
       

    def send_goal(self, waypoint_name, yaml_path):
        try:
            with open(yaml_path, 'r') as f:
                poses = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Error opening waypoint file: {str(e)}")
            raise e

        pose = poses['waypoints'][waypoint_name]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = pose['position']['x']
        goal_msg.pose.pose.position.y = pose['position']['y']
        goal_msg.pose.pose.position.z = pose['position']['z']

        goal_msg.pose.pose.orientation.x = pose['orientation']['x']
        goal_msg.pose.pose.orientation.y = pose['orientation']['y']
        goal_msg.pose.pose.orientation.z = pose['orientation']['z']
        goal_msg.pose.pose.orientation.w = pose['orientation']['w']

        try:
            self.action_client.wait_for_server()
            self.action_client.send_goal_async(goal_msg)
        except Exception as e:
            self.get_logger().error(f"Error sending goal: {str(e)}")
            raise e
        
def main():
    rclpy.init()
    node = NavToSavedPose()

    rclpy.spin(node)

if __name__ == '__main__':
    main()
