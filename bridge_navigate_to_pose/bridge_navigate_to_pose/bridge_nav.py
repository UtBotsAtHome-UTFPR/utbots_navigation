import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
import time

class NavigationNode(Node):
    def __init__(self):
        super().__init__('bridge_nav')

        # Initialize BasicNavigator
        self.navigator = BasicNavigator()

        # Set the initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = -1.999406229747971
        initial_pose.pose.position.y = -0.5000001448860989
        initial_pose.pose.orientation.z = 0.00017372500724032591
        initial_pose.pose.orientation.w = 0.9999945227842333
        self.navigator.setInitialPose(initial_pose)

        self.bt = ""

        # Wait for navigation to fully activate
        self.navigator.waitUntilNav2Active()

        # Subscribe to the goal topic
        self.create_subscription(
            PoseStamped,
            '/bridge_navigate_to_pose/goal',
            self.goal_callback,
            QoSProfile(depth=10)
        )
        self.create_subscription(
            String,
            'bridge_navigate_to_pose/bt',
            self.bt_callback,
            QoSProfile(depth=10)
        )

        # Create a publisher for the result topic
        self.result_publisher = self.create_publisher(String, '/bridge_navigate_to_pose/result', QoSProfile(depth=10))

    def bt_callback(self, msg):
        self.bt = msg.data

    def goal_callback(self, goal_pose):
        # Start navigating to the goal pose
        
        if self.bt == "":
            self.get_logger().info("Using default BT")
            bt_path = ""
        else:
            self.get_logger().info(self.bt)

        if self.bt == "follow_point":
            bt_path = "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/follow_point.xml"

        self.navigator.goToPose(goal_pose, behavior_tree=bt_path)#"/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/follow_point.xml"

        time.sleep(1.5)

        i = 0
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()

            if feedback and i % 5 == 0:
                self.get_logger().info('Estimated time of arrival: ' + 
                                        '{0:.0f}'.format(
                                            Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + 
                                            ' seconds.')

                # Navigation timeout for cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()
                if feedback is not None and self.bt == "follow_point":
                    if feedback.estimated_time_remaining.sec == 0:
                        self.navigator.cancelTask()
            i += 1

        # Handle the result of the navigation
        result = self.navigator.getResult()
        result_message = String()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
            result_message.data = 'Succeeded'
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
            result_message.data = 'Canceled'
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')
            result_message.data = 'Failed'
        else:
            self.get_logger().info('Goal has an invalid return status!')
            result_message.data = 'Invalid'

        # Publish the result
        print(result_message)
        self.result_publisher.publish(result_message)

def main(args=None):
    rclpy.init(args=args)
    navigation_node = NavigationNode()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

