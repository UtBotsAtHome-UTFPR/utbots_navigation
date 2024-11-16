import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
import time

class nav_pub(Node):
    def __init__(self):
        
        rclpy.init()
        super().__init__('nav_test')

        logger = self.get_logger()
        logger.info("Calling basic navigator")

        self.navigator = BasicNavigator()

        logger.info("Calling waitUntilNav2Active")
        self.navigator.waitUntilNav2Active()

        goal = PoseStamped()
        goal.header.frame_id = "map"

        goal.pose.position.x = 2.0
        goal.pose.position.y = 0.0

        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 0.0

        
        logger.info("Calling go to pose")

        self.navigator.goToPose(goal, behavior_tree="")
        time.sleep(3)

        i = 0
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()

            if feedback and i % 5 == 0:
                self.get_logger().info('Estimated time of arrival: ' + 
                                        '{0:.0f}'.format(
                                            Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + 
                                            ' seconds.')
            i += 1

            if feedback.estimated_time_remaining.sec == 0:
                break

        time.sleep(5)

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



def main():
    a = nav_pub()


if __name__ == '__main__':
    main()
