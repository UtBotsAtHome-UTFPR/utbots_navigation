import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/bridge_navigate_to_pose/result',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(PoseStamped, '/bridge_navigate_to_pose/goal', 10)

        bt_pub = self.create_publisher(String, 'bridge_navigate_to_pose/bt', 10)
        msg = String()
        msg.data = "follow_point"
        bt_pub.publish(msg)

        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = 0.5
        msg.pose.position.y = 0.5
        self.publisher_.publish(msg)
                
        self.positions_x = [-1.5, -1.0, -0.5, 0.5, 1.0, 1.5]
        self.positions_y = [-1.5, -1.0, -0.5, 0.5, 1.0, 1.5]

        self.i = 0
        self.j = 0

    def listener_callback(self, msg):
        self.get_logger().info(msg.data)
        self.timer_callback()

    def timer_callback(self):
        msg = PoseStamped()
        
        msg.header.frame_id = "map"

        self.j += 1
        if self.j == 6:
            self.j = 0
            self.i += 1
        if self.i == 6:
            self.i = 0

        msg.pose.position.x = self.positions_x[self.i]
        msg.pose.position.y = self.positions_y[self.j]


        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.pose)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

