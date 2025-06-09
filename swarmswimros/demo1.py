import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist


class TwistPublisherNode(Node):
    def __init__(self):
        super().__init__('demo1')
        self.publisher = self.create_publisher(TwistStamped, 'A01/cmd', 10)
        self.timer = self.create_timer(5.0, self.publish_twist)
        self.get_logger().info('TwistPublisherNode started, publishing every 5 seconds.')

        self.depth_oscillation = 1.0
        self.cmd_heading = 0.0

    def publish_twist(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.twist = self.new_cmd()
        self.publisher.publish(msg)
        self.get_logger().info(f'Published cmd depth {msg.twist.linear.z}, heading {msg.twist.angular.z}.')

    def new_cmd(self):
        msg_twist = Twist()
        msg_twist.linear.x = 1.5 # N
        msg_twist.linear.z = 70 + self.depth_oscillation # m
        msg_twist.angular.z = self.cmd_heading

        self.depth_oscillation *= -1
        self.cmd_heading = (self.cmd_heading + 90) % 360
        return msg_twist 

def main(args=None):
    rclpy.init(args=args)
    node = TwistPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()