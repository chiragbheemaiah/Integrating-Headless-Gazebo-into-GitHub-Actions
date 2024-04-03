import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class MyPublisherNode(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer_ = self.create_timer(10.0, self.publish_message)
        self.message_published = False

    def publish_message(self):
        time.sleep(60)
        if not self.message_published:
            msg = PoseWithCovarianceStamped()
            msg.header.stamp.sec = 0
            msg.header.stamp.nanosec = 0
            msg.header.frame_id = "map"
            msg.pose.pose.position.x = -2.00
            msg.pose.pose.position.y = -0.50
            msg.pose.pose.position.z = 0.01
            self.publisher_.publish(msg)
            self.message_published = True

def main(args=None):
    rclpy.init(args=args)
    node = MyPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
