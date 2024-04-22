import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class MyPublisherNode(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('x_position', -2.00),
                ('y_position', -0.50),
                ('z_position', 0.01)
            ]
        )

        # Get parameter values
        self.x_position = self.get_parameter("x_position").value
        self.y_position = self.get_parameter("y_position").value
        self.z_position = self.get_parameter("z_position").value

        # Use parameter values
        self.timer_ = self.create_timer(10.0, self.publish_message)
        self.message_published = False

    def publish_message(self):
        if not self.message_published:
            msg = PoseWithCovarianceStamped()
            msg.header.stamp.sec = 0
            msg.header.stamp.nanosec = 0
            msg.header.frame_id = "map"

            # Use parameter values
            msg.pose.pose.position.x = self.x_position
            msg.pose.pose.position.y = self.y_position
            msg.pose.pose.position.z = self.z_position

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
