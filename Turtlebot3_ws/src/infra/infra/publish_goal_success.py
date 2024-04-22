import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
from nav2_msgs.action import NavigateToPose
import time


class NavigateToPoseActionClient(Node):

    def __init__(self):
        super().__init__('navigate_to_pose_action_client')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        # self.distance_threshold = 0.4

        # Define parameters
        self.declare_parameters(
                namespace='',
                parameters=[
                    ('position_x', 1.8),
                    ('position_y', 1.1),
                    ('position_z', 0.01),
                    ('orientation_x', 0.0),
                    ('orientation_y', 0.0),
                    ('orientation_z', 0.0),
                    ('orientation_w', 1.0),
                    ]
                )

    def send_goal(self):
        pose = PoseStamped()
        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0
        pose.header.frame_id = 'map'
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        pose.pose.position.x = self.get_parameter('position_x').value
        pose.pose.position.y = self.get_parameter('position_y').value
        pose.pose.position.z = self.get_parameter('position_z').value
        pose.pose.orientation.x = self.get_parameter('orientation_x').value
        pose.pose.orientation.y = self.get_parameter('orientation_y').value
        pose.pose.orientation.z = self.get_parameter('orientation_z').value
        pose.pose.orientation.w = self.get_parameter('orientation_w').value

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        # self.get_logger().info('Result: Error Code - {0}'.format(result.error_code))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

def main(args=None):
    rclpy.init(args=args)

    action_client = NavigateToPoseActionClient()

    pose = PoseStamped()
    pose.header.stamp.sec = 0
    pose.header.stamp.nanosec = 0
    pose.header.frame_id = 'map'
    pose.pose.position.x = 1.8
    pose.pose.position.y = 1.1
    pose.pose.position.z = 0.01
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0

    time.sleep(40)
    # print('GOAL PUBLISHING STARTS!!!!!!!!')
    action_client.send_goal(pose)

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
