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

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

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
        # self.get_logger().info('Received feedback:')
        # print('Current Pose:', feedback.current_pose)
        # print('Navigation Time:', feedback.navigation_time.sec, 'seconds')
        # print('Estimated Time Remaining:', feedback.estimated_time_remaining.sec, 'seconds')
        # print('Number of Recoveries:', feedback.number_of_recoveries)
        # print('Distance Remaining:', feedback.distance_remaining)
        # if (feedback.distance_remaining < self.distance_threshold):
        #     print('GOAL STATE REACHED!')


def main(args=None):
    rclpy.init(args=args)

    action_client = NavigateToPoseActionClient()

    pose = PoseStamped()
    pose.header.stamp.sec = 0
    pose.header.stamp.nanosec = 0
    pose.header.frame_id = 'map'
    pose.pose.position.x = 1.8
    pose.pose.position.y = 5.0
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
