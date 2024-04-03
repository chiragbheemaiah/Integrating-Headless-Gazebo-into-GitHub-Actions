import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('goal_publisher_node')

    # Create an action client
    action_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')

    # Wait for the action server to become available
    while not action_client.wait_for_server(timeout_sec=1.0):
        print('Waiting for action server...')
    
    # Create a goal message
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.stamp.sec = 0
    goal_msg.pose.header.stamp.nanosec = 0
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.pose.position.x = 1.8
    goal_msg.pose.pose.position.y = 1.1
    goal_msg.pose.pose.position.z = 0.01
    goal_msg.pose.pose.orientation.x = 0.0
    goal_msg.pose.pose.orientation.y = 0.0
    goal_msg.pose.pose.orientation.z = 0.0
    goal_msg.pose.pose.orientation.w = 1.0

    # Send the goal
    send_goal_future = action_client.send_goal_async(goal_msg)
    
    rclpy.spin_until_future_complete(node, send_goal_future)

    if send_goal_future.result() is not None:
        print('Goal sent successfully!')
    else:
        print('Failed to send goal.')

    # TODO: Add code to validate if the goal pose was reached!

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
