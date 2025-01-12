# navigator.py
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

class Navigator:
    def __init__(self, node):
        self.node = node
        self.nav_action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    def navigate_to(self, pose_stamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        self.nav_action_client.wait_for_server()

        self._send_goal_future = self.nav_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose
        self.node.get_logger().info(f'현재 위치: ({current_pose.position.x:.2f}, {current_pose.position.y:.2f})')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal이 거부되었습니다.')
            return
        self.node.get_logger().info('Goal이 수락되었습니다.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info('Waypoint에 성공적으로 도착했습니다!')
        else:
            self.node.get_logger().info(f'Goal 실패 상태: {result.status}')