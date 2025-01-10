import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        # Waypoints 정의 (A, B, C)
        self.waypoints = {
            'A': self.create_pose_stamped(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            'B': self.create_pose_stamped(3.0, 4.0, 0.0, 0.0, 0.0, 0.7071, 0.7071),
            'C': self.create_pose_stamped(5.0, 6.0, 0.0, 0.0, 0.0, 1.0, 0.0)
        }

        # Waypoint 명령을 수신하는 토픽 구독
        self.subscription = self.create_subscription(
            String,
            '/waypoint_command',
            self.waypoint_callback,
            10
        )

        # Navigation2 액션 클라이언트 설정
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def create_pose_stamped(self, x, y, z, ox, oy, oz, ow):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = Pose()
        pose_stamped.pose.position = Point(x=x, y=y, z=z)
        pose_stamped.pose.orientation = Quaternion(x=ox, y=oy, z=oz, w=ow)
        return pose_stamped

    def waypoint_callback(self, msg):
        waypoint_id = msg.data.strip().upper()  # 공백 제거 및 대문자 변환
        if waypoint_id not in self.waypoints:
            self.get_logger().error(f"알 수 없는 Waypoint: {waypoint_id}")
            return

        # Waypoint 가져오기
        waypoint = self.waypoints[waypoint_id]
        # 현재 시간으로 헤더 업데이트
        waypoint.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(f"Waypoint {waypoint_id}로 이동 중: "
                               f"({waypoint.pose.position.x}, {waypoint.pose.position.y})")

        # NavigateToPose 액션 Goal 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint

        # 액션 서버가 준비될 때까지 대기
        self.nav_action_client.wait_for_server()

        # Goal 전송
        self._send_goal_future = self.nav_action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose
        self.get_logger().info(f'현재 위치: ({current_pose.position.x:.2f}, {current_pose.position.y:.2f})')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal이 거부되었습니다.')
            return

        self.get_logger().info('Goal이 수락되었습니다.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Waypoint에 성공적으로 도착했습니다!')
        else:
            self.get_logger().info(f'Goal 실패 상태: {result.status}')
def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

#