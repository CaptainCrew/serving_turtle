import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from turtlebot3_msgs.action import OrderFood
from turtlebot3_msgs.srv import RobotCommand  # ✅ 서비스 클라이언트로 사용
import time

class KitchenROSNode(Node):
    def __init__(self, gui=None):
        super().__init__('kitchen_ros_node')
        self.gui = gui  # ✅ GUI 연결

        # ✅ Action 서버 생성 (주문 처리)
        self._action_server = ActionServer(
            self,
            OrderFood,
            'order_food',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # ✅ Service 클라이언트 생성 (로봇 명령 전송)
        self.robot_command_client = self.create_client(RobotCommand, 'navigate_to_waypoint')
        while not self.robot_command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for robot_command service...')

    def goal_callback(self, goal_request):
        """주문 수신 시 실행"""
        order_text = f"테이블 {goal_request.table_id}: {goal_request.quantity} x {goal_request.menu_item}"
        self.get_logger().info(f"Order received: {order_text}")
        if self.gui:
            self.gui.add_order(order_text)
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """주문 취소 요청 처리"""
        self.get_logger().info("Order canceled.")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """조리 및 서빙 진행"""
        self.get_logger().info("Processing order...")

        feedback_msg = OrderFood.Feedback()
        steps = ["조리 중", "조리 완료", "서빙 중", "서빙 완료"]

        for i, step in enumerate(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Order canceled during processing.")
                return OrderFood.Result(success=False, message="주문 취소됨")

            # ✅ 피드백 전송
            feedback_msg.status = step
            feedback_msg.progress = (i + 1) / len(steps)
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"{step} ({feedback_msg.progress * 100:.0f}%)")

            time.sleep(2)  # ✅ 단계별 대기

        goal_handle.succeed()

        # ✅ 주문 완료 시 로봇에게 이동 명령 전송
        self.send_robot_command("S1")

        # ✅ 주문 완료 결과 반환
        result = OrderFood.Result()
        result.success = True
        result.message = "서빙 완료"
        return result

    # ✅ 로봇 제어 명령 전송 (서비스 클라이언트 역할)
    def send_robot_command(self, command):
        """로봇 제어 서비스 호출"""
        request = RobotCommand.Request()
        request.command = command

        future = self.robot_command_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info(f"로봇 명령 성공: {future.result().message}")
            if self.gui:
                self.gui.update_status(f"로봇 명령 성공: {future.result().message}")
        else:
            self.get_logger().warn("로봇 명령 실패")
            if self.gui:
                self.gui.update_status("로봇 명령 실패")
