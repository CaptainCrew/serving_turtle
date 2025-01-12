import rclpy
from rclpy.node import Node
from turtlebot3_msgs.msg import Order, OrderStatusMsg
#from database_handler import DatabaseHandler
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from turtlebot3_msgs.action import OrderFood
import time

class KitchenROSNode(Node):
    def __init__(self, gui=None):
        super().__init__('kitchen_ros_node')
        self.gui = gui  # ✅ GUI 연결

        # ✅ Action 서버 생성
        self._action_server = ActionServer(
            self,
            OrderFood,
            'order_food',  # ✅ Action 이름
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request):
        """주문 수신 시 실행"""
        order_text = f"테이블 {goal_request.table_id}: {goal_request.quantity} x {goal_request.menu_item}"
        self.get_logger().info(f"Order received: {order_text}")
        if self.gui:
            self.gui.add_order(order_text)
        return GoalResponse.ACCEPT  # ✅ 주문 수락

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

        # ✅ 주문 완료 결과 반환
        result = OrderFood.Result()
        result.success = True
        result.message = "서빙 완료"
        return result