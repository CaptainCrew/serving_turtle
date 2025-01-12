import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from turtlebot3_msgs.action import OrderFood

class OrderActionClient(Node):
    def __init__(self, gui=None):
        super().__init__('order_action_client')
        self._action_client = ActionClient(self, OrderFood, 'order_food')
        self.gui = gui  # ✅ GUI 연결
        self.get_logger().info("Order Action Client Node started.")

    def send_order(self, menu_items, total_quantity, table_id=1):
        self._action_client.wait_for_server()

        goal_msg = OrderFood.Goal()
        goal_msg.menu_item = ", ".join(menu_items)
        goal_msg.quantity = total_quantity
        goal_msg.table_id = str(table_id)

        self.get_logger().info(f"Sending Order: {goal_msg.menu_item} (Quantity: {goal_msg.quantity}) to Table {goal_msg.table_id}")
        self.gui.update_status("주문 전송 중...", "blue")  # ✅ 상태 업데이트

        self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Order was rejected by the server.")
            self.gui.update_status("주문이 거절되었습니다.", "red")  # ✅ 상태 업데이트
            return
        self.get_logger().info("Order accepted. Processing...")
        self.gui.update_status("주문이 접수되었습니다.", "green")  # ✅ 상태 업데이트
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        status = f"[진행 상태]: {feedback.status} | [진행률]: {feedback.progress * 100:.0f}%"
        self.get_logger().info(status)
        self.gui.update_status(status, "orange")  # ✅ 실시간 피드백

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"✅ 주문 완료: {result.message}")
            self.gui.update_status(f"✅ 주문 완료: {result.message}", "green")  # ✅ 결과 업데이트
        else:
            self.get_logger().info(f"❌ 주문 실패: {result.message}")
            self.gui.update_status(f"❌ 주문 실패: {result.message}", "red")  # ✅ 결과 업데이트
