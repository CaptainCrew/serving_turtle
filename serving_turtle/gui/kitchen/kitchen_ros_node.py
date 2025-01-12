import rclpy
from rclpy.node import Node
from turtlebot3_msgs.msg import Order, OrderStatusMsg
#from database_handler import DatabaseHandler

class KitchenROSNode(Node):
    def __init__(self, gui=None):
        super().__init__('kitchen_ros_node')
        #self.db_handler = DatabaseHandler()
        self.gui = gui  # ✅ GUI 객체 저장
        self.subscription = self.create_subscription(Order, 'order_topic', self.order_callback, 10)
        self.status_publisher = self.create_publisher(OrderStatusMsg, 'order_status_topic', 10)

    def order_callback(self, msg):
        #self.db_handler.save_order(msg.table_id, msg.menu_item, msg.quantity)
        order_text = f"테이블 {msg.table_id}: {msg.quantity} x {msg.menu_item}"
        self.get_logger().info(f"Order received: {msg.menu_item}")
        if self.gui:
            self.gui.add_order(order_text)

    def publish_status(self, table_id, status):
        status_msg = OrderStatusMsg()
        status_msg.table_number = table_id
        status_msg.success = (status == 'Completed')
        status_msg.message = f"Order {status}"
        self.status_publisher.publish(status_msg)


