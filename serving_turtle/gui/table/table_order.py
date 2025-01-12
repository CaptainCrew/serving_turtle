import rclpy
from rclpy.node import Node
from turtlebot3_msgs.msg import Order

class OrderPublisherNode(Node):
    def __init__(self):
        super().__init__('order_publisher_node')
        self.publisher_ = self.create_publisher(Order, 'order_topic', 10)
        self.get_logger().info("Order Publisher Node started.")

    def publish_order(self, menu_items, total_quantity, table_id=1):
        msg = Order()
        msg.menu_item = ", ".join(menu_items)
        msg.quantity = total_quantity
        msg.table_id = table_id

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published Order: {msg.menu_item} (Quantity: {msg.quantity})")
