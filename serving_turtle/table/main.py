import rclpy
from rclpy.executors import MultiThreadedExecutor
from serving_turtle.table.table_ros_node import OrderPublisherNode
from serving_turtle.table.table_gui import TableGUI
import threading  # ✅ 추가
def main(args=None):
    rclpy.init(args=args)

    publisher_node = OrderPublisherNode()
    gui = TableGUI(publisher_node)

    # ✅ 멀티스레드 스피너 사용 (ROS와 GUI 병렬 실행)
    executor = MultiThreadedExecutor()
    executor.add_node(publisher_node)

    try:
        gui_thread = threading.Thread(target=gui.run)
        gui_thread.start()
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
