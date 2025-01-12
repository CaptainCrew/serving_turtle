import rclpy
from threading import Thread
from serving_turtle.kitchen.kitchen_server import KitchenROSNode
from serving_turtle.kitchen.kitchen_gui import KitchenGUI

def ros_spin(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)

    # ✅ ROS 노드 먼저 생성
    ros_node = KitchenROSNode()

    # ✅ ROS 노드를 GUI와 연결
    gui = KitchenGUI(ros_node)

    # ✅ GUI를 ROS 노드와 연결
    ros_node.gui = gui

    # ✅ ROS 노드와 GUI 병렬 실행
    ros_thread = Thread(target=ros_spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # ✅ GUI 실행
    gui.run()

    # ✅ 노드 종료 처리
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
