import rclpy
from rclpy.executors import MultiThreadedExecutor
from serving_turtle.table.table_client import OrderActionClient
from serving_turtle.table.table_gui import TableGUI
import threading

def ros_spin(executor):
    """ROS 2 노드를 실행하는 함수 (서브 스레드)"""
    executor.spin()

def main(args=None):
    rclpy.init(args=args)

    # ✅ Action Client 노드 생성
    action_client = OrderActionClient()

    # ✅ 멀티스레드 실행기 생성
    executor = MultiThreadedExecutor()
    executor.add_node(action_client)

    # ✅ ROS 2 노드는 서브 스레드에서 실행
    ros_thread = threading.Thread(target=ros_spin, args=(executor,), daemon=True)
    ros_thread.start()

    # ✅ GUI는 메인 스레드에서 실행
    gui = TableGUI(action_client)
    action_client.gui = gui  # 🔥 추가된 부분
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()
        rclpy.shutdown()
        ros_thread.join()

if __name__ == "__main__":
    main()
