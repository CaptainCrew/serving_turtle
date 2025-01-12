import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        # ✅ 퍼블리셔 생성
        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # ✅ 2초 후 초기 위치 퍼블리시
        self.timer = self.create_timer(2.0, self.publish_initial_pose)
        self.get_logger().info('Initial Pose Publisher is ready.')

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # ✅ 로봇의 초기 위치 (x, y, z)
        msg.pose.pose.position.x = 2.0
        msg.pose.pose.position.y = 1.0
        msg.pose.pose.position.z = 0.0

        # ✅ 로봇의 방향 (Quaternion - 회전 정보)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        # ✅ 위치 오차 정보 (Covariance)
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942
        ]

        # ✅ 퍼블리시
        self.publisher_.publish(msg)
        self.get_logger().info('Initial pose published.')

        # ✅ 한 번 퍼블리시 후 종료
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
