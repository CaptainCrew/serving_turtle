import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
import numpy as np

class ZoneMaskPublisher(Node):
    def __init__(self):
        super().__init__('zone_mask_publisher')

        # 근접 금지구역(K-Zone) 마스크 발행기
        self.kzone_publishers = {
            1: self.create_publisher(OccupancyGrid, '/kzone1_mask', 10),
            2: self.create_publisher(OccupancyGrid, '/kzone2_mask', 10),
            3: self.create_publisher(OccupancyGrid, '/kzone3_mask', 10),
            4: self.create_publisher(OccupancyGrid, '/kzone4_mask', 10)
        }

        # 우회 구역(A-Zone) 마스크 발행기
        self.azone_publishers = {
            1: self.create_publisher(OccupancyGrid, '/azone1_mask', 10),
            2: self.create_publisher(OccupancyGrid, '/azone2_mask', 10),
            3: self.create_publisher(OccupancyGrid, '/azone3_mask', 10),
            4: self.create_publisher(OccupancyGrid, '/azone4_mask', 10)
        }

        # 마스크 ON/OFF 제어를 위한 토픽 구독
        self.mask_enabled = False  # 초기 상태는 OFF
        self.create_subscription(Bool, '/mask_toggle', self.toggle_mask_callback, 10)

        # 주기적으로 마스크 발행
        self.timer = self.create_timer(2.0, self.publish_masks)
        self.get_logger().info('Zone Mask Publisher Started. (Mask OFF)')

    def toggle_mask_callback(self, msg):
        """마스크 ON/OFF 상태 전환"""
        self.mask_enabled = msg.data
        state = "ON" if self.mask_enabled else "OFF"
        self.get_logger().info(f'Mask state changed to: {state}')

    def publish_masks(self):
        """마스크 발행 (ON 상태일 때만)"""
        if self.mask_enabled:
            # 근접 금지구역(K-Zone) 마스크 발행
            for zone_id in self.kzone_publishers:
                mask = self.create_mask(zone_id, 'k', 255)
                self.kzone_publishers[zone_id].publish(mask)

            # 우회 구역(A-Zone) 마스크 발행
            for zone_id in self.azone_publishers:
                mask = self.create_mask(zone_id, 'a', 100)
                self.azone_publishers[zone_id].publish(mask)

    def create_mask(self, zone_id, filter_type, cost_value):
        """OccupancyGrid 마스크 생성"""
        mask = OccupancyGrid()
        mask.header.frame_id = "map"
        mask.info.resolution = 0.05
        mask.info.width = 200
        mask.info.height = 200
        mask.info.origin.position.x = 2.5
        mask.info.origin.position.y = 1.0

        # 마스크 데이터 생성
        data = np.zeros((200, 200), dtype=np.int8)

        # 구역별 위치 지정 (버그 수정 포함)
        if filter_type == 'k':
            if zone_id == 1:
                data[55:85, 55:85] = cost_value
            elif zone_id == 2:
                data[85:115, 55:85] = cost_value
            elif zone_id == 3:
                data[55:85, 85:115] = cost_value
            elif zone_id == 4:
                data[85:115, 115:145] = cost_value  # ✅ 수정됨
        elif filter_type == 'a':
            if zone_id == 1:
                data[35:85, 35:85] = cost_value
            elif zone_id == 2:
                data[35:85, 135:185] = cost_value  # ✅ 수정됨
            elif zone_id == 3:
                data[85:135, 85:135] = cost_value
            elif zone_id == 4:
                data[85:135, 135:185] = cost_value  # ✅ 수정됨

        mask.data = data.flatten().tolist()
        return mask

def main(args=None):
    rclpy.init(args=args)
    node = ZoneMaskPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
