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
        mask.info.resolution = 0.05  # 해상도 유지
        mask.info.width = 25         # ✅ 가로 크기 절반으로 줄임
        mask.info.height = 25        # ✅ 세로 크기 절반으로 줄임
        mask.info.origin.position.x = 2.5
        mask.info.origin.position.y = 1.0

        # ✅ 마스크 데이터 크기 조정
        data = np.zeros((25, 25), dtype=np.int8)

        # 구역별 위치 지정 (축소된 범위에 맞게 수정)
        if filter_type == 'k':
            if zone_id == 1:
                data[0:15, 10:20] = cost_value  # ✅ 축소
            elif zone_id == 2:
                data[15:25, 10:20] = cost_value  # ✅ 축소
            elif zone_id == 3:
                data[0:15, 20:25] = cost_value  # ✅ 축소
            elif zone_id == 4:
                data[15:25, 20:25] = cost_value  # ✅ 축소

        elif filter_type == 'a':
            if zone_id == 1:
                data[5:15, 5:15] = cost_value  # ✅ 축소
            elif zone_id == 2:
                data[5:15, 15:25] = cost_value  # ✅ 축소
            elif zone_id == 3:
                data[15:25, 5:15] = cost_value  # ✅ 축소
            elif zone_id == 4:
                data[15:25, 15:25] = cost_value  # ✅ 축소

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
