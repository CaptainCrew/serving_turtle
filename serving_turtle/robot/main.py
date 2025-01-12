# main.py
import rclpy
from rclpy.node import Node
from waypoints import Waypoints
from navigator import Navigator
from service_server import WaypointServiceServer

class WaypointNavigatorNode(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        waypoints = Waypoints(self)
        navigator = Navigator(self)
        WaypointServiceServer(self, waypoints, navigator)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
