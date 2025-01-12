# main.py
import rclpy
from rclpy.node import Node
from serving_turtle.robot.waypoints import Waypoints
from serving_turtle.robot.navigator import Navigator
from serving_turtle.robot.service_server import WaypointServiceServer

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
