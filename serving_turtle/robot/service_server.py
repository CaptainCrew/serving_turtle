# service_server.py
from turtlebot3_msgs.srv import RobotCommand

class WaypointServiceServer:
    def __init__(self, node, waypoints, navigator):
        self.node = node
        self.waypoints = waypoints
        self.navigator = navigator
        self.service = node.create_service(RobotCommand, 'navigate_to_waypoint', self.handle_service)

    def handle_service(self, request, response):
        waypoint_id = request.command.strip().upper()
        if waypoint_id not in self.waypoints.waypoints:
            response.success = False
            response.message = f"알 수 없는 Waypoint: {waypoint_id}"
            self.node.get_logger().error(response.message)
            return response

        waypoint = self.waypoints.waypoints[waypoint_id]
        waypoint.header.stamp = self.node.get_clock().now().to_msg()
        self.navigator.navigate_to(waypoint)
        response.success = True
        response.message = f"Waypoint {waypoint_id}로 이동 중입니다."
        self.node.get_logger().info(response.message)
        return response