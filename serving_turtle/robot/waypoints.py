# waypoints.py
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

class Waypoints:
    def __init__(self, node):
        self.node = node
        self.waypoints = {
            'HOME': self.create_pose_stamped(7.0, 9.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            'KITCHEN': self.create_pose_stamped(2.0, 7.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            'S1': self.create_pose_stamped(2.0, 4.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            'S2': self.create_pose_stamped(2.0, 2.5, 0.0, 0.0, 0.0, 0.0, 1.0),
            'S3': self.create_pose_stamped(5.0, 2.0, 0.0, 0.0, 0.0, 0.7071, 0.7071),
            'S4': self.create_pose_stamped(4.0, 2.0, 0.0, 0.0, 0.0, 1.0, 0.0)
        }

    def create_pose_stamped(self, x, y, z, ox, oy, oz, ow):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        pose_stamped.pose = Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(x=ox, y=oy, z=oz, w=ow))
        return pose_stamped
