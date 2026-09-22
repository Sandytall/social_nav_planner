"""Visualize /social_nav/humans as RViz markers: a body cylinder, a heading/velocity
arrow, and anisotropic comfort/caution social-zone outlines."""
import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.parameter import Parameter
from social_nav_msgs.msg import HumanArray
from visualization_msgs.msg import Marker, MarkerArray


def social_zone_points(cx, cy, yaw, front, side, rear, n=48):
    """Egg-shaped outline: radius `front` ahead, `rear` behind, `side` laterally."""
    pts = []
    c, s = math.cos(yaw), math.sin(yaw)
    for i in range(n + 1):
        t = -math.pi + 2.0 * math.pi * i / n
        lon = (front if math.cos(t) >= 0 else rear) * math.cos(t)
        lat = side * math.sin(t)
        pts.append(Point(x=cx + c * lon - s * lat, y=cy + s * lon + c * lat, z=0.02))
    return pts


class HumanMarkers(Node):
    def __init__(self):
        super().__init__("human_markers", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.declare_parameter("comfort", [1.2, 0.9, 0.6])   # front, side, rear (m)
        self.declare_parameter("caution", [2.0, 1.5, 1.0])
        self.comfort = self.get_parameter("comfort").value
        self.caution = self.get_parameter("caution").value
        self.pub = self.create_publisher(MarkerArray, "/social_nav/markers", 10)
        self.create_subscription(HumanArray, "/social_nav/humans", self.cb, 10)

    def cb(self, msg):
        arr = MarkerArray()
        # Wipe previous markers so departed people disappear.
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, h in enumerate(msg.humans):
            frame = msg.header.frame_id or "map"
            x, y = h.pose.position.x, h.pose.position.y
            q = h.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

            body = Marker()
            body.header.frame_id = frame
            body.header.stamp = msg.header.stamp
            body.ns, body.id, body.type, body.action = "person", i, Marker.CYLINDER, Marker.ADD
            body.pose.position.x, body.pose.position.y, body.pose.position.z = x, y, 0.85
            body.pose.orientation.w = 1.0
            body.scale.x = body.scale.y = 0.5
            body.scale.z = 1.7
            body.color.r, body.color.g, body.color.b, body.color.a = 0.2, 0.4, 0.9, 0.9
            arr.markers.append(body)

            for name, radii, rgba, mid in [
                ("comfort", self.comfort, (1.0, 0.7, 0.0, 0.8), 1000 + i),
                ("caution", self.caution, (1.0, 0.9, 0.3, 0.5), 2000 + i)]:
                zone = Marker()
                zone.header.frame_id = frame
                zone.header.stamp = msg.header.stamp
                zone.ns, zone.id, zone.type, zone.action = name, mid, Marker.LINE_STRIP, Marker.ADD
                zone.scale.x = 0.04
                zone.color.r, zone.color.g, zone.color.b, zone.color.a = rgba
                zone.pose.orientation.w = 1.0
                zone.points = social_zone_points(x, y, yaw, radii[0], radii[1], radii[2])
                arr.markers.append(zone)
        self.pub.publish(arr)


def main():
    rclpy.init()
    node = HumanMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
