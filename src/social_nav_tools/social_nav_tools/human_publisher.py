"""Simulated human publisher: publishes /social_nav/humans so the controller has people to
reason about without a real detector. Each human is "x,y,vx,vy" in the map frame (m, m/s);
stationary if vx=vy=0, otherwise it walks in a straight line. Runs on sim time.

Example:
  ros2 run social_nav_tools human_publisher --ros-args -p humans:="['1.5,1.9,0,0']"
"""
import math

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.parameter import Parameter
from social_nav_msgs.msg import HumanArray, HumanState


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class HumanPublisher(Node):
    def __init__(self):
        # Sim time so stamps match the controller's clock (else it drops us as stale).
        super().__init__("human_publisher", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.declare_parameter("humans", ["1.5,1.9,0.0,0.0"])
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("loop_period", 0.0)  # >0 => bounce back and forth
        self.declare_parameter("frame_id", "map")

        specs = self.get_parameter("humans").get_parameter_value().string_array_value
        self.starts = []
        self.vels = []
        for s in specs:
            x, y, vx, vy = (float(v) for v in s.split(","))
            self.starts.append((x, y))
            self.vels.append((vx, vy))

        self.frame = self.get_parameter("frame_id").get_parameter_value().string_value
        self.loop_period = self.get_parameter("loop_period").get_parameter_value().double_value
        rate = self.get_parameter("rate_hz").get_parameter_value().double_value
        self.pub = self.create_publisher(HumanArray, "/social_nav/humans", 10)
        self.t0 = self.get_clock().now()
        self.create_timer(1.0 / max(rate, 1.0), self.tick)
        self.get_logger().info(f"Publishing {len(self.starts)} human(s) on /social_nav/humans")

    def tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        # Triangle wave in [0,1] for optional bouncing motion.
        phase = 1.0
        if self.loop_period > 0.0:
            frac = (t % self.loop_period) / self.loop_period
            phase = 1.0 - abs(2.0 * frac - 1.0)

        msg = HumanArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        for i, ((x0, y0), (vx, vy)) in enumerate(zip(self.starts, self.vels)):
            h = HumanState()
            h.id = i
            if self.loop_period > 0.0:
                h.pose.position.x = x0 + vx * self.loop_period * phase
                h.pose.position.y = y0 + vy * self.loop_period * phase
            else:
                h.pose.position.x = x0 + vx * t
                h.pose.position.y = y0 + vy * t
            speed = math.hypot(vx, vy)
            h.pose.orientation = yaw_to_quat(math.atan2(vy, vx) if speed > 1e-6 else 0.0)
            h.velocity.linear.x = vx
            h.velocity.linear.y = vy
            h.confidence = 1.0
            h.stationary = speed < 1e-6
            msg.humans.append(h)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = HumanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
