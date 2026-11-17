"""Starter behaviour tree: dive, drive out, turn around, drive back.

Run it against the Unity simulator after selecting the mission source:

    ros2 param set /velocity_setpoint_mux active_source mission
    ros2 run tardigrade_mission starter_tree --ros-args -p use_sim_time:=true
"""

import math

import py_trees
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from tardigrade_mission.bt_behaviours import create_starter_tree


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class RosVehicle:
    """The vehicle interface from bt_behaviours, backed by ROS topics."""

    def __init__(self, node, odometry_topic, command_topic):
        self.node = node
        self.odometry = None
        self.odometry_time = None
        self.publisher = node.create_publisher(
            TwistStamped, command_topic, 10)
        node.create_subscription(
            Odometry, odometry_topic, self.on_odometry, 10)

    def on_odometry(self, message):
        self.odometry = message
        self.odometry_time = self.now()

    def now(self):
        return self.node.get_clock().now().nanoseconds / 1e9

    def odometry_age(self):
        if self.odometry_time is None:
            return math.inf
        return self.now() - self.odometry_time

    def pose(self):
        pose = self.odometry.pose.pose
        return (pose.position.x, pose.position.y, pose.position.z,
                yaw_from_quaternion(pose.orientation))

    def yaw_rate(self):
        return self.odometry.twist.twist.angular.z

    def command(self, surge, heave, yaw_rate):
        message = TwistStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.twist.linear.x = float(surge)
        message.twist.linear.z = float(heave)
        message.twist.angular.z = float(yaw_rate)
        self.publisher.publish(message)

    def stop(self):
        self.command(0.0, 0.0, 0.0)


class StarterTreeNode(Node):
    """Tick the starter tree at a fixed rate until it finishes."""

    def __init__(self):
        super().__init__('starter_tree')
        self.declare_parameter(
            'odometry_topic', '/tardigrade/state/odometry/filtered')
        self.declare_parameter(
            'command_topic', '/tardigrade/control/velocity_setpoint/mission')
        self.declare_parameter('depth_m', 1.0)
        self.declare_parameter('distance_m', 3.0)
        self.declare_parameter('speed', 0.2)
        self.declare_parameter('tick_hz', 10.0)

        self.vehicle = RosVehicle(
            self,
            str(self.get_parameter('odometry_topic').value),
            str(self.get_parameter('command_topic').value))
        root = create_starter_tree(
            self.vehicle,
            depth_m=float(self.get_parameter('depth_m').value),
            distance_m=float(self.get_parameter('distance_m').value),
            speed=float(self.get_parameter('speed').value))
        self.tree = py_trees.trees.BehaviourTree(root)
        self.tree.setup(timeout=5.0)
        self.finished = None
        self.last_running = None
        rate = max(1.0, float(self.get_parameter('tick_hz').value))
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            'Starter tree loaded:\n' + py_trees.display.unicode_tree(root))

    def tick(self):
        if self.finished is not None:
            return
        self.tree.tick()
        root = self.tree.root
        running = root.tip().name if root.tip() is not None else None
        if running != self.last_running:
            self.last_running = running
            self.get_logger().info(
                f'Now running: {running}\n'
                + py_trees.display.unicode_tree(root, show_status=True))
        if root.status in (py_trees.common.Status.SUCCESS,
                           py_trees.common.Status.FAILURE):
            self.vehicle.stop()
            self.finished = root.status
            self.get_logger().info(f'Mission finished: {root.status.value}')


def main(args=None):
    rclpy.init(args=args)
    node = StarterTreeNode()
    try:
        while rclpy.ok() and node.finished is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.vehicle.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
