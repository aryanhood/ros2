#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

def normalize_angle(a):
    # normalize to [-pi, pi]
    return math.atan2(math.sin(a), math.cos(a))

def angle_diff(current, target):
    # shortest signed difference (target - current) normalized to [-pi,pi]
    d = target - current
    return math.atan2(math.sin(d), math.cos(d))

class TurtleSquare(Node):
    def __init__(self):
        super().__init__('turtle_square')
        self.pose = None
        self.pose_sub = self.create_subscription(Pose, 'turtle1/pose', self.pose_cb, 10)
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.get_logger().info('turtle_square node started')

    def pose_cb(self, msg: Pose):
        self.pose = msg

    def wait_for_pose(self, timeout=5.0):
        start = time.time()
        while rclpy.ok() and self.pose is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose is not None

    def stop(self):
        self.pub.publish(Twist())
        rclpy.spin_once(self, timeout_sec=0.05)

    def move_forward(self, distance, speed=1.0):
        start_x = self.pose.x
        start_y = self.pose.y
        twist = Twist()
        twist.linear.x = abs(speed)
        twist.angular.z = 0.0
        rate = 0.02
        while rclpy.ok():
            dx = self.pose.x - start_x
            dy = self.pose.y - start_y
            if math.hypot(dx, dy) >= distance:
                break
            self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=rate)
        self.stop()

    def rotate_by(self, angle, angular_speed=1.0):
        # angle in radians, positive = CCW
        target = normalize_angle(self.pose.theta + angle)
        rate = 0.02
        while rclpy.ok():
            diff = angle_diff(self.pose.theta, target)
            if abs(diff) < 0.01:  # ~0.6 degrees tolerance
                break
            twist = Twist()
            twist.angular.z = (angular_speed if diff > 0 else -angular_speed)
            self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=rate)
        self.stop()

    def draw_square(self, side_length=2.0, linear_speed=1.0, angular_speed=1.0):
        if not self.wait_for_pose():
            self.get_logger().error('No pose messages received within timeout.')
            return
        # do 4 sides
        for i in range(4):
            self.get_logger().info(f'Side {i+1}: forward {side_length}m')
            self.move_forward(side_length, speed=linear_speed)
            self.get_logger().info('Turning 90 deg')
            self.rotate_by(math.pi/2, angular_speed=angular_speed)
        self.get_logger().info('Finished square.')
        self.stop()

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSquare()
    try:
        # tweak these params if you need larger/smaller square
        node.draw_square(side_length=2.0, linear_speed=1.5, angular_speed=1.2)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()