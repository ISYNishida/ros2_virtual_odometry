#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """
    yaw（ラジアン）から Quaternion を生成する。
    今回は平面移動のみなので roll, pitch は 0。
    """
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class VirtualOdometryNode(Node):
    """
    /cmd_vel を入力として、仮想的に自己位置 (x, y, theta) を計算し、
    /odom (nav_msgs/Odometry) を publish するノード。
    """

    def __init__(self):
        super().__init__('virtual_odometry_node')

        # ===== 内部状態（自己位置）=====
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # yaw [rad]

        # ===== 最新の速度指令 =====
        self.v = 0.0  # linear.x [m/s]
        self.w = 0.0  # angular.z [rad/s]

        # ===== パラメータ =====
        self.dt = 0.1  # 更新周期 [s]（10Hz）

        # ===== Subscriber: /cmd_vel =====
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # ===== Publisher: /odom =====
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        # ===== タイマー =====
        self.create_timer(self.dt, self.update_odometry)

        self.get_logger().info('Virtual Odometry Node started.')

    def cmd_vel_callback(self, msg: Twist):
        """
        /cmd_vel を受信し、最新の速度指令を保存する
        """
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update_odometry(self):
        """
        速度指令を積分して自己位置を更新し、/odom を publish
        """
        # --- オドメトリ計算 ---
        self.x += self.v * math.cos(self.theta) * self.dt
        self.y += self.v * math.sin(self.theta) * self.dt
        self.theta += self.w * self.dt

        # --- Odometry メッセージ作成 ---
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # 位置
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # 姿勢（yaw → quaternion）
        odom.pose.pose.orientation = yaw_to_quaternion(self.theta)

        # 速度（そのまま反映）
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w

        # --- publish ---
        self.odom_pub.publish(odom)

        # --- ログ出力 ---
        self.get_logger().info(
            f'x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = VirtualOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

