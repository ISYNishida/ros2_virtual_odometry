#!/usr/bin/env python3

import math
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomAnalyzerNode(Node):
    """
    /odom を subscribe し、
    ロボットの向きに対する前進／後退・旋回を投影計算で判定するノード
    """

    def __init__(self):
        super().__init__('odom_analyzer_node')

        self.prev_x = None
        self.prev_y = None
        self.prev_theta = None

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            'Odom Analyzer Node started (projection-based interpretation).'
        )

    def odom_callback(self, msg: Odometry):
        # --- 現在の位置 ---
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # --- quaternion → yaw ---
        q = msg.pose.pose.orientation
        theta = math.atan2(
            2.0 * (q.w * q.z),
            1.0 - 2.0 * (q.z * q.z)
        )

        # 初回は初期化のみ
        if self.prev_x is None:
            self.prev_x = x
            self.prev_y = y
            self.prev_theta = theta
            return

        # --- 差分 ---
        dx = x - self.prev_x
        dy = y - self.prev_y
        dtheta = theta - self.prev_theta

        # --- ロボット前方向への投影 ---
        forward = dx * math.cos(theta) + dy * math.sin(theta)

        # --- 状態判定 ---
        if abs(forward) < 0.005:
            move_str = 'STOP'
        elif forward > 0:
            move_str = 'FORWARD'
        else:
            move_str = 'BACKWARD'

        if abs(dtheta) < 0.02:
            turn_str = 'STRAIGHT'
        elif dtheta > 0:
            turn_str = 'LEFT'
        else:
            turn_str = 'RIGHT'

        # --- 1行ステータス表示 ---
        sys.stdout.write(
            f'\r'
            f'[STATUS] '
            f'x={x:6.2f}  '
            f'y={y:6.2f}  '
            f'theta={theta:6.2f} | '
            f'move={move_str:8s} ({forward:6.3f}m) | '
            f'turn={turn_str:8s} ({dtheta:6.3f}rad)   '
        )
        sys.stdout.flush()

        # 更新
        self.prev_x = x
        self.prev_y = y
        self.prev_theta = theta


def main(args=None):
    rclpy.init(args=args)
    node = OdomAnalyzerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

