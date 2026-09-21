#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AutoInitLocalizer(Node):
    def __init__(self):
        super().__init__('auto_align_node')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_recovery')
        # Recovery tools feed the arbiter; they never publish the real chassis
        # output directly.
        self.pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10
        )
        self.get_logger().info('等待 25 秒以确保 Nav2 和雷达底盘完全启动激活...')
        self.timer_count = 0
        # 将原先的 10 秒等待延长为 25 秒，避免与 AMCL 配置启动期冲突
        self.startup_timer = self.create_timer(25.0, self.start_rotation)

    def start_rotation(self):
        self.startup_timer.cancel()  # 取消启动延时器
        self.get_logger().info('==== 开始自动原地旋转，帮助 AMCL 粒子收敛 ====')
        # 创建一个 0.2 秒触发一次的定时器来持续发送旋转指令
        self.spin_timer = self.create_timer(0.2, self.spin_step)

    def spin_step(self):
        self.timer_count += 1
        msg = Twist()
        msg.angular.z = 0.5  # 以 0.5 rad/s 的角速度旋转
        self.pub.publish(msg)

        # 0.5 rad/s * (65 * 0.2s) = 6.5 rad ≈ 372度 (转稍微超过一圈，以确保获取全向环境特征)
        if self.timer_count >= 65:
            self.spin_timer.cancel()  # 停止计时器
            msg.angular.z = 0.0      # 发送停止指令
            self.pub.publish(msg)
            self.get_logger().info('==== 旋转校准完成！地图对齐完毕，随时可下发目标点。 ====')
            raise SystemExit  # 安全退出节点


def main(args=None):
    rclpy.init(args=args)
    node = AutoInitLocalizer()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
