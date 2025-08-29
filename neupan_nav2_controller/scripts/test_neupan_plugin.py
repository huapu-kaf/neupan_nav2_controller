#!/usr/bin/env python3
"""
NeuPAN Nav2 控制器插件功能测试脚本
用于测试 NeuPAN 控制器插件的基本功能和性能

许可证: GNU General Public License v3.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import time
import threading
from math import cos, sin, pi, sqrt, atan2

# ROS2 消息类型
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray


class NeuPANControllerTester(Node):
    """NeuPAN 控制器功能测试节点"""
    
    def __init__(self):
        super().__init__('neupan_controller_tester')
        
        # 测试参数
        self.test_duration = 30.0  # 测试时长(秒)
        self.test_start_time = None
        self.test_results = {}
        
        # 数据收集
        self.cmd_vel_count = 0
        self.laser_scan_count = 0
        self.last_cmd_vel_time = None
        self.last_laser_time = None
        self.cmd_vel_frequencies = []
        self.processing_times = []
        
        # QoS 配置
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 订阅话题
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, sensor_qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # 发布话题
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/test_markers', 10)
        
        # 创建模拟激光雷达发布器(用于无硬件测试)
        self.sim_laser_pub = self.create_publisher(LaserScan, '/scan', sensor_qos)
        self.sim_odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.sim_tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        
        # 定时器
        self.test_timer = self.create_timer(0.1, self.test_update_callback)
        self.sim_timer = self.create_timer(0.05, self.simulate_sensor_data)  # 20Hz
        self.report_timer = self.create_timer(5.0, self.print_interim_results)
        
        # 机器人状态仿真
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.robot_vx = 0.0
        self.robot_vy = 0.0
        self.robot_vtheta = 0.0
        
        self.get_logger().info("🚀 NeuPAN 控制器测试节点启动")
        self.get_logger().info(f"📊 测试将运行 {self.test_duration} 秒")
        
    def cmd_vel_callback(self, msg):
        """处理速度指令回调"""
        current_time = time.time()
        self.cmd_vel_count += 1
        
        # 更新机器人状态
        self.robot_vx = msg.linear.x
        self.robot_vy = msg.linear.y
        self.robot_vtheta = msg.angular.z
        
        # 计算频率
        if self.last_cmd_vel_time is not None:
            dt = current_time - self.last_cmd_vel_time
            if dt > 0:
                freq = 1.0 / dt
                self.cmd_vel_frequencies.append(freq)
        
        self.last_cmd_vel_time = current_time
        
        # 记录处理时间
        if self.last_laser_time is not None:
            processing_time = (current_time - self.last_laser_time) * 1000  # ms
            self.processing_times.append(processing_time)
            
        self.get_logger().debug(
            f"📦 收到速度指令: vx={msg.linear.x:.3f}, vy={msg.linear.y:.3f}, "
            f"vz={msg.angular.z:.3f}")
    
    def laser_callback(self, msg):
        """处理激光雷达数据回调"""
        self.laser_scan_count += 1
        self.last_laser_time = time.time()
        
        # 分析激光雷达数据质量
        valid_points = sum(1 for r in msg.ranges 
                          if msg.range_min <= r <= msg.range_max)
        total_points = len(msg.ranges)
        
        self.get_logger().debug(
            f"🔍 激光雷达数据: {valid_points}/{total_points} 有效点")
        
    def odom_callback(self, msg):
        """处理里程计数据回调"""
        # 更新机器人位置用于可视化
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
    def simulate_sensor_data(self):
        """仿真传感器数据 - 用于无硬件测试"""
        current_time = self.get_clock().now()
        
        # 更新机器人位置
        dt = 0.05  # 20Hz
        self.robot_x += self.robot_vx * dt * cos(self.robot_theta) - self.robot_vy * dt * sin(self.robot_theta)
        self.robot_y += self.robot_vx * dt * sin(self.robot_theta) + self.robot_vy * dt * cos(self.robot_theta)
        self.robot_theta += self.robot_vtheta * dt
        
        # 发布模拟里程计
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.robot_x
        odom_msg.pose.pose.position.y = self.robot_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = sin(self.robot_theta / 2.0)
        odom_msg.pose.pose.orientation.w = cos(self.robot_theta / 2.0)
        odom_msg.twist.twist.linear.x = self.robot_vx
        odom_msg.twist.twist.linear.y = self.robot_vy
        odom_msg.twist.twist.angular.z = self.robot_vtheta
        
        self.sim_odom_pub.publish(odom_msg)
        
        # 发布模拟激光雷达数据
        laser_msg = LaserScan()
        laser_msg.header.stamp = current_time.to_msg()
        laser_msg.header.frame_id = "laser_link"
        laser_msg.angle_min = -pi
        laser_msg.angle_max = pi
        laser_msg.angle_increment = pi / 180.0  # 1度
        laser_msg.range_min = 0.1
        laser_msg.range_max = 10.0
        
        # 创建一些虚拟障碍物
        ranges = []
        for i in range(360):
            angle = laser_msg.angle_min + i * laser_msg.angle_increment
            # 创建虚拟环境 - 圆形房间边界
            distance = 5.0
            
            # 添加一些随机障碍物
            if abs(angle) < pi/4:  # 前方有障碍物
                if i % 30 == 0:  # 稀疏障碍物
                    distance = 2.0 + 0.5 * sin(time.time() + angle)
                    
            ranges.append(max(laser_msg.range_min, 
                            min(laser_msg.range_max, distance)))
        
        laser_msg.ranges = ranges
        self.sim_laser_pub.publish(laser_msg)
        
        # 发布 TF 变换
        tf_msg = TFMessage()
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.robot_x
        transform.transform.translation.y = self.robot_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = sin(self.robot_theta / 2.0)
        transform.transform.rotation.w = cos(self.robot_theta / 2.0)
        tf_msg.transforms = [transform]
        
        # base_link -> laser_link 变换
        laser_transform = TransformStamped()
        laser_transform.header.stamp = current_time.to_msg()
        laser_transform.header.frame_id = "base_link"
        laser_transform.child_frame_id = "laser_link"
        laser_transform.transform.translation.x = 0.1
        laser_transform.transform.translation.y = 0.0
        laser_transform.transform.translation.z = 0.2
        laser_transform.transform.rotation.w = 1.0
        tf_msg.transforms.append(laser_transform)
        
        self.sim_tf_pub.publish(tf_msg)
        
    def test_update_callback(self):
        """测试更新回调"""
        if self.test_start_time is None:
            self.test_start_time = time.time()
            self.send_test_goal()
            
        elapsed_time = time.time() - self.test_start_time
        
        # 每10秒发送一个新目标
        if int(elapsed_time) % 10 == 0 and int(elapsed_time) > 0:
            if int(elapsed_time) not in getattr(self, '_sent_goals', set()):
                self.send_test_goal()
                if not hasattr(self, '_sent_goals'):
                    self._sent_goals = set()
                self._sent_goals.add(int(elapsed_time))
        
        # 测试结束
        if elapsed_time >= self.test_duration:
            self.finalize_test()
            
    def send_test_goal(self):
        """发送测试目标点"""
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        
        # 生成随机目标点
        import random
        angle = random.uniform(0, 2*pi)
        distance = random.uniform(2.0, 4.0)
        
        goal_msg.pose.position.x = self.robot_x + distance * cos(angle)
        goal_msg.pose.position.y = self.robot_y + distance * sin(angle)
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal_msg)
        
        self.get_logger().info(
            f"🎯 发送测试目标: ({goal_msg.pose.position.x:.2f}, "
            f"{goal_msg.pose.position.y:.2f})")
        
    def print_interim_results(self):
        """打印中期测试结果"""
        if self.test_start_time is None:
            return
            
        elapsed_time = time.time() - self.test_start_time
        
        # 计算统计信息
        avg_cmd_freq = np.mean(self.cmd_vel_frequencies[-20:]) if self.cmd_vel_frequencies else 0
        avg_processing_time = np.mean(self.processing_times[-20:]) if self.processing_times else 0
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"📊 中期测试结果 (运行时间: {elapsed_time:.1f}s)")
        self.get_logger().info(f"📦 速度指令数量: {self.cmd_vel_count}")
        self.get_logger().info(f"🔍 激光数据数量: {self.laser_scan_count}")
        self.get_logger().info(f"🔄 平均控制频率: {avg_cmd_freq:.2f} Hz")
        self.get_logger().info(f"⏱️  平均处理时间: {avg_processing_time:.2f} ms")
        self.get_logger().info("=" * 60)
        
    def finalize_test(self):
        """完成测试并生成报告"""
        total_time = time.time() - self.test_start_time
        
        # 计算最终统计
        self.test_results = {
            'total_time': total_time,
            'cmd_vel_count': self.cmd_vel_count,
            'laser_scan_count': self.laser_scan_count,
            'avg_cmd_frequency': np.mean(self.cmd_vel_frequencies) if self.cmd_vel_frequencies else 0,
            'max_cmd_frequency': np.max(self.cmd_vel_frequencies) if self.cmd_vel_frequencies else 0,
            'min_cmd_frequency': np.min(self.cmd_vel_frequencies) if self.cmd_vel_frequencies else 0,
            'avg_processing_time': np.mean(self.processing_times) if self.processing_times else 0,
            'max_processing_time': np.max(self.processing_times) if self.processing_times else 0,
            'min_processing_time': np.min(self.processing_times) if self.processing_times else 0,
        }
        
        self.print_final_report()
        self.save_test_report()
        
        # 关闭节点
        self.get_logger().info("✅ 测试完成，正在关闭节点...")
        rclpy.shutdown()
        
    def print_final_report(self):
        """打印最终测试报告"""
        results = self.test_results
        
        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("🎉 NEUPAN NAV2 CONTROLLER 测试报告")
        self.get_logger().info("=" * 80)
        self.get_logger().info(f"⏰ 测试总时长: {results['total_time']:.2f} 秒")
        self.get_logger().info("")
        
        # 基础统计
        self.get_logger().info("📊 基础统计:")
        self.get_logger().info(f"  - 速度指令总数: {results['cmd_vel_count']}")
        self.get_logger().info(f"  - 激光数据总数: {results['laser_scan_count']}")
        self.get_logger().info("")
        
        # 性能指标
        self.get_logger().info("🚀 性能指标:")
        self.get_logger().info(f"  - 平均控制频率: {results['avg_cmd_frequency']:.2f} Hz")
        self.get_logger().info(f"  - 最大控制频率: {results['max_cmd_frequency']:.2f} Hz")
        self.get_logger().info(f"  - 最小控制频率: {results['min_cmd_frequency']:.2f} Hz")
        self.get_logger().info("")
        
        # 延迟统计
        self.get_logger().info("⏱️  延迟统计:")
        self.get_logger().info(f"  - 平均处理时间: {results['avg_processing_time']:.2f} ms")
        self.get_logger().info(f"  - 最大处理时间: {results['max_processing_time']:.2f} ms")
        self.get_logger().info(f"  - 最小处理时间: {results['min_processing_time']:.2f} ms")
        self.get_logger().info("")
        
        # 评估结果
        self.get_logger().info("🎯 性能评估:")
        
        # 控制频率评估
        if results['avg_cmd_frequency'] >= 15.0:
            freq_status = "✅ 优秀"
        elif results['avg_cmd_frequency'] >= 10.0:
            freq_status = "⚠️  良好"
        else:
            freq_status = "❌ 需要优化"
        self.get_logger().info(f"  - 控制频率: {freq_status}")
        
        # 处理延迟评估
        if results['avg_processing_time'] <= 50.0:
            latency_status = "✅ 优秀"
        elif results['avg_processing_time'] <= 100.0:
            latency_status = "⚠️  良好"
        else:
            latency_status = "❌ 需要优化"
        self.get_logger().info(f"  - 处理延迟: {latency_status}")
        
        # 数据接收评估
        expected_laser_count = int(results['total_time'] * 20)  # 20Hz expected
        data_ratio = results['laser_scan_count'] / expected_laser_count if expected_laser_count > 0 else 0
        
        if data_ratio >= 0.9:
            data_status = "✅ 优秀"
        elif data_ratio >= 0.7:
            data_status = "⚠️  良好"
        else:
            data_status = "❌ 数据丢失严重"
        self.get_logger().info(f"  - 数据完整性: {data_status} ({data_ratio:.1%})")
        
        self.get_logger().info("=" * 80)
        
    def save_test_report(self):
        """保存测试报告到文件"""
        import json
        import os
        from datetime import datetime
        
        # 创建报告目录
        report_dir = "/tmp/neupan_test_reports"
        os.makedirs(report_dir, exist_ok=True)
        
        # 生成报告文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_file = os.path.join(report_dir, f"neupan_test_{timestamp}.json")
        
        # 保存结果
        with open(report_file, 'w') as f:
            json.dump(self.test_results, f, indent=2)
            
        self.get_logger().info(f"📁 测试报告已保存: {report_file}")


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        tester = NeuPANControllerTester()
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print("\n⚠️  测试被用户中断")
    except Exception as e:
        print(f"❌ 测试过程中发生错误: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()