#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from status_interface.msg import SystemStatus
import psutil
import platform

import time
from builtin_interfaces.msg import Time

class SystemStatusPublisher(Node):
    """系统状态发布节点，用于监控并发布系统资源使用情况"""
    
    def __init__(self):
        super().__init__('system_status_publisher')
        # 创建发布者，发布SystemStatus消息到'system_status'话题
        self.publisher_ = self.create_publisher(SystemStatus, 'system_status', 10)
        # 设置定时器，每1秒触发一次回调函数
        self.timer = self.create_timer(1.0, self.timer_callback)
        # 获取主机名
        self.hostname = platform.node()
        self.get_logger().info('系统状态发布节点已启动')
        
    def timer_callback(self):
        """定时器回调函数，收集系统信息并发布"""
        # 创建SystemStatus消息
        msg = SystemStatus()
        
        # 设置时间戳
        msg.stamp = self.get_clock().now().to_msg()
        
        # 设置主机名
        msg.host_name = self.hostname
        
        # 获取CPU使用率
        msg.cpu_percent = psutil.cpu_percent()
        
        # 获取内存信息
        memory = psutil.virtual_memory()
        msg.memory_percent = memory.percent
        msg.memory_total = float(memory.total) / (1024 * 1024 * 1024)  # 转换为GB
        msg.memory_available = float(memory.available) / (1024 * 1024 * 1024)  # 转换为GB
        
        # 获取网络信息
        net_io = psutil.net_io_counters()
        msg.net_set = float(net_io.bytes_sent) / (1024 * 1024)  # 转换为MB
        msg.net_recv = float(net_io.bytes_recv) / (1024 * 1024)  # 转换为MB
        
        # 发布消息
        self.publisher_.publish(msg)
        self.get_logger().info(f'已发布系统状态: CPU: {msg.cpu_percent}%, 内存: {msg.memory_percent}%')

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = SystemStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
