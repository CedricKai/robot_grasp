#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
六自由度机械臂控制器
专门用于眼在手外抓取系统
"""

import numpy as np
import sys
import time
from math import pi
import threading


class RobotController:
    """机械臂控制器"""
    
    def __init__(self, robot_ip='192.168.31.100'):
        """
        初始化机械臂控制器
        
        Args:
            robot_ip: 机械臂IP地址
        """
        self.robot_ip = robot_ip
        self.arm = None
        self.is_connected = False
        
        # 机械臂参数
        self.joint_limits = {
            'joint1': [-360, 360],  # 度
            'joint2': [-360, 360],
            'joint3': [-360, 360],
            'joint4': [-360, 360],
            'joint5': [-360, 360],
            'joint6': [-360, 360]
        }
        
        # 工作空间限制 (mm)
        self.workspace_limits = {
            'x': [200, 600],
            'y': [-300, 300],
            'z': [0, 400]
        }
        
        # 运动参数
        self.default_speed = 50  # mm/s
        self.default_acc = 500   # mm/s²
        self.default_wait = True
        
        # 夹爪参数
        self.gripper_open_pos = 85   # 开启位置
        self.gripper_close_pos = 0   # 关闭位置
        
        # 初始位置
        self.init_pose = [350, 0, 150, 180, 0, 0]  # [x, y, z, rx, ry, rz]
        
        self.connect_robot()
    
    def connect_robot(self):
        """连接机械臂"""
        try:
            from robot_sdk.xArm.xarm.wrapper import XArmAPI
            self.arm = XArmAPI(self.robot_ip)
            
            # 检查连接状态
            if self.arm.connected:
                self.is_connected = True
                print(f"机械臂连接成功: {self.robot_ip}")
                self.init_robot()
            else:
                print(f"机械臂连接失败: {self.robot_ip}")
                self.is_connected = False
                
        except ImportError:
            print("警告: 未找到xArm SDK，使用模拟模式")
            self.is_connected = False
        except Exception as e:
            print(f"机械臂连接错误: {e}")
            self.is_connected = False
    
    def init_robot(self):
        """初始化机械臂"""
        if not self.is_connected:
            print("机械臂未连接，跳过初始化")
            return
        
        try:
            # 启用运动
            self.arm.motion_enable(enable=True)
            
            # 设置模式：位置控制模式
            self.arm.set_mode(0)
            
            # 设置状态：运动状态
            self.arm.set_state(state=0)
            
            # 设置暂停时间
            self.arm.set_pause_time(1)
            
            # 移动到初始位置
            self.move_to_init_pose()
            
            print("机械臂初始化完成")
            
        except Exception as e:
            print(f"机械臂初始化失败: {e}")
    
    def move_to_init_pose(self):
        """移动到初始位置"""
        if self.is_connected:
            try:
                self.arm.set_position(*self.init_pose, 
                                   speed=self.default_speed, 
                                   mvacc=self.default_acc, 
                                   wait=self.default_wait)
                print("已移动到初始位置")
            except Exception as e:
                print(f"移动到初始位置失败: {e}")
        else:
            print("模拟: 移动到初始位置")
    
    def move_to_position(self, position, speed=None, acc=None):
        """
        移动到指定位置
        
        Args:
            position: [x, y, z, rx, ry, rz] 位置和姿态
            speed: 速度 (mm/s)
            acc: 加速度 (mm/s²)
        """
        if not self.is_connected:
            print(f"模拟: 移动到位置 {position}")
            return True
        
        try:
            # 检查工作空间限制
            if not self.check_workspace_limits(position[:3]):
                print(f"位置 {position} 超出工作空间限制")
                return False
            
            # 设置默认参数
            speed = speed or self.default_speed
            acc = acc or self.default_acc
            
            # 执行运动
            self.arm.set_position(*position, 
                               speed=speed, 
                               mvacc=acc, 
                               wait=self.default_wait)
            
            print(f"已移动到位置: {position}")
            return True
            
        except Exception as e:
            print(f"移动失败: {e}")
            return False
    
    def check_workspace_limits(self, position):
        """
        检查工作空间限制
        
        Args:
            position: [x, y, z] 位置
            
        Returns:
            bool: 是否在工作空间内
        """
        x, y, z = position
        
        return (self.workspace_limits['x'][0] <= x <= self.workspace_limits['x'][1] and
                self.workspace_limits['y'][0] <= y <= self.workspace_limits['y'][1] and
                self.workspace_limits['z'][0] <= z <= self.workspace_limits['z'][1])
    
    def get_current_position(self):
        """
        获取当前位置
        
        Returns:
            position: [x, y, z, rx, ry, rz] 当前位置和姿态
        """
        if not self.is_connected:
            print("模拟: 获取当前位置")
            return self.init_pose.copy()
        
        try:
            pose_info = self.arm.get_position()
            if pose_info[0] == 0:
                return pose_info[1]
            else:
                print("获取位置失败")
                return None
        except Exception as e:
            print(f"获取位置失败: {e}")
            return None
    
    def set_gripper_position(self, position, wait=True):
        """
        设置夹爪位置
        
        Args:
            position: 夹爪位置 (0-100)
            wait: 是否等待完成
        """
        if not self.is_connected:
            print(f"模拟: 设置夹爪位置 {position}")
            return True
        
        try:
            self.arm.set_gripper_position(position, wait=wait)
            print(f"夹爪位置设置为: {position}")
            return True
        except Exception as e:
            print(f"设置夹爪位置失败: {e}")
            return False
    
    def open_gripper(self):
        """打开夹爪"""
        return self.set_gripper_position(self.gripper_open_pos)
    
    def close_gripper(self):
        """关闭夹爪"""
        return self.set_gripper_position(self.gripper_close_pos)
    
    def execute_grasp_sequence(self, target_coords, place_coords, 
                              grasp_height=50, approach_height=100):
        """
        执行抓取序列
        
        Args:
            target_coords: [x, y, z] 抓取目标坐标
            place_coords: [x, y, z] 放置目标坐标
            grasp_height: 抓取高度偏移
            approach_height: 接近高度
        """
        print(f"开始执行抓取序列:")
        print(f"抓取坐标: {target_coords}")
        print(f"放置坐标: {place_coords}")
        
        try:
            # 1. 移动到初始位置
            self.move_to_init_pose()
            
            # 2. 计算抓取位姿
            grasp_pose = [
                target_coords[0], target_coords[1], 
                target_coords[2] + grasp_height, 0, 0, 0
            ]
            
            place_pose = [
                place_coords[0], place_coords[1], 
                place_coords[2] + grasp_height, 0, 0, 0
            ]
            
            # 3. 移动到接近位置
            approach_pose = grasp_pose.copy()
            approach_pose[2] += approach_height
            self.move_to_position(approach_pose)
            
            # 4. 打开夹爪
            self.open_gripper()
            
            # 5. 移动到抓取位置
            self.move_to_position(grasp_pose)
            
            # 6. 关闭夹爪
            self.close_gripper()
            
            # 7. 提升到安全高度
            lift_pose = grasp_pose.copy()
            lift_pose[2] += approach_height
            self.move_to_position(lift_pose)
            
            # 8. 移动到放置接近位置
            place_approach_pose = place_pose.copy()
            place_approach_pose[2] += approach_height
            self.move_to_position(place_approach_pose)
            
            # 9. 移动到放置位置
            self.move_to_position(place_pose)
            
            # 10. 打开夹爪释放物体
            self.open_gripper()
            
            # 11. 提升到安全高度
            self.move_to_position(place_approach_pose)
            
            # 12. 返回初始位置
            self.move_to_init_pose()
            
            print("抓取序列执行完成!")
            return True
            
        except Exception as e:
            print(f"抓取序列执行失败: {e}")
            # 紧急停止并返回初始位置
            self.move_to_init_pose()
            return False
    
    def emergency_stop(self):
        """紧急停止"""
        if self.is_connected:
            try:
                self.arm.emergency_stop()
                print("紧急停止已执行")
            except Exception as e:
                print(f"紧急停止失败: {e}")
        else:
            print("模拟: 紧急停止")
    
    def disconnect(self):
        """断开连接"""
        if self.is_connected and self.arm:
            try:
                self.arm.disconnect()
                print("机械臂连接已断开")
            except Exception as e:
                print(f"断开连接失败: {e}")
        self.is_connected = False


def demo_robot_controller():
    """演示机械臂控制器"""
    print("机械臂控制器演示")
    print("=" * 30)
    
    # 创建控制器
    controller = RobotController()
    
    # 演示基本功能
    print("\n1. 获取当前位置")
    current_pos = controller.get_current_position()
    print(f"当前位置: {current_pos}")
    
    print("\n2. 测试夹爪")
    controller.open_gripper()
    time.sleep(1)
    controller.close_gripper()
    
    print("\n3. 执行抓取序列")
    target = [400, 0, 100]
    place = [400, 100, 100]
    success = controller.execute_grasp_sequence(target, place)
    
    if success:
        print("演示完成!")
    else:
        print("演示失败!")
    
    # 断开连接
    controller.disconnect()


if __name__ == "__main__":
    demo_robot_controller() 