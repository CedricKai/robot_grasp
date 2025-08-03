#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
六自由度机械臂眼在手外（Eye-to-Hand）视觉-机械臂抓取系统
支持接受导入坐标并自主抓取物体到指定位置
"""

import numpy as np
import cv2
import sys
import os
from math import pi
from pathlib import Path

# 添加项目路径
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
sys.path.append(str(ROOT))

from core.wrap_xarm import xarm6
from core.realsense2 import Realsense2
from core.calibrate import Calibration


class EyeToHandGraspSystem:
    """眼在手外抓取系统"""
    
    def __init__(self, robot_ip='192.168.31.100'):
        """
        初始化抓取系统
        
        Args:
            robot_ip: 机械臂IP地址
        """
        self.robot = xarm6(robot_ip)
        self.camera = Realsense2()
        self.calibration = Calibration()
        
        # 标定参数（需要根据实际标定结果调整）
        self.RT_cam2base = None  # 相机到基座的变换矩阵
        self.camera_intrinsics = None
        self.dist_coeffs = None
        
        # 抓取参数
        self.grasp_height = 50  # 抓取高度偏移(mm)
        self.approach_height = 100  # 接近高度(mm)
        self.gripper_open_angle = 85  # 夹爪开启角度
        self.gripper_close_angle = 0  # 夹爪关闭角度
        
        # 工作空间限制
        self.workspace_limits = {
            'x': [200, 600],  # mm
            'y': [-300, 300],  # mm  
            'z': [0, 400]      # mm
        }
        
        self.init_system()
    
    def init_system(self):
        """初始化系统"""
        print("正在初始化眼在手外抓取系统...")
        
        # 初始化机械臂
        self.robot.init_robot()
        print("机械臂初始化完成")
        
        # 初始化相机
        print("相机初始化完成")
        
        # 加载标定参数（如果有的话）
        self.load_calibration()
        
        # 移动到初始位置
        self.robot.move_init_pose()
        print("系统初始化完成")
    
    def load_calibration(self):
        """加载标定参数"""
        cal_file = ROOT / "output" / "calibration_data.npy"
        if cal_file.exists():
            try:
                cal_data = np.load(cal_file, allow_pickle=True).item()
                self.RT_cam2base = cal_data['RT_cam2base']
                self.camera_intrinsics = cal_data['camera_intrinsics']
                self.dist_coeffs = cal_data['dist_coeffs']
                print("标定参数加载成功")
            except Exception as e:
                print(f"标定参数加载失败: {e}")
                self.set_default_calibration()
        else:
            print("未找到标定文件，使用默认参数")
            self.set_default_calibration()
    
    def set_default_calibration(self):
        """设置默认标定参数"""
        # 默认相机内参（需要根据实际相机调整）
        self.camera_intrinsics = np.array([
            [640, 0, 320],
            [0, 640, 240],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.zeros(5)
        
        # 默认相机到基座的变换矩阵（需要根据实际标定调整）
        self.RT_cam2base = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 500],  # 相机在基座上方500mm
            [0, 0, 0, 1]
        ])
    
    def pixel_to_robot_coordinates(self, pixel_coords, depth):
        """
        将像素坐标转换为机器人坐标系
        
        Args:
            pixel_coords: [x, y] 像素坐标
            depth: 深度值(mm)
            
        Returns:
            robot_coords: [x, y, z] 机器人坐标系坐标(mm)
        """
        if self.camera_intrinsics is None:
            raise ValueError("相机内参未设置")
        
        # 像素坐标转相机坐标
        fx = self.camera_intrinsics[0, 0]
        fy = self.camera_intrinsics[1, 1]
        cx = self.camera_intrinsics[0, 2]
        cy = self.camera_intrinsics[1, 2]
        
        x = (pixel_coords[0] - cx) * depth / fx
        y = (pixel_coords[1] - cy) * depth / fy
        z = depth
        
        # 相机坐标转机器人坐标
        camera_coords = np.array([x, y, z, 1])
        robot_coords = self.RT_cam2base @ camera_coords
        
        return robot_coords[:3]
    
    def get_grasp_pose(self, target_coords, approach_angle=0):
        """
        计算抓取位姿
        
        Args:
            target_coords: [x, y, z] 目标坐标(mm)
            approach_angle: 接近角度(度)
            
        Returns:
            grasp_pose: [x, y, z, rx, ry, rz] 抓取位姿
        """
        # 检查工作空间限制
        x, y, z = target_coords
        if not (self.workspace_limits['x'][0] <= x <= self.workspace_limits['x'][1] and
                self.workspace_limits['y'][0] <= y <= self.workspace_limits['y'][1] and
                self.workspace_limits['z'][0] <= z <= self.workspace_limits['z'][1]):
            raise ValueError(f"目标坐标 {target_coords} 超出工作空间限制")
        
        # 计算抓取位姿
        grasp_pose = [
            x, y, z + self.grasp_height,  # 位置
            approach_angle, 0, 0  # 姿态（欧拉角）
        ]
        
        return grasp_pose
    
    def execute_grasp_sequence(self, target_coords, place_coords):
        """
        执行抓取序列
        
        Args:
            target_coords: [x, y, z] 抓取目标坐标(mm)
            place_coords: [x, y, z] 放置目标坐标(mm)
        """
        print(f"开始执行抓取任务:")
        print(f"抓取坐标: {target_coords}")
        print(f"放置坐标: {place_coords}")
        
        try:
            # 1. 移动到初始位置
            self.robot.move_init_pose()
            
            # 2. 计算抓取位姿
            grasp_pose = self.get_grasp_pose(target_coords)
            place_pose = self.get_grasp_pose(place_coords)
            
            # 3. 移动到接近位置
            approach_pose = grasp_pose.copy()
            approach_pose[2] += self.approach_height
            self.robot.move(approach_pose)
            
            # 4. 打开夹爪
            self.robot.arm.set_gripper_position(self.gripper_open_angle, wait=True)
            
            # 5. 移动到抓取位置
            self.robot.move(grasp_pose)
            
            # 6. 关闭夹爪
            self.robot.arm.set_gripper_position(self.gripper_close_angle, wait=True)
            
            # 7. 提升到安全高度
            lift_pose = grasp_pose.copy()
            lift_pose[2] += self.approach_height
            self.robot.move(lift_pose)
            
            # 8. 移动到放置接近位置
            place_approach_pose = place_pose.copy()
            place_approach_pose[2] += self.approach_height
            self.robot.move(place_approach_pose)
            
            # 9. 移动到放置位置
            self.robot.move(place_pose)
            
            # 10. 打开夹爪释放物体
            self.robot.arm.set_gripper_position(self.gripper_open_angle, wait=True)
            
            # 11. 提升到安全高度
            self.robot.move(place_approach_pose)
            
            # 12. 返回初始位置
            self.robot.move_init_pose()
            
            print("抓取任务完成!")
            
        except Exception as e:
            print(f"抓取任务失败: {e}")
            # 紧急停止并返回初始位置
            self.robot.move_init_pose()
            raise
    
    def grasp_from_coordinates(self, target_coords, place_coords):
        """
        从坐标执行抓取
        
        Args:
            target_coords: [x, y, z] 抓取目标坐标(mm)
            place_coords: [x, y, z] 放置目标坐标(mm)
        """
        try:
            self.execute_grasp_sequence(target_coords, place_coords)
            return True
        except Exception as e:
            print(f"抓取失败: {e}")
            return False
    
    def grasp_from_pixel_coordinates(self, pixel_coords, depth, place_coords):
        """
        从像素坐标执行抓取
        
        Args:
            pixel_coords: [x, y] 像素坐标
            depth: 深度值(mm)
            place_coords: [x, y, z] 放置目标坐标(mm)
        """
        try:
            # 转换像素坐标到机器人坐标
            target_coords = self.pixel_to_robot_coordinates(pixel_coords, depth)
            print(f"像素坐标 {pixel_coords} 转换为机器人坐标: {target_coords}")
            
            # 执行抓取
            return self.grasp_from_coordinates(target_coords, place_coords)
            
        except Exception as e:
            print(f"坐标转换失败: {e}")
            return False
    
    def demo_grasp(self):
        """演示抓取功能"""
        print("开始演示抓取功能...")
        
        # 示例坐标（需要根据实际情况调整）
        target_coords = [400, 0, 100]  # 抓取位置
        place_coords = [400, 100, 100]  # 放置位置
        
        success = self.grasp_from_coordinates(target_coords, place_coords)
        
        if success:
            print("演示完成!")
        else:
            print("演示失败!")


def main():
    """主函数"""
    print("六自由度机械臂眼在手外抓取系统")
    print("=" * 50)
    
    # 创建抓取系统
    grasp_system = EyeToHandGraspSystem()
    
    # 示例用法
    print("\n示例1: 直接坐标抓取")
    target = [400, 0, 100]  # 抓取位置
    place = [400, 100, 100]  # 放置位置
    grasp_system.grasp_from_coordinates(target, place)
    
    print("\n示例2: 像素坐标抓取")
    pixel_coords = [320, 240]  # 像素坐标
    depth = 500  # 深度值(mm)
    place = [400, -100, 100]  # 放置位置
    grasp_system.grasp_from_pixel_coordinates(pixel_coords, depth, place)
    
    print("\n示例3: 演示抓取")
    grasp_system.demo_grasp()


if __name__ == "__main__":
    main() 