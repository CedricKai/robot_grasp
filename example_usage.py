#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
眼在手外抓取系统使用示例
演示如何接受导入坐标并执行抓取任务
"""

import numpy as np
import sys
from pathlib import Path

# 添加项目路径
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
sys.path.append(str(ROOT))

from core.robot_controller import RobotController


def example_direct_coordinates():
    """示例1: 直接使用机器人坐标"""
    print("=" * 50)
    print("示例1: 直接使用机器人坐标")
    print("=" * 50)
    
    # 创建机械臂控制器
    controller = RobotController()
    
    # 定义抓取和放置坐标
    target_coords = [400, 0, 100]    # 抓取位置 (mm)
    place_coords = [400, 100, 100]   # 放置位置 (mm)
    
    print(f"抓取坐标: {target_coords}")
    print(f"放置坐标: {place_coords}")
    
    # 执行抓取任务
    success = controller.execute_grasp_sequence(target_coords, place_coords)
    
    if success:
        print("抓取任务完成!")
    else:
        print("抓取任务失败!")
    
    # 断开连接
    controller.disconnect()


def example_multiple_objects():
    """示例2: 抓取多个物体"""
    print("=" * 50)
    print("示例2: 抓取多个物体")
    print("=" * 50)
    
    # 创建机械臂控制器
    controller = RobotController()
    
    # 定义多个物体的抓取和放置坐标
    objects = [
        {
            'name': '物体1',
            'grasp': [400, -50, 100],
            'place': [400, 50, 100]
        },
        {
            'name': '物体2', 
            'grasp': [400, 0, 100],
            'place': [400, 100, 100]
        },
        {
            'name': '物体3',
            'grasp': [400, 50, 100], 
            'place': [400, 150, 100]
        }
    ]
    
    # 依次抓取每个物体
    for i, obj in enumerate(objects, 1):
        print(f"\n开始抓取 {obj['name']} ({i}/{len(objects)})")
        print(f"抓取坐标: {obj['grasp']}")
        print(f"放置坐标: {obj['place']}")
        
        success = controller.execute_grasp_sequence(
            obj['grasp'], obj['place']
        )
        
        if success:
            print(f"{obj['name']} 抓取完成!")
        else:
            print(f"{obj['name']} 抓取失败!")
            break
    
    # 断开连接
    controller.disconnect()


def example_with_coordinate_transformation():
    """示例3: 带坐标转换的抓取"""
    print("=" * 50)
    print("示例3: 带坐标转换的抓取")
    print("=" * 50)
    
    # 创建机械臂控制器
    controller = RobotController()
    
    # 模拟从视觉系统获得的像素坐标和深度
    pixel_coords = [320, 240]  # 像素坐标
    depth = 500                # 深度值 (mm)
    
    # 相机内参（需要根据实际相机调整）
    camera_intrinsics = np.array([
        [640, 0, 320],
        [0, 640, 240], 
        [0, 0, 1]
    ])
    
    # 相机到基座的变换矩阵（需要根据实际标定调整）
    RT_cam2base = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 500],  # 相机在基座上方500mm
        [0, 0, 0, 1]
    ])
    
    # 像素坐标转机器人坐标
    fx = camera_intrinsics[0, 0]
    fy = camera_intrinsics[1, 1]
    cx = camera_intrinsics[0, 2]
    cy = camera_intrinsics[1, 2]
    
    # 像素坐标转相机坐标
    x = (pixel_coords[0] - cx) * depth / fx
    y = (pixel_coords[1] - cy) * depth / fy
    z = depth
    
    # 相机坐标转机器人坐标
    camera_coords = np.array([x, y, z, 1])
    robot_coords = RT_cam2base @ camera_coords
    
    target_coords = robot_coords[:3]
    place_coords = [400, 100, 100]
    
    print(f"像素坐标: {pixel_coords}")
    print(f"深度: {depth} mm")
    print(f"转换后的机器人坐标: {target_coords}")
    print(f"放置坐标: {place_coords}")
    
    # 执行抓取任务
    success = controller.execute_grasp_sequence(target_coords, place_coords)
    
    if success:
        print("抓取任务完成!")
    else:
        print("抓取任务失败!")
    
    # 断开连接
    controller.disconnect()


def example_custom_grasp_parameters():
    """示例4: 自定义抓取参数"""
    print("=" * 50)
    print("示例4: 自定义抓取参数")
    print("=" * 50)
    
    # 创建机械臂控制器
    controller = RobotController()
    
    # 定义坐标
    target_coords = [400, 0, 100]
    place_coords = [400, 100, 100]
    
    # 自定义抓取参数
    grasp_height = 30      # 抓取高度偏移 (mm)
    approach_height = 80   # 接近高度 (mm)
    
    print(f"抓取坐标: {target_coords}")
    print(f"放置坐标: {place_coords}")
    print(f"抓取高度偏移: {grasp_height} mm")
    print(f"接近高度: {approach_height} mm")
    
    # 执行抓取任务
    success = controller.execute_grasp_sequence(
        target_coords, place_coords,
        grasp_height=grasp_height,
        approach_height=approach_height
    )
    
    if success:
        print("抓取任务完成!")
    else:
        print("抓取任务失败!")
    
    # 断开连接
    controller.disconnect()


def example_error_handling():
    """示例5: 错误处理"""
    print("=" * 50)
    print("示例5: 错误处理")
    print("=" * 50)
    
    # 创建机械臂控制器
    controller = RobotController()
    
    # 测试超出工作空间的坐标
    invalid_coords = [1000, 1000, 1000]  # 超出工作空间
    place_coords = [400, 100, 100]
    
    print(f"测试无效坐标: {invalid_coords}")
    
    try:
        success = controller.execute_grasp_sequence(invalid_coords, place_coords)
        print(f"结果: {'成功' if success else '失败'}")
    except Exception as e:
        print(f"捕获到异常: {e}")
    
    # 测试有效坐标
    valid_coords = [400, 0, 100]
    print(f"\n测试有效坐标: {valid_coords}")
    
    try:
        success = controller.execute_grasp_sequence(valid_coords, place_coords)
        print(f"结果: {'成功' if success else '失败'}")
    except Exception as e:
        print(f"捕获到异常: {e}")
    
    # 断开连接
    controller.disconnect()


def main():
    """主函数"""
    print("眼在手外抓取系统使用示例")
    print("=" * 60)
    
    # 运行所有示例
    examples = [
        example_direct_coordinates,
        example_multiple_objects,
        example_with_coordinate_transformation,
        example_custom_grasp_parameters,
        example_error_handling
    ]
    
    for i, example in enumerate(examples, 1):
        print(f"\n运行示例 {i}/{len(examples)}")
        try:
            example()
        except Exception as e:
            print(f"示例 {i} 运行失败: {e}")
        
        print("\n" + "="*60)
    
    print("所有示例运行完成!")


if __name__ == "__main__":
    main() 