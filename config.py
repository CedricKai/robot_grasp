#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
眼在手外抓取系统配置文件
包含机械臂、相机、抓取等参数设置
"""

import numpy as np


class GraspConfig:
    """抓取系统配置类"""
    
    def __init__(self):
        # 机械臂配置
        self.ROBOT_IP = '192.168.31.100'
        self.ROBOT_PORT = 8080
        
        # 工作空间限制 (mm)
        self.WORKSPACE_LIMITS = {
            'x': [200, 600],
            'y': [-300, 300], 
            'z': [0, 400]
        }
        
        # 关节限制 (度)
        self.JOINT_LIMITS = {
            'joint1': [-360, 360],
            'joint2': [-360, 360],
            'joint3': [-360, 360],
            'joint4': [-360, 360],
            'joint5': [-360, 360],
            'joint6': [-360, 360]
        }
        
        # 运动参数
        self.MOTION_CONFIG = {
            'default_speed': 50,      # mm/s
            'default_acc': 500,       # mm/s²
            'default_wait': True,
            'pause_time': 1           # s
        }
        
        # 夹爪配置
        self.GRIPPER_CONFIG = {
            'open_position': 85,      # 开启位置
            'close_position': 0,      # 关闭位置
            'wait_time': 1           # s
        }
        
        # 抓取参数
        self.GRASP_CONFIG = {
            'grasp_height': 50,       # 抓取高度偏移 (mm)
            'approach_height': 100,   # 接近高度 (mm)
            'lift_height': 150,       # 提升高度 (mm)
            'approach_angle': 0       # 接近角度 (度)
        }
        
        # 初始位置
        self.INIT_POSE = [350, 0, 150, 180, 0, 0]  # [x, y, z, rx, ry, rz]
        
        # 相机配置
        self.CAMERA_CONFIG = {
            'width': 640,
            'height': 480,
            'fps': 30
        }
        
        # 相机内参（需要根据实际相机调整）
        self.CAMERA_INTRINSICS = np.array([
            [640, 0, 320],
            [0, 640, 240],
            [0, 0, 1]
        ])
        
        # 相机畸变系数
        self.CAMERA_DIST_COEFFS = np.zeros(5)
        
        # 相机到基座的变换矩阵（需要根据实际标定调整）
        self.RT_CAM2BASE = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 500],  # 相机在基座上方500mm
            [0, 0, 0, 1]
        ])
        
        # 标定参数
        self.CALIBRATION_CONFIG = {
            'chessboard_size': (9, 6),    # 标定板内角点数
            'square_size': 7,              # 标定板方格尺寸 (mm)
            'num_images': 16,              # 标定图片数量
            'min_detection_quality': 0.8   # 最小检测质量
        }
        
        # 视觉检测配置
        self.VISION_CONFIG = {
            'model_path': '../ultralytics/yolov8n.pt',
            'confidence_threshold': 0.5,
            'nms_threshold': 0.4,
            'target_classes': [47]  # 目标类别ID
        }
        
        # 安全配置
        self.SAFETY_CONFIG = {
            'emergency_stop_enabled': True,
            'collision_detection': True,
            'max_force': 100,        # N
            'max_torque': 10         # Nm
        }
        
        # 日志配置
        self.LOG_CONFIG = {
            'log_level': 'INFO',
            'log_file': 'grasp_system.log',
            'save_images': True,
            'save_trajectories': True
        }
    
    def get_robot_config(self):
        """获取机械臂配置"""
        return {
            'ip': self.ROBOT_IP,
            'port': self.ROBOT_PORT,
            'workspace_limits': self.WORKSPACE_LIMITS,
            'joint_limits': self.JOINT_LIMITS,
            'motion_config': self.MOTION_CONFIG,
            'gripper_config': self.GRIPPER_CONFIG,
            'init_pose': self.INIT_POSE
        }
    
    def get_camera_config(self):
        """获取相机配置"""
        return {
            'config': self.CAMERA_CONFIG,
            'intrinsics': self.CAMERA_INTRINSICS,
            'dist_coeffs': self.CAMERA_DIST_COEFFS,
            'RT_cam2base': self.RT_CAM2BASE
        }
    
    def get_grasp_config(self):
        """获取抓取配置"""
        return self.GRASP_CONFIG
    
    def get_calibration_config(self):
        """获取标定配置"""
        return self.CALIBRATION_CONFIG
    
    def get_vision_config(self):
        """获取视觉配置"""
        return self.VISION_CONFIG
    
    def get_safety_config(self):
        """获取安全配置"""
        return self.SAFETY_CONFIG
    
    def get_log_config(self):
        """获取日志配置"""
        return self.LOG_CONFIG
    
    def update_calibration(self, RT_cam2base, camera_intrinsics, dist_coeffs):
        """
        更新标定参数
        
        Args:
            RT_cam2base: 相机到基座的变换矩阵
            camera_intrinsics: 相机内参
            dist_coeffs: 相机畸变系数
        """
        self.RT_CAM2BASE = RT_cam2base
        self.CAMERA_INTRINSICS = camera_intrinsics
        self.CAMERA_DIST_COEFFS = dist_coeffs
        print("标定参数已更新")
    
    def save_config(self, filepath):
        """保存配置到文件"""
        import json
        import numpy as np
        
        # 转换numpy数组为列表
        config_dict = {
            'ROBOT_IP': self.ROBOT_IP,
            'WORKSPACE_LIMITS': self.WORKSPACE_LIMITS,
            'MOTION_CONFIG': self.MOTION_CONFIG,
            'GRIPPER_CONFIG': self.GRIPPER_CONFIG,
            'GRASP_CONFIG': self.GRASP_CONFIG,
            'INIT_POSE': self.INIT_POSE,
            'CAMERA_CONFIG': self.CAMERA_CONFIG,
            'CAMERA_INTRINSICS': self.CAMERA_INTRINSICS.tolist(),
            'CAMERA_DIST_COEFFS': self.CAMERA_DIST_COEFFS.tolist(),
            'RT_CAM2BASE': self.RT_CAM2BASE.tolist(),
            'CALIBRATION_CONFIG': self.CALIBRATION_CONFIG,
            'VISION_CONFIG': self.VISION_CONFIG,
            'SAFETY_CONFIG': self.SAFETY_CONFIG,
            'LOG_CONFIG': self.LOG_CONFIG
        }
        
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(config_dict, f, indent=2, ensure_ascii=False)
        
        print(f"配置已保存到: {filepath}")
    
    def load_config(self, filepath):
        """从文件加载配置"""
        import json
        import numpy as np
        
        with open(filepath, 'r', encoding='utf-8') as f:
            config_dict = json.load(f)
        
        # 更新配置
        self.ROBOT_IP = config_dict.get('ROBOT_IP', self.ROBOT_IP)
        self.WORKSPACE_LIMITS = config_dict.get('WORKSPACE_LIMITS', self.WORKSPACE_LIMITS)
        self.MOTION_CONFIG = config_dict.get('MOTION_CONFIG', self.MOTION_CONFIG)
        self.GRIPPER_CONFIG = config_dict.get('GRIPPER_CONFIG', self.GRIPPER_CONFIG)
        self.GRASP_CONFIG = config_dict.get('GRASP_CONFIG', self.GRASP_CONFIG)
        self.INIT_POSE = config_dict.get('INIT_POSE', self.INIT_POSE)
        self.CAMERA_CONFIG = config_dict.get('CAMERA_CONFIG', self.CAMERA_CONFIG)
        self.CAMERA_INTRINSICS = np.array(config_dict.get('CAMERA_INTRINSICS', self.CAMERA_INTRINSICS.tolist()))
        self.CAMERA_DIST_COEFFS = np.array(config_dict.get('CAMERA_DIST_COEFFS', self.CAMERA_DIST_COEFFS.tolist()))
        self.RT_CAM2BASE = np.array(config_dict.get('RT_CAM2BASE', self.RT_CAM2BASE.tolist()))
        self.CALIBRATION_CONFIG = config_dict.get('CALIBRATION_CONFIG', self.CALIBRATION_CONFIG)
        self.VISION_CONFIG = config_dict.get('VISION_CONFIG', self.VISION_CONFIG)
        self.SAFETY_CONFIG = config_dict.get('SAFETY_CONFIG', self.SAFETY_CONFIG)
        self.LOG_CONFIG = config_dict.get('LOG_CONFIG', self.LOG_CONFIG)
        
        print(f"配置已从文件加载: {filepath}")


# 创建全局配置实例
config = GraspConfig()


def get_config():
    """获取全局配置实例"""
    return config


def demo_config():
    """演示配置功能"""
    print("抓取系统配置演示")
    print("=" * 40)
    
    # 获取配置
    cfg = get_config()
    
    print("机械臂配置:")
    robot_config = cfg.get_robot_config()
    print(f"  IP地址: {robot_config['ip']}")
    print(f"  工作空间: {robot_config['workspace_limits']}")
    print(f"  初始位置: {robot_config['init_pose']}")
    
    print("\n相机配置:")
    camera_config = cfg.get_camera_config()
    print(f"  分辨率: {camera_config['config']['width']}x{camera_config['config']['height']}")
    print(f"  帧率: {camera_config['config']['fps']}")
    
    print("\n抓取配置:")
    grasp_config = cfg.get_grasp_config()
    print(f"  抓取高度偏移: {grasp_config['grasp_height']} mm")
    print(f"  接近高度: {grasp_config['approach_height']} mm")
    
    print("\n安全配置:")
    safety_config = cfg.get_safety_config()
    print(f"  紧急停止: {'启用' if safety_config['emergency_stop_enabled'] else '禁用'}")
    print(f"  碰撞检测: {'启用' if safety_config['collision_detection'] else '禁用'}")
    
    # 保存配置示例
    cfg.save_config('grasp_config.json')
    
    print("\n配置演示完成!")


if __name__ == "__main__":
    demo_config() 