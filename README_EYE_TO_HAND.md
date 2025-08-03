# 六自由度机械臂眼在手外（Eye-to-Hand）抓取系统

## 概述

本项目是一个针对六自由度机械臂的眼在手外（Eye-to-Hand）视觉-机械臂抓取系统。系统能够接受导入的坐标，并自主抓取物体到指定位置。

## 系统特点

- **眼在手外配置**: 相机固定在机械臂外部，提供稳定的视觉反馈
- **六自由度控制**: 支持完整的空间位姿控制
- **坐标转换**: 支持像素坐标到机器人坐标的自动转换
- **安全保护**: 包含工作空间限制和紧急停止功能
- **模块化设计**: 易于扩展和维护

## 文件结构

```
robot_grasp/
├── core/
│   ├── robot_controller.py      # 机械臂控制器
│   ├── wrap_xarm.py            # xArm机械臂封装
│   ├── realsense2.py           # RealSense相机封装
│   ├── calibrate.py            # 标定功能
│   └── utils.py                # 工具函数
├── eye_to_hand_grasp.py        # 主抓取系统
├── example_usage.py            # 使用示例
├── config.py                   # 配置文件
└── README_EYE_TO_HAND.md      # 本说明文档
```

## 安装要求

### 硬件要求
- 六自由度机械臂（如xArm6、CR5等）
- RealSense深度相机
- 夹爪装置

### 软件要求
```bash
pip install numpy opencv-python transforms3d
```

### 机械臂SDK
根据您使用的机械臂型号安装相应的SDK：
- xArm: `pip install xarm`
- CR5: 参考Dobot官方文档

## 快速开始

### 1. 基本使用

```python
from core.robot_controller import RobotController

# 创建机械臂控制器
controller = RobotController(robot_ip='192.168.31.100')

# 定义抓取和放置坐标
target_coords = [400, 0, 100]    # 抓取位置 (mm)
place_coords = [400, 100, 100]   # 放置位置 (mm)

# 执行抓取任务
success = controller.execute_grasp_sequence(target_coords, place_coords)

if success:
    print("抓取任务完成!")
else:
    print("抓取任务失败!")

# 断开连接
controller.disconnect()
```

### 2. 像素坐标抓取

```python
from eye_to_hand_grasp import EyeToHandGraspSystem

# 创建抓取系统
grasp_system = EyeToHandGraspSystem()

# 从像素坐标执行抓取
pixel_coords = [320, 240]  # 像素坐标
depth = 500                # 深度值 (mm)
place_coords = [400, 100, 100]  # 放置位置

success = grasp_system.grasp_from_pixel_coordinates(
    pixel_coords, depth, place_coords
)
```

### 3. 批量抓取

```python
# 定义多个物体的抓取任务
objects = [
    {'grasp': [400, -50, 100], 'place': [400, 50, 100]},
    {'grasp': [400, 0, 100], 'place': [400, 100, 100]},
    {'grasp': [400, 50, 100], 'place': [400, 150, 100]}
]

controller = RobotController()

for obj in objects:
    success = controller.execute_grasp_sequence(
        obj['grasp'], obj['place']
    )
    if not success:
        break
```

## 配置说明

### 机械臂配置

在 `config.py` 中修改机械臂相关参数：

```python
# 机械臂IP地址
ROBOT_IP = '192.168.31.100'

# 工作空间限制 (mm)
WORKSPACE_LIMITS = {
    'x': [200, 600],
    'y': [-300, 300], 
    'z': [0, 400]
}

# 初始位置
INIT_POSE = [350, 0, 150, 180, 0, 0]  # [x, y, z, rx, ry, rz]
```

### 抓取参数配置

```python
# 抓取参数
GRASP_CONFIG = {
    'grasp_height': 50,       # 抓取高度偏移 (mm)
    'approach_height': 100,    # 接近高度 (mm)
    'lift_height': 150,        # 提升高度 (mm)
    'approach_angle': 0        # 接近角度 (度)
}
```

### 相机标定

系统需要相机标定参数，包括：
- 相机内参矩阵
- 畸变系数
- 相机到基座的变换矩阵

可以使用 `eye-to-hand.py` 进行标定，或手动设置参数。

## 坐标系统

### 坐标系定义

- **基座坐标系**: 机械臂基座坐标系
- **工具坐标系**: 机械臂末端工具坐标系
- **相机坐标系**: 相机坐标系
- **像素坐标系**: 图像像素坐标系

### 坐标转换

系统支持以下坐标转换：
1. 像素坐标 → 相机坐标
2. 相机坐标 → 基座坐标
3. 基座坐标 → 工具坐标

## 安全注意事项

### 1. 工作空间限制
系统会自动检查目标位置是否在工作空间内，超出范围会拒绝执行。

### 2. 紧急停止
- 机械臂控制器提供紧急停止功能
- 异常情况下会自动返回初始位置

### 3. 碰撞检测
建议启用碰撞检测功能，防止机械臂与障碍物碰撞。

## 故障排除

### 常见问题

1. **机械臂连接失败**
   - 检查网络连接
   - 确认IP地址正确
   - 检查机械臂电源和网络状态

2. **抓取精度问题**
   - 检查相机标定参数
   - 验证坐标转换是否正确
   - 调整抓取参数

3. **运动失败**
   - 检查工作空间限制
   - 确认目标位置可达
   - 检查机械臂状态

### 调试模式

启用调试模式获取详细信息：

```python
import logging
logging.basicConfig(level=logging.DEBUG)

controller = RobotController()
```

## 扩展功能

### 1. 视觉检测集成

可以集成YOLO等目标检测算法：

```python
from ultralytics import YOLO

# 加载检测模型
model = YOLO('yolov8n.pt')

# 检测目标
results = model(image)
for result in results:
    if result.conf > 0.5:
        # 获取目标坐标
        bbox = result.boxes.xyxy[0]
        center = [(bbox[0] + bbox[2])/2, (bbox[1] + bbox[3])/2]
        # 执行抓取
        grasp_system.grasp_from_pixel_coordinates(center, depth, place_coords)
```

### 2. 多相机支持

可以扩展支持多个相机：

```python
class MultiCameraGraspSystem:
    def __init__(self, cameras):
        self.cameras = cameras
        self.controllers = []
        
    def select_best_camera(self, target_coords):
        # 选择最佳视角的相机
        pass
```

### 3. 力反馈控制

集成力传感器实现更精确的抓取：

```python
def force_controlled_grasp(self, target_coords, max_force=10):
    # 基于力反馈的抓取控制
    pass
```

## 性能优化

### 1. 运动规划优化
- 使用更高效的运动规划算法
- 优化路径以减少运动时间

### 2. 视觉处理优化
- 使用GPU加速视觉处理
- 优化图像预处理流程

### 3. 通信优化
- 使用更高效的通信协议
- 减少不必要的网络传输

## 许可证

本项目基于MIT许可证开源。

## 贡献

欢迎提交Issue和Pull Request来改进这个项目。

## 联系方式

如有问题，请通过以下方式联系：
- 提交GitHub Issue
- 发送邮件至项目维护者

---

**注意**: 使用本系统前请确保：
1. 机械臂已正确安装和配置
2. 相机已正确标定
3. 工作环境安全
4. 已阅读并理解安全注意事项 