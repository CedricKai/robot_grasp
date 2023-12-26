import numpy as np
import pyrealsense2 as rs
import os
import sys
import cv2
import time
import threading
from threading import Thread
from pathlib import Path
from core.utils import *


FILE = Path(__file__).resolve()       # __file__:当前路径 .resolve():获取绝对路径
ROOT = FILE.parents[0]                # .parents():路径的父目录
if str(ROOT) not in sys.path:         # sys.path是python的搜索模块的路径集，是一个list
    sys.path.append(str(ROOT))        # sys.path.append():添加相关路径，但在退出python环境后自己添加的路径就会消失
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # Path.cwd():返回当前工作目录 .relpath()得到相对路径


class Realsense2(object):
    def __init__(self, id=0, rgb_width=640, rgb_height=480, rgb_fps=60,
                 is_depth=True, is_depth_pix=False, is_colormap=True, is_filter=True, is_fps=True):
        """
        创建实时获取realsense相机帧的类
        :param id:                  打开指定相机
        :param rgb_width:           RGB的宽度(pix)
        :param rgb_height:          RGB的高度
        :param rgb_fps:             RGB的fps
        :param is_depth:            是否需要深度图
        :param is_depth_pix:        是否计算每个像素点的深度信息（m）
        :param is_colormap:         是否在深度图作用可视化RGB彩图
        :param is_filter:           是否在深度图的基础上填充未检测的到的深度像素
        :param is_fps:              是否在rgb图上显示图像帧数
        """

        # Data options
        self.id_camera = id
        self.im_width = rgb_width
        self.im_height = rgb_height
        self.im_fps = rgb_fps
        self.is_depth = is_depth
        self.is_depth_pix = is_depth_pix
        self.is_colormap = is_colormap
        self.is_filter = is_filter
        self.is_fps = is_fps

        self.depth_fps = self.im_fps
        self.align = None
        self.config = None
        self.pipeline = None
        self.start_time = time.time()
        self.frame_count = 0

        # achieve
        self.intrinsics = None
        self.depth_image = np.zeros((self.im_height, self.im_width), dtype=np.float64)
        self.rgb_image = np.zeros((self.im_height, self.im_width), dtype=np.float64)
        self.rgbd_image = np.zeros((self.im_height, self.im_width), dtype=np.float64)
        self.depth_frame = np.zeros((self.im_height, self.im_width), dtype=np.float64)

        self.camera_config()
        self.start_camera()

    def camera_config(self):
        """
        为realsense配置参数
        """
        # 配置realsense参数
        self.align = rs.align(rs.stream.color)            # 创建对齐对象与color流对齐
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.im_width, self.im_height, rs.format.bgr8, self.im_fps)
        self.config.enable_stream(rs.stream.depth, self.im_width,  self.im_height, rs.format.z16, self.depth_fps)
        self.config.enable_stream(rs.stream.infrared, 1, self.im_width,  self.im_height, rs.format.y8, self.depth_fps)
        self.config.enable_stream(rs.stream.infrared, 2, self.im_width,  self.im_height, rs.format.y8, self.depth_fps)

        # 检测当前存在的realsense
        connect_device = []
        for d in rs.context().devices:
            # d.hardware_reset()
            print('Found device: ',
                  d.get_info(rs.camera_info.name), ' ',
                  d.get_info(rs.camera_info.serial_number))
            if d.get_info(rs.camera_info.name).lower() != 'platform camera':
                connect_device.append(d.get_info(rs.camera_info.serial_number))

        # 确认开启相机/streaming流开始
        try:
            self.config.enable_device(connect_device[self.id_camera])
        except Exception as e:
            # 处理异常的代码
            print_and_write(None, f"Camera device not detected, program terminated.", 31)
            sys.exit()
        self.pipeline = rs.pipeline()
        self.pipeline.start(self.config)

        time.sleep(3)         # Give camera some time to load data

    def start_camera(self):
        # per thread
        thread = Thread(target=self.get_data, daemon=True)
        thread.start()

    def __iter__(self):
        # define classes as iterable object
        return self

    def __next__(self):
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            # Stop streaming
            self.pipeline.stop()
            cv2.destroyAllWindows()
            raise StopIteration
        if key & 0xFF == ord('s'):
            cv2.imwrite(f'./core/images/rgb.jpg', self.rgb_image)
            np.save('./core/images/depth.npy', self.depth_frame)
            if self.is_depth:
                cv2.imwrite('./core/images/rgbd.jpg', self.rgbd_image)
            print_and_write(None, f"Save success.", 32)

        return self.rgb_image, self.rgbd_image, self.depth_frame

    def __len__(self):
        return 0  # 1E12 frames = 32 streams at 30 FPS for 30 years

    def pixel2point(self, xy):
        dis = self.aligned_depth_frame.get_distance(xy[0], xy[1])  # 深度单位是m
        return rs.rs2_deproject_pixel_to_point(self.intrinsics, xy, dis)

    def _get_depth_frame(self, depth):
        """
        Get the depth frame's dimensions
        """
        for i in range(self.im_height):
            for j in range(self.im_width):
                self.depth_frame[i][j] = round(depth.get_distance(j, i), 3)

        return self.depth_frame

    def get_data(self):
        """
        获取实时图像（RGB-D）
        """
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = self.pipeline.wait_for_frames()                # 等待获取图像帧，获取颜色和深度的框架集
            aligned_frames = self.align.process(frames)             # 获取对齐帧，将深度框与颜色框对齐
            aligned_color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的的color帧
            aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的的depth帧
            self.aligned_depth_frame = aligned_depth_frame

            # 获取来自每个像素点的深度信息(m)
            if self.is_depth_pix:
                self.depth_frame = self._get_depth_frame(aligned_depth_frame)   # 获取depth的深度信息

            # 深度信息填补过滤器
            if self.is_filter:
                hole_filling = rs.hole_filling_filter()
                aligned_depth_frame = hole_filling.process(aligned_depth_frame)

            # 获取相机参数
            '''
            默认：
                D435i:[ 640x480  p[314.694 247.336]  f[610.18 610.833]   Inverse Brown Conrady [0 0 0 0 0] ]
                D415: [ 640x480  p[314.114 248.192]  f[608.732 608.237]  Inverse Brown Conrady [0 0 0 0 0] ]
            '''
            depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  # 获取深度参数
            self.intrinsics = aligned_color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参
            # print(color_intrin)

            # Convert images to numpy arrays
            self.depth_image = np.asanyarray(aligned_depth_frame.as_frame().get_data())
            self.rgb_image = np.asanyarray(aligned_color_frame.as_frame().get_data())
            if self.rgb_image is None or self.depth_image is None:
                print('Frame acquisition failed.')

            # 彩色深度图
            if self.is_depth:
                if self.is_colormap:
                    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                    self.rgbd_image = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.1), cv2.COLORMAP_JET)
                else:
                    self.rgbd_image = cv2.convertScaleAbs(self.depth_image, alpha=0.03)

            # 实时显示帧数
            if self.is_fps:
                self.frame_count += 1
                if (spend_time := time.time() - self.start_time) != 0:
                    fps = round(self.frame_count / spend_time, 1)
                    cv2.putText(self.rgb_image, f"FPS: {fps}", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    self.frame_count = 0
                    self.start_time = time.time()