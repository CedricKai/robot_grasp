import numpy as np
import pyrealsense2 as rs
import os
import sys
import cv2
import time
from pathlib import Path

FILE = Path(__file__).resolve()       # __file__:当前路径 .resolve():获取绝对路径
ROOT = FILE.parents[0]                # .parents():路径的父目录
if str(ROOT) not in sys.path:         # sys.path是python的搜索模块的路径集，是一个list
    sys.path.append(str(ROOT))        # sys.path.append():添加相关路径，但在退出python环境后自己添加的路径就会消失
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # Path.cwd():返回当前工作目录 .relpath()得到相对路径


class Realsense2(object):
    def __init__(self, id=0, rgb_width=640, rgb_height=480, rgb_fps=60,
                 is_depth=False, is_depth_pix=False, is_colormap=False, is_filter=False, is_imshow=False):
        """
        id:          打开指定相机
        rgb_width：  RGB的宽度(pix)
        rgb_height： RGB的高度
        rgb_fps：    RGB的fps
        d_width：    深度图的宽度
        d_height：   深度图的高度
        d_fps：      深度图的fps
        is_depth：   是否需要深度图
        is_depth_pix：是否计算每个像素点的深度信息（m）
        is_colormap：是否在深度图作用可视化RGB彩图
        is_filter：  是否在深度图的基础上填充未检测的到的深度像素
        is_imshow：  是否窗口显示化

        输出指定realsense的RGB彩图和深度图
        """
        # Data options
        self.id_camera = id
        self.im_width = rgb_width
        self.im_height = rgb_height
        self.im_fps = rgb_fps
        self.depth_fps = 30
        self.is_depth = is_depth
        self.is_depth_pix = is_depth_pix
        self.is_colormap = is_colormap
        self.is_filter = is_filter
        self.is_imshow = is_imshow
        self.align = None
        self.config = None
        self.pipeline = None
        self.intrinsics = None
        self.images = None
        self.images_d = None
        self.depth_frame = np.zeros((self.im_height, self.im_width), dtype=np.float)
        self.camera_config()

    def _get_depth_frame(self, depth):
        """
        Get the depth frame's dimensions
        """
        for i in range(self.im_height):
            for j in range(self.im_width):
                self.depth_frame[i][j] = round(depth.get_distance(j, i), 3)

        return self.depth_frame

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
            print('Found device: ',
                  d.get_info(rs.camera_info.name), ' ',
                  d.get_info(rs.camera_info.serial_number))
            if d.get_info(rs.camera_info.name).lower() != 'platform camera':
                connect_device.append(d.get_info(rs.camera_info.serial_number))

        # 确认开启相机/streaming流开始
        self.pipeline = rs.pipeline()
        self.config.enable_device(connect_device[self.id_camera])
        self.pipeline.start(self.config)

        time.sleep(1)         # Give camera some time to load data

    def get_data(self):
        """
        获取实时图像（RGB-D）
        """
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()                # 等待获取图像帧，获取颜色和深度的框架集
        aligned_frames = self.align.process(frames)             # 获取对齐帧，将深度框与颜色框对齐
        aligned_color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的的color帧
        aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的的depth帧

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
        color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参
        self.intrinsics = color_intrin
        # print(color_intrin)
        # Convert images to numpy arrays
        depth_image = np.asanyarray(aligned_depth_frame.as_frame().get_data())
        color_image = np.asanyarray(aligned_color_frame.as_frame().get_data())

        if color_image is None or depth_image is None:
            print('Failed to open camera.')

        if self.is_depth:
            if self.is_colormap:
                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                self.images_d = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.1), cv2.COLORMAP_JET)
            else:
                self.images_d = cv2.convertScaleAbs(depth_image, alpha=0.03)

        self.images = color_image

        # Show images
        if self.is_imshow:
            cv2.namedWindow('RealSense', cv2.WINDOW_NORMAL)
            cv2.imshow('RealSense', self.images)
            if self.is_depth:
                cv2.namedWindow('RealSense_d', cv2.WINDOW_NORMAL)
                cv2.imshow('RealSense_d', self.images_d)

            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
            if key & 0xFF == ord('s'):
                cv2.imwrite('./rgb.jpg', self.images)
                np.save('./depth.npy', self.depth_frame)
                if self.is_depth:
                    cv2.imwrite('./rgb_d.jpg', self.images_d)
                print("save success.")

        return self.images, self.images_d, self.depth_frame

    def stop(self):
        """
        terminate stream
        """
        # Stop streaming
        self.pipeline.stop()
        cv2.destroyAllWindows()
