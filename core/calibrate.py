import cv2 as cv
import numpy as np
import os
import sys
from core.utils import *


class Calibration(object):
    def __init__(self, width: int = 640, height: int = 480, size: tuple = (9, 6), size_step: float = 1.0,
                 img_num=16, cal_path=None):
        self.w = width
        self.h = height
        self.size = size
        self.size_step = size_step
        self.img_num = img_num
        self.cal_path = cal_path

        self.camera_Intrinsics = None
        self.distCoeffs = None
        self.rvecs = None
        self.tvecs = None
        self.RT_cam2gripper = None           # core: AX=XB

        obj_atr = ['all_tcp', 'all_image', 'all_real_points', 'all_pix_points',
                   'RT_gripper2base', 'R_gripper2base', 'T_gripper2base',
                   'RT_target2cam', 'R_target2cam', 'T_target2cam']
        for atr in obj_atr:
            setattr(self, atr, list())

    def __call__(self):
        """Calls the run() method with given arguments to perform object detection."""
        return self.run()

    @staticmethod
    def homogeneous(MT, flag=0):
        """
        MT: 2维矩阵

        齐次坐标与齐次坐标相互转换
        """
        assert MT.ndim == 2, '矩阵维度应为2维'
        if flag == 0:
            # 非齐次坐标转齐次坐标
            _MT = cv.convertPointsToHomogeneous(MT)
            print('非齐次坐标：{:<20}转为齐次坐标：{}'.format(str(MT), str(_MT)))
        else:
            # 齐次坐标转非齐次坐标
            _MT = cv.convertPointsFromHomogeneous(MT)
            print('齐次坐标：{:<22}转为非齐次坐标：{}'.format(str(MT), str(_MT)))

        return _MT

    @staticmethod
    def Cartesian2Homogeneous(Ola_pose):
        tx, ty, tz, rx, ry, rz = Ola_pose
        R = euler2rotm([rx / 180 * pi, ry / 180 * pi, rz / 180 * pi])      # 欧拉角—>旋转矩阵
        T = np.array([[tx], [ty], [tz]])

        # 统一坐标（mm->m）
        return np.row_stack((np.column_stack([R, T * 0.001]), np.array([0, 0, 0, 1])))

    def chessboard(self, index, image):
        """
        image: 原始图像

        输出生成棋盘格内角点的二维像素坐标(pix)以及生成棋盘格内角点的三维坐标(cm)
        """
        # 生成棋盘格内角点的二维像素坐标(pix)
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)                           # 转为灰度图像
        board_size = self.size                                                 # 定义数目尺寸
        _, points = cv.findChessboardCorners(gray, board_size)                 # 检测角点
        try:
            _, points = cv.find4QuadCornerSubpix(gray, points, (5, 5))             # 细化角点坐标
        except Exception as e:
            # 处理异常的代码
            print_and_write(None, f"Partial calibration board image corner calibration failed, may be img_{index+1}.jpg, please recalibrate.", 31,1)
            sys.exit()
        image = cv.drawChessboardCorners(image, board_size, points, True)      # 绘制角点检测结果

        # 显示角点图
        # cv2.namedWindow('123',cv2.WINDOW_NORMAL)
        # cv2.imshow('123', image)
        # cv2.waitKey(0)
        # save
        cv2.imwrite(os.path.join(self.cal_path, f"corner_{index + 1}.jpg"), image)

        # 生成棋盘格内角点的三维坐标(cm)
        obj_points = np.zeros((self.size[0]*self.size[1], 3), np.float32)
        obj_points[:, :2] = np.mgrid[0:self.size[0]*self.size_step:self.size_step,
                                     0:self.size[1]*self.size_step:self.size_step].T.reshape(-1, 2)
        obj_points = np.reshape(obj_points, (self.size[0]*self.size[1], 1, 3))

        return image, obj_points, points

    def calibrate(self, all_obj_points: list=None, all_points: list=None, default=False):
        """
        all_obj_points:世界三维坐标系 序列
        all_points：二维像素角点坐标系 序列

        计算相机内参矩阵和畸变系数
        """
        if all_obj_points is not None and all_points is not None:
            pass
        else:
            all_obj_points = self.all_real_points
            all_points = self.all_pix_points

        assert len(all_obj_points) == len(all_points) == self.img_num

        _, self.camera_Intrinsics, self.distCoeffs, self.rvecs, self.tvecs = \
            cv.calibrateCamera(all_obj_points, all_points, (self.w, self.h), None, None)

        # D435i: [640x480  p[314.694 247.336] f[610.18 610.833]   Inverse Brown Conrady[0 0 0 0 0]]
        if default:
            self.camera_Intrinsics = np.array([[610.18, 0., 314.694], [0., 610.833, 247.336], [0., 0., 1]])
            self.distCoeffs = np.array([[0., 0., 0., 0., 0.]])

        print(f'Camera Intrinsics: {self.camera_Intrinsics}')
        print(f'Distortion Coefficient: {self.distCoeffs}')

        # # 此结果在后面ProjectPoints.py函数中有使用到，此处先进行计算
        # print('旋转向量为：\n{}'.format(rvecs))
        # print('平移向量为：\n{}'.format(tvecs))

        return self.camera_Intrinsics, self.distCoeffs, self.rvecs, self.tvecs

    def undistort(self, image, flag=0):
        """
        image： 原始图像

        畸变矫正后的图像
        """
        if flag == 0:
            # 校正图像（使用cv.initUndistortRectifyMap()函数和cv.remap()函数）
            map1, map2 = cv.initUndistortRectifyMap(self.camera_Intrinsics, self.distCoeffs, None, None,
                                                    (self.w, self.h), 5)
            result = cv.remap(image, map1, map2, cv.INTER_LINEAR)
        else:
            # 校正图像（使用cv.undistort()函数）
            result = cv.undistort(image, self.camera_Intrinsics, self.distCoeffs, newCameraMatrix=None)

        return result

    def projectpoints(self, obj_points, points, rvecs, tvecs):
        """
        obj_points: 世界三维坐标系
        points： 二维像素角点坐标系
        rvecs: 旋转向量
        tvecs：平移向量

        计算角点误差（mm）
        """
        # 根据三维坐标和相机与世界坐标系时间的关系估计内角点像素坐标
        points1, _ = cv.projectPoints(obj_points, rvecs, tvecs, self.camera_Intrinsics, self.distCoeffs)

        # 计算图像中内角点的真实坐标误差
        error = 0
        for j in range(len(points)):
            error += np.sqrt(
                np.power((points[j][0][0] - points1[j][0][0]), 2) + np.power((points[j][0][1] - points1[j][0][1]), 2))
        print('图像中内角点的真实坐标误差为：{}'.format(round(error / len(points), 6)))

    def pnpandransac(self, obj_points, points, is_Ransac, method):
        """
        obj_points: 世界三维坐标系
        points：    二维像素角点坐标系
        is_Ransac： 是否结合Ransac计算
        method：    计算方法参考https://docs.opencv.org/4.4.0/d9/d0c/group__calib3d.html#gad10a5ef12ee3499a0774c7904a801b99

        计算单张图片的旋转向量和平移向量
        """
        if is_Ransac:
            # 用PnP+Ransac算法计算旋转向量和平移向量
            _, rvec, tvec, inliers = cv.solvePnPRansac(obj_points, points, self.camera_Intrinsics, self.distCoeffs,
                                                       flags=method)
        else:
            # 用PnP算法计算旋转和平移向量
            _, rvec, tvec = cv.solvePnP(obj_points, points, self.camera_Intrinsics, self.distCoeffs)

        # 旋转向量转换为旋转矩阵
        rvec_transport, _ = cv.Rodrigues(rvec)

        # 输出结果
        # print('世界坐标系变换到相机坐标系的旋转向量：\n', rvec)
        # print('对应旋转矩阵为：\n', rvec_transport)
        # print('世界坐标系变换到相机坐标系的平移向量：\n', tvec)

        return tvec, rvec, rvec_transport


    # 读取tcp位姿
    def tcp_pose(self):
        # 以下部分是加载相关记载的数据(手动记录每一组机械臂的坐标系，可在示教器或者exe端获得)
        txt_path = os.path.join(self.cal_path, "calibration.txt")
        with open(txt_path, "r", encoding="utf-8") as file:  # 打开文件
            while True:
                r_data = []
                data = file.readline()  # 包括换行符
                data = data[:-1]  # 去掉换行符
                if data:
                    for i in range(6):
                        r_data.append(float(data.split(' ')[i]))
                    self.all_tcp.append(r_data)
                else:
                    break
        file.close()
        # print(t_r_datas)

    def get_pix_and_points(self):
        self.tcp_pose()
        for i in range(self.img_num):
            img = cv.imread(os.path.join(self.cal_path, f"img_{i + 1}.jpg"))
            if img is None:
                print(f'Failed to read {i + 1}.jpg.')
                sys.exit()
            self.all_image.append(img)
            _, obj_points, points = self.chessboard(i, img)
            self.all_real_points.append(obj_points)
            self.all_pix_points.append(points)

    def get_relative_pose(self):
        for i in range(self.img_num):
            # 计算两个坐标系之间的旋转向量及旋转矩阵,计算target2cam,统一坐标（cm->m）
            tvec, rvec, rvec_transport = self.pnpandransac(self.all_real_points[i], self.all_pix_points[i],
                                                           is_Ransac=True, method=cv.SOLVEPNP_ITERATIVE)
            self.R_target2cam.append(rvec_transport)
            self.T_target2cam.append(tvec * 0.01)
            self.RT_target2cam.append(np.row_stack((np.column_stack([rvec_transport, tvec * 0.01]), np.array([0, 0, 0, 1]))))

            # 用于根据欧拉角计算旋转矩阵，统一计算维度，计算gripper2base，以及计算base2gripper
            base_H_gri = self.Cartesian2Homogeneous(self.all_tcp[i])          # 计算gripper2base
            self.R_gripper2base.append(base_H_gri[:3, :3])
            self.T_gripper2base.append(base_H_gri[:3, 3].reshape((3, 1)))
            self.RT_gripper2base.append(base_H_gri)

    def get_cam2gri_pose(self):
        # 手眼标定
        # 旋转矩阵的转置矩阵 = 旋转矩阵的逆矩阵
        # 数据应与实际距离一致
        R_cam2gripper, T_cam2gripper = cv.calibrateHandEye(self.R_gripper2base, self.T_gripper2base, self.R_target2cam, self.T_target2cam)
        self.RT_cam2gripper = np.row_stack((np.column_stack([R_cam2gripper, T_cam2gripper]), np.array([0, 0, 0, 1])))
        print(f"RT_cam2gripper: {self.RT_cam2gripper}")

    def test_and_verify(self):
        # 验证，basel_H_cal = base_H_tool * tool_H_cam * cam_H_cal
        # 即 RT_cal2base = RT_gripper2base * RT_cam2gripper * RT_target2cam
        # 以上矩阵大小全为 4*4
        RT_cal2base = np.matmul(np.matmul(self.RT_gripper2base, self.RT_cam2gripper), self.RT_target2cam)   # num*4*4
        T_cal2base = np.squeeze(RT_cal2base[:, :3, 3]) * 1000                                       # num*3*1 -> num*3
        # print(T_cal2base)
        std_x, std_y, std_z = [round(s, 3) for s in np.std(T_cal2base, axis=0)]
        mean_std = round((std_x + std_y + std_z) / 3.0, 3)
        dev_z = np.var(T_cal2base[:, 2])
        print_and_write(None,
                        f"x_std: {std_x}, y_std: {std_y}, z_std: {std_z}, z_dec: {dev_z}, "
                        f"mean_std: {mean_std}",
                        32)

        if 0 < mean_std < 2:
            print_and_write(None, f"mean_std: {mean_std}, average error is small, within ±5mm, calibration successful", 32, 1)

        else:
            print_and_write(None, f"mean_std: {mean_std}, average error is big, beyond ±5mm, recommend to recalibrate", 31, 1)

        # save

        np.save(f'{self.cal_path}/data.npy', vars(self))
        print_and_write(None, f"--Data has already save.--", 32, 1)

    def run(self):
        # 整个标定流程
        self.get_pix_and_points()                                         # 获取图片的角点坐标和对应的三维坐标
        self.calibrate(default=True)                                      # 获取相机的参数
        self.get_relative_pose()                                          # 获取相对坐标位姿参数
        self.get_cam2gri_pose()                                           # 获取手眼矩阵
        self.test_and_verify()                                            # 测试并验证标定精度

