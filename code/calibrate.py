import cv2 as cv
import numpy as np


class Calibration(object):
    def __init__(self, width: int = 640, height: int = 480, size: tuple = (9, 6), size_step: float = 1.0,
                 default: bool = False):
        self.w = width
        self.h = height
        self.size = size
        self.size_step = size_step
        self.is_default = default
        self.camera_Intrinsics = None
        self.distCoeffs = None
        self.rvecs = None
        self.tvecs = None

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

    def chessboard(self, image):
        """
        image: 原始图像

        输出生成棋盘格内角点的二维像素坐标(pix)以及生成棋盘格内角点的三维坐标(cm)
        """
        # 生成棋盘格内角点的二维像素坐标(pix)
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)                           # 转为灰度图像
        board_size = self.size                                                 # 定义数目尺寸
        _, points = cv.findChessboardCorners(gray, board_size)                 # 检测角点
        _, points = cv.find4QuadCornerSubpix(gray, points, (5, 5))             # 细化角点坐标
        image = cv.drawChessboardCorners(image, board_size, points, True)      # 绘制角点检测结果

        # 显示角点图
        # cv2.namedWindow('123',cv2.WINDOW_NORMAL)
        # cv2.imshow('123', image)
        # cv2.waitKey(0)

        # 生成棋盘格内角点的三维坐标(cm)
        obj_points = np.zeros((self.size[0]*self.size[1], 3), np.float32)
        obj_points[:, :2] = np.mgrid[0:self.size[0]*self.size_step:self.size_step,
                                     0:self.size[1]*self.size_step:self.size_step].T.reshape(-1, 2)
        obj_points = np.reshape(obj_points, (self.size[0]*self.size[1], 1, 3))

        return image, obj_points, points

    def calibrate(self, all_obj_points: list, all_points: list):
        """
        all_obj_points:世界三维坐标系 序列
        all_points：二维像素角点坐标系 序列

        计算相机内参矩阵和畸变系数
        """

        _, self.camera_Intrinsics, self.distCoeffs, self.rvecs, self.tvecs = \
            cv.calibrateCamera(all_obj_points, all_points, (self.w, self.h), None, None)

        # D435i: [640x480  p[314.694 247.336] f[610.18 610.833]   Inverse Brown Conrady[0 0 0 0 0]]
        if self.is_default:
            self.camera_Intrinsics = np.array([[610.18, 0., 314.694], [0., 610.833, 247.336], [0., 0., 1]])
            self.distCoeffs = np.array([[0., 0., 0., 0., 0.]])

        print('内参矩阵为：\n{}'.format(self.camera_Intrinsics))
        print('畸变系数为：\n{}'.format(self.distCoeffs))

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


# =====================Debug======================================
# a=calibration()
# all_obj_points = []
# all_points = []
# for i in range(1, 17):
#     # 读取图像
#     image = cv.imread('./Cal_images/{}.jpg'.format(i))
#     if image is None:
#         print('Failed to read {}.jpg.'.format(i))
#     image,p,m=a.chessboard(image)
#     # 计算三维坐标
#     all_obj_points.append(m)
#     # 计算二维坐标
#     all_points.append(p)
#
# a.calibrate(all_obj_points,all_points)
