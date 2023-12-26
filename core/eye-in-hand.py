'''
# 请参考同目录下的eye-to-hand文件并修改！！！
'''
import cv2 as cv
import numpy as np
import sys
import os
from math import *
from calibrate import Calibration


# Get rotation matrix from euler angles
def euler2rotm(theta):
    R_x = np.array([[1,         0,             0              ],
                    [0,         cos(theta[0]), -sin(theta[0]) ],
                    [0,         sin(theta[0]), cos(theta[0])  ]
                    ])
    R_y = np.array([[cos(theta[1]),    0,      sin(theta[1])  ],
                    [0,                1,      0              ],
                    [-sin(theta[1]),   0,      cos(theta[1])  ]
                    ])
    R_z = np.array([[cos(theta[2]),    -sin(theta[2]),    0],
                    [sin(theta[2]),    cos(theta[2]),     0],
                    [0,                     0,            1]
                    ])
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R


# 用于根据位姿计算变换矩阵
def pose_robot(Ola_pose):
    x = Ola_pose[3]
    y = Ola_pose[4]
    z = Ola_pose[5]
    Tx = Ola_pose[0]
    Ty = Ola_pose[1]
    Tz = Ola_pose[2]
    thetaX = x / 180 * pi
    thetaY = y / 180 * pi
    thetaZ = z / 180 * pi
    R = euler2rotm([thetaX, thetaY, thetaZ])
    t = np.array([[Tx], [Ty], [Tz]])
    # 统一坐标（mm->m）
    RT1 = np.column_stack([R, t * 0.001])
    RT1 = np.row_stack((RT1, np.array([0, 0, 0, 1])))
    return RT1


# 读取标定图片
def read_image():
    img_path = []
    file_dir = './imgSave/'
    for root, dirs, files in os.walk(file_dir, topdown=False):
        for png in files:
            if i[-4:] == '.png':
                img_path.append(file_dir + png)


# 读取tcp位姿
def tcp_pose():
    # 以下部分是加载相关记载的数据(手动记录每一组机械臂的坐标系，可在示教器或者exe端获得)
    txt_path = "./imgSave/calibration.txt"
    t_r_datas = []
    with open(txt_path, "r", encoding="utf-8") as file:  # 打开文件
        while True:
            t_r_data = []
            data = file.readline()  # 包括换行符
            data = data[:-1]  # 去掉换行符
            if data:
                for i in range(6):
                    t_r_data.append(float(data.split(' ')[i]))
                t_r_datas.append(t_r_data)
            else:
                break
    file.close()
    # print(t_r_datas)

    return t_r_datas


all_image = []
all_obj_points = []
all_points = []
all_tcp = []
num_images = 16

RT_gripper2base = []
R_gripper2base = []
T_gripper2base = []

RT_target2cam = []
R_target2cam = []
T_target2cam = []

if __name__ == '__main__':
    cal = Calibration(size=(9, 6), size_step=2, default=False)  # size_step由实际距离决定（cm），可由最后平均误差反验证
    all_tcp = tcp_pose()

    for i in range(1, num_images + 1):
        img = cv.imread('./imgSave/' + str(i) + '.png')
        if img is None:
            print('Failed to read {}.jpg.'.format(i))
            sys.exit()
        all_image.append(img)
        _, obj_points, points = cal.chessboard(img)
        all_obj_points.append(obj_points)
        all_points.append(points)

    camera_Intrinsics, distCoeffs, _, _ = cal.calibrate(all_obj_points, all_points)

    for i in range(len(all_image)):
        # 计算两个坐标系之间的旋转向量及旋转矩阵，计算target2cam
        tvec, rvec, rvec_transport = cal.PnPAndRansa(all_obj_points[i], all_points[i])
        # 统一坐标（cm->m）
        R_target2cam.append(rvec_transport)
        T_target2cam.append(tvec * 0.01)
        tmp1 = np.column_stack([rvec_transport, tvec * 0.01])
        tmp1 = np.row_stack((tmp1, np.array([0, 0, 0, 1])))
        RT_target2cam.append(tmp1)

        # 用于根据欧拉角计算旋转矩阵，统一计算维度，计算gripper2base，以及计算base2gripper
        base_H_gri = pose_robot(all_tcp[i])
        RT_gripper2base.append(base_H_gri)
        R_gripper2base.append(base_H_gri[:3, :3])
        T_gripper2base.append(base_H_gri[:3, 3].reshape((3, 1)))

    # 手眼标定
    # 旋转矩阵的转置矩阵 = 旋转矩阵的逆矩阵
    # 数据应与实际距离一致
    R_cam2gripper, T_cam2gripper = cv.calibrateHandEye(R_gripper2base, T_gripper2base, R_target2cam, T_target2cam)
    RT_cam2gripper = np.column_stack([R_cam2gripper, T_cam2gripper])
    RT_cam2gripper = np.row_stack((RT_cam2gripper, np.array([0, 0, 0, 1])))
    print("RT_cam2gripper的坐标(即相机当前基于robot基坐标系的空间位姿):")
    print(RT_cam2gripper)

    # 验证，basel_H_cal = base_H_tool * tool_H_cam * cam_H_cal
    # 即 RT_cal2base = RT_gripper2base * RT_cam2gripper * RT_target2cam
    # 以上矩阵大小全为 4*4
    error_x = []
    error_y = []
    error_depth = []
    for i in range(len(all_image)):
        RT_cal2base = np.dot(np.dot(RT_gripper2base[i], RT_cam2gripper), RT_target2cam[i])

        # 计算每个方向的标定的标准差
        error_x.append(RT_cal2base[0, 3])
        error_y.append(RT_cal2base[1, 3])
        error_depth.append(RT_cal2base[2, 3])

    # 验证标定的方差和标准差，是否需要重新标定
    dev_depth = np.var(error_depth)
    std_depth = np.std(error_depth, ddof=1)
    std_x = np.std(error_x, ddof=1)
    std_y = np.std(error_y, ddof=1)
    print("x标准差为:±%f" % (std_x * 1000))
    print("y标准差为:±%f" % (std_y * 1000))
    print("z⽅差为：%f" % dev_depth)
    if (0 < std_depth * 1000 < 1):
        print("z标定差为：±%fmm" % (std_depth * 1000))
        print("深度z误差小于±1mm，标定成功")
        # with open(write_RT_cal2base_txt, "w+", encoding="utf-8") as f2:
        #     f2.write(str(RT_cam2tool))
        #     f2.close()
        # print("--数据已保存--")
    else:
        print("z标定误差为：±%fmm" % (std_depth * 1000))
        print("z深度误差较大，建议重新标定")
    print("各方向平均误差为：%fmm" % ((std_x + std_y + std_depth) / 3.0 * 1000))
