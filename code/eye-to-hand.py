'''
# Current time is 2022/07/18.
# 参考代码：
# https://blog.csdn.net/weixin_42203839/article/details/103882739
# https://blog.csdn.net/weixin_38258767/article/details/104956154
# 备注：
# RT_gripper2base = base_H_gripper  两者为同一变换矩阵，意义：机械臂坐标相对于基坐标
# realsense的参考矩阵在光心处，相机右侧花了黑线
# 根据最后计算得出的拍摄的标定板x，y，z标定误差来验证数据的稳定性
# 误差大于2mm的原因：1.标定板不平，可在标定前用深度相机查看整幅标定本的深度是否一致
                  2.计算单位一致
'''
import cv2 as cv
import numpy as np
import sys
import os
from math import *
from calibrate import Calibration


# 用于根据欧拉角计算旋转矩阵
def abRxyz(x, y, z):
    Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
    Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
    Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
    R = Rz@Ry@Rx
    return R

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

# 用于根据位姿计算变换矩阵（4×4）
def pose_robot(Ola_pose):
    Tx = Ola_pose[0]
    Ty = Ola_pose[1]
    Tz = Ola_pose[2]
    x = Ola_pose[3]
    y = Ola_pose[4]
    z = Ola_pose[5]
    thetaX = x / 180 * pi
    thetaY = y / 180 * pi
    thetaZ = z / 180 * pi
    #R = abRxyz(thetaX, thetaY, thetaZ)
    R = euler2rotm([thetaX, thetaY, thetaZ])
    t = np.array([[Tx], [Ty], [Tz]])
    # 统一坐标（mm->m）
    RT = np.column_stack([R, t * 0.001])
    RT = np.row_stack((RT, np.array([0, 0, 0, 1])))
    return RT

def read_image():
    img_path = []
    file_dir = './imgSave/'
    for root, dirs, files in os.walk(file_dir, topdown=False):
        for png in files:
            if i[-4:] == '.png':
                img_path.append(file_dir + png)

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

RT_base2gripper = []
R_base2gripper = []
T_base2gripper = []

RT_target2cam = []
R_target2cam = []
T_target2cam = []


if __name__ == '__main__':
    cal = Calibration(size=(9, 6), size_step=0.7, default=True)      # size_step由实际距离决定（cm），可由最后平均误差反验证
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
        tvec, rvec, rvec_transport = cal.pnpandransac(all_obj_points[i], all_points[i], is_Ransac=True,
                                                     method=cv.SOLVEPNP_ITERATIVE)
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

        tmp = np.linalg.inv(base_H_gri)
        RT_base2gripper.append(tmp)
        R_base2gripper.append(tmp[:3, :3])
        T_base2gripper.append(tmp[:3, 3].reshape((3, 1)))

    # 手眼标定
    # 旋转矩阵的转置矩阵 = 旋转矩阵的逆矩阵
    # 结论数据应与实际距离基本一致
    # 注意：calibrateHandEye计算AX=XB的方法（5）种 \
    # 可参考：https://docs.opencv.org/4.4.0/d9/d0c/group__calib3d.html#gad10a5ef12ee3499a0774c7904a801b99
    R_cam2base, T_cam2base = cv.calibrateHandEye(R_base2gripper, T_base2gripper, R_target2cam, T_target2cam,
                                                 method=cv.CALIB_HAND_EYE_DANIILIDIS)
    RT_cam2base = np.column_stack([R_cam2base, T_cam2base])
    RT_cam2base = np.row_stack((RT_cam2base, np.array([0, 0, 0, 1])))
    print("RT_cam2base的坐标(即相机当前基于robot基坐标系的空间位姿):")
    print(RT_cam2base)

    # 验证，tool_H_cal = tool_H_base * base_H_cam * cam_H_cal
    # 即 RT_cal2tool = RT_base2gripper * RT_cam2base * RT_target2cam
    # 以上矩阵大小全为 4*4
    error_x = []
    error_y = []
    error_depth = []
    for i in range(len(all_image)):
        RT_cal2tool = np.dot(np.dot(RT_base2gripper[i], RT_cam2base), RT_target2cam[i])

        # 计算每个方向的标定的标准差
        error_x.append(RT_cal2tool[0, 3])
        error_y.append(RT_cal2tool[1, 3])
        error_depth.append(RT_cal2tool[2, 3])

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
    print("各方向平均误差为：%fmm" % ((std_x+std_y+std_depth)/3.0*1000))

    # Undistort
    # undistort_image = cal.undistort(all_image[2])
    # cv2.imshow("undistort_image",undistort_image)
    # cv2.waitKey(0)