import cv2
import numpy as np
from realsense2 import Realsense2
import utils

camera = Realsense2(0, 640, 480, 60, is_depth=True, is_depth_pix=True, is_colormap=True, is_filter=False, is_imshow=True)
while True:
    cap_img, cap_img_d, cap_depth = camera.get_data()


workspace_limits = np.asarray([[0.2, 0.6], [-0.2, 0.2], [-0.15, 1]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
heightmap_resolution = 0.001

# A->B
# cam_intrinsics = np.array([[658.8982692,    0.,         309.51533836],
#  [  0.,         664.8721658,  207.1107828 ],
#  [  0. ,          0.     ,      1.        ]])
# # B
# RT_cam2base = np.array([[-1.04875453e-02,  7.66114691e-01, -6.42618309e-01,  6.88924258e-01],
#  [ 9.99801673e-01,  1.89147120e-02 , 6.23289653e-03,  6.63602258e-04],
#  [ 1.69300538e-02 ,-6.42425492e-01, -7.66161119e-01 , 4.12990706e-01],
#  [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]])
# C->D
cam_intrinsics = np.array([[610.18,   0.,        314.694],
                           [0.,       610.833,   247.336],
                           [0.,       0.,        1.     ]])
# D
RT_cam2base = np.array([[-0.11072305 , 0.90152823, -0.41831477,  0.68252624],
 [ 0.99368857,  0.10803762, -0.03018126,  0.01566926],
 [ 0.01798447, -0.41901637, -0.90780055 , 0.4240398 ],
 [ 0.    ,      0.    ,      0.    ,      1.        ]])

depth = np.load('./depth.npy', allow_pickle=True)
np.savetxt(r'depth.txt', depth, fmt='%f', delimiter=',')
obj_pix = [187, 279]                                             # [h, w]
cam_pose, obj_pose = utils.get_object(depth, cam_intrinsics, RT_cam2base, obj_pix)
obj_offset = [0, 0, 0]                                           # constant offset(mm)，raise 30mm in Z direction
aaa = np.add(obj_pose, obj_offset)
print(aaa)


# cap_img = cv2.imread('./rgb.jpg')
# cap_img_d = cv2.imread('./rgb_d.jpg', cv2.IMREAD_UNCHANGED)
# color_heightmap, depth_heightmap = utils.get_heightmap(cap_img, depth, cam_intrinsics, RT_cam2base, workspace_limits, heightmap_resolution)
# cv2.namedWindow("color_heightmap", cv2.WINDOW_NORMAL)
# cv2.namedWindow("depth_heightmap", cv2.WINDOW_NORMAL)
# cv2.imshow("color_heightmap", color_heightmap)
# cv2.imshow("depth_heightmap", depth_heightmap)
# cv2.imwrite("./color_heightmap.jpg", color_heightmap)
# cv2.waitKey(0)
# input()


