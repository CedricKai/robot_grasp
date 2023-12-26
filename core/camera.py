import argparse
from core.iter_capture import LoadStreams
from core.utils import *
from core.realsense2 import *


def create_parser():
    print_and_write(None, "------------------------config------------------------", 33)
    parser = argparse.ArgumentParser(description='Hand-Eye Calibration')                                      # config
    parser.add_argument('--source', type=str, default='1,2', help='camera index')                  # file/folder, 0 for webcam
    parser.add_argument('--realsense', action='store_true', help='whether to use realsense')
    parser.add_argument('--depth_data', action='store_true', help='whether to obtain depth information')
    parser.add_argument('--dis_img', action='store_true', help='whether to display images')

    return parser


if __name__ == "__main__":
    # input para config
    parser = create_parser()
    args = parser.parse_args()
    print_and_write(None, f"args: {args}", 33)

    # set camera
    if args.realsense:
        # 注意开启部分功能（获取深度图像素的深度信息）会导致fps下降
        realcam = Realsense2(0, 640, 480, 60, is_depth_pix=args.depth_data)
        for img in realcam:
            rgb_img, rgbd_img, img_depth = img          # rgb图，深度图，深度信息
            # Show images
            if args.dis_img:
                cv2.namedWindow('RealSense_rgb', cv2.WINDOW_NORMAL)
                cv2.imshow('RealSense_rgb', rgb_img)
                cv2.namedWindow('RealSense_rgbd', cv2.WINDOW_NORMAL)
                cv2.imshow('RealSense_rgbd', rgbd_img)
    else:
        # 普通相机，可并行开启，标定部分代码有待补全，可参考上述realsense代码
        cams = eval(args.source) if isinstance(eval(args.source), tuple) else tuple(args.source)  # signal:(1), double:(1,2)
        for img, fps in (dataset := LoadStreams(*cams)):
            for i in range(len(img)):
                cv2.imshow(f"video_{i + 1}", img[i])
