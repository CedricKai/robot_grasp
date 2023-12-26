import argparse
import sys
import time
from tqdm import tqdm
from core.iter_capture import LoadStreams
from core.utils import *
from core.realsense2 import *
from core.wrap_xarm import xarm6
from core.calibrate import Calibration


def create_parser():
    print_and_write(None, "------------------------config------------------------", 33)
    parser = argparse.ArgumentParser(description='Hand Eye Calibration')                                      # config
    parser.add_argument('--source', type=str, default='1,2', help='camera index')
    parser.add_argument('--dis_img', action='store_true', help='whether to display images')
    parser.add_argument('--robot_ip', type=str, default='192.168.31.100', help='robot ip')
    parser.add_argument('--depth_data', action='store_true', help='whether to obtain depth information')
    parser.add_argument('--fps', action='store_true', help='whether to display fps')
    parser.add_argument('--num_img', type=int, default=16, help='number of calibration boards')

    return parser


def save_path():
    # 在当前目录下创建文件夹
    folder_name = "Folder_" + time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
    pro_dir = os.path.join(os.path.dirname(__file__), '..')
    dir_path = f'{pro_dir}/output/imgSave/{folder_name}'
    if not os.path.exists(dir_path):
        os.mkdir(dir_path)
        print_and_write(None, f"Generate folder: {dir_path}", 32)
    return dir_path


def save_img(display, index, img, save_path):
    if display:
        cv2.namedWindow(f'RealSense_rgb', cv2.WINDOW_NORMAL)
        cv2.imshow(f'RealSense_rgb', rgb_img)
    print_and_write(None, f"Save the {index+1} image.", 32)
    cv2.imwrite(os.path.join(save_path, f"img_{index+1}.jpg"), img)


def capture(obj, flag):
    while not flag.is_set():
        rgb_img, _, _ = next(obj)
        cv2.namedWindow(f'realsense', cv2.WINDOW_NORMAL)
        cv2.imshow(f'realsense', rgb_img)


def caution(obj):
    #
    stop_flag = threading.Event()
    loop = Thread(target=capture, args=([obj, stop_flag]), daemon=True)
    loop.start()

    print_and_write(None,
                    f"1. Please keep away from the robot arm to avoid personal injury or equipment damage.\n"
                         f"2. Make sure to do a safety assessment before moving to prevent collisions.\n"
                         f"3. Protect the arm before unlocking the joint.\n")
    print_and_write(None, f"Place the calibration board horizontally in the center of the camera's field of view.\n"
                          f"Please place the paper 8~10cm below the base of the robotic arm.", 32, 1)
    y_n = input("Enter 'y' to continue: ")
    if y_n == 'y':
        stop_flag.set()
    else:
        print_and_write(None, f"Calibration termination." ,31)
        sys.exit()


if __name__ == "__main__":
    # input para config
    parser = create_parser()
    args = parser.parse_args()
    print_and_write(None, f"args: {args}", 33)

    # paras
    dir_path = save_path()                                          # 保存图片的路径
    image_num = args.num_img if args.num_img <= 20 else 16          # 所需图片个数<=20
    tcp_file_path = os.path.join(dir_path, 'calibration.txt')       # 保存robot信息

    # robot
    robot = xarm6(args.robot_ip)

    # camera
    # 注意开启部分功能（获取深度图像素的深度信息）会导致fps下降
    realcam = Realsense2(0, 640, 480, 60, is_depth_pix=args.depth_data, is_fps=args.fps)

    # caution
    caution(obj=realcam)

    # Loop
    for index in tqdm(range(image_num), desc='Progress: '):
        # random action
        robot.rand_action()

        # get state
        rgb_img, rgbd_img, img_depth = next(realcam)                     # rgb图，深度图，深度信息
        save_img(args.dis_img, index, rgb_img, dir_path)                 # 显示并保存rgb图像

        # save robot tcp
        tcp_pose = robot.get_current_pose()
        with open(tcp_file_path, "a+", encoding="utf-8") as file:
            for data in tcp_pose:
                file.write(str(data) + ' ')
            file.write('\n')

        # next
        time.sleep(1)

    print_and_write(None, f"Collection completed, Save img path: {dir_path} ", 32, 1)
    robot.move_init_pose()     # 采集完毕
    file.close()

    # calibration   size_step由实际距离决定（cm），可由最后平均误差反验证
    # image_num =16
    # dir_path = 'S:\\pycharm\\project\\robot_arm\\output\\imgSave\\test\\'
    Calibration(size=(9, 6), size_step=2.6, img_num=image_num, cal_path=dir_path)()

    print_and_write(None, f"Calibration END.", 33, 1)
