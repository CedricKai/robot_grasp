import os
import sys
import cv2
from xArm.xarm.wrapper import XArmAPI
from realsense2 import Realsense2
from pathlib import Path

FILE = Path(__file__).resolve()       # __file__:当前路径 .resolve():获取绝对路径
ROOT = FILE.parents[0]                # .parents():路径的父目录
if str(ROOT) not in sys.path:         # sys.path是python的搜索模块的路径集，是一个list
    sys.path.append(str(ROOT))        # sys.path.append():添加相关路径，但在退出python环境后自己添加的路径就会消失
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # Path.cwd():返回当前工作目录 .relpath()得到相对路径


def move_init_pose(arm):
    arm.set_position(300, 0, 100, 180, 0, 0, speed=50, mvacc=500, wait=True)

def save_path():
    dir_path = 'imgSave'
    if not os.path.exists(dir_path):
        os.mkdir(dir_path)
    return dir_path

'''
注意：
   1.该文件会覆盖原有imgSave文件内的所有标定图片和tcp数据，慎用！！！
   2.重新标定，请删除imgSave内的所有文件
'''

if __name__ == '__main__':
    arm = XArmAPI('192.168.31.100')
    camera = Realsense2()
    dir_path = save_path()
    index = 1
    image_num = 16
    file_path = ROOT/dir_path/'calibration.txt'

    # state
    arm.motion_enable(enable=True)  # enable motion
    arm.set_mode(0)  # set mode: position control mode
    arm.set_state(state=0)  # set state: sport state
    arm.set_pause_time(1)  # wait 1 s
    move_init_pose(arm)  # move init pose

    # collect calibration parameters
    while True:
        # save img
        cap_img, _, _ = camera.get_data()
        cv2.imshow("capture video", cap_img)
        key = cv2.waitKey(1) & 0xFF

        if key == 27:
            break
        elif key == ord(" "):
            cv2.imshow("save" + str(index), cap_img)
            cv2.waitKey(2000)
            cv2.destroyWindow("save" + str(index))
            cv2.imwrite(str(ROOT) + '/' + str(dir_path) + '/' + str(index) + '.png', cap_img)

            # save robot tcp
            cur_pose = arm.get_position()
            tcp_pose = [*cur_pose[1]]
            with open(file_path, "a+", encoding="utf-8") as file:
                for data in tcp_pose:
                    file.write(str(data) + ' ')
                file.write('\n')
            print("Input {} success.".format(index))

            # next
            if index == image_num:
                break

            index += 1
        else:
            pass
    file.close()






