from ultralytics import YOLO
from core.realsense2 import *
from core.wrap_xarm import xarm6
from core.calibrate import Calibration
import argparse

FILE = Path(__file__).resolve()       # __file__:当前路径 .resolve():获取绝对路径
ROOT = FILE.parents[0]                # .parents():路径的父目录


def hsv2bgr(h, s, v):
    h_i = int(h * 6)
    f = h * 6 - h_i
    p = v * (1 - s)
    q = v * (1 - f * s)
    t = v * (1 - (1 - f) * s)

    r, g, b = 0, 0, 0

    if h_i == 0:
        r, g, b = v, t, p
    elif h_i == 1:
        r, g, b = q, v, p
    elif h_i == 2:
        r, g, b = p, v, t
    elif h_i == 3:
        r, g, b = p, q, v
    elif h_i == 4:
        r, g, b = t, p, v
    elif h_i == 5:
        r, g, b = v, p, q

    return int(b * 255), int(g * 255), int(r * 255)


def random_color(id):
    h_plane = (((id << 2) ^ 0x937151) % 100) / 100.0
    s_plane = (((id << 3) ^ 0x315793) % 100) / 100.0
    return hsv2bgr(h_plane, s_plane, 1)


def show(rgb_img, names, boxes):
    center = []
    if boxes:
        boxes = np.array(boxes)
        boxes = boxes[boxes[:, -1] == 47].tolist()          # 只检测苹果
        if boxes:
            boxes = [boxes[0]]
            for obj in boxes:
                left, top, right, bottom = int(obj[0]), int(obj[1]), int(obj[2]), int(obj[3])
                center = [left+abs(round((right-left)/2)), top+abs(round((bottom-top)/2))]
                confidence = obj[4]
                label = int(obj[5])
                color = random_color(label)
                cv2.rectangle(rgb_img, (left, top), (right, bottom), color=color, thickness=2, lineType=cv2.LINE_AA)
                caption = f"{names[label]} {confidence:.2f}"
                w, h = cv2.getTextSize(caption, 0, 1, 2)[0]
                cv2.rectangle(rgb_img, (left - 3, top - 33), (left + w + 10, top), color, -1)
                cv2.putText(rgb_img, caption, (left, top - 5), 0, 1, (0, 0, 0), 2, 16)

    cv2.imshow(f'realsense', rgb_img)
    cv2.waitKey(1000)

    return center


def create_parser():
    print_and_write(None, "------------------------config------------------------", 33)
    parser = argparse.ArgumentParser(description='GRASP')                                      # config
    parser.add_argument('--date', type=str, default='test', help='calibration file')

    return parser


if __name__ == "__main__":
    # input para config
    parser = create_parser()
    args = parser.parse_args()

    model = YOLO(f"../ultralytics/yolov8n.pt")  # 加载模型
    robot = xarm6()                             # robot
    realcam = Realsense2()                      # camera

    # 获取标定数据
    cal_data = np.load(f'{ROOT}\\..\\output\\imgSave\\{args.date}\\data.npy', allow_pickle=True).item()
    RT_cam2gri = cal_data['RT_cam2gripper']
    RT_cam2gri[:3, 3] *= 1000
    RT_obj2cam = np.identity(4)
    RT_gri2base = np.identity(4)

    # LOOP
    for img in realcam:
        robot.move_init_pose()                   # 初始位姿
        rgb_img, _, _ = img                      # rgb图，深度图，深度信息
        if len(np.array(rgb_img).shape) == 3:
            # 使用模型对图像进行预测
            results = model(rgb_img)[0]
            img_xy = show(rgb_img, results.names, results.boxes.data.tolist())
            if img_xy:
                real_xyz = realcam.pixel2point(img_xy)
                RT_obj2cam[:3, 3] = np.array(real_xyz) * 1000
                RT_gri2base = Calibration.Cartesian2Homogeneous(robot.get_current_pose())
                RT_gri2base[:3, 3] *= 1000
                RT_obj2base = RT_gri2base.dot(RT_cam2gri.dot(RT_obj2cam))
                print(f"RT_obj2base: {RT_obj2base}")

                # action
                robot(RT_obj2base[:3, 3])
                break
    print("End.")






