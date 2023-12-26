from robot_sdk.xArm.xarm.wrapper import XArmAPI
from core.utils import *
import sys
import random


class xarm6(object):
    def __init__(self, ip='192.168.31.100'):
        try:
            self.arm = XArmAPI(ip)
        except Exception as e:
            # 处理异常的代码
            print_and_write(None, f"Xarm6 connect failed, check if the connection is correct", 31)
            sys.exit()
        self.init_robot()

    def __call__(self, action):
        """Calls the move(action) method with given arguments to perform grasp."""
        return self.move(action)

    def init_robot(self):
        self.arm.motion_enable(enable=True)            # enable motion
        self.arm.set_mode(0)                           # set mode: position control mode
        self.arm.set_state(state=0)                    # set state: sport state
        self.arm.set_pause_time(1)                     # wait 1 s
        self.move_init_pose()                          # move to init pose

        # rand sample xyz pose
        self.rand_pos = [[350, 0, 195, 180, 0, 0], [355, -60, 202, -170, 0, 0], [355, 70, 205, 170, 0, 0],
                         [345, 55, 200, 168, 0, 10], [350, 85, 199, 168, 0, -15], [345, 55, 188, 172, 0, 0],
                         [335, 5, 230, -178, 0, -15], [330, -55, 190, -165, 2, -16], [390, -65, 187, -167, 5, 10],
                         [420, -65, 200, -166, 6, 25], [320, 135, 225, 162, -12, -22], [423, -25, 227, -178, 10, 24],
                         [403, 30, 209, 176, 14, -24], [463, 0, 225, -175, 16, 30], [480, -18, 219, -171, 22, 20],
                         [387, -40, 212, -171, 11, -5], [310, -60, 202, -171, -10, -5], [356, -68, 198, -166, 3, -5],
                         [400, 20, 187, 175, 13, 5], [438, 0, 189, -178, 20, -3]]
        self.limit_xyz = [[220, 580], [-300, 300], [-70, 450]]

    def move_init_pose(self):
        self.arm.set_position(350, 0, 150, 180, 0, 0, speed=50, mvacc=500, wait=True)

    def rand_action(self):
        sample_index = random.randint(0, len(self.rand_pos) - 1)
        sample_pos = self.rand_pos[sample_index]
        self.rand_pos.pop(sample_index)
        rand_noise = [random.randint(-20, 20), random.randint(-15, 15), random.randint(-20, 15),
                      random.randint(-4, 4), random.randint(-4, 4), random.randint(-4, 4)]
        action = np.array(sample_pos) + np.array(rand_noise)
        self.arm.set_position(*action, speed=50, mvacc=500, wait=True)
        self.arm.set_pause_time(1)  # wait 1 s

    def _move_boundary(self, action):
        """
        Limiting the range of motion.
        """
        new_action = []
        action = action if isinstance(action, list) else action.tolist()
        assert len(action) == 3, 'action dimension error'
        for l, a in zip(self.limit_xyz, action):
            new_action.append(max(l[0], min(l[1], a)))
        return new_action

    def move(self, action):
        self.arm.set_position(*self._move_boundary(action), speed=50, mvacc=500, wait=True)
        self.arm.set_pause_time(1)  # wait 1 s

    def get_current_pose(self):
        pose_info = self.arm.get_position()
        if pose_info[0] == 0:
            return pose_info[1]
        else:
            self.move_init_pose()
            # 处理异常的代码
            print_and_write(None, f"Robot pose acquisition failed.", 31)
            sys.exit()

