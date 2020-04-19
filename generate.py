"""
@File    : select.py
@Author  : Hyunsoo Shin
@Date    : 20. 4. 17.
@Contact : hyunsoo.shin@outlook.com
"""

import kinematics as kin
import numpy as np
import math

class Pose:
    def __init__(self, init_x, init_y, init_z, init_rx, init_ry, init_rz):
        self.init_T = kin.homogeneousMatrix(init_x, init_y, init_z, init_rx, init_ry, init_rz, False)
        self.init_R = kin.rotationFromHmgMatrix(self.init_T)
        self.init_t = kin.translationFromHmgMatrix(self.init_T)

    def generate(self, num_of_pose, distance_target, angle):
        rvec, ang = kin.rotMatrixToRodVector(self.init_R)
        # vec
        print(self.init_t)
        print((self.init_R[:, 2] * distance_target))

        set_robot_pose = rvec
        return set_robot_pose


    def savePose(self):
        print('Saved.')


    def loadPose(self):
        data = np.loadtxt('pose.txt', delimiter=' ', dtype=float)
        return data



