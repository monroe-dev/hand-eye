"""
@File    : calibrate.py
@Author  : Hyunsoo Shin
@Date    : 20. 4. 28.
@Contact : hyunsoo.shin@outlook.com
"""

import numpy as np
import cv2

robot_pose = np.loadtxt('data/kuka_2/RobotPosesVec.txt', delimiter='\t', dtype=float)
robot_pose = np.reshape(robot_pose, (28, 4, 4))

