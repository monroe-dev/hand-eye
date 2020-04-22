"""
@File    : select.py
@Author  : Hyunsoo Shin
@Date    : 20. 4. 17.
@Contact : hyunsoo.shin@outlook.com
"""

import kinematics as kin
import numpy as np
import math
import itertools
from matplotlib import pyplot as plt
import draw as dR


class Pose:
    def __init__(self, init_x, init_y, init_z, init_rx, init_ry, init_rz):
        self.init_T = kin.homogeneousMatrix(init_x, init_y, init_z, init_rx, init_ry, init_rz, False)
        self.init_R = kin.rotationFromHmgMatrix(self.init_T)
        self.init_t = kin.translationFromHmgMatrix(self.init_T)

    def generate(self, num_of_pose, distance_target, angle_btw_rvec):
        rvec, ang = kin.rotMatrixToRodVector(self.init_R)

        theta = np.pi / 180 * 90
        temp = np.array([0, 1, 0])

        next_rvec = self.rotateAbout(a, rvec, theta)

        set_of_robot_pose = next_rvec
        return set_of_robot_pose

    # def makeUnit(x):
    #     """Normalize entire input to norm 1. Not what you want for 2D arrays!"""
    #     return x / np.linalg.norm(x)

    def rotateAbout(self, a, b, theta):
        """Rotate vector a about vector b by theta radians."""
        # Thanks user MNKY at http://math.stackexchange.com/a/1432182/81266
        proj = self.xProjectV(a, b)
        w = np.cross(b, proj['perp'])
        return (proj['par'] +
                proj['perp'] * np.cos(theta) +
                np.linalg.norm(proj['perp']) * (w / np.linalg.norm(w)) * np.sin(theta))

    def xParV(self, x, v):
        """Project x onto v. Result will be parallel to v."""
        # (x' * v / norm(v)) * v / norm(v)
        # = (x' * v) * v / norm(v)^2
        # = (x' * v) * v / (v' * v)
        return np.dot(x, v) / np.dot(v, v) * v

    def xProjectV(self, x, v):
        """Project x onto v, returning parallel and perpendicular components
        >> d = xProject(x, v)
        >> np.allclose(d['par'] + d['perp'], x)
        True
        """
        par = self.xParV(x, v)
        perp = x - par
        return {'par': par, 'perp': perp}

    def savePose(self):
        print('Saved.')

    def loadPose(self, file_name):
        data = np.loadtxt(file_name, delimiter=' ', dtype=float)
        return data

if __name__ == "__main__":
    p = Pose(init_x=0, init_y=0, init_z=0, init_rx=0, init_ry=np.pi, init_rz=0)
    # robot_pose = p.generate(num_of_pose=10, distance_target=0.3, angle_btw_rvec=10)
    # print(robot_pose)

    ur_pose = p.loadPose('pose.txt')
    comb = list(itertools.combinations(list(range(len(ur_pose))), 2))
    ur_ori = ur_pose[:, -3:]

    # for c in comb:
    #     print(kin.angleBtwVectors(ur_ori[c[0]], ur_ori[c[1]]))
        # _, ur_rvec = kin.rodVectorToAngle(ur_ori[])
        # ur_rvec.


    ORG = np.array([0, 0, 0])
    X_axis = np.array([1, 0, 0])
    Y_axis = np.array([0, 1, 0])
    Z_axis = np.array([0, 0, 1])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    n = 0
    for i in range(len(ur_pose)):
        _, ur_rvec = kin.rodVectorToAngle(ur_ori[i])
        dR.drawVector(ax, ORG, ur_rvec, arrowstyle='-|>', proj=False, pointEnable=False) #, annotationString=' $ ^{A}P $ '

    dR.drawPointWithAxis(ax, ORG, X_axis, Y_axis, Z_axis, pointEnable=True, lineWidth=2)
    # dR.drawPointWithAxis(ax, BORG, hat_X_atB, hat_Y_atB, hat_Z_atB, pointEnable=False, lineStyle='--')

    ax.set_xlim([-1.5, 1.5]), ax.set_ylim([-1.5, 1.5]), ax.set_zlim([-1, 1.5])
    ax.set_xlabel('X axis'), ax.set_ylabel('Y axis'), ax.set_zlabel('Z axis')
    # ax.view_init(azim=-90, elev=90)
    plt.show()
