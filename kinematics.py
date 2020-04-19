"""
@File    : kinematics.py
@Author  : Hyunsoo Shin
@Date    : 20. 4. 16.
@Contact : hyunsoo.shin@outlook.com
"""
import numpy as np
from math import *
import transformations as tf
import cv2
import math

# dummy = np.array([0, 0, 0, 1])
dummy = np.array([[0, 0, 0, 1]])


def homogeneousMatrix(R, tvec):
    T = np.concatenate((R, tvec), axis=1)
    T = np.concatenate((T, dummy), axis=0)
    return T


def homogeneousInverse(T):
    R = T[:3, :3]
    t = T[:3, 3]
    t = np.expand_dims(t, axis=1)
    invT = np.concatenate((R.T, np.dot(-R.T, t)), axis=1)
    invT = np.concatenate((invT, dummy), axis=0)
    return invT

def rotateXaxis(ang, is_deg):
    """
    Input: Angle for rotation, bool type(degree or radian)
    Return: Rotation matrix(type: numpy, shape: 3,3)
    """
    if is_deg:
        ang = np.deg2rad(ang)
    rotX = np.array([[1, 0, 0], [0, cos(ang), -sin(ang)], [0, sin(ang), cos(ang)]])
    return rotX


def rotateYaxis(ang, is_deg):
    """
    Input: Angle for rotation, bool type(degree or radian)
    Return: Rotation matrix(type: numpy, shape: 3,3)
    """
    if is_deg:
        ang = np.deg2rad(ang)
    rotY = np.array([[cos(ang), 0, sin(ang)], [0, 1, 0], [-sin(ang), 0, cos(ang)]])

    return rotY


def rotateZaxis(ang, is_deg):
    """
    Input: Angle for rotation, bool type(degree or radian)
    Return: Rotation matrix(type: numpy, shape: 3,3)
    """
    if is_deg:
        ang = np.deg2rad(ang)
    rotZ = np.array([[cos(ang), -sin(ang), 0], [sin(ang), cos(ang), 0], [0, 0, 1]])
    return rotZ


def rotationMatrix(r, p, y, is_deg, is_homog):
    """
    Generate a rotation matrix with 3 rotation such as roll, pitch, yaw
    Input: roll, pitch, yaw, bool type(degree or radian), bool type(3x3 or 4x4)
    Output: Rotation matrix(type: numpy, shape: 3,3)
    """
    R1 = rotateZaxis(y, is_deg)
    R2 = rotateYaxis(p, is_deg)
    R3 = rotateXaxis(r, is_deg)
    R = R1.dot(R2).dot(R3)
    if is_homog:
        R = np.hstack((R, np.zeros((3, 1))))
        R = np.vstack((R, dummy))
    return R


def translationVector(x, y, z):
    """
    Generate a translation vector
    Input: x, y, z
    Output: Vector(type: numpy, shape: 3,1)
    """
    t_vec = np.array([[x], [y], [z]])
    return t_vec


def translationMatrix(x, y, z):
    """
    Generate a translation matrix
    Input: x, y, z
    Output: Matrix(type: numpy, shape: 4,4)
    """
    t_mat = np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])
    return t_mat


def homogeneousMatrix(x, y, z, rx, ry, rz, is_deg):
    """
    Generate a translation matrix
    Input: x, y, z, r, p, y, bool type(degree or radian)
    Output: Matrix(type: numpy, shape: 4,4)
    """
    is_homog = True
    homog_matrix = np.dot(translationMatrix(x, y, z), rotationMatrix(rx, ry, rz, is_deg, is_homog))
    return homog_matrix

def generatePointSet(num, dim, min, max):
    """
    Generate a point set
    :param num: The number of points
    :param dim: The dimension of points(2d or 3d)
    :param min: The minimum of each point value
    :param max: The maximum of each point value
    :return: Point set(Nx2 or Nx3)
    """
    return np.random.uniform(min, max, (num, 3))

def transform(points, transformation_matrix_a2b):
    """
    Transform point from A to B
    :param points: Points in 3d
    :param transformation_matrix_a2b: Homogeneous matrix(4x4)
    :return: Transformed point
    """
    if points.shape[0] > 1:
        transformed = []
        for p in points:
            point = np.append(p, [1])
            transformed.append(np.dot(transformation_matrix_a2b, point))
    else:
        point = np.append(points, [1])
        transformed = np.dot(transformation_matrix_a2b, point)

    # List to Numpy
    transformed = np.array(transformed)
    return transformed[:, 0:3]

def rotationFromHmgMatrix(T):
    R = T[:3, :3]
    return R

def translationFromHmgMatrix(T):
    t = T[:3, 3]
    return t



def rotMatrixToRodVector(R):
    # a = (np.trace(R) - 1) / 2
    # if a == -1:
    #     rvec = np.array([sqrt((1 + R[0, 0]) / 2), sqrt((1 + R[1, 1]) / 2), sqrt((1 + R[2, 2]) / 2)])
    # elif a == 1:
    #     rvec = np.zeros(3)
    # else:
    #     to_normal = np.sqrt((3 - np.trace(R)) * (1 + np.trace(R)))
    #     rvec = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]] / to_normal)
    # angle = math.acos((np.trace(R) - 1) / 2)
    # print(np.linalg.inv(R))
    # print(np.transpose(R))
    # print(np.array_equal(np.transpose(R), np.linalg.inv(R)))
    # print(np.sum(np.linalg.inv(R) - np.transpose(R)))
    # print(math.isclose(np.sum(np.linalg.inv(R) - np.transpose(R)), 0.1e-30, rel_tol=0.01))
    if np.linalg.det(R) == 1:
        a = np.array(np.transpose(R))
        b = np.linalg.inv(R)
        if abs(np.sum(a - b)) < 1e-15:
            rvec = np.empty((3, 1))
            cv2.Rodrigues(R, rvec)
            angle = np.linalg.norm(rvec)
            norm_rvec = rvec / angle
            return norm_rvec, angle
    else:
        print('This is not a rotation matrix...')

