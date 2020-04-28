"""
@File    : simulate.py
@Author  : Hyunsoo Shin
@Date    : 20. 4. 16.
@Contact : hyunsoo.shin@outlook.com
"""
import cv2
import numpy as np
import kinematics as kin

import draw as dR
from Arrow3D import *


def newPose(type_of_motion, min_theta, max_theta, min_tvec, max_tvec):
    """
    Generate new random poses within the specific range (rvec and tvec)
    Pure translation : PT
    Pure rotation : PR
    Planar motion : PM
    General motion : GM
    """
    if type_of_motion == 'PT':
        tvec = np.random.uniform(min_tvec, max_tvec, (3, 1))
        # theta is static angle..
        arb_theta = max_theta - min_theta
        R = kin.RotationMatrix(arb_theta, arb_theta * 2, arb_theta * 3, True, False)
    elif type_of_motion == 'PR':
        # translation is static vector..
        arb_translation = max_tvec - min_tvec
        tvec = kin.translationVector(arb_translation, arb_translation * 2, arb_translation * 3)
        axis = np.random.uniform(-1, 1, (3, 1))
        normal_axis = axis / np.linalg.norm(axis)
        theta = np.random.uniform(min_theta, max_theta)
        rvec = normal_axis * theta
        R = np.empty((3, 3))
        cv2.Rodrigues(rvec, R)
    elif type_of_motion == 'GM':
        axis = np.random.uniform(-1, 1, (3, 1))
        normal_axis = axis / np.linalg.norm(axis)
        theta = np.random.uniform(min_theta, max_theta)
        rvec = normal_axis * theta
        tvec = np.random.uniform(min_tvec, max_tvec, (3, 1))
        R = np.empty((3, 3))
        cv2.Rodrigues(rvec, R)
    elif type_of_motion == 'PM':
        tvec = np.random.uniform(min_tvec, max_tvec, (3, 1))
        # it should be write code...
    elif type_of_motion == 'temp':
        axis = np.random.uniform(0.5, 1, (3, 1))
        normal_axis = axis / np.linalg.norm(axis)
        theta = np.random.uniform(min_theta, max_theta)
        rvec = normal_axis * theta
        tvec = np.random.uniform(min_tvec, max_tvec, (3, 1))
        R = np.empty((3, 3))
        cv2.Rodrigues(rvec, R)
    elif type_of_motion == 'temp2':
        axis = np.random.uniform(-0.4, 0.1, (3, 1))
        normal_axis = axis / np.linalg.norm(axis)
        theta = np.random.uniform(min_theta, max_theta)
        rvec = normal_axis * theta
        tvec = np.random.uniform(min_tvec, max_tvec, (3, 1))
        R = np.empty((3, 3))
        cv2.Rodrigues(rvec, R)
    return R, tvec


def generateNewPose(nPose, motion):
    """
    T_hand2eye, T_base2world are constant(fixed) matrix.
    T_base2hand, T_eye2world are different matrix at the each pose.
    """
    # Generate the pose
    general_motion = 'GM'
    pure_translation = 'PT'
    pure_rotation = 'PR'

    R_hand2eye, t_hand2eye = newPose(general_motion, np.deg2rad(10), np.deg2rad(50), 0.1, 0.3)
    T_hand2eye = kin.homogMatfromRotAndTrans(R_hand2eye, t_hand2eye)
    R_base2world, t_base2world = newPose(general_motion, np.deg2rad(5), np.deg2rad(85), 0.5, 3.5)
    T_base2world = kin.homogMatfromRotAndTrans(R_base2world, t_base2world)

    # Generation of robot motion
    robot_motion = [np.deg2rad(5), np.deg2rad(180), 0.5, 2.0]    # [min_theta, max_theta, min_t, max_t]

    # Generation of noise
    noise = True
    eye_noise_ratio = 0.02
    eye_noise_theta = [-2, 2]
    robot_noise_ratio = 0.005
    robot_noise_theta = [-1, 1]

    # [min_theta, max_theta, min_t, max_t]
    eye_noise = [eye_noise_theta[0], eye_noise_theta[1], robot_motion[2] * eye_noise_ratio, robot_motion[3] * eye_noise_ratio]
    robot_noise = [robot_noise_theta[0], robot_noise_theta[1], robot_motion[2] * robot_noise_ratio, robot_motion[3] * robot_noise_ratio]
    noise_data = [eye_noise, robot_noise]
    rvec_noise_ = np.random.uniform(-1, 1, (3, 1))
    rvec_noise = rvec_noise_ / np.linalg.norm(rvec_noise_)

    R_eye2world = np.zeros((1, 3, 3))
    t_eye2world = np.zeros((1, 3, 1))
    R_base2hand = np.zeros((1, 3, 3))
    t_base2hand = np.zeros((1, 3, 1))

    for i in range(nPose):
        if motion == 'temp':
            R_base2hand_, t_base2hand_ = newPose('temp', robot_motion[0], robot_motion[1], robot_motion[2], robot_motion[3])
        elif motion == 'temp2':
            R_base2hand_, t_base2hand_ = newPose('temp2', robot_motion[0], robot_motion[1], robot_motion[2], robot_motion[3])
        elif motion == 'GM':
            R_base2hand_, t_base2hand_ = newPose('GM', robot_motion[0], robot_motion[1], robot_motion[2], robot_motion[3])

        T_base2hand = kin.homogMatfromRotAndTrans(R_base2hand_, t_base2hand_)
        T_eye2base = np.dot(kin.homogeneousInverse(T_hand2eye), kin.homogeneousInverse(T_base2hand))
        T_eye2world = np.dot(T_eye2base, T_base2world)
        R_eye2world_ = T_eye2world[:3, :3]
        t_eye2world_ = T_eye2world[:3, 3]
        t_eye2world_ = np.expand_dims(t_eye2world_, axis=1)

        if noise:
            """ Estimation of world coordinate using camera(eye) has some error,
                and the robot also usually exist positioning error."""
            # --- World coordinate --- #
            # Add to arbitrary noise.
            R_eye2world_noise = T_eye2world[:3, :3]
            rvec_eye2world_noise = np.empty((3, 1))
            # Orientation
            cv2.Rodrigues(R_eye2world_noise, rvec_eye2world_noise)
            angle_eye_noise = np.random.uniform(np.deg2rad(eye_noise[0]), np.deg2rad(eye_noise[1]))
            rvec_eye_noise = rvec_noise * angle_eye_noise
            rvec_eye2world_noise = np.add(rvec_eye2world_noise, rvec_eye_noise)
            cv2.Rodrigues(rvec_eye2world_noise, R_eye2world_noise)
            R_eye2world_ = R_eye2world_noise
            # Position
            t_eye2world_noise = T_eye2world[:3, 3]
            t_eye2world_noise = np.add(np.expand_dims(t_eye2world_noise, axis=1), np.random.uniform(eye_noise[2], eye_noise[3], (3, 1)))
            t_eye2world_ = t_eye2world_noise

            # --- Robot hand --- #
            # Orientation
            R_base2hand_noise = T_base2hand[:3, :3]
            rvec_base2hand_noise = np.empty((3, 1))
            angle_robot_noise = np.random.uniform(np.deg2rad(robot_noise[0]), np.deg2rad(robot_noise[1]))
            rvec_robot_noise = rvec_noise * angle_robot_noise
            cv2.Rodrigues(R_base2hand_noise, rvec_base2hand_noise)
            rvec_base2hand_noise = np.add(rvec_base2hand_noise, rvec_robot_noise)
            cv2.Rodrigues(rvec_base2hand_noise, R_base2hand_noise)
            R_base2hand_ = R_base2hand_noise

            # Position
            t_base2hand_noise = T_base2hand[:3, 3]
            t_base2hand_noise = np.add(np.expand_dims(t_base2hand_noise, axis=1), np.random.uniform(robot_noise[2], robot_noise[3], (3, 1)))
            t_base2hand_ = t_base2hand_noise

        # Rotation matrix and translation vector at the every pose
        R_eye2world = np.concatenate((R_eye2world, np.expand_dims(R_eye2world_, axis=0)), axis=0)
        t_eye2world = np.concatenate((t_eye2world, np.expand_dims(t_eye2world_, axis=0)), axis=0)
        R_base2hand = np.concatenate((R_base2hand, np.expand_dims(R_base2hand_, axis=0)), axis=0)
        t_base2hand = np.concatenate((t_base2hand, np.expand_dims(t_base2hand_, axis=0)), axis=0)

    # Because first array was zeros..
    return R_eye2world[1:], t_eye2world[1:], R_base2hand[1:], t_base2hand[1:], R_hand2eye, t_hand2eye, T_base2world, robot_motion, noise_data


def validate(num_of_test, num_of_pose, num_of_start_method, num_of_method, motion, test_motion):
    rvec_hand2eye_true = np.empty((3, 1))
    R_hand2eye_est = np.empty((3, 3))
    t_hand2eye_est = np.empty((3, 1))
    rvec_hand2eye_est = np.empty((3, 1))
    rvec_diff = np.empty((1, num_of_method - num_of_start_method))
    t_diff = np.empty((1, num_of_method - num_of_start_method))

    ori_error = np.empty((1, num_of_method - num_of_start_method))
    pos_error = np.empty((1, num_of_method - num_of_start_method))

    for i in range(num_of_test):
        # Generate a number of poses of robot and camera
        R_eye2world, t_eye2world, R_base2hand, t_base2hand, R_hand2eye_true, t_hand2eye_true, T_base2world, robot_motion, noise_data = \
            generateNewPose(num_of_pose, motion)
        rvec_diff_ = np.empty(1)
        t_diff_ = np.empty(1)
        ori_error_ = np.empty(1)
        pos_error_ = np.empty(1)

        for j in range(num_of_start_method, num_of_method):
            # Estimation of transformation from robot hand to camera
            cv2.calibrateHandEye(R_base2hand, t_base2hand, R_eye2world, t_eye2world, R_hand2eye_est, t_hand2eye_est, method=j)
            # --- Validate --- #
            # - Method 1 - #
            # error = X_true - X_estimation
            # Orientation
            cv2.Rodrigues(R_hand2eye_true, rvec_hand2eye_true)
            cv2.Rodrigues(R_hand2eye_est, rvec_hand2eye_est)
            # temp #
            print('t_hand2eye_est\n', t_hand2eye_est)
            print('t_hand2eye_true\n', t_hand2eye_true)
            print('R_hand2eye_est\n', R_hand2eye_est)
            print('R_hand2eye_true\n', R_hand2eye_true)
            _, rvec_diff__ = kin.rotMatrixToRodVector(np.dot(np.transpose(R_hand2eye_est), R_hand2eye_true))
            # rvec_diff__ = kin.rotMatrixToRodVector(temp_rvec_diff__)
            # original #
            # rvec_diff__ = np.linalg.norm(np.subtract(rvec_hand2eye_true, rvec_hand2eye_est))
            # Translation
            t_diff__ = np.linalg.norm(np.subtract(t_hand2eye_true, t_hand2eye_est)) / np.linalg.norm(t_hand2eye_true)
            # t_diff__ = np.linalg.norm(np.subtract(t_hand2eye_true, t_hand2eye_est))

            # The result vectors at the each method
            rvec_diff_ = np.append(rvec_diff_, np.expand_dims(rvec_diff__, axis=0), axis=0)
            t_diff_ = np.append(t_diff_, np.expand_dims(t_diff__, axis=0), axis=0)

            # - Method 2 - #
            # $ error = (AX)^-1 XB $ #
            X = kin.homogMatfromRotAndTrans(R_hand2eye_est, t_hand2eye_est)
            _, _, R_base2hand_test, t_base2hand_test, _, _, _, _, _ = generateNewPose(num_of_pose, test_motion)
            T_eye2hand = kin.homogeneousInverse(X)
            T_eye2world = np.zeros((1, 4, 4))
            T_base2hand = np.zeros((1, 4, 4))

            for R_base2hand_, t_base2hand_ in zip(R_base2hand_test, t_base2hand_test):
                T_base2hand_ = kin.homogMatfromRotAndTrans(R_base2hand_, t_base2hand_)
                T_hand2base_ = kin.homogeneousInverse(T_base2hand_)
                # eye2hand * hand2base * base2world
                T_eye2world_ = T_eye2hand.dot(T_hand2base_).dot(T_base2world)
                # Generation of noise
                eye_noise = noise_data[0]
                rvec_noise_ = np.random.uniform(-1, 1, (3, 1))
                rvec_noise = rvec_noise_ / np.linalg.norm(rvec_noise_)
                # --- World coordinate --- #
                # Add to arbitrary noise.
                R_eye2world_noise = T_eye2world_[:3, :3]
                rvec_eye2world_noise = np.empty((3, 1))
                # Orientation
                cv2.Rodrigues(R_eye2world_noise, rvec_eye2world_noise)
                angle_eye_noise = np.random.uniform(np.deg2rad(eye_noise[0]), np.deg2rad(eye_noise[1]))
                rvec_eye_noise = rvec_noise * angle_eye_noise
                rvec_eye2world_noise = np.add(rvec_eye2world_noise, rvec_eye_noise)
                cv2.Rodrigues(rvec_eye2world_noise, R_eye2world_noise)
                R_eye2world_ = R_eye2world_noise
                # Position
                t_eye2world_noise = T_eye2world_[:3, 3]
                t_eye2world_noise = np.add(np.expand_dims(t_eye2world_noise, axis=1),
                                           np.random.uniform(eye_noise[2], eye_noise[3], (3, 1)))
                t_eye2world_ = t_eye2world_noise
                T_eye2world_noise = kin.homogMatfromRotAndTrans(R_eye2world_, t_eye2world_)
                T_eye2world = np.concatenate((T_eye2world, np.expand_dims(T_eye2world_noise, axis=0)), axis=0)
                T_base2hand = np.concatenate((T_base2hand, np.expand_dims(T_base2hand_, axis=0)), axis=0)
            # print('--------------------------------------')
            # print('Method', j)
            ori_error__, pos_error__ = computeHandeyeError(T_base2hand[1:], T_eye2world[1:], X)
            # ori_error = np.concatenate((ori_error, np.expand_dims(ori_error_, axis=0)))
            ori_error_ = np.append(ori_error_, np.expand_dims(ori_error__, axis=0), axis=0)
            pos_error_ = np.append(pos_error_, np.expand_dims(pos_error__, axis=0), axis=0)


        rvec_diff_ = rvec_diff_.reshape(1, num_of_method - num_of_start_method + 1)
        t_diff_ = t_diff_.reshape(1, num_of_method - num_of_start_method + 1)

        ori_error_ = ori_error_.reshape(1, num_of_method - num_of_start_method + 1)
        pos_error_ = pos_error_.reshape(1, num_of_method - num_of_start_method + 1)
        # The result vectors at the each test
        rvec_diff = np.concatenate((rvec_diff, rvec_diff_[:, 1:num_of_method - num_of_start_method + 1]), axis=0)
        t_diff = np.concatenate((t_diff, t_diff_[:, 1:num_of_method - num_of_start_method + 1]), axis=0)

        ori_error = np.concatenate((ori_error, ori_error_[:, 1:num_of_method - num_of_start_method + 1]), axis=0)
        pos_error = np.concatenate((pos_error, pos_error_[:, 1:num_of_method - num_of_start_method + 1]), axis=0)

    mean_rvec_diff = np.mean(rvec_diff[1:, :], axis=0)
    mean_t_diff = np.mean(t_diff[1:, :], axis=0)

    std_rvec_diff = np.std(rvec_diff[1:, :], axis=0)
    std_t_diff = np.std(t_diff[1:, :], axis=0)

    max_rvec_diff = np.max(rvec_diff[1:, :], axis=0)
    max_t_diff = np.max(t_diff[1:, :], axis=0)

    mean_ori = np.mean(ori_error[1:, :], axis=0)
    mean_pos = np.mean(pos_error[1:, :], axis=0)

    return R_base2hand, R_base2hand_test, rvec_diff[1:], t_diff[1:], mean_rvec_diff, mean_t_diff, std_rvec_diff, std_t_diff, max_rvec_diff, max_t_diff, ori_error[1:], pos_error[1:], mean_ori, mean_pos


def computeHandeyeError(A, B, X):
    n = len(A)
    sum_angular_error = 0
    sum_transl_error = 0
    cnt_error = 0
    for A_1, B_1 in zip(A, B):
        n -= 1
        for i in range(n):
            A_2 = A[-i - 1]
            B_2 = B[-i - 1]
            A_rel = np.dot(kin.homogeneousInverse(A_2), A_1)
            B_rel = np.dot(B_2, kin.homogeneousInverse(B_1))
            AX = np.dot(kin.rotationFromHmgMatrix(A_rel), kin.rotationFromHmgMatrix(X))
            XB = np.dot(kin.rotationFromHmgMatrix(X), kin.rotationFromHmgMatrix(B_rel))
            rot_error = np.dot(np.linalg.inv(AX), XB)
            transl_error = (np.dot(kin.rotationFromHmgMatrix(A_rel), kin.translationFromHmgMatrix(X)) + kin.translationFromHmgMatrix(A_rel)) \
                           - (np.dot(kin.rotationFromHmgMatrix(X), kin.translationFromHmgMatrix(B_rel)) + kin.translationFromHmgMatrix(X))

            _, angular_error = kin.rotMatrixToRodVector(kin.rotationFromHmgMatrix(rot_error))
            sum_angular_error = sum_angular_error + angular_error
            sum_transl_error = sum_transl_error + np.linalg.norm(transl_error)
            cnt_error += 1
    # print('sum angular error:', sum_angular_error)
    # print('sum translation error:', sum_transl_error)
    # print('cnt_error', cnt_error)
    return  sum_angular_error, sum_transl_error


if __name__=="__main__":
    import matplotlib.pyplot as plt
    num_of_test = 100
    num_of_pose = 30
    num_of_method = 4
    num_of_start_method = 0


    # CALIB_HAND_EYE_DANIILIDIS = 4
    # CALIB_HAND_EYE_ANDREFF = 3
    # CALIB_HAND_EYE_HORAUD = 2
    # CALIB_HAND_EYE_PARK = 1
    # CALIB_HAND_EYE_TSAI = 0

    # --- Demo --- #
    motion = 'GM'
    test_motion = 'temp'
    R_base2hand, R_base2hand_test, rvec_diff, t_diff, mean_rvec_diff, mean_t_diff, std_rvec_diff, std_t_diff, \
    max_rvec_diff, max_t_diff, ori_error, pos_error, mean_ori, mean_pos = validate(num_of_test, num_of_pose, num_of_start_method, num_of_method, motion, test_motion)

    # test_motion = 'temp2'
    # R_base2hand, R_base2hand_test, rvec_diff, t_diff, mean_rvec_diff, mean_t_diff, std_rvec_diff, std_t_diff, max_rvec_diff, max_t_diff = validate(
    #     num_of_test, num_of_pose, num_of_method, motion, test_motion)

    # rvec_diff, t_diff, mean_rvec_diff, mean_t_diff, std_rvec_diff, std_t_diff, max_rvec_diff, max_t_diff = validate(
    #     num_of_test, num_of_pose, num_of_method)

    ORG = np.array([0, 0, 0])
    X_axis = np.array([1, 0, 0])
    Y_axis = np.array([0, 1, 0])
    Z_axis = np.array([0, 0, 1])
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for k in range(num_of_pose):
        rvec, _ = kin.rotMatrixToRodVector(R_base2hand[k])
        _, ur_rvec = kin.rodVectorToAngle(np.squeeze(rvec))
        dR.drawVector(ax, ORG, ur_rvec, arrowstyle='-|>', proj=False, pointEnable=False)

        rvec_test, _ = kin.rotMatrixToRodVector(R_base2hand_test[k])
        _, ur_rvec_test = kin.rodVectorToAngle(np.squeeze(rvec_test))
        dR.drawVector(ax, ORG, ur_rvec_test, arrowstyle='-|>', proj=False, pointEnable=False, lineColor='b')

    dR.drawPointWithAxis(ax, ORG, X_axis, Y_axis, Z_axis, pointEnable=True, lineWidth=2)
    ax.set_xlim([-1.5, 1.5]), ax.set_ylim([-1.5, 1.5]), ax.set_zlim([-1, 1.5])
    ax.set_xlabel('X axis'), ax.set_ylabel('Y axis'), ax.set_zlabel('Z axis')
    plt.show()
    # # Print
    print('Error analyis')
    for i in range(num_of_method - num_of_start_method):
        print('Method', i + num_of_start_method, ': (Mean / Std / Max)')
        print('Orientation: ', mean_rvec_diff[i], '/', std_rvec_diff[i], '/', max_rvec_diff[i])
        print('Position: ', mean_t_diff[i], '/', std_t_diff[i], '/', max_t_diff[i])
        print('---Different test set---')
        print('Orientation: ', mean_ori[i])
        print('Position: ', mean_pos[i])

    # Graph
    fig_error = plt.figure(figsize=(12, 12))
    # ax = fig.add_subplot(111)

    plt1 = fig_error.add_subplot(221)
    plt2 = fig_error.add_subplot(222)
    plt3 = fig_error.add_subplot(223)
    plt4 = fig_error.add_subplot(224)

    for i in range(num_of_method - num_of_start_method):
        plt1.plot(rvec_diff[:, i])
        plt2.plot(t_diff[:, i])
        plt1.set_xlabel('Measurement number')
        plt1.set_ylabel('')
        plt2.set_xlabel('Measurement number')

    plt1.grid(True)
    plt2.grid(True)
    plt3.grid(True)
    plt4.grid(True)
    plt1.set_title('Orientation error')
    plt2.set_title('Position error')
    plt1.legend(('TSA', 'PAR', 'HOR', 'AND', 'DAN'))
    plt2.legend(('TSA', 'PAR', 'HOR', 'AND', 'DAN'))

    plt3.boxplot(rvec_diff)
    plt4.boxplot(t_diff)
    plt3.set_xlabel('Method')
    plt4.set_xlabel('Method')

    # plt.grid(True)
    fig_error.savefig('result_handeye_calibration.png', dpi=300)
    plt.show()