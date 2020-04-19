"""
@File    : capture.py
@Author  : Hyunsoo Shin
@Date    : 20. 4. 16.
@Contact : hyunsoo.shin@outlook.com
"""
import cv2
import numpy as np
import os


class ChessBoard:
    def __init__(self, frame_width, frame_height, num_of_width, num_of_height, size_of_square):
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.num_of_width = num_of_width
        self.num_of_height = num_of_height
        self.size_of_square = size_of_square
        self.cap = cv2.VideoCapture(2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.save_cnt = 0

        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((self.num_of_width * self.num_of_height, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.num_of_width, 0:self.num_of_height].T.reshape(-1, 2) * self.size_of_square

        # Arrays to store object points and image points from all the images.
        self.objpoints = []  # 3d point in real world space
        self.imgpoints = []  # 2d points in image plane.

        dirname = 'image'
        os.makedirs(dirname, exist_ok=True)

    def start(self):
        while True:
            ret, frame = self.cap.read()
            # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cv2.imshow('img', frame)
            cv2.waitKey(1)
        cv2.destroyAllWindows()

    def save(self):
        cnt = 0
        threshold = 10
        while True:
            cnt += 1
            ret, frame = self.cap.read()
            if cnt > threshold:
                cv2.imwrite('image/img' + str(self.save_cnt) + '.png', frame)
                self.save_cnt += 1
                break
            cv2.waitKey(1)

    def calibrate(self, total_frame, time_btw_frame):
        # total_frame : total of the calibration frame
        # time_btw_frame : milliseconds
        num_of_frame = 1
        while True:
            ret, frame = self.cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (self.num_of_width, self.num_of_height), None)
            # If found, add object points, image points (after refining them)
            if ret == True:
                self.objpoints.append(self.objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                self.imgpoints.append(corners)
                # Draw and display the corners
                cv2.drawChessboardCorners(frame, (self.num_of_width, self.num_of_height), corners2, ret)
                cv2.putText(frame, 'Camera Calibration', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                cv2.putText(frame, str(num_of_frame) + '/' + str(total_frame), (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                cv2.imshow('img', frame)
                cv2.waitKey(time_btw_frame)
                num_of_frame += 1
            if num_of_frame > total_frame:
                break

        cv2.destroyAllWindows()
        # Calibrate
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, gray.shape[::-1], None, None)
        # Re-projection error
        mean_error = 0
        for i in range(len(self.objpoints)):
            imgpoints2, _ = cv2.projectPoints(self.objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error
        print("total error: {}".format(mean_error / len(self.objpoints)))
        # Save camera parameters
        np.savez('cam_parameter', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

    def estimatePose(self, check_cam_coordinate):
        # check the coordinate of camera
        rect_width = 200
        rect_height = 150
        length_of_axis = 15
        start_point = (int(self.frame_width / 2 - rect_width / 2), int(self.frame_height / 2 - rect_height / 2))
        end_point = (int(self.frame_width / 2 + rect_width / 2), int(self.frame_height / 2 + rect_height / 2))
        ref_pts = np.zeros((3, 1))

        # Load previously saved data
        with np.load('cam_parameter.npz') as X:
            mtx, dist, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]

        def draw(img, corners, imgpts):
            corner = tuple(corners[0].ravel())
            img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 3)
            img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 3)
            img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 3)
            return img
        axis = np.float32([[length_of_axis, 0, 0], [0, length_of_axis, 0], [0, 0, -length_of_axis]]).reshape(-1, 3)

        while True:
            ret, frame = self.cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (self.num_of_width, self.num_of_height), None)

            if ret == True:
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                # Find the rotation and translation vectors.
                ret, rvecs, tvecs = cv2.solvePnP(self.objp, corners2, mtx, dist)
                # project 3D points to image plane
                imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
                img = draw(frame, corners2, imgpts)
                if check_cam_coordinate:
                    cv2.rectangle(img, start_point, end_point, (255, 0, 0), 1)

                cv2.imshow('img', img)
                key = cv2.waitKey(1)
                if key == 27:
                    break
                elif key == ord('s'):
                    print('Saved the reference point')
                    print('tvec:', tvecs)
                    ref_pts = tvecs
                elif key == ord(' '):
                    if np.array_equal(ref_pts, np.zeros((3, 1))):
                        print('Press "s" key before "space bar" key...')
                    else:
                        diff_vec = tvecs - ref_pts
                        print('diff_vec:', diff_vec)
                        if check_cam_coordinate:
                            if np.argmax(abs(diff_vec)) == 0:
                                if diff_vec[np.argmax(abs(diff_vec))] > 0:
                                    print('+ X axis')
                                else:
                                    print('- X axis')
                            elif np.argmax(abs(diff_vec)) == 1:
                                if diff_vec[np.argmax(abs(diff_vec))] > 0:
                                    print('+ Y axis')
                                else:
                                    print('- Y axis')
                            elif np.argmax(abs(diff_vec)) == 2:
                                if diff_vec[np.argmax(abs(diff_vec))] > 0:
                                    print('+ Z axis')
                                else:
                                    print('- Z axis')
                    print('distance from camera', np.linalg.norm(tvecs))
        self.cap.release()
        cv2.destroyAllWindows()
