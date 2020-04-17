"""
@File    : main.py
@Author  : Hyunsoo Shin
@Date    : 20. 4. 16.
@Contact : hyunsoo.shin@outlook.com
"""

import remote_ur
import capture
import time
import chessboard

print('Start!')
# remote_ur.init()
# # t_x =
# # t_y =
# # t_z =
# # t_rx =
# # t_ry =
# # t_rz =
# remote_ur.moveUR(t_x, t_y, t_z, t_rx, t_ry, t_rz)
# # delay!!!!
# x, y, z, rx, ry, rz = urcommand.ur_get_pose()

num_of_poses = 3
cam = capture.ChessBoard(640, 480, 7, 9, 14)

# Camera Calibration
# cam.calibrate(30, 500)

# Pose estimation(chessboard)
# cam.estimatePose(False)

for i in range(num_of_poses):
    # remote_ur.moveUR(des_x, des_y, des_z, des_rx, des_ry, des_rz)
    time.sleep(3)
    cam.save()
    time.sleep(0.1)
    print(i + 1,  "th image was captured.")


print('Finished.')
