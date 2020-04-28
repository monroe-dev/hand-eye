"""
@File    : control.py
@Author  : Hyunsoo Shin
@Date    : 20. 4. 16.
@Contact : hyunsoo.shin@outlook.com
"""

import remote_ur
import capture
import generate
import time
import math
print('Start!')

# -- Connect the robot --- #
# ur_ip = "192.168.0.10"
# ur_script_port = 30002
# ur = remote_ur.UniversalRobot(ur_ip, ur_script_port)

# --- Set frame size and chessboard
cam = capture.ChessBoard(0, 1920, 1280, 7, 9, 14)

# --- Video capture --- #
# cam.start()

# --- Camera Calibration --- #
# cam.calibrate(30, 500)

# ---Pose estimation(chessboard) --- #
# cam.estimatePose(False)

init_x = -0.1
init_y = -0.3
init_z = 0.4
init_rx = 0
init_ry = math.pi
init_rz = 0
p = generate.Pose(init_x, init_y, init_z, init_rx, init_ry, init_rz)
# robotPose = p.generate(1, 10, 60)
# p.savePose()

robotPose = p.loadPose()
num_of_poses = len(robotPose)

for i in range(num_of_poses):
    # ur.movel(robotPose[i][0], robotPose[i][1], robotPose[i][2], robotPose[i][3], robotPose[i][4], robotPose[i][5], 0.1, 0.1)
    # time.sleep(3)
    cam.save()
    time.sleep(0.1)
    print(i + 1,  "th image was captured.")

print('Finished.')
