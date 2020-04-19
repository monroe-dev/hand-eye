"""
@File    : remote_ur.py
@Author  : Hyunsoo Shin
@Date    : 20. 4. 16.
@Contact : hyunsoo.shin@outlook.com
"""
import socket
import struct
import binascii

class UniversalRobot:
    def __init__(self, ur_ip, ur_port):
        self.URMove = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.URMove.connect((ur_ip, ur_port))
        print('Connect to UR')

    def ur_get_pose(self):
        ur_port_pose = 30003
        URPose = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        URPose.connect((self.ur_ip, ur_port_pose))
        s = URPose
        packet_1 = s.recv(4)
        packet_2 = s.recv(8)
        packet_3 = s.recv(48)
        packet_4 = s.recv(48)
        packet_5 = s.recv(48)
        packet_6 = s.recv(48)
        packet_7 = s.recv(48)
        packet_8 = s.recv(48)
        packet_9 = s.recv(48)
        packet_10 = s.recv(48)
        packet_11 = s.recv(48)

        packet_12 = s.recv(8)
        packet_12 = binascii.hexlify(packet_12)  # convert the data from \x hex notation to plain hex
        x = struct.unpack('!d', binascii.unhexlify(packet_12))[0]

        packet_13 = s.recv(8)
        packet_13 = binascii.hexlify(packet_13)
        y = struct.unpack('!d', binascii.unhexlify(packet_13))[0]

        packet_14 = s.recv(8)
        packet_14 = binascii.hexlify(packet_14)
        z = struct.unpack('!d', binascii.unhexlify(packet_14))[0]

        packet_15 = s.recv(8)
        packet_15 = binascii.hexlify(packet_15)
        Rx = struct.unpack('!d', binascii.unhexlify(packet_15))[0]

        packet_16 = s.recv(8)
        packet_16 = binascii.hexlify(packet_16)
        Ry = struct.unpack('!d', binascii.unhexlify(packet_16))[0]

        packet_17 = s.recv(8)
        packet_17 = binascii.hexlify(packet_17)
        Rz = struct.unpack('!d', binascii.unhexlify(packet_17))[0]

        URPose.close()
        return x, y, z, Rx, Ry, Rz

    def moveUR(self, x, y, z, rx, ry, rz, vel, acc):
        move_msg = "movel(p[" + str(x) + "," + str(y) + "," + str(z) + "," + str(rx) + "," + str(ry) + "," + str(rz) \
                   + "]," + str(acc) + "," + str(vel) + ",0,0)\n"
        print(move_msg)
        self.URMove.send(move_msg.encode())

