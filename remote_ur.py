"""
@File    : remote_ur.py
@Author  : Hyunsoo Shin
@Date    : 20. 4. 16.
@Contact : hyunsoo.shin@outlook.com
"""
import socket
import struct
import binascii

ur_ip = "192.168.0.10"
ur_acc = 0.01
ur_vel = 0.01
ur_script_port = 30002


def init():
    URMove = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    URMove.connect((ur_ip, ur_script_port))
    print('Connect to UR')


def ur_get_pose():
    ur_port_pose = 30003
    URPose = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    URPose.connect((ur_ip, ur_port_pose))
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

def moveUR(x, y, z, rx, ry, rz):
    x, y, z, Rx, Ry, Rz = str(x), str(y), str(z), str(rx), str(ry), str(rz)
    move_msg = "movel(p[" + x + "," + y + "," + z + "," + rx + "," + ry + "," + rz + "],0.05,0.15,0,0)\n"
    URMove.send(move_msg.encode())