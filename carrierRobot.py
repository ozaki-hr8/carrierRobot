import cv2
import numpy as np
import sys
import select
import os
import time
import serial
import re

# Uses Dynamixel SDK library
from dynamixel_sdk import *
from dynamixel_sdk import Protocol2PacketHandler as P2PH
from dynamixel_sdk import PortHandler
from dynamixel_sdk import PacketHandler

# Control table address
# Control table address is different in Dynamixel model
AX_TORQUE_ENABLE = 24
AX_GOAL_POSITION = 30
AX_PRESENT_POSITION = 36
AX_MOVING_SPEED = 32
AX_MOVING = 46
# Protocol version
# See which protocol version is used in the Dynamixel
PROTOCOL_VERSION = 1

# Default setting
DXL_ID = 16     # 左後
DXL_ID2 = 12    # 左前
DXL_ID3 = 4     # 右後
DXL_ID4 = 17    # 右前
DXL_ID5 = 9     # アーム
BAUDRATE = 1000000
# Check which port is being used on your controller
DEVICENAME = "/dev/tty.usbserial-AL03ERS9"
# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

TORQUE_ENABLE = 1                             # Value for enabling the torque
TORQUE_DISABLE = 0                            # Value for disabling the torque
# Dynamixel will rotate between this value
DXL_MINIMUM_POSITION_VALUE = 819
DXL_MAXIMUM_POSITION_VALUE = 1023

# and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

ESC_ASCII_VALUE = 0x1b

COMM_SUCCESS = 0                             # Communication Success result value
COMM_TX_FAIL = -1001                         # Communication Tx Failed

flag = 1
flag2 = 0
# Initialize PortHandler Structs
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)
cap = cv2.VideoCapture(2)

def red_detect(img):
    # HSV色空間に変換
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 赤色のHSVの値域1
    hsv_min = np.array([0,64,0])
    hsv_max = np.array([10,255,255])
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

    # 赤色のHSVの値域2
    hsv_min = np.array([170,64,0])
    hsv_max = np.array([179,255,255])
    mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

    return mask1 + mask2

def analysis_blob(binary_img):
    # 2値画像のラベリング処理
    label = cv2.connectedComponentsWithStats(binary_img)

    # ブロブ情報を項目別に抽出
    n = label[0] - 1
    data = np.delete(label[2], 0, 0)
    center = np.delete(label[3], 0, 0)

    # ブロブ面積最大のインデックス
    max_index = np.argmax(data[:, 4])

    # 面積最大ブロブの情報格納用
    maxblob = {}

    # 面積最大ブロブの各種情報を取得
    maxblob["upper_left"] = (data[:, 0][max_index], data[:, 1][max_index]) # 左上座標
    maxblob["width"] = data[:, 2][max_index]  # 幅
    maxblob["height"] = data[:, 3][max_index]  # 高さ
    maxblob["area"] = data[:, 4][max_index]   # 面積
    maxblob["center"] = center[max_index]  # 中心座標

    return maxblob

class DXL():

    def __init__(self):

        # Open port
        if portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            quit()

        # Set port baudrate
        if portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            quit()

        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, AX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")

    def moveDXL(self):
        global flag
        # Read moving state
        dxl_moving_state, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED)
        dxl_moving_state, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED)
        dxl_moving_state, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED)
        dxl_moving_state, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED)
        #dxl_moving_state, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID5, AX_MOVING)

        # Write goal position
        if dxl_moving_state == 0:
            if flag == 1:
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED, 200)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED, 200)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED, 1223)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED, 1223)
                #dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID5, AX_GOAL_POSITION, DXL_MINIMUM_POSITION_VALUE)

                time.sleep(0.5)

                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED, 0)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED, 0)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED, 1024)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED, 1024)

                #右回り
            elif flag ==2:

                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED, 200)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED, 200)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED, 200)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED, 200)
                #flag=2

                time.sleep(0.5)

                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED, 0)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED, 0)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED, 0)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED, 0)


              #左回り
            elif flag ==3:
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED, 1223)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED, 1223)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED, 1223)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED, 1223)

                time.sleep(0.5)

                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED, 1024)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED, 1024)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED, 1024)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED, 1024)

            elif flag ==4:
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED, 1223)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED, 1223)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED, 200)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED, 200)

    def moveDXLARM(self):
        global flag2
        # Read moving state
        dxl_moving_state, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED)
        dxl_moving_state, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED)
        dxl_moving_state, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED)
        dxl_moving_state, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED)
        dxl_moving_state, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID5, AX_MOVING)
        # Write goal position
        if dxl_moving_state == 0:
            if flag2 == 1:
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID5, AX_GOAL_POSITION, DXL_MINIMUM_POSITION_VALUE)
                flag2=0
                #time.sleep(3)
            else:
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID5, AX_GOAL_POSITION, DXL_MAXIMUM_POSITION_VALUE)
                flag2 = 1
                #time.sleep(3)
################## Main Loop ################################################

if __name__ == "__main__":

    dx = DXL()

    try:
        while True:
            #　カメラの読み込み
            _, img = cap.read()

            size = (img.shape[1]//2, img.shape[0]//2)
            img = cv2.resize(img, size)

            mask = red_detect(img)

            # マスク画像をブロブ解析（面積最大のブロブ情報を取得）
            target = analysis_blob(mask)

            # 面積最大ブロブの中心座標を取得
            center_x = int(target["center"][0])
            center_y = int(target["center"][1])

            # フレームに面積最大ブロブの中心周囲を円で描く
            cv2.circle(img, (center_x, center_y), 30, (0, 200, 0),
                       thickness=3, lineType=cv2.LINE_AA)

            """
            # 結果表示
            cv2.imshow("Frame", img)
            cv2.imshow("Mask", mask)
            """
            device_name = '/dev/tty.usbmodem143301'
            BaudRate = 115200
            ser = serial.Serial(port=device_name,
                                baudrate=BaudRate,
                                bytesize=serial.EIGHTBITS,
                                parity=serial.PARITY_NONE
                                )

            DATA_SIZE = 30
            disnum =0

            raw_value = ser.readline(DATA_SIZE)  #行末記号 '\n' まで読み込む ただしDATA_SIZEビットまで
            str_value = raw_value.decode('utf-8')
            disnum=int(re.sub(r'\D', '', str_value))
            print('disnum:',disnum)

            if 0<abs((img.shape[1]//2)-(center_x))<50:
                flag =1
                dx.moveDXL()
            if ((img.shape[1]//2)-(center_x))<-50:
                flag =2
                dx.moveDXL()
            if ((img.shape[1]//2)-(center_x))>50:
                flag =3
                dx.moveDXL()

            if disnum>150:
                flag2 =1
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED, 0)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED, 0)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED, 1024)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED, 1024)
                time.sleep(1)
                dx.moveDXLARM()
                time.sleep(2)
                flag=4
                dx.moveDXL()
                break
            else :
                flag2 =0

            print('flag:',flag,'flag2:',flag2)
            if cv2.waitKey(1) & 0xff == 27:
                break
##############################################################################

    finally:
        # Close port
        cap.release()
        cv2.destroyAllWindows()
        portHandler.closePort()
