import cv2
import numpy as np
import sys, select, os
import time
from dynamixel_sdk import *
from dynamixel_sdk import Protocol2PacketHandler as P2PH                    # Uses Dynamixel SDK library
from dynamixel_sdk import PortHandler
from dynamixel_sdk import PacketHandler

# Control table address
AX_TORQUE_ENABLE       = 24                          # Control table address is different in Dynamixel model
AX_GOAL_POSITION       = 30
AX_PRESENT_POSITION    = 36
AX_MOVING_SPEED        = 32
AX_MOVING              = 46
# Protocol version
PROTOCOL_VERSION            = 1                             # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 16     # 左後
DXL_ID2                     = 12    # 左前
DXL_ID3                     = 4    # 右後
DXL_ID4                     = 17    #右前
DXL_ID5                     = 9    # アーム
BAUDRATE                    = 1000000
DEVICENAME                  = "/dev/tty.usbserial-AL03ERS9"                # Check which port is being used on your controller
                                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 819                       # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1023

                       # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

ESC_ASCII_VALUE             = 0x1b

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

flag = 0


# Initialize PortHandler Structs
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)
cap = cv2.VideoCapture(2) #cameraの設定

def red_detect(img):
    # HSV色空間に変換
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 赤色のHSVの値域1
    hsv_min = np.array([0,127,0])
    hsv_max = np.array([30,255,255])
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

    # 赤色のHSVの値域2
    hsv_min = np.array([150,127,0])
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
            if cv2.waitKey(0) & 0xFF == ord('k'):
                if (img.shape[1]//2==center_x):
                    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED, 200)
                    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED, 200)
                    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED, 1223)
                    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED, 1223)
                #dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID5, AX_GOAL_POSITION, DXL_MINIMUM_POSITION_VALUE)
                #flag=0
                #ストップ
                    if cv2.waitKey(0) & 0xFF == ord('1'):
                        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED, 0)
                        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED, 0)
                        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED, 1024)
                        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED, 1024)
                        flag=0
                #右回り
                elif (img.shape[1]//2<center_x):
                    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED, 200)
                    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED, 200)
                    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED, 200)
                    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED, 200)
                    #flag=2

                    if 0<abs(img.shape[1]//2-center_x)<20:
                        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED, 0)
                        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED, 0)
                        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED, 1024)
                        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED, 1024)
                        flag=0

                    if cv2.waitKey(0) & 0xFF == ord('1'):
                        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED, 0)
                        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED, 0)
                        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED, 1024)
                        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED, 1024)
                        flag=0
              #左回り
                elif (img.shape[1]//2>center_x):
                    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED, 1223)
                    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED, 1223)
                    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED, 1223)
                    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED, 1223)
                    #123flag=3

                    if 0<abs(img.shape[1]//2-center_x)<50:
                        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED, 0)
                        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED, 0)
                        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED, 1024)
                        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED, 1024)
                        flag=0

                    if cv2.waitKey(0) & 0xFF == ord('1'):
                        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED, 0)
                        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED, 0)
                        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED, 1024)
                        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED, 1024)
                        flag=0
            elif flag == 0:
                print("a");
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED, 0)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED, 0)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED, 1024)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED, 1024)
                #dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID5, AX_GOAL_POSITION, DXL_MAXIMUM_POSITION_VALUE)
                #flag = 1

                if cv2.waitKey(0) & 0xFF == ord('6'):
                    flag=1

    def moveDXLARM(self):
        global flag
        # Read moving state
        dxl_moving_state, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID, AX_MOVING_SPEED)
        dxl_moving_state, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID2, AX_MOVING_SPEED)
        dxl_moving_state, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID3, AX_MOVING_SPEED)
        dxl_moving_state, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID4, AX_MOVING_SPEED)
        dxl_moving_state, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID5, AX_MOVING)
        # Write goal position
        if dxl_moving_state == 0:
            if flag == 1:
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID5, AX_GOAL_POSITION, DXL_MINIMUM_POSITION_VALUE)
                flag=0
                time.sleep(3)
            else:
                print("a");

                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID5, AX_GOAL_POSITION, DXL_MAXIMUM_POSITION_VALUE)
                flag = 1
                time.sleep(3)




################## Main Loop ################################################

if __name__ == "__main__":

    dx = DXL()

    try:
         while True:
             #　カメラの読み込み
             #ret, frame = cap.read()
             #img = frame
             #print("press ESC to quit!")
             #cv2.namedWindow('Camera', cv2.WINDOW_AUTOSIZE)
             #cv2.imshow('Camera',img)
             #if cv2.waitKey(1) & 0xff == 27:
                 #break

            dx.moveDXLARM()# dynamixelを動かすメソッド


##############################################################################

    finally:
        # Close port
        cap.release()
        cv2.destroyAllWindows()
        portHandler.closePort()
