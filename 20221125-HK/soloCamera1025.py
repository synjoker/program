# 双光融合主程序

import sys
import threading
import msvcrt
import cv2
from cv2 import INTER_NEAREST
import numpy as np
import time
import math
import seaborn as sns
import matplotlib.pyplot as plt
import matplotlib
import datetime
import os
# from numba import jit, njit # 高性能程序加速(事实证明没什么用)

from ctypes import *

sys.path.append("../MvImport")
from HKclass.MvCameraControl_class import *

g_bExit = False
# 图像翻转标志
ReverseX_YES = 1
ReverseX_NO = 0

#-------------------------参数设置-----------------------------
global deviceList
global tlayerType
global bench  #标定温度 和灰度
global benchtemperature
bench = 3500
benchtemperature = 2045.125
global Lamda1, Lambda2  # 双光使用的波长
Lambda1 = 808  # (nm)
Lambda2 = 700  # (nm)
global threshold  # 计算温度时阈值灰度，小于此值进行处理
threshold = 0
#------------------------------------------------------

import functools
import time


def run_time(fn):

    @functools.wraps(fn)
    def wrapper(*args, **kw):
        start = time.time()
        res = fn(*args, **kw)
        print('%s 运行了 %f 秒' % (fn.__name__, time.time() - start))
        return res

    return wrapper


# 相机开发的基本函数
class HKCameraInitialization:

    def __init__(self) -> None:
        self.minET = 100 # us
        self.maxET = 1000000 # us
        self.GreyUp = 4000
        self.GreyDn = 200
        pass

    # 侦察设备并并开启设备
    def DetectDevice(self):
        #input:  NULL
        #output: deviceList
        # global Lambda1
        # global Lambda2
        # Lambda1 = 800 # (nm)
        # Lambda2 = 700 # (nm)
        deviceList = MV_CC_DEVICE_INFO_LIST()
        tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE

        # ch:枚举设备 | en:Enum device
        ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
        if ret != 0:
            print("enum devices fail! ret[0x%x]" % ret)
            sys.exit()

        if deviceList.nDeviceNum == 0:
            print("find no device!")
            sys.exit()

        print("Find %d devices!" % deviceList.nDeviceNum)

        for i in range(0, deviceList.nDeviceNum):
            mvcc_dev_info = cast(deviceList.pDeviceInfo[i],
                                 POINTER(MV_CC_DEVICE_INFO)).contents
            if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
                print("\ngige device: [%d]" % i)
                strModeName = ""
                for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName:
                    if per == 0:
                        break
                    strModeName = strModeName + chr(per)
                print("device model name: %s" % strModeName)

                nip1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp
                         & 0xff000000) >> 24)
                nip2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp
                         & 0x00ff0000) >> 16)
                nip3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp
                         & 0x0000ff00) >> 8)
                nip4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp
                        & 0x000000ff)
                print("current ip: %d.%d.%d.%d\n" % (nip1, nip2, nip3, nip4))
                strSerialNumber = ""
                for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chSerialNumber:
                    if per == 0:
                        break
                    strSerialNumber = strSerialNumber + chr(per)
                print("device model name: %s" % strModeName)
                print("serial number: %s" % strSerialNumber)

            elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
                print("\nu3v device: [%d]" % i)
                strModeName = ""
                for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName:
                    if per == 0:
                        break
                    strModeName = strModeName + chr(per)
                print("device model name: %s" % strModeName)

                strSerialNumber = ""
                for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
                    if per == 0:
                        break
                    strSerialNumber = strSerialNumber + chr(per)
                print("user serial number: %s" % strSerialNumber)
        return deviceList

    # 连接设备并设置参数
    def ConnectDevice(self, nConnectionNum, ReverseFlag):
        #input:  连接设备号码, ReverseFlag 双光融合时一台相机图像需要翻转的标志位
        #output: MvCamera实例Cam
        # 设置相机的各项参数

        # nConnectionNum = input("please input the number of the device to connect:")
        if int(nConnectionNum) >= deviceList.nDeviceNum:
            print("intput error!")
            sys.exit()

        # ch:创建相机实例 | en:Creat Camera Object
        cam = MvCamera()

        # ch:选择设备并创建句柄 | en:Select device and create handle
        stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)],
                            POINTER(MV_CC_DEVICE_INFO)).contents

        ret = cam.MV_CC_CreateHandle(stDeviceList)
        if ret != 0:
            print("create handle fail! ret[0x%x]" % ret)
            sys.exit()

        # ch:打开设备 | en:Open device
        ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            print("open device fail! ret[0x%x]" % ret)
            sys.exit()

        # ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if stDeviceList.nTLayerType == MV_GIGE_DEVICE:
            nPacketSize = cam.MV_CC_GetOptimalPacketSize()
            # nPacketRate = 1410  # 实现带宽500MB
            nPacketRate = 100
            if int(nPacketSize) > 0:
                # 设置两项参数实现带宽500MB
                ret = cam.MV_CC_SetIntValue("GevSCPSPacketSize", nPacketSize)
                ret = cam.MV_CC_SetIntValue("GevSCPD", nPacketRate)
                if ret != 0:
                    print("Warning: Set Packet Size fail! ret[0x%x]" % ret)
            else:
                print("Warning: Get Packet Size fail! ret[0x%x]" % nPacketSize)

        stBool = c_bool(False)
        ret = cam.MV_CC_GetBoolValue("AcquisitionFrameRateEnable", stBool)
        if ret != 0:
            print("get AcquisitionFrameRateEnable fail! ret[0x%x]" % ret)
            sys.exit()

        # ch:设置曝光时间
        # - 增益 Node Name: Gain Type: Float
        self.nExposureTime = 2500
        ret = cam.MV_CC_SetFloatValue("ExposureTime", self.nExposureTime)
        nGain = 0
        ret = cam.MV_CC_SetFloatValue("Gain", nGain)

        # Y轴图像翻转   Node Name: ReverseX         Type: Boolean
        # 自动曝光      Node Name: ExposureAuto     Type: Enumeration
        # 自动增益      Node Name: GainAuto         Type: Enumeration
        # 软触发        Enum Entry Name: Software   Enum Entry Value: 7
        reverseBool = c_bool(ReverseFlag)
        ret = cam.MV_CC_SetBoolValue("ReverseX", reverseBool)
        nTriggerValue = 7  # 软触发
        cam.MV_CC_SetEnumValue("software", nTriggerValue)
        cam.MV_CC_SetEnumValueByString("GainAuto", "Off")
        cam.MV_CC_SetEnumValueByString("ExposureAuto", "Off")

        # 设置width和height 以及layoutx y来调节图像大小，进而实现图像的帧率
        # width 1832 height 1500 layoutx 700 layouty 400
        self.nWidth = 1000
        self.nHeight = 1000
        self.offsetX = 700
        self.offsetY = 400
        # self.nWidth = 3072
        # self.nHeight = 2048
        # self.offsetX = 0
        # self.offsetY = 0
        cam.MV_CC_SetIntValue("Width", self.nWidth)
        cam.MV_CC_SetIntValue("Height", self.nHeight)
        cam.MV_CC_SetIntValue("OffsetX", self.offsetX)
        cam.MV_CC_SetIntValue("OffsetY", self.offsetY)

        # ch:设置触发模式为off | en:Set trigger mode as off
        ret = cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
        if ret != 0:
            print("set trigger mode fail! ret[0x%x]" % ret)
            sys.exit()

        # ch:设置图像像素 | en:Set the pixelFormat of the image
        ret = cam.MV_CC_SetEnumValue("PixelFormat", PixelType_Gvsp_Mono12)
        if ret != 0:
            print("set PixelFormat fail! nRet [0x%x]" % ret)
            sys.exit()

        return cam
    
    def AutoExposure(self, cam, picgrey):
        if picgrey < self.GreyDn:
            self.nExposureTime = self.nExposureTime * 2
            if self.nExposureTime > self.maxET:
                self.nExposureTime = self.nExposureTime / 2
            
        if picgrey > self.GreyUp:
            self.nExposureTime = self.nExposureTime / 2
            if self.nExposureTime > self.maxET:
                self.nExposureTime = self.nExposureTime * 2
        # print("ET is %d us" % self.nExposureTime)
        return self.SetExposure(cam, self.nExposureTime)
    
    def SetExposure(self, cam, _nExposureTime):
        self.nExposureTime = _nExposureTime
        return cam.MV_CC_SetFloatValue("ExposureTime", self.nExposureTime)


    # 自动校正两个相机的连接
    def CorrectDevice(self, deviceList):
        # 判断并保证相机输出
        mvcc_dev_info = cast(deviceList.pDeviceInfo[0],
                             POINTER(MV_CC_DEVICE_INFO)).contents
        strSerialNumber = ""
        for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chSerialNumber:
            if per == 0:
                break
            strSerialNumber = strSerialNumber + chr(per)
        print("serial number: %s" % strSerialNumber)
        # 00J76177348为800
        # 00J76177577为700
        # 这样修改让cam1为800nm cam2为700nm
        if strSerialNumber == '00J76177348':
            cam1 = self.ConnectDevice(0, ReverseX_NO)
            cam2 = self.ConnectDevice(1, ReverseX_YES)
        elif strSerialNumber == '00J76177577':
            cam1 = self.ConnectDevice(1, ReverseX_NO)
            cam2 = self.ConnectDevice(0, ReverseX_YES)

        # ch:开始取流 | en:Start grab image
        ret1 = cam1.MV_CC_StartGrabbing()
        ret2 = cam2.MV_CC_StartGrabbing()
        if ret1 | ret2 != 0:
            print("start grabbing fail! ret[0x%x]" % ret1)
            print("start grabbing fail! ret[0x%x]" % ret2)
            sys.exit()
        return cam1, cam2

    # 获取设备状态信息
    def GetCameraParameters(self, cam):
        # # 获取带宽、曝光、像素格式、帧率、最大值、最小值、触发模式、增益、单张图片大小等等
        # nWidth = c_uint(0)
        # nHeight = c_uint(0)
        # cam.MV_CC_GetIntValue("Width", nWidth)
        # cam.MV_CC_GetIntValue("Height", nHeight)

        # # ch:获取图像像素 | en:Get the pixelFormat of the image
        # stEnumValue = MVCC_ENUMVALUE()
        # memset(byref(stEnumValue), 0 ,sizeof(MVCC_ENUMVALUE))
        # ret = cam.MV_CC_GetEnumValue("PixelFormat", stEnumValue)
        # if ret != 0:
        #     print ("get PixelFormat fail! nRet [0x%x]" % ret)
        #     sys.exit()
        # if stEnumValue.nCurValue == PixelType_Gvsp_Mono12:
        #     print("set PixelFormat succeed!")

        # # ch:打开属性配置GUI | en:Open Parameter Configuration GUI
        # nRet = cam.MV_CC_OpenParamsGUI();
        # if ret != 0:
        #     printf("Open Parameters Configuration GUI fail! nRet [0x%x]\n", nRet);
        #     sys.exit()
        # print("Press a key to close camera.\n");
        # msvcrt.getch()

        return 0

    # 关闭设备
    def CloseDevice(self, cam):
        ##############################

        # try:
        #     hThreadHandle = threading.Thread(target=work_thread, args=(cam, None, None))
        #     hThreadHandle.start()
        # except:
        #     print ("error: unable to start thread")
        # g_bExit = True
        # hThreadHandle.join()

        # ch:停止取流 | en:Stop grab image
        ret = cam.MV_CC_StopGrabbing()
        if ret != 0:
            print("stop grabbing fail! ret[0x%x]" % ret)
            sys.exit()

        # ch:关闭设备 | Close device
        ret = cam.MV_CC_CloseDevice()
        if ret != 0:
            print("close deivce fail! ret[0x%x]" % ret)
            sys.exit()

        # ch:销毁句柄 | Destroy handle
        ret = cam.MV_CC_DestroyHandle()
        if ret != 0:
            print("destroy handle fail! ret[0x%x]" % ret)
            sys.exit()

        keyValue = cv2.waitKey()
        cv2.destroyAllWindows()

    # 获取图像并预处理
    def GetImage(self, cam):
        sec = 0
        data_buf = None
        stOutFrame = MV_FRAME_OUT()
        memset(byref(stOutFrame), 0, sizeof(stOutFrame))
        ret = cam.MV_CC_GetImageBuffer(stOutFrame, 1000)
        if None != stOutFrame.pBufAddr and 0 == ret:
            if data_buf == None:
                data_buf = (c_ubyte * stOutFrame.stFrameInfo.nFrameLen)()
            # print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
            #     stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum))
            cdll.msvcrt.memcpy(byref(data_buf), stOutFrame.pBufAddr,
                               stOutFrame.stFrameInfo.nFrameLen)
            temp = np.asarray(data_buf)
            # 对缓存区的mono12图像数据进行处理
            nWidth = c_uint(0)
            nHeight = c_uint(0)
            cam.MV_CC_GetIntValue("Width", nWidth)
            cam.MV_CC_GetIntValue("Height", nHeight)

            OriginalData = self.Mono12toImg16(temp.astype(np.uint8),
                                              nHeight.value, nWidth.value)
            ImshowData = self.Img16toImg8(OriginalData)

            nRet = cam.MV_CC_FreeImageBuffer(stOutFrame)
            return OriginalData, ImshowData
        else:
            print("get one frame fail, ret[0x%x]" % ret)

    # ch:将缓存区mono12图像数据转换成16位图像，单字节用uint16表示
    def Mono12toImg16(self, Mono12, IHeight, IWidth):
        Mat = np.reshape(Mono12, (-1, 2)).astype(np.uint16)
        cell = np.array([[1], [256]])
        Img16 = np.matmul(Mat, cell)
        Img16 = np.reshape(Img16, (IHeight, IWidth))
        # print(np.shape(Img16))
        # print(Img16)
        return Img16

    # ch:将16位图像映射到8位图像进行显示
    def Img16toImg8(self, Img16):
        # return (Img16/16).astype(np.uint8)
        return cv2.normalize(Img16,
                             None,
                             0,
                             255,
                             cv2.NORM_MINMAX,
                             dtype=cv2.CV_8U)

    @run_time
    def CameraOperation(self, cam1, fra1, img1, num, temFlag):  # num用于命名imshow窗口
        self.int16src1, src1 = HKcam.GetImage(cam1)  # int16src为12位，src为8位 # 800

        # 图像平均化
        img1.append(self.int16src1)
        if (len(img1) > 1):
            img1.remove(img1[0])
        img11 = sum(img1) / len(img1)

        fra1.append(np.max(img11))
        if (len(fra1) > 1):
            fra1.remove(fra1[0])
        mean1 = sum(fra1) / len(fra1)

        # self.AutoExposure(cam1, int(np.max(self.int16src1)))

        # 12位图像数据8位化
        self.int8src1 = cv2.normalize(
            self.int16src1, None, 0, 255, cv2.NORM_MINMAX,
            dtype=cv2.CV_8U)  #把最大值放到255(eg:11-23放到0-255，因此比较明显)
        # 观察窗口
        int16xmin = 500
        int16ymin = 450
        int16xmax = 550
        int16ymax = 500
        int16window = self.int16src1[int16ymin:int16ymax, int16xmin:int16xmax] # cv2 和矩阵间的转换
        self.int8src1[int16ymin:int16ymax, int16xmin:int16xmax] = 0
        cv2.putText(self.int8src1,
                    "maxpoint=" + str(int(np.max(self.int16src1))), (0, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(self.int8src1, "meanmaxtemperature: " + str(int(mean1)),
                    (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(self.int8src1, "mean grey: " + str(int(np.mean(int16window))),
                    (0, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(self.int8src1, "ET is %s us" % str(int(self.nExposureTime)),
                    (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.rectangle(self.int8src1, (int16xmin, int16ymin), (int16xmax, int16ymax), (0,0,255), 2)         
        picname1 = "origin" + str(num)
        picname2 = "temperature" + str(num)
        cv2.namedWindow(picname1, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.imshow(picname1, self.int8src1)
        # cv2.imshow("temperature1", np.uint8(img11))
        # # 彩色图&等温线
        # amplify = 10
        if temFlag is False:
            return self.int8src1, self.int16src1
        if temFlag is True:
            src_T16_1, src_T_1, src_maxT1, src_minT1 = GetTemperaturePic(
                self.int16src1.astype(np.uint16), num)  # 这个函数是针对8为定制的需要改善
            cv2.imshow(picname2, src_T_1)

            # # # 制作等温线
            # # dx = 0.01
            # # dy = 0.01
            # # x=np.arange(-5.0,8.0,dx)
            # # y=np.arange(-5.0,5.0,dy)
            # # X,Y=np.meshgrid(x,y)

            # # 等温线
            # # plt.figure(picname2)
            # # plt.clf()
            # # C=plt.contour(X,Y,src_T16_1,[1710,1740, 1770,  1800,  1830,  1860,  1890],colors='black')  #生成等值线图
            # # plt.contourf(X,Y,src_T16_1,[1710, 1740, 1770,  1800,  1830,  1860,  1890])
            # # plt.colorbar()
            # # plt.clabel(C,inline=0.01,fontsize=2)
            # # print(np.max(src_T16_1))

            # 热彩色图
            plt.figure(picname2)
            plt.clf()
            sns.heatmap(src_T16_1,
                        linewidths=0,
                        vmax=980,
                        vmin=940,
                        cmap='jet')
            plt.title('800nm')  # 图像题目
            plt.axis('off')
            plt.xticks(rotation=90)

            # # plt.pause(0.01)
            # # keyValue = cv2.waitKey(1)

            # # print("图像1", src_maxT1, src_minT1)

            return self.int8src1, self.int16src1, src_T16_1

    def SaveIMG(self, tem1, picname):
        #打印时间戳保存
        timeNow = datetime.datetime.now().strftime("%Y-%m-%d")
        timeNow1 = datetime.datetime.now().strftime("%H-%M-%S-")
        if not os.path.isdir("20221125-HK/" + timeNow):
            os.makedirs("20221125-HK/" + timeNow)
        img16 = np.uint16(tem1)
        cv2.imwrite(
            "20221125-HK/" + timeNow + '/' + timeNow1 + picname + '.tif',
            img16)
        print("the %s image is refreshed" % picname)

        # # if input key 's', refresh compare image
        # if self.key == ord('s'):
        #     # 绘制magnitude图片
        #     print("mag.shape: ", np.shape(self.mag))
        #     timeNow = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        #     os.makedirs("save_" + timeNow)
        #     np.savetxt("save_" + timeNow + "/mag.txt", self.mag)
        #     np.savetxt("save_" + timeNow + "/angle.txt", self.ang)


# ***************************************************************************************************************
# 各项功能的函数
# ***************************************************************************************************************


# 计算温度值
def GetTemperature(greylevel):
    A = 3357.86668129799
    B = -0.346495384076732
    C = 14665.1652494004
    D = 668.451983297123
    T = (A - D) / (1 + np.power((greylevel / C), B)) + D
    return T


# 过滤极小值，避免算法处理中的偏差
def ThresholdProcess(pic, deviceFlag):
    # input:    8位or16位numpy array数据
    # output:   处理过的 8位or16位numpy array数据
    # 值过小对于图像影响很大，需要算法上处理
    pic = pic.astype(np.float16)  # 转化成float格式以防下面相减出现数值溢出
    # pic[np.where(pic <= threshold)] = np.inf
    if deviceFlag == 0:
        pic = pic - 22
    if deviceFlag == 1:
        pic = pic - 11
    pic[np.where(pic <= 0)] = 0
    return pic


# 高斯滤波
def filterGuassian(pic):
    return 0


def TemperatureNormalization(Pic, MaxT, MinT):
    _range = MaxT - MinT
    Tpic = (Pic - MinT) / _range
    Tpic[np.where(Tpic < 0)] = 0
    return Tpic


# def ABCD(greypic, A, B, C, D):
#     return T16 = (A - D)/(1 + np.power((greypic/C), B))+D


# 全图温度计算，归一化，彩色显示
# @run_time
def GetTemperaturePic(greypic, deviceFlag):
    # greypic =  ThresholdProcess(greypic, deviceFlag) # 用于两个相机底噪不一致
    # time_start = time.time()
    greypic = ThresholdProcess(greypic, 0)
    greypic = greypic.astype(np.uint64)
    # deviceFlag 为相机的序号，也就是不同相机的温标
    # print("thresholdprocess time:", time.time()-time_start)
    if deviceFlag == 1:
        #     A = 2683.29152774192
        #     B = -0.573096109328184
        #     C = 1469.52876963474
        #     D = 946.791773859162
        T16 = 846.88572 + np.power((greypic), 1) * 0.23588 + np.power(
            (greypic), 2) * (-1.04153E-4) + np.power(
                (greypic), 3) * 2.50137E-8 + np.power(
                    (greypic), 4) * (-2.2922E-12)
    if deviceFlag == 2:
        #     A = 6123.93764350186
        #     B = -0.251928479353211
        #     C = 314600.403422154
        #     D = 630.098415725083
        T16 = 865.16058 + np.power((greypic), 1) * 0.61838 + np.power(
            (greypic), 2) * (-7.59338E-4) + np.power(
                (greypic), 3) * 4.84034E-7 + np.power(
                    (greypic), 4) * (-1.15252E-10)
    # # print("selection:", time.time()-time_start)
    # T16 = (A - D)/(1 + np.power((greypic/C), B))+D
    # print("ABCD:", time.time()-time_start)
    T_max = (int(np.max(T16) / 100) + 1) * 100  #额定温度上限
    # T_min = (int(np.min(T16)/100)-1)*100
    T_min = (int(np.mean(T16) / 100) - 1) * 100
    # print("TMAXMIN:", time.time()-time_start)
    # # 方案1：传统测温归一化
    # T16[np.where(T16 <= 1000)] = 0
    # T = T16/T_max*255
    # 用平均数作为最小值！！
    # 方案2：新归一化 注意和cv2.normalize是不一样的
    T = TemperatureNormalization(T16, T_max, T_min) * 255
    # print("normalizaiton:", time.time()-time_start)

    T = T.astype(np.uint8)
    # print("typeconvert:", time.time()-time_start)
    # 返回8位色彩图
    return T16, T, T_max, T_min


# 加入图例和温度曲线
def PseudoColor(temperaturepic):
    return cv2.applyColorMap(cv2.convertScaleAbs(temperaturepic, alpha=1),
                             cv2.COLORMAP_HOT)


if __name__ == "__main__":
    HKcam = HKCameraInitialization()
    deviceList = HKcam.DetectDevice()
    print("OUT find %d device(s)" % deviceList.nDeviceNum)
    cam1, cam2 = HKcam.CorrectDevice(deviceList)

    fra1 = []
    fra2 = []
    img1 = []
    img2 = []
    count = 0
    key = []
    import cv2
    import numpy as np
    import time
    import csv
    import os
    time_start = time.time()
    plt.ion()
    tem = 1000
    timeNow = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    if not os.path.exists('./20221125-HK/CameraGreyOutput2'): #判断所在目录下是否有该文件名的文件夹
        os.makedirs("./20221125-HK/CameraGreyOutput2")
    with open('./20221125-HK/CameraGreyOutput2/Output_%s+%s+%sus.csv'%(timeNow, str(tem), str(HKcam.nExposureTime)),'w',newline='')as csv_file:
        writer=csv.writer(csv_file)
        # writerow 写入一行数据
        writeDatum = ["Time"]
        for i in range(500, 550):
                for j in range(500, 550):
                    writeDatum.append('(%s, %s)'%(str(i), str(j)))
        writer.writerow(writeDatum)
        while key != ord('q'):
            # 从cam句柄获取12位原始数据和8位换算数据
            # 如果双光还要翻转图像

            count += 1

            # int8src1, int16src1 = HKcam.CameraOperation(
            #     cam1, fra1, img1, 1, False)
            int8src2, int16src2 = HKcam.CameraOperation(
                cam2, fra2, img2, 2, False)
            
            timeNow1 = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            # max min min variance 
            # 获取一个csv对象进行内容写入
            writer=csv.writer(csv_file)
            writeDatum = [timeNow1]
            # writerow 写入一行数据
            for i in range(500, 550):
                for j in range(500, 550):
                    writeDatum.append(str(int(int16src2[i,j])))
            writer.writerow(writeDatum)        

            # # 计算帧数
            # time_sum = time.time() - time_start
            # print(count/time_sum)
            key = cv2.waitKey(1)
            if key == ord('r'):  # if input key 'r', refresh compare image
                # HKcam.SaveIMG(int8src1, "int8src1")
                # HKcam.SaveIMG(int16src1, "int16src1")
                
                HKcam.SaveIMG(int8src2, "int8src2+%d+%dus"%(tem, HKcam.nExposureTime))
                HKcam.SaveIMG(int16src2, "int16src2+%d+%dus"%(tem, HKcam.nExposureTime))

                # HKcam.SaveIMG(src_T16_1, "src_T16_1")
                # HKcam.SaveIMG(src_T16_2, "src_T16_2")
        plt.ioff()
        plt.show()
        cv2.destroyAllWindows()
        HKcam.CloseDevice(cam1)
        HKcam.CloseDevice(cam2)



