# 双光融合主程序

import sys
import threading
import msvcrt
import cv2
import numpy as np
import time
import math
 
 
from ctypes import *

sys.path.append("../MvImport")
from MvCameraControl_class import *
 
g_bExit = False
# 图像翻转标志
ReverseX_YES = 1
ReverseX_NO = 0 

#-------------------------参数设置-----------------------------
global deviceList
global tlayerType
global bench #标定温度 和灰度
global benchtemperature
bench = 3500
benchtemperature = 2045.125
global Lamda1, Lambda2 # 双光使用的波长
Lambda1 = 800 # (nm)
Lambda2 = 850 # (nm)
global threshold # 计算温度时阈值灰度，小于此值进行处理
threshold = 50
#------------------------------------------------------


# 侦察设备并并开启设备 
def DetectDevice():
    #input:  NULL
    #output: deviceList  

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
        mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
            print("\ngige device: [%d]" % i)
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName:
                strModeName = strModeName + chr(per)
            print("device model name: %s" % strModeName)

            nip1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
            nip2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
            nip3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
            nip4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
            print("current ip: %d.%d.%d.%d\n" % (nip1, nip2, nip3, nip4))
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
def ConnectDevice(nConnectionNum, ReverseFlag):         
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
    stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents

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
        nPacketRate = 1410 # 实现带宽500MB
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
    nExposureTime = 100000
    ret = cam.MV_CC_SetFloatValue("ExposureTime", nExposureTime)
    nGain = 0
    ret = cam.MV_CC_SetFloatValue("Gain", nGain)

    # Y轴图像翻转   Node Name: ReverseX         Type: Boolean
    # 自动曝光      Node Name: ExposureAuto     Type: Enumeration
    # 自动增益      Node Name: GainAuto         Type: Enumeration
    # 软触发        Enum Entry Name: Software   Enum Entry Value: 7
    reverseBool = c_bool(ReverseFlag)
    ret = cam.MV_CC_SetBoolValue("ReverseX", reverseBool)
    nTriggerValue = 7 # 软触发
    cam.MV_CC_SetEnumValue("software", nTriggerValue)
    cam.MV_CC_SetEnumValueByString("GainAuto", "Off")
    cam.MV_CC_SetEnumValueByString("ExposureAuto", "Off")

    # 设置width和height 以及layoutx y来调节图像大小，进而实现图像的帧率
    # width 1832 height 1500 layoutx 700 layouty 400
    nWidth  = 1832
    nHeight = 1500
    offsetX = 700
    offsetY = 400
    cam.MV_CC_SetIntValue("Width", nWidth)
    cam.MV_CC_SetIntValue("Height", nHeight)
    cam.MV_CC_SetIntValue("OffsetX", offsetX)
    cam.MV_CC_SetIntValue("OffsetY", offsetY)

    # ch:设置触发模式为off | en:Set trigger mode as off
    ret = cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
    if ret != 0:
        print("set trigger mode fail! ret[0x%x]" % ret)
        sys.exit()

    # ch:设置图像像素 | en:Set the pixelFormat of the image
    ret = cam.MV_CC_SetEnumValue("PixelFormat", PixelType_Gvsp_Mono12)
    if ret != 0: 
        print ("set PixelFormat fail! nRet [0x%x]" % ret)
        sys.exit()

    return cam

# 获取设备状态信息
def GetCameraParameters(cam):
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
def CloseDevice(cam):
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
    
# ***************************************************************************************************************    
# 各项功能的函数
# ***************************************************************************************************************   

# 计算温度值
def GetTemperature(greylevel):
    t1 = 1/benchtemperature
    t2 = 5.56e-5*math.log(bench/greylevel,math.e)
    T = 1/(t1+t2)
    return T

# 过滤极小值，避免算法处理中的偏差
def ThresholdProcess(pic):
    # input:    8位or16位numpy array数据
    # output:   处理过的 8位or16位numpy array数据
    # 值过小对于图像影响很大，需要算法上处理
    pic = pic.astype(np.float16)
    pic[np.where(pic < threshold)] = np.inf
    return pic

# 高斯滤波
def filterGuassian(pic):
    return 0

# 全图温度计算，归一化，彩色显示
def GetTemperaturePic(greypic):
    greypic =  ThresholdProcess(greypic)
    # 根据灰度值计算色彩
    t1 = 1/benchtemperature
    t2 = 5.56e-5*np.log(bench/greypic)
    T = 1/(t1+t2)
    # T_max = np.max(T)
    T_max = 3500 #额定温度上限
    T = T/T_max*255
    T = T.astype(np.uint8)
    # 返回8位色彩图
    return T

# 加入图例和温度曲线
def PseudoColor(temperaturepic):  
    return cv2.applyColorMap(cv2.convertScaleAbs(temperaturepic, alpha=1), cv2.COLORMAP_JET)

# 双光融合  ！！！！！！！！
def pyrometricfunc(img1, img2):
    # img为8位图像
    # parameters setting
    T = 300 # temperature(K)
    C1 = 3.7419e-16 # first planck's constant(W*m^2)
    C2 = 1.4388e-2 # second planck's constant(m*K)

    m2nm = 1e9 # m to nm

    G1 = img1 # grey level
    G2 = img2
    G2 = ThresholdProcess(G2)
    # S_lambda1 = 0.1911*G1 + 2.9188 # spectral sensitivity (Tungsten lamp factor)
    # S_lambda2 = 0.1911*G2 + 2.9188
    S_lambda1 = 1 # spectral sensitivity (Tungsten lamp factor)
    S_lambda2 = 1
    Epsilon_lambda1 = 1 # spectral emissivity 
    Epsilon_lambda2 = 1

    # 注意C2和lambda间的单位换算
    Uppart = C2*(1/Lambda2-1/Lambda1)*m2nm
    # ------------------------------------------这里后面处理
    Gnumber = np.log((G1)/(G2))
    Snumber = np.log(0.3815*Gnumber + 0.7914)
    Snumber = np.log(S_lambda2/S_lambda1)
    Gnumber[np.where(Gnumber > 0)] = -3.5
    Downpart = Gnumber + math.log(math.pow(Lambda1/Lambda2, 6),math.e) + Snumber # 这里为二维数据
    T = Uppart / Downpart
    
    # 做个预处理
    T = T.astype(np.float16)
    # T[np.where(T < 0)] = 0
    temperature1 = np.max(T)
    T = T/np.max(T)*255
    T = T.astype(np.uint8)
    # ------------------------------------------这里后面处理
    return T, temperature1

#  计算特征点提取&生成描述时间
def siftCam(image1, image2):
    start = time.time()
    sift = cv2.SIFT_create()
    #  使用SIFT查找关键点key points和描述符descriptors

    kp1, des1 = sift.detectAndCompute(image1, None)
    kp2, des2 = sift.detectAndCompute(image2, None)
    end = time.time()
    print("特征点提取&生成描述运行时间:%.2f秒"%(end-start))

    ratio = 0.85 #  如果最接近和次接近的比值大于一个既定的值

    #  计算匹配点匹配时间
    # start = time.time()

    #  K近邻算法求取在空间中距离最近的K个数据点，并将这些数据点归为一类
    matcher = cv2.BFMatcher()
    raw_matches = matcher.knnMatch(des1, des2, k = 2)
    good_matches = []
    for m1, m2 in raw_matches:
        #  如果最接近和次接近的比值大于一个既定的值，那么我们保留这个最接近的值，认为它和其匹配的点为good_match
        if m1.distance < ratio * m2.distance:
            good_matches.append([m1])
    # end = time.time()
    # print("匹配点匹配运行时间:%.2f秒"%(end-start))

    matches = cv2.drawMatchesKnn(image1, kp1, image2, kp2, good_matches, None, flags = 2)

    #  单应性矩阵有八个参数，每一个对应的像素点可以产生2个方程(x一个，y一个)，那么需要四个像素点就能解出单应性矩阵
    if len(good_matches) > 4:
        #  计算匹配时间
        # start    = time.time()
        ptsA    = np.float32([kp1[m[0].queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        ptsB    = np.float32([kp2[m[0].trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        ransacReprojThreshold    = 4
        #  单应性矩阵可以将一张图通过旋转、变换等方式与另一张图对齐
        H, status =cv2.findHomography(ptsA,ptsB,cv2.RANSAC,ransacReprojThreshold);
        # imgOut = cv2.warpPerspective(image2, H, (image1.shape[1],image1.shape[0]),flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP)
        # end = time.time()
        # print("匹配运行时间:%.2f秒"%(end-start))
        return H, status
    else:
        print ("siftcam failed!")
        sys.exit()

# ch:将缓存区mono12图像数据转换成16位图像，单字节用uint16表示
def Mono12toImg16(Mono12, IHeight, IWidth):
    Mat = np.reshape(Mono12, (-1, 2)).astype(np.int16)
    cell = np.array([[1],[256]])
    Img16 = np.matmul(Mat, cell)
    Img16 = np.reshape(Img16, (IHeight, IWidth))
    # print(np.shape(Img16))
    # print(Img16)
    return Img16

# ch:将16位图像映射到8位图像进行显示
def Img16toImg8(Img16):
    return (Img16/16).astype(np.uint8)

# 获取图像并预处理
def GetImage(cam):
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
        cdll.msvcrt.memcpy(byref(data_buf), stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nFrameLen)
        temp = np.asarray(data_buf)
        # 对缓存区的mono12图像数据进行处理
        nWidth = c_uint(0)
        nHeight = c_uint(0)
        cam.MV_CC_GetIntValue("Width", nWidth)
        cam.MV_CC_GetIntValue("Height", nHeight)

        OriginalData = Mono12toImg16(temp.astype(np.uint8), nHeight.value, nWidth.value)
        ImshowData = Img16toImg8(OriginalData)

        nRet = cam.MV_CC_FreeImageBuffer(stOutFrame)
        return OriginalData, ImshowData
    else:
        print ("get one frame fail, ret[0x%x]" % ret)

if __name__ == "__main__":
    deviceList = DetectDevice()
    print("OUT find %d device(s)" % deviceList.nDeviceNum)
    cam1 = ConnectDevice(0, ReverseX_NO)
    cam2 = ConnectDevice(1, ReverseX_YES) 
    # ch:开始取流 | en:Start grab image
    ret1 = cam1.MV_CC_StartGrabbing()
    ret2 = cam2.MV_CC_StartGrabbing()
    if ret1 | ret2 != 0:
        print("start grabbing fail! ret[0x%x]" % ret1)
        print("start grabbing fail! ret[0x%x]" % ret2)
        sys.exit()

    ##############################这里获得照片
    # ## 采集单张
    #     src = GetImage(cam)
    #     cv2.imshow("1",src)
    #     # cv2.imwrite("1.jpg", src)

    ##实时显示
    keyValue = 0
    fra1 = []
    fra2 = []
    FirstGetSiftPara = True
    while keyValue != ord('q'):
        
        # 从cam句柄获取12位原始数据和8位换算数据
        # 如果双光还要翻转图像
        originalsrc1, src1 = GetImage(cam1) # originalsrc为12位，src为8位
        originalsrc2, src2 = GetImage(cam2)
        # 可以考虑假如数字滤波，较小噪声

        

        # sift调试代码
        if FirstGetSiftPara | (keyValue == ord('f')):
            homographyMat, status = siftCam(src1, src2)
            FirstGetSiftPara = False
            keyValue = 0
        if (keyValue == ord('g')):
            nGain = 20
            ret = cam1.MV_CC_SetFloatValue("Gain", nGain)
            ret = cam2.MV_CC_SetFloatValue("Gain", nGain)
            print("set gain %d" % nGain)
        if (keyValue == ord('c')):
            nGain = 0
            ret = cam1.MV_CC_SetFloatValue("Gain", nGain)
            ret = cam2.MV_CC_SetFloatValue("Gain", nGain)
            print("set gain %d" % nGain)
        if (keyValue == ord('h')):
            nExposureTime = 100000
            ret = cam1.MV_CC_SetFloatValue("ExposureTime", nExposureTime)
            ret = cam2.MV_CC_SetFloatValue("ExposureTime", nExposureTime)
            print("set exposure %d" % nExposureTime)
        if (keyValue == ord('v')):
            nExposureTime = 50000
            ret = cam1.MV_CC_SetFloatValue("ExposureTime", nExposureTime)
            ret = cam2.MV_CC_SetFloatValue("ExposureTime", nExposureTime)
            print("set exposure %d" % nExposureTime)

        # start = time.time()
        HomoImage2 = cv2.warpPerspective(originalsrc2, homographyMat, (originalsrc1.shape[1],originalsrc1.shape[0]),flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP)
        # end = time.time()
        # print("图像转换:%.2f秒"%(end-start))
        imgOut = (HomoImage2*0.5 + originalsrc1 *0.5).astype(np.uint8)
        cv2.imshow("sifttest", cv2.resize(imgOut,(600,400)))

        
        
        # cv2.waitKey()
        # cv2.destroyAllWindows()

        # 图像配准，选用两幅原始图像进行灰度处理后配准

        
        # # 图像融合  
        # # 相机1单色光测温
        # src_T1 = GetTemperaturePic(originalsrc1) # 这个函数是针对8为定制的需要改善
        # src_TC1 = PseudoColor(src_T1) # 伪彩色图像
        # fra1.append(np.max(originalsrc1))
        # mean1 = sum(fra1)/len(fra1)
        # if(len(fra1)>50):
        #     fra1.remove(fra1[0])
        # cv2.putText(src1, "maxgrayLevel="+str(np.max(originalsrc1)), (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
        # cv2.putText(src1, "meangrayLevel="+str(int(mean1)), (0, 90), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
        # cv2.putText(src1, "temperature="+str(int(GetTemperature(mean1))), (0, 120), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
        # cv2.imshow("1", cv2.resize(src1, (900, 600)))
        # cv2.putText(src_TC1, "maxgrayLevel="+str(np.max(src_T1)), (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 2, (128,128,128), 2)
        # cv2.imshow("2", cv2.resize(src_TC1,(900,600)))
        # # 相机2单色光测温
        # src_T2 = GetTemperaturePic(originalsrc2) # 这个函数是针对8为定制的需要改善
        # src_TC2 = PseudoColor(src_T2) # 伪彩色图像
        # fra2.append(np.max(originalsrc2))
        # mean2 = sum(fra2)/len(fra2)
        # if(len(fra2)>50):
        #     fra2.remove(fra2[0])
        # cv2.putText(src2, "maxgrayLevel="+str(np.max(originalsrc2)), (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
        # cv2.putText(src2, "meangrayLevel="+str(int(mean2)), (0, 90), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
        # cv2.putText(src2, "temperature="+str(int(GetTemperature(mean2))), (0, 120), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
        # cv2.imshow("3", cv2.resize(src2, (900, 600)))
        # cv2.putText(src_TC2, "maxgrayLevel="+str(np.max(src_T2)), (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 2, (128,128,128), 2)
        # cv2.imshow("4", cv2.resize(src_TC2,(900,600)))
        
        # 相机1&2双光测温
        # # originalsrc2 = cv2.warpPerspective(originalsrc2, homographyMat, (originalsrc1.shape[1],originalsrc1.shape[0]),flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP)
        # originalsrc2 = cv2.warpPerspective(originalsrc2, homographyMat, (originalsrc1.shape[1],originalsrc1.shape[0]))
        temperature, maxtemperature = pyrometricfunc(src1, HomoImage2)
        temperature = PseudoColor(temperature)



        fra1.append(np.max(originalsrc1))
        mean1 = sum(fra1)/len(fra1)
        if(len(fra1)>50):
            fra1.remove(fra1[0])
        cv2.putText(src1, "maxgrayLevel="+str(np.max(originalsrc1)), (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
        cv2.putText(src1, "meangrayLevel="+str(int(mean1)), (0, 90), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
        
        fra2.append(np.max(originalsrc2))
        mean2 = sum(fra2)/len(fra2)
        if(len(fra2)>50):
            fra2.remove(fra2[0])
        cv2.putText(src2, "maxgrayLevel="+str(np.max(originalsrc2)), (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
        cv2.putText(src2, "meangrayLevel="+str(int(mean2)), (0, 90), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
        
        cv2.imshow("test1", cv2.resize(src1,(600,400)))
        cv2.imshow("test2", cv2.resize(src2,(600,400)))
        cv2.imshow("5", cv2.resize(temperature,(900,600)))
        print("maxtemperature: ", maxtemperature)
        # cv2.waitKey(10)  # time.sleep 延时没有用
        keyValue = cv2.waitKey(10)
    cv2.destroyAllWindows()
    CloseDevice(cam1)
    CloseDevice(cam2)