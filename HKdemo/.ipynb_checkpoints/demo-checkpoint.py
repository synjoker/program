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
 
def main():
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
 
    # nConnectionNum = input("please input the number of the device to connect:")
    nConnectionNum = 0
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
        if int(nPacketSize) > 0:
            ret = cam.MV_CC_SetIntValue("GevSCPSPacketSize", nPacketSize)
            if ret != 0:
                print("Warning: Set Packet Size fail! ret[0x%x]" % ret)
        else:
            print("Warning: Get Packet Size fail! ret[0x%x]" % nPacketSize)
 
    stBool = c_bool(False)
    ret = cam.MV_CC_GetBoolValue("AcquisitionFrameRateEnable", stBool)
    if ret != 0:
        print("get AcquisitionFrameRateEnable fail! ret[0x%x]" % ret)
        sys.exit()
 
    # ch:设置触发模式为off | en:Set trigger mode as off
    ret = cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
    if ret != 0:
        print("set trigger mode fail! ret[0x%x]" % ret)
        sys.exit()
 
    # ch:开始取流 | en:Start grab image
    ret = cam.MV_CC_StartGrabbing()
    if ret != 0:
        print("start grabbing fail! ret[0x%x]" % ret)
        sys.exit()
##############################这里获得照片
# ## 采集单张
#     src = GetImage(cam)
    cv2.namedWindow('1', cv2.WINDOW_NORMAL)
#     cv2.imshow("1",src)
#     # cv2.imwrite("1.jpg", src)
 
##实时显示
    keyValue = 0
    fra = []
    while keyValue != ord('q'):
        src = GetImage(cam)
        src_T = GetTemperaturePic(src)
        src_TC = PseudoColor(src_T)
        fra.append(np.max(src))
        mean = sum(fra)/len(fra)
        if(len(fra)>50):
            fra.remove(fra[0])
        cv2.putText(src, "maxgrayLevel="+str(np.max(src)), (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(src, "meangrayLevel="+str(int(mean)), (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(src, "temperature="+str(int(GetTemperature(mean))), (0, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow("1", cv2.resize(src, (900, 600)))
        
        cv2.putText(src_TC, "maxgrayLevel="+str(np.max(src_T)), (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow("2", cv2.resize(src_TC,(900,600)))
    
        # cv2.waitKey(10)  # time.sleep 延时没有用
        keyValue = cv2.waitKey(10)
    cv2.destroyAllWindows()
 
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

# 计算温度值
def GetTemperature(greylevel):
    bench = 241
    t1 = 1/2045.125
    t2 = 5.56e-5*math.log(241/greylevel,math.e)
    T = 1/(t1+t2)
    return T

# 全图温度计算，归一化，彩色显示
def GetTemperaturePic(greypic):
    bench = 241
    t1 = 1/2045.125
    t2 = 5.56e-5*np.log(241/greypic[:,:,0])
    T = 1/(t1+t2)
    # T_max = np.max(T)
    T_max = 2000 #额定温度上限
    T = T/T_max*255
    T = T.astype(np.uint8)
    return T

def PseudoColor(temperaturepic):

    return cv2.applyColorMap(cv2.convertScaleAbs(temperaturepic, alpha=1), cv2.COLORMAP_JET)

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
        temp = temp.reshape((2048, 3072))  #注意 这里要改成使用相机的宽和高
        temp = cv2.cvtColor(temp, cv2.COLOR_BayerBG2BGR)
        nRet = cam.MV_CC_FreeImageBuffer(stOutFrame)
        return temp
    else:
        print("no data[0x%x]" % ret)
 
if __name__ == "__main__":
    main()