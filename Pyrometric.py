import math
import cv2 as cv

# CCD read module
# def read_CCD()

img1 = cv.imread("pic\808LD+1.4A+NBP800-10.tif")
img2 = cv.imread("pic\808LD+1.4A+LBP850.tif")


# parameters setting
T = 300 # temperature(K)
C1 = 3.7419e-16 # first planck's constant(W*m^2)
C2 = 1.4388e-2 # second planck's constant(m*K)
Lambda1 = 800 # (nm)
Lambda2 = 850 # (nm)
m2nm = 1e9 # m to nm

G1 = 0 # grey level
G2 = 0
S_lambda1 = 0.1911*G1 + 2.9188 # spectral sensitivity (Tungsten lamp factor)
S_lambda2 = 0.1911*G2 + 2.9188
Epsilon_lambda1 = 1 # spectral emissivity 
Epsilon_lambda2 = 1

# 注意C2和lambda间的单位换算

Uppart = C2*(1/Lambda2-1/Lambda1)
Downpart = math.log(G1/G2,math.e) # 这里为二维数据
+ math.log(S_lambda2/S_lambda1,math.e)
+ math.log(Epsilon_lambda2/Epsilon_lambda1,math.e)
+ math.log(math.pow(Lambda1/Lambda2, 5),math.e)

T = Uppart / Downpart

# 显示模块
# def fake_color_display(input: T(2-D))