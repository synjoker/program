def pyrometricfunc(img1, img2):
    import math
    import cv2 as cv
    import numpy as np
    # parameters setting
    T = 300 # temperature(K)
    C1 = 3.7419e-16 # first planck's constant(W*m^2)
    C2 = 1.4388e-2 # second planck's constant(m*K)
    Lambda1 = 800 # (nm)
    Lambda2 = 850 # (nm)
    m2nm = 1e9 # m to nm

    G1 = img1 # grey level
    G2 = img2
    S_lambda1 = 0.1911*G1 + 2.9188 # spectral sensitivity (Tungsten lamp factor)
    S_lambda2 = 0.1911*G2 + 2.9188
    Epsilon_lambda1 = 1 # spectral emissivity 
    Epsilon_lambda2 = 1

    # 注意C2和lambda间的单位换算
    Uppart = C2*(1/Lambda2-1/Lambda1)*m2nm
    Downpart = np.log((G1)/(G2+0.1)) + math.log(math.pow(Lambda1/Lambda2, 6),math.e) + np.log((S_lambda2)/(S_lambda1+0.1),math.e) # 这里为二维数据
    T = Uppart / Downpart
    return T