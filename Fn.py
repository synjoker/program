

from ctypes import sizeof
import numpy as np

def Mono12toImg8(Mono12, IHeight, IWidth):
    Mat = np.reshape(Mono12, (-1, 2)).astype(np.int16)
    cell = np.array([[1],[256]])
    Img8 = np.matmul(Mat, cell)
    Img8 = np.reshape(Img8, (IHeight, IWidth))
    print(np.shape(Img8))
    print(Img8)
    return Img8

mono = np.ones(2048*3072*2,).astype(np.int8)
Mono12toImg8(mono, 2048, 3072)
