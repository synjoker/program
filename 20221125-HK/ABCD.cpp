#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <thread>
#include <mutex>
#include <unistd.h>
using namespace cv;
int main()
{
    Mat img=imread("../cut_image1.png");
    imshow("image",img);
    waitKey();
    return 0;
}
 