#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;//包含cv命名空间
IplImage *g_pGrayImage = NULL;
IplImage *g_pBinaryImage = NULL;
const char *pstrWindowsBinaryTitle = "二值图(http://blog.csdn.net/MoreWindows)";

void on_trackbar(int pos) {
    // 转为二值图
    cvThreshold(g_pGrayImage, g_pBinaryImage, pos, 255, CV_THRESH_BINARY);
    // 显示二值图
    cvShowImage(pstrWindowsBinaryTitle, g_pBinaryImage);
}

float getMax(float a, float b) {
    if (a > b)
        return a;
    else
        return b;
}

float getMin(float a, float b) {
    if (a > b) {
        return b;
    } else return a;
}

float division(float a, float b) {
    if (a > b) {
        return b / a;
    }
    return a / b;
}

int main(int argc, char **argv) {
    /*const char *pstrWindowsSrcTitle = "原图(http://blog.csdn.net/MoreWindows)";
    const char *pstrWindowsToolBarName = "二值图阈值";

  // 从文件中加载原图
    IplImage *pSrcImage = cvLoadImage("/home/arch1tect/图片/test.png", CV_LOAD_IMAGE_UNCHANGED);

    // 转为灰度图
    g_pGrayImage =  cvCreateImage(cvGetSize(pSrcImage), IPL_DEPTH_8U, 1);
    cvCvtColor(pSrcImage, g_pGrayImage, CV_BGR2GRAY);

    // 创建二值图
    g_pBinaryImage = cvCreateImage(cvGetSize(g_pGrayImage), IPL_DEPTH_8U, 1);*/





/*    // 显示原图
    cvNamedWindow(pstrWindowsSrcTitle, CV_WINDOW_AUTOSIZE);
    cvShowImage(pstrWindowsSrcTitle, pSrcImage);
    // 创建二值图窗口
    cvNamedWindow(pstrWindowsBinaryTitle, CV_WINDOW_AUTOSIZE);*/

    // 滑动条
/*    int nThreshold = 0;
   cvCreateTrackbar(pstrWindowsToolBarName, pstrWindowsBinaryTitle, &nThreshold, 254, on_trackbar);

    on_trackbar(1);

    cvWaitKey(0);

    cvDestroyWindow(pstrWindowsSrcTitle);
    cvDestroyWindow(pstrWindowsBinaryTitle);
    cvReleaseImage(&pSrcImage);
    cvReleaseImage(&g_pGrayImage);
    cvReleaseImage(&g_pBinaryImage);*/

    // 读取视频流
    cv::VideoCapture capture("/home/arch1tect/桌面/vision/1.mov");
    // 检测视频是否读取成功
    if (!capture.isOpened()) {
        std::cout << "No Input Image";
        return 1;
    }

    // 获取图像帧率
    double rate = capture.get(CV_CAP_PROP_FPS);
    bool stop(false);
    cv::Mat frame; // 当前视频帧
    cv::namedWindow("Extracted Frame");

    // 每一帧之间的延迟
    int delay = 1000 / rate;

    // 遍历每一帧
    while (!stop) {
        // 尝试读取下一帧
        if (!capture.read(frame))
            break;
        //cvtColor(frame, frame, CV_BGR2GRAY);
        //threshold(frame, frame, 254, 255, CV_THRESH_BINARY);
        /*  for(int i =0;i<frame.rows;i++){
              Vec3b* data = frame.ptr<Vec3b>(i);
              for(int j=0;j<frame.cols;j++){
                  if(!(data[j][0]==255&&data[j][1]==255&&data[j][2]==255)){
                      data[j] = Vec3b(0,0,0);
                  }
              }
          }*/
        Mat temp = frame;
        Rect rect(temp.cols / 2, 0, temp.cols / 2, temp.rows);
        temp = temp(rect);
        cvtColor(temp, temp, CV_BGR2GRAY);
        threshold(temp, temp, 254, 255, CV_THRESH_BINARY);

        medianBlur(temp, temp, 5);
        std::vector<std::vector<Point>> contours;
        findContours(temp, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

        int size = contours.size();
        //标示识别到的灯条
        std::vector<RotatedRect> rotatedRects;
        std::vector<RotatedRect> vertical;//竖线
        std::vector<RotatedRect> horizon;//横线
        for (int i = 0; i < size; i++) {
            RotatedRect temp1 = minAreaRect(Mat(contours[i]));
            if ((temp1.angle < -45 && temp1.size.width > temp1.size.height &&
                 temp1.size.height * temp1.size.width >= 75 && temp1.size.width / temp1.size.height > 1.7)
                || (temp1.angle > -45 && temp1.size.width < temp1.size.height &&
                    temp1.size.height * temp1.size.width >= 75 && temp1.size.height / temp1.size.width > 1.7)) {
                vertical.push_back(temp1);
                rotatedRects.push_back(temp1);
                temp1.center.x += temp.cols;
                circle(frame, temp1.center, getMax(temp1.size.height, temp1.size.width) / 2, CV_RGB(100, 200, 50), 2, 8,
                       0);
            } else if (temp1.size.height * temp1.size.width >= 100 &&
                       (temp1.size.width / temp1.size.height > 2 || temp1.size.height / temp1.size.width > 2)) {
                horizon.push_back(temp1);
                rotatedRects.push_back(temp1);
                temp1.center.x += temp.cols;
                circle(frame, temp1.center, getMax(temp1.size.height, temp1.size.width) / 2, CV_RGB(255, 0, 255), 2, 8,
                       0);
            }

            // minRect.angle
        }
        size = rotatedRects.size();
        int min = 0;
        Point2f point2f;

        for (int i = 0; i < size; i++) {

            for (int j = 0; j < size; j++) {
                // Point2f point = (rotatedRects[i].center+rotatedRects[j].center)/2;
                if (j != i) {
                    float distance = sqrt(abs(vertical[i].center.x - vertical[j].center.x) *
                                          abs(vertical[i].center.x - vertical[j].center.x) +
                                          abs(vertical[i].center.y - vertical[j].center.y) *
                                          abs(vertical[i].center.y - vertical[j].center.y));
                    if (abs(vertical[i].center.y - vertical[j].center.y) <
                        (vertical[i].size.height + vertical[j].size.height) / 2
                        && ((getMax(vertical[i].size.width, vertical[i].size.height) +
                             getMax(vertical[j].size.width, vertical[j].size.height)) / 2 > 0.4 * distance
                            && (getMax(vertical[i].size.width, vertical[i].size.height) +
                                getMax(vertical[j].size.width, vertical[j].size.height)) / 2 < 0.6 * distance
                        )) {

                        float similar = division(getMax(vertical[i].size.height, vertical[i].size.width),
                                                 getMax(vertical[j].size.height, vertical[j].size.width))
                                        * division(getMin(vertical[i].size.height, vertical[i].size.width),
                                                   getMin(vertical[j].size.height, vertical[j].size.width));

                        if (similar > min) {
                            min = similar;
                            Point2f point2f1((vertical[i].center.x + vertical[j].center.x) / 2 + temp.cols,
                                             (vertical[i].center.y + vertical[j].center.y) / 2);
                            point2f = point2f1;
                        }
                    }
                }
            }
            circle(frame, point2f, 10, CV_RGB(0, 0, 255), 2, 8, 0);
        }

        //drawContours(frame,contours,-1,Scalar(0),2);

        imshow("Extracted Frame", frame);
        // 引入延迟
        if (cv::waitKey(delay) >= 0)
            stop = true;
    }
    return 0;
}