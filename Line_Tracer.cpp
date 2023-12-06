#include <iostream>
#include <unistd.h>
#include <sys/time.h> 
#include <signal.h>
#include "opencv2/opencv.hpp"
#include "dxl.hpp"
using namespace std; 
using namespace cv; 
bool ctrl_c_pressed = false;
bool mode =false;
void ctrlc_handler(int){ ctrl_c_pressed = true; } 
int main(void)
{
    string src = "nvarguscamerasrc sensor-id=0 ! \
        video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
        format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
        width=(int)640, height=(int)360, format=(string)BGRx ! \
        videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    VideoCapture source(src, CAP_GSTREAMER); 
    if (!source.isOpened()){ cout << "Camera error" << endl; return -1; }

    string dst1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \ 
        nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
        h264parse ! rtph264pay pt=96 ! \
        udpsink host=192.168.0.108 port=9001 sync=false";

    VideoWriter writer1(dst1, 0, (double)30, Size(640, 360), true);
    if (!writer1.isOpened()) { cerr << "Writer open failed!" << endl; return -1;} 

    string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \ 
        nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
        h264parse ! rtph264pay pt=96 ! \
        udpsink host=192.168.0.108 port=9002 sync=false";

    VideoWriter writer2(dst2, 0, (double)30, Size(640, 360), true);
    if (!writer2.isOpened()) { cerr << "Writer open failed!" << endl; return -1;} 
    
    Dxl mx;
    signal(SIGINT, ctrlc_handler);
    if(!mx.open()) { cout << "dynamixel open error"<<endl; return -1; }

    struct timeval start,end1; 
    double time1;
    int vel1 = 0,vel2 = 0, error;
    int pointX = source.get(CAP_PROP_FRAME_WIDTH) / 2;

    while(true) 
    {
        gettimeofday(&start,NULL);
        Mat frame;
        if (!source.read(frame)) {
            cerr << "Error reading frame from camera." << endl;
            break;
        }

        //밝기 조정
        double Bright = 100.0;
        Scalar frameMean = mean(frame);
        double currentBright = frameMean[0];
        double brightDif = Bright - currentBright;
        frame += Scalar(brightDif, brightDif, brightDif);

        //관심 영역
        Rect roiRect(0, 270, 640, 90);
        Mat roi = frame(roiRect).clone();
        cvtColor(roi, roi, COLOR_BGR2GRAY);

        //레이블링 영역
        Mat binary, cp1;
        threshold(roi, binary, 160, 255, THRESH_BINARY);
        cvtColor(binary, cp1, COLOR_GRAY2BGR);

        Mat labels, stats, centroids;
        int numLabels = connectedComponentsWithStats(binary, labels, stats, centroids);

        bool redRectangleFound = false;
        int closestLineIdx = -1;
        int minDistance = INT_MAX;

        //라인 추적
        for (int i = 1; i < numLabels; ++i) {
            int* stat = stats.ptr<int>(i);
            int x = stat[ConnectedComponentsTypes::CC_STAT_LEFT];
            int y = stat[ConnectedComponentsTypes::CC_STAT_TOP];
            int width = stat[ConnectedComponentsTypes::CC_STAT_WIDTH];
            int height = stat[ConnectedComponentsTypes::CC_STAT_HEIGHT];
            int area = stat[ConnectedComponentsTypes::CC_STAT_AREA];

            Scalar color;
            if (area > 500) {
                int centerX = cp1.cols / 2;
                int centerY = cp1.rows / 2;
                int rectCenterX = x + width / 2;
                int rectCenterY = y + height / 2;

                int distanceThresholdX = cp1.cols / 4;
                int distanceThresholdY = cp1.rows / 4;

                int distance = sqrt(pow(rectCenterX - pointX, 2) + pow(rectCenterY - centerY, 2));

                if (distance < minDistance) {
                    minDistance = distance;
                    closestLineIdx = i;
                }

                if (rectCenterX > centerX - distanceThresholdX && rectCenterX < centerX + distanceThresholdX &&
                    rectCenterY > centerY - distanceThresholdY && rectCenterY < centerY + distanceThresholdY) {
                    if (!redRectangleFound) {
                        color = Scalar(0, 0, 255);
                        redRectangleFound = true;
                    }
                    else {
                        color = Scalar(255, 0, 0);
                    }
                }
                else {
                    color = Scalar(255, 0, 0);
                }

                rectangle(cp1, Point(x, y), Point(x + width, y + height), color, 2);
                Point center(rectCenterX, rectCenterY);
                circle(cp1, center, 3, color, -1);
            }
        }
        // 처음 잡은 붉은 사각형의 중심
        if (closestLineIdx != -1) {
            int* selectedStat = stats.ptr<int>(closestLineIdx);
            int selectedX = selectedStat[ConnectedComponentsTypes::CC_STAT_LEFT];
            int selectedWidth = selectedStat[ConnectedComponentsTypes::CC_STAT_WIDTH];
            pointX = selectedX + selectedWidth / 2;
        }

        error = 320 - pointX;
        vel1 = 50 - error/3;
        vel2 = -(50 + error/3);

        writer1 << frame;
        writer2 << cp1;

        if (mx.kbhit()) //키보드입력 체크
        {
            char c = mx.getch();
            if(c == 'q') break;
            else if(c == 's') mode = true; 
        }
        if (ctrl_c_pressed) break; //Ctrl+c입력시 탈출
        if(mode) mx.setVelocity(vel1, vel2);
        usleep(20*1000); 

        gettimeofday(&end1,NULL); 
        time1 =end1.tv_sec-start.tv_sec+(end1.tv_usec-start.tv_usec)/1000000.0;
        cout <<"err: "<< error <<", vel1:"<< vel1 <<", vel2:"<< vel2 <<", time:"<< time1 << endl;

    }
    mx.close();
    return 0;
}
