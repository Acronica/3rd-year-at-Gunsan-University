라인 트레이서 시뮬레이션
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;
int main() {
    //비디오 캡처
    string videoPath = "7_lt_ccw_100rpm_in.mp4";
    VideoCapture cap(videoPath);
    if (!cap.isOpened()) {
        cerr << "Error: Unable to open video file." << endl;
        return -1;
    }

    int64 startTick, endTick; // 시작 카운터 sys/time.h는 윈도우에 없음

    int pointX = cap.get(CAP_PROP_FRAME_WIDTH) / 2;
    int prevPointX = pointX;
    int error, vel1, vel2;
    int prevPointX1 = 0;
    int redFound = 1;
    int blueFound = 1;
    while (true) {
        startTick = getTickCount();

        Mat frame;
        cap >> frame;

        if (frame.empty()) {
            break;
        }

        //밝기 조정
        Scalar frameMean = mean(frame);
        double Bright = 100.0;
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

        // 라인 추적 부분
        for (int i = 1; i < numLabels; ++i) {
            // 현재 라벨의 통계 정보 가져오기
            int* stat = stats.ptr<int>(i);
            int x = stat[ConnectedComponentsTypes::CC_STAT_LEFT];
            int y = stat[ConnectedComponentsTypes::CC_STAT_TOP];
            int width = stat[ConnectedComponentsTypes::CC_STAT_WIDTH];
            int height = stat[ConnectedComponentsTypes::CC_STAT_HEIGHT];
            int area = stat[ConnectedComponentsTypes::CC_STAT_AREA];

            Scalar color;

            // 라벨의 면적이 500보다 큰 경우에만 처리
            if (area <= 500) continue;

            // 라벨의 중심 좌표 계산
            int rectCenterX = x + width / 2;
            int rectCenterY = y + height / 2;
            int pointx;
            // 라벨 중심과 이미지 중심 사이의 거리 계산
            int distance = sqrt(pow(rectCenterX - pointX, 2) + pow(rectCenterY - cp1.rows / 2, 2));

            // 현재 라벨이 이전까지의 최소 거리보다 더 가까우면 업데이트
            if (distance < minDistance) {
                minDistance = distance;
                closestLineIdx = i;
                prevPointX = pointX;
            }
            
            // 라벨이 일정 범위 안에 있는 경우 빨간색, 아니면 파란색으로 표시
            if (redFound == 1 && rectCenterX > prevPointX - 75  && rectCenterX < prevPointX + 75) {
                color = redRectangleFound ? Scalar(255, 0, 0) : Scalar(0, 0, 255);
                redRectangleFound = true;
                blueFound = 1;
            }
            else if (redFound == 1 && (rectCenterX > prevPointX + 200 || rectCenterX < prevPointX - 200)) {
                prevPointX1 = prevPointX;
                redFound = 0;
                blueFound = 0;
                color = Scalar(255, 0, 0);
            }
            else {
                color = Scalar(255, 0, 0);
                blueFound = 0;
                if (redFound == 0 && rectCenterX > prevPointX1 - 75 && rectCenterX < prevPointX1 + 75) {
                    color = Scalar(0, 0, 255);
                    redFound = 1;
                }
            }
            if (prevPointX1 > 400 || prevPointX1 < 50) {
                prevPointX1 = 320;
            }

            cout << "Current X: " << rectCenterX << ", Previous X: " << prevPointX << ", found:" << redFound << ", x1:" << prevPointX1 << endl;

            // 라벨에 사각형과 중심을 그림
            Point center(rectCenterX, rectCenterY);
            rectangle(cp1, Point(x, y), Point(x + width, y + height), color, 2);
            circle(cp1, center, 3, color, -1);
        }

        // 가장 가까운 라인의 정보를 이용하여 라인 중심 갱신
        if (closestLineIdx != -1) {
            int* selectedStat = stats.ptr<int>(closestLineIdx);
            int selectedX = selectedStat[ConnectedComponentsTypes::CC_STAT_LEFT];
            int selectedWidth = selectedStat[ConnectedComponentsTypes::CC_STAT_WIDTH];
            pointX = selectedX + selectedWidth / 2;

        }

        endTick = getTickCount();
        double elapsedTime = (endTick - startTick) / getTickFrequency();
        int base = 320;
        int speed = 50;
        error = base - pointX;
        if (blueFound == 0) {
            error = -error;
        }
        vel1 = speed - error / 3;
        vel2 = -(speed + error / 3);

        circle(cp1, Point(cp1.cols / 2, cp1.rows / 2), 5, Scalar(0, 255, 0), -1);

        cout << "error: " << error << ", l: " << vel1 << ", r: " << vel2 << ", time: " << elapsedTime << endl;
        imshow("Processed Video", cp1);

        if (waitKey(30) == 27) {
            break;
        }
    }
    destroyAllWindows();
    return 0;
}
