#include "opencv2/opencv.hpp"
#include <iostream>
#include <cmath>
#include <ctime>
using namespace cv;
using namespace std;

// 벡터에 값을 추가하는 함수
void addValueToVector(vector<double>* vec, double value, int condition) {
    vec->insert(vec->begin(), value);
    if (condition)
        vec->push_back(value);
}

int main(void) {
    // 비디오 소스 열기
    VideoCapture videoSource("lanefollow_100rpm_ccw.mp4");
    VideoCapture cap(videoSource);

    if (!cap.isOpened()) {
        cerr << "Error: Unable to open video file." << endl;
        return -1;
    }

    Mat frame, labels, stats, centroids;
    vector<double> lane1X(2);    vector<double> lane1Y(2);
    vector<double> lane2X(2);    vector<double> lane2Y(2);

    int velocity1 = 0, velocity2 = 0, error = 0;
    float gain = 0.3;

    double currentMinDist1 = 0;    double currentMinDist2 = 0;
    double minDist1 = 0;    double minDist2 = 0;

    bool startProcessing = true;
    bool isFirstLane1 = true;
    bool isFirstLane2 = true;

    Point2d befVec;

    while (true) {
        if (startProcessing) {
            minDist1 = 0;
            minDist2 = 0;
            currentMinDist1 = 0;
            currentMinDist2 = 0;

            // 비디오에서 프레임 읽기
            videoSource >> frame;

            if (frame.empty()) {
                cerr << "Frame is empty!" << endl;
                break;
            }

            // 관심 영역 추출
            Mat regionOfInterest = frame(Rect(0, frame.rows / 4 * 3, frame.cols, 90));
            cvtColor(regionOfInterest, regionOfInterest, COLOR_BGR2GRAY);
            regionOfInterest = regionOfInterest + (100 - mean(regionOfInterest)[0]);
            threshold(regionOfInterest, regionOfInterest, 150, 255, THRESH_BINARY);
            int componentCount = connectedComponentsWithStats(regionOfInterest, labels, stats, centroids);
            cvtColor(regionOfInterest, regionOfInterest, COLOR_GRAY2BGR);

            for (int i = 1; i < componentCount; i++) {
                // 각 컴포넌트의 중심과 통계 정보 얻기
                double* centroid = centroids.ptr<double>(i);
                int* statistics = stats.ptr<int>(i);

                // 각 컴포넌트를 사각형으로 표시 (파란색)
                rectangle(regionOfInterest, Rect(statistics[0], statistics[1], statistics[2], statistics[3]), Scalar(255, 0, 0));

                // 첫 번째 차선인지 확인하고 조건을 만족하면 벡터에 추가
                if (isFirstLane1 && (centroid[0] > 440 && centroid[1] > 20)) {
                    addValueToVector(&lane1X, centroid[0], 1);
                    addValueToVector(&lane1Y, centroid[1], 1);

                    // 첫 번째 차선을 찾았으므로 플래그와 최소 거리 초기화
                    isFirstLane1 = false;
                    currentMinDist1 = sqrt(pow(centroid[0] - regionOfInterest.cols / 2.0, 2) +
                        pow(centroid[1] - regionOfInterest.rows / 2.0, 2));
                }

                // 두 번째 차선인지 확인하고 조건을 만족하면 벡터에 추가
                if (isFirstLane2 && (centroid[0] < 200 && centroid[1] > 20)) {
                    addValueToVector(&lane2X, centroid[0], 1);
                    addValueToVector(&lane2Y, centroid[1], 1);

                    // 두 번째 차선을 찾았으므로 플래그와 최소 거리 초기화
                    isFirstLane2 = false;
                    currentMinDist2 = sqrt(pow(centroid[0] - regionOfInterest.cols / 2.0, 2) +
                        pow(centroid[1] - regionOfInterest.rows / 2.0, 2));
                }

                // 각 차선과 현재 컴포넌트 간의 거리 계산
                minDist1 = sqrt(pow(lane1X.back() - centroid[0], 2) + pow(lane1Y.back() - centroid[1], 2));
                minDist2 = sqrt(pow(lane2X.back() - centroid[0], 2) + pow(lane2Y.back() - centroid[1], 2));

                // 처음 컴포넌트일 때 초기 거리 값 설정
                if (i == 1)
                    currentMinDist1 = minDist1, currentMinDist2 = minDist2;

                // 첫 번째 차선에 대한 조건을 만족하면 현재 컴포넌트를 벡터에 추가하고 표시 (빨간색)
                if ((abs(lane1X.back() - centroid[0]) <= 70 && abs(lane1Y.back() - centroid[1] <= 60)) &&
                    (minDist1 <= currentMinDist1)) {
                    currentMinDist1 = minDist1;
                    addValueToVector(&lane1X, centroid[0], 0);
                    addValueToVector(&lane1Y, centroid[1], 0);
                    rectangle(regionOfInterest, Rect(statistics[0], statistics[1], statistics[2], statistics[3]),
                        Scalar(0, 0, 255));
                }

                // 두 번째 차선에 대한 조건을 만족하면 현재 컴포넌트를 벡터에 추가하고 표시 (보라색)
                if ((abs(lane2X.back() - centroid[0]) <= 70 && abs(lane2Y.back() - centroid[1] <= 60)) &&
                    (minDist2 <= currentMinDist2)) {
                    currentMinDist2 = minDist2;
                    addValueToVector(&lane2X, centroid[0], 0);
                    addValueToVector(&lane2Y, centroid[1], 0);
                    rectangle(regionOfInterest, Rect(statistics[0], statistics[1], statistics[2], statistics[3]),
                        Scalar(255, 0, 255));
                }
            }

            // 차선 중점과 오차 계산
            error = (regionOfInterest.cols / 2.0) - ((lane1X.front() + lane2X.front()) / 2.0);
            circle(regionOfInterest, Point2d(lane1X.front(), lane1Y.front()), 4, Scalar(0, 0, 255), -1);
            circle(regionOfInterest, Point2d(lane2X.front(), lane2Y.front()), 4, Scalar(0, 0, 255), -1);

            // 속도 계산
            velocity1 = 30 - gain * error;
            velocity2 = -(30 + gain * error);

            // 벡터에 현재 값을 추가 (순환을 위해)
            lane1X.push_back(lane1X.front());
            lane1Y.push_back(lane1Y.front());
            lane2X.push_back(lane2X.front());
            lane2Y.push_back(lane2Y.front());

            // 결과 출력
            cout << "Error: " << error << ", Vel1: " << velocity1 << ", Vel2: " << velocity2 << endl;
            imshow("Region of Interest", regionOfInterest);
            imshow("Frame", frame);

            waitKey(30);
        }
    }
    return 0;
}
