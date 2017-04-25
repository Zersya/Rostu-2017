#pragma once

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2\imgproc\imgproc.hpp"

#include "SFML\Network\TcpSocket.hpp"
#include "SFML\Network.hpp"

#include "SerialClass.h"

#include <iostream>
#include <stdio.h>
#include <cmath>

using namespace std;

//variable
int LOW[3];
int HIGH[3];
char* dir;
vector<vector<cv::Point>> contours;
vector<cv::Vec4i> hierarchy;
cv::Mat _ori;
cv::Mat _ori2;
cv::Point centerBall[3];
bool ball;
char* COM = "COM4";

bool writeres = false;
int writelen = 0;
const double PI = 3.14;
int radiusCircle[2] = { 0, 0 };
int locBola[2] = { 0,0 };


//function
cv::Mat processThreshold(cv::Mat _pros, cv::Scalar low, cv::Scalar high);
void trackBars();
void updateData(int, void*);
cv::Scalar inisiateScalarHighBola();
cv::Scalar inisiateScalarLowBola();
cv::Scalar inisiateScalarLowLapang();
cv::Scalar inisiateScalarHighLapang();
cv::Scalar inisiateScalarLowGaris();
cv::Scalar inisiateScalarHighGaris();
cv::Scalar inisiateScalarLowObstacle();
cv::Scalar inisiateScalarHighObstacle();
void erodeDilate(cv::Mat _pros);
int processFindContour(cv::Mat _ths, int id);
void drawing(int largeI, int id);
void circleRoi();
void getData(int angle);
char* intToChar(int data);
void cameraDepan();
void cameraAtas();
int getDirection();
void perintahKeRobot(string aksinya);
void sendToArduino(int writelen, bool writeres, Serial SP);
void DataForCoach();