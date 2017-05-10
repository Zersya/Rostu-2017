#pragma once

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2\imgproc\imgproc.hpp"

#include "SFML\Network\TcpSocket.hpp"
#include "SFML\Network.hpp"

#include "SerialClass.h"
#include "PidController.h"

#include <iostream>
#include <stdio.h>
#include <cmath>
#include <thread>
#include <chrono>

using namespace std;

//variable
int LOW[3];
int HIGH[3];
char* dir = "";
char* dataKirimServer = "";
vector<vector<cv::Point>> contours[3];
vector<vector<cv::Point>> frontContours;
vector<vector<cv::Point>> topContours;
vector<cv::Vec4i> hierarchy[3];
cv::Mat _ori;
cv::Mat _ori2;
cv::Point centerBall[3];
bool ball = false;//ball camera atas
bool _ball = false;//ball camera depan
char* COMmega = "COM7";
char* COMnano = "COM5";
string statusGame = "";
string lastState = "";
sf::IpAddress ipServer = "172.16.0.100";
PidControllerRotation pidRotation;
PidControllerDestionation pidDestination;
PidControllerArahGawang pidArahGawang;
PidControllerOmniCamera pidOmniView;

bool writeres = false;
int writelen = 0;
const double PI = 3.14;
int radiusCircle[2] = { 0, 0 };
int locBola[2] = { 0,0 };
int kondisi = 0;
int dataHeading = 0;
int d; //mengambil nilai jauh bola dari camera atas
string dataReceivefromCoach[5];

int j = 0;

double pidJarakBola = 0;
double pidArahBola = 0;
double pidNgarahGawang = 0;
double pidOmniCamera = 0;

//motorRPM Jarak dan Rotasi
int motor[4] = { 0, 0, 0, 0 };
//motorRPM arahkanKegawangLawan
int motorArahGawang[4] = { 0, 0, 0, 0 };

//AUTOMATIC COLOR FILTER BY KYLE

bool calibrationMode;//used for showing debugging windows, trackbars etc.

bool mouseIsDragging;//used for showing a rectangle on screen as user clicks and drags mouse
bool mouseMove;
bool rectangleSelected;
cv::Point initialClickPoint, currentMousePoint; //keep track of initial point clicked and current position of mouse
cv::Rect rectangleROI; //this is the ROI that the user has selected
vector<int> H_ROI, S_ROI, V_ROI;// HSV values from the click/drag ROI region stored in separate vectors so that we can sort them easily


//function
void kejarBola();
void kondisiMotor();
cv::Mat processThreshold(cv::Mat _pros, cv::Scalar low, cv::Scalar high);
void trackBars();
void updateData(int, void*);
cv::Scalar inisiateScalarHighBola();
cv::Scalar inisiateScalarLowBola();
cv::Scalar inisiateScalarLowBolaDepan();
cv::Scalar inisiateScalarHighBolaDepan();
cv::Scalar inisiateScalarLowBolaGloss();
cv::Scalar inisiateScalarHighBolaGloss();
cv::Scalar inisiateScalarLowBolaFar();
cv::Scalar inisiateScalarHighBolaFar();
cv::Scalar inisiateScalarLowLapang();
cv::Scalar inisiateScalarHighLapang();
cv::Scalar inisiateScalarLowGaris();
cv::Scalar inisiateScalarHighGaris();
cv::Scalar inisiateScalarLowObstacle();
cv::Scalar inisiateScalarHighObstacle();
void erodeDilate(cv::Mat _pros);
int processFindContour_base(cv::Mat _ths1, cv::Mat _ths2, cv::Mat _ths3, int id, vector<vector<cv::Point>> cekContours);
int implementContour(vector<vector<cv::Point>> _contours, int id, int largestIdContours);
void drawing(int largeI, int id, vector<vector<cv::Point>> cekContours);
void circleRoi();
void getData(int m1, int m2, int m3, int m4, int kon1, int kon2, int kon3, int kon4);
char* intToChar(int data);
void cameraDepan();
void cameraAtas();
int getDirection();
void perintahKeRobot(string aksinya);
void dapatkanPerintahWasit();
void testThread();
void getDataHeading();
void clickAndDrag_Rectangle(int event, int x, int y, int flags, void* param);
void recordHSV_Values(cv::Mat frame, cv::Mat hsv_frame);
float euclideanDist(cv::Point p, cv::Point q);