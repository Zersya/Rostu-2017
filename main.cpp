#include "main.h"

int main(int argc, char** argv) {
	
	//Serial
	Serial SP(COM);
	char dataReceive4Robot[100];

	std::size_t received4Robot;

	
	char* State = "";
	char* lastDir = "";

	//socket for communication processing base and robot
	sf::TcpSocket socketRobot;
	sf::Socket::Status statusRobot = socketRobot.connect(ipServer, 444);
	
	//cv::VideoCapture cap(0);
	cv::VideoCapture caps(0);

	//Multi-Thread
	std::thread thread4Server;
	thread4Server = std::thread(&dapatkanPerintahWasit);
	//trackBars();

	//if (!cap.isOpened())
	//	return 0;
	if (!caps.isOpened())
		return 0;

	if (statusRobot == sf::Socket::Done) {
		cout << "Server Connected" << endl;
	}
	if (SP.IsConnected()){
		cout << "Arduino Connected" << endl;
	}

	string aksinyaBuff = "";
	string aksinya = "";

	while (true) {
		//cap >> _ori;
		caps >> _ori2;

		//cameraAtas();
		cameraDepan();
		
		//Robot Kirim data ke Pelatih
		//char* rad = intToChar(radiusCircle[1]);
		char* rad = intToChar(centerBall[1].x/3.5);
		socketRobot.send(rad, strlen(rad));
		
		//Robot Terima data dari Pelatih
		socketRobot.receive(dataReceive4Robot, 3, received4Robot);
		
		 
		for (int i = 0; i < received4Robot; i++) {
			//cout << dataReceive[i] << endl;
			if (dataReceive4Robot[i] != '\n') {
				aksinyaBuff += dataReceive4Robot[i];
			}
			else {
				aksinya = aksinyaBuff;
				//cout << aksinya << endl;
				aksinyaBuff = "";
			}
		}
	
		perintahKeRobot(aksinya);
		//toArduino
		if (SP.IsConnected()) {
			writelen = strlen(dir);
			writeres = SP.WriteData(dir, writelen);
			//cout << lastDir << " , " << dir << endl;
			//readres = SP.ReadData(incomingData, datalen);
			//printf("%s", incomingData);
			//cout << dir << endl;
			
			delete[] dir;
		}
		else if (!writeres) {
			SP.ReConnect(COM);
		}
		
		cout << centerBall[1].x/3.5 << endl;

		//cv::imshow("Camera Atas", _ori);
		cv::imshow("Camera Depan", _ori2);
		if (cv::waitKey(30) > 0) break;

	}
	return 0;
}

void testThread() {
	while(true)cout << "Test Thread\n";
}

void dapatkanPerintahWasit() {

	//socket for communication refreebox base and robot
	sf::TcpSocket socketBaseRefree;
	sf::Socket::Status statusRefree = socketBaseRefree.connect(ipServer, 11306);

	char dataReceive4Game[100];
	std::size_t received4Game;

	if (statusRefree == sf::Socket::Done) {
		cout << "\nWasit Connected" << endl;
	}

	while (true) {

		//Robot Terima dari Wasit/Refree
		socketBaseRefree.receive(dataReceive4Game, 2, received4Game);

		string statusPerintahGame = "";

		for (int i = 0; i < received4Game; i++) {
			//cout << dataReceive[i] << endl;
			
			statusPerintahGame = dataReceive4Game[i];
		}

		statusGame = statusPerintahGame;
		//lastState = statusGame;
		cout << statusGame;
	}
}

void cameraAtas() {
	//bola
	locBola[0] = processFindContour(processThreshold(_ori, inisiateScalarLowBola(), inisiateScalarHighBola()), 1);
	//cout << locBola[0] << " , ";
	//lapangan
	 //processFindContour(processThreshold(_ori, inisiateScalarLowLapang(), inisiateScalarHighLapang()), 0);

	//garis
	cv::Mat destImg;
	cv::bitwise_not(processThreshold(_ori, inisiateScalarLowGaris(), inisiateScalarHighGaris()), destImg);
	processFindContour(destImg, 3);

	//obstacle
	processFindContour(processThreshold(_ori, inisiateScalarLowObstacle(), inisiateScalarHighObstacle()), 4);

	 //circle
	 circleRoi();
}

void cameraDepan() {
	//bola
	processFindContour(processThreshold(_ori2, inisiateScalarLowBola(), inisiateScalarHighBola()), 2);
	//cout << centerBall[1];
}

void perintahKeRobot(string aksinya){

	if (statusGame == "s") {
		if (aksinya == "RM") {
			getData(centerBall[1].x/3.5);
		}
		else if (aksinya == "RJ") {
			getData(centerBall[1].x/3.5);
		}
		else
			getData(90);
	}
	else if(statusGame == "S"){
		getData(90);
	}
	else {
		getData(0);
	}
}

void circleRoi() {
	int radius = 235;

	cv::Point center(_ori.cols / 2, _ori.rows / 2);
	cv::Rect r(center.x - radius, center.y - radius, radius * 2, radius * 2);
	cv::Mat roi(_ori, r);
	cv::Mat mask(roi.size(), roi.type(), cv::Scalar::all(0));

	
	cv::circle(mask, cv::Point(radius, radius), radius, cv::Scalar::all(255), -1);
	_ori = roi & mask;;
	cv::line(_ori, cv::Point(roi.cols / 2, 0), cv::Point(roi.cols / 2, roi.rows / 2), cv::Scalar(255, 0, 0), 2);
}

cv::Mat processThreshold(cv::Mat _pros, cv::Scalar low, cv::Scalar high) {
	cv::Mat hsv;
	cv::cvtColor(_pros, hsv, cv::COLOR_BGR2HSV);
	

	cv::inRange(_pros, low, high, _pros);
	erodeDilate(_pros);

	return _pros;
}

int processFindContour(cv::Mat _ths, int id) {

	cv::Mat tmp;

	//get more resource
	//_ths.copyTo(tmp);

	cv::findContours(_ths, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	
	int largestContourIndex, largestArea = 0;
	int rotAngle = 0;
	if (contours.size() > 0) {
		ball = true;
		for (int i = 0; i < contours.size(); i++) {
			
			double a = cv::contourArea(contours[i], false);
			if (a > largestArea) {
				largestArea = (int) a;
				largestContourIndex = i;
			}

		}
		if (id == 1 ) {
			drawing(largestContourIndex, id);
			rotAngle = getDirection();

			return rotAngle;
		}
		else if (id == 2) {
			drawing(largestContourIndex, id);
		}
		else if (id == 0)
			cv::drawContours(_ori, contours, -1, cv::Scalar(12,255,26), CV_FILLED);
		else if (id == 3)
			cv::drawContours(_ori, contours, -1, cv::Scalar(170,205,60), CV_FILLED);
		else if (id == 4)
			cv::drawContours(_ori, contours, -1, cv::Scalar(255, 255, 250), CV_FILLED);
	}
	else {
		ball = false;
		if(id == 2)
			centerBall[1] = cv::Point(0, 0);
		return 0;

	}
	
	contours.clear();
}

int getDirection() {

	int rotAngle, quadAngle, direction = 0;
	rotAngle = 0;

	cv::Point centerCamera(_ori.cols / 2, _ori.rows / 2);
	if (ball) {
		int derajat = 180;

		direction = derajat * atan2(centerCamera.y - centerBall[0].y, centerCamera.x - centerBall[0].x) / PI;
		quadAngle = derajat * atan2(centerBall[0].y, centerBall[0].x) / PI;

		rotAngle = -(quadAngle - direction) + 100;
		if (rotAngle < 0)
			rotAngle += 360;

		return rotAngle;
	}
	else
		return 0;
}

char* intToChar(int data) {
	string dataStr = to_string(data);
	char* rad = new char[dataStr.length() + 1];

	std::strcpy(rad, dataStr.c_str());

	return rad;

}

void getData(int prt) {
	string data = to_string(prt);
	data += "\n";

	dir = new char[data.length()];
	std::strcpy(dir, data.c_str());
}

void drawing(int largeI, int id) {

	cv::Mat _output;


	if (id == 1) {
		_output = _ori;
	}
	else if (id == 2) {
		_output = _ori2;
	}

	int cX, cY = 0;
	
	for (int i = 0; i < contours.size(); i++) {

		cv::Rect r = cv::boundingRect(contours[largeI]);
		cX = r.x + r.width / 2;
		cY = r.y + r.height / 2;
		
		switch (id) {
			case 1:
				centerBall[0] = cv::Point(cX, cY);
				radiusCircle[0] = r.height / 2;
				cv::circle(_output, cv::Point(cX, cY), radiusCircle[0], cv::Scalar::all(255), 1);
				break;
			case 2:
				centerBall[0] = cv::Point(cX, cY);
				radiusCircle[1] = r.height / 2;
				centerBall[1] = centerBall[0];
				cv::circle(_output, cv::Point(cX, cY), radiusCircle[1], cv::Scalar::all(255), 1);
				break;
		}
		cv::circle(_output, cv::Point(cX, cY), 3, cv::Scalar(255, 255, 0), 1);
		
		if(id == 2)
			cv::line(_output, cv::Point(_ori2.cols / 2, _ori2.rows), cv::Point(cX, cY), cv::Scalar(16, 50, 220), 5);
		//cv::rectangle(_ori, r, cv::Scalar::all(255), 1);
	}

}

void erodeDilate(cv::Mat _pros) {
	cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	
	cv::erode(_pros, _pros, erodeElement);
	cv::erode(_pros, _pros, erodeElement);

	cv::dilate(_pros, _pros, dilateElement);
	cv::dilate(_pros, _pros, dilateElement);
}

void trackBars() {
	cv::namedWindow("Kalibrasi", 0);
	cv::createTrackbar("H_MIN", "Kalibrasi", &LOW[0], 180, updateData);
	cv::createTrackbar("H_HIGH", "Kalibrasi", &HIGH[0], 180, updateData);
	cv::createTrackbar("S_MIN", "Kalibrasi", &LOW[1], 255, updateData);
	cv::createTrackbar("S_HIGH", "Kalibrasi", &HIGH[1], 255, updateData);
	cv::createTrackbar("V_MIN", "Kalibrasi", &LOW[2], 256, updateData);
	cv::createTrackbar("V_HIGH", "Kalibrasi", &HIGH[2], 256, updateData);
}

void updateData(int, void*) {
	cout << "H Low\t" << LOW[0] << endl;
	cout << "H High\t" << HIGH[0] << endl;
	cout << "S Low\t" << LOW[1] << endl;
	cout << "S High\t" << HIGH[1] << endl;
	cout << "V Low\t" << LOW[2] << endl;
	cout << "V High\t" << HIGH[2] << endl;
	cout << endl;
}

cv::Scalar inisiateScalarLowBola() {
	cv::Scalar LOWBola = cv::Scalar(0,0,210);
	
	return LOWBola;
}
cv::Scalar inisiateScalarHighBola() {

	cv::Scalar HIGHBola = cv::Scalar(117, 205, 256);
	
	return HIGHBola;
}

cv::Scalar inisiateScalarLowLapang() {
	cv::Scalar LOWLapang = cv::Scalar(0, 109, 0);

	return LOWLapang;
}
cv::Scalar inisiateScalarHighLapang() {
	cv::Scalar HIGHLapang = cv::Scalar(140, 256, 99);

	return HIGHLapang;
}

cv::Scalar inisiateScalarLowGaris() {
	cv::Scalar LOWGaris = cv::Scalar(0, 0, 0, 0);
	return LOWGaris;
}
cv::Scalar inisiateScalarHighGaris() {
	cv::Scalar HIGHGaris = cv::Scalar(180, 255, 255, 0);

	return HIGHGaris;
}

cv::Scalar inisiateScalarLowObstacle() {
	cv::Scalar LOWObstacle = cv::Scalar(0, 0, 0, 0);
	return LOWObstacle;
}
cv::Scalar inisiateScalarHighObstacle() {
	cv::Scalar HIGHObstacle = cv::Scalar(10, 10, 256, 0);

	return HIGHObstacle;
}