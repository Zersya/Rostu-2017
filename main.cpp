#include "main.h"


int main(int argc, char** argv) {

	//Serial
	Serial SP(COMmega);
	char dataReceive4Robot[100];
	std::size_t received4Robot;

	
	char* State = "";
	char* lastDir = "";

	//socket for communication processing base and robot
	sf::TcpSocket socketRobot;
	sf::Socket::Status statusRobot = socketRobot.connect(ipServer, 444);
	
	//Multi-Thread
	std::thread thread4Server;
	thread4Server = std::thread(&dapatkanPerintahWasit);
	//std::thread thread4Compass;
	//thread4Compass = std::thread(&getDataHeading);


	cv::VideoCapture cap(1);
	cv::VideoCapture caps(0);
	//initiate mouse move and drag to false 
	mouseIsDragging = false;
	mouseMove = false;
	rectangleSelected = false;
	calibrationMode = true;


	cap.set(CV_CAP_PROP_FRAME_WIDTH, 720);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	//set mouse callback function to be active on "Webcam Feed" window
	//we pass the handle to our "frame" matrix so that we can draw a rectangle to it
	//as the user clicks and drags the mouse	
	cv::namedWindow("Camera Atas");
	cv::setMouseCallback("Camera Atas", clickAndDrag_Rectangle, &_ori);

	cv::namedWindow("Camera Depan");
	cv::setMouseCallback("Camera Depan", clickAndDrag_Rectangle, &_ori2);

	
	//trackBars();

	if (!cap.isOpened())
		return 0;
	if (!caps.isOpened())
		return 0;

	if (statusRobot == sf::Socket::Done) {
		cout << "Server Connected" << endl;
	}
	if (SP.IsConnected()){
		cout << "Arduino Mega Connected" << endl;
	}
	string dataReceivefromCoach[4];
	string aksinya = "";
	PidControllerArahGawang pidAG;
	int pidArahGawang = pidAG.setPoint;

	while (true) {
		cap >> _ori;
		caps >> _ori2;
		
		//set HSV values from user selected region
		recordHSV_Values(_ori, _ori);

		//set HSV values from user selected region
		recordHSV_Values(_ori2, _ori2);
		//Robot Kirim data ke Pelatih
		char* radius = intToChar(radiusCircle[1]);
		char* posisiBola = intToChar(centerBall[1].x/3.5);


		string konversi = to_string(radiusCircle[1]);
		konversi += "," + to_string(centerBall[1].x);
		konversi += "," + to_string(dataHeading);
		konversi += "," + to_string(pidArahGawang);

		dataKirimServer = new char[konversi.length()+2];
		std::strcpy(dataKirimServer, konversi.c_str());

		socketRobot.send(dataKirimServer, strlen(dataKirimServer));
		
		//Robot Terima data dari Pelatih
		socketRobot.receive(dataReceive4Robot, 8, received4Robot);

		int k = 0;
		dataReceivefromCoach[k] = "";
		for (int i = 0; i < received4Robot; i++) {
			if (dataReceive4Robot[i] != '\n') {
				if (dataReceive4Robot[i] != ',') {
					dataReceivefromCoach[k] += dataReceive4Robot[i];
				}
				else {
					k++;
					dataReceivefromCoach[k] = "";
				}
			}
			else {
				aksinya = dataReceivefromCoach[0];
				aksinya += "," + dataReceivefromCoach[1];
			}
		}
		//cout << aksinya << endl;
		//pidAG.setPoint = stoi(dataReceivefromCoach[0]);
	
		perintahKeRobot(aksinya);
		//toArduino
		writeres = false;
		if (SP.IsConnected()) {
			writelen = strlen(dir);
			writeres = SP.WriteData(dir, writelen);
			//cout << kondisi << "," << pidJarakBola << "," << dataHeading << "," << dir;
			delete[] dir;
		}
	//	cout << kondisi <<  "," << dir;
		//cout << pidJarakBola << endl;
		//cout << radiusCircle[0] << "," << (radiusCircle[0] + pidOmniCamera) << endl;
		
		if (!writeres) {
			SP.ReConnect(COMmega);
		}
		
		
		cv::imshow("Camera Atas", _ori);
		cv::imshow("Camera Depan", _ori2);
		//if (cv::waitKey(30) == 99) calibrationMode != calibrationMode;
		 if (cv::waitKey(30) > 0) break;

	}
 	return 0;
}

void testThread() {
	while(true)cout << "Test Thread\n";
}

void getDataHeading() {	
	Serial SP(COMnano);
	char read[50] = "";
	string _dataNano = "";

	if (SP.IsConnected()) {
		cout << "Arduino Nano Connected" << endl;
	}
	while (true) {
		int len = 0;
		if (SP.IsConnected()) {
			len = SP.ReadData(read, 8);
			dataHeading = atoi(read);
			read[len] = 0;
			Sleep(200);
		}
		else if(len <= 0){
			SP.ReConnect(COMnano);
		}
		//cout << pidNgarahGawang << "," << dataHeading << endl;

		
	}
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

void perintahKeRobot(string aksinya) {
	//kejarBola();
	//kondisiMotor();


	if (statusGame == "s") {
		//kejarBola();
		cameraAtas();
		cout << "Start" << endl;
		if (aksinya == "RM") {
			
		}
		else if (aksinya == "RJ") {
	
		}
	
	}
	else if (statusGame == "S") {
		cout << "Stop" << endl;
		getData(0, 0, 0, 0, 2, 2, 2, 2);
	}
	else if (statusGame == "k" || statusGame == "K") {
		cout << "Kick off" << endl;
		cameraDepan();
	}
	else {
		getData(0, 0, 0, 0, 0, 0, 0, 0);
	}
}

float euclideanDist(cv::Point p, cv::Point q) {
	cv::Point diff =p - q  ;
	return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
}
int tempLocBola = 0;
void kejarBola(int kondisiKamera) {
	kondisiMotor(kondisiKamera);
	if (kondisiKamera == 1) {
		d = euclideanDist(cv::Point(_ori.rows / 2, _ori.cols / 2), cv::Point(centerBall[0].x, centerBall[0].y));
		if (locBola[0] <= 0) {
			locBola[0] = tempLocBola;
		}
		pidJarakBola = pidDestination._pidControlleDestination(d);
		pidNgarahGawang = pidArahGawang._pidControllerArahGawang(dataHeading);
		pidOmniCamera = pidOmniView._pidControllerOmniCamera(locBola[0]);

		int motor = (pidJarakBola + pidOmniCamera);
		//cout << d << endl;
		

		if (ball) {
			tempLocBola = locBola[0];
			if (kondisi == 1 || kondisi == -1) {
				if (kondisi == 1) {
					getData(motor / 3, pidJarakBola, pidJarakBola, motor, 1, 1, 1, 1);
				}
				else if (kondisi == -1) {
					getData(motor, pidJarakBola, pidJarakBola, motor / 3, 1, 1, 1, 1);
				}
			}
			else if (kondisi == 2 || kondisi == -2) {
				if (kondisi == 2) {
					getData(pidOmniCamera / 3, pidOmniCamera / 3, pidOmniCamera / 3, pidOmniCamera / 3, 1, 1, 0, 0);
				}
				else if (kondisi == -2) {
					getData(pidOmniCamera / 3, pidOmniCamera / 3, pidOmniCamera / 3, pidOmniCamera / 3, 0, 0, 1, 1);
				}
			}
			else if (kondisi == 3 || kondisi == -3) {
				if (kondisi == -3) {
					getData(pidNgarahGawang / 2, pidNgarahGawang / 2, pidNgarahGawang, pidNgarahGawang, 1, 0, 1, 1);
				}
				else if (kondisi == 3) {
					getData(pidNgarahGawang, pidNgarahGawang, pidNgarahGawang / 2, pidNgarahGawang / 2, 1, 1, 0, 1);
				}
			}
			else {
				getData(0, 0, 0, 0, 0, 0, 0, 0);
			}
		}
		else {
			getData(0, 0, 0, 0, 0, 0, 0, 0);
			/*	ball = false;
				_ball = true;
				if (kondisi == 1 || kondisi == -1) {
					if (kondisi == 1) {
						getData(motor / 4, pidJarakBola, pidJarakBola, motor, 1, 1, 1, 1);
					}
					else if (kondisi == -1) {
						getData(motor, pidJarakBola, pidJarakBola, motor / 4, 1, 1, 1, 1);
					}
				}
				*/
		}
	}
	else if (kondisiKamera == 2) {
		pidDestination.setPoint = 30;
		pidArahBola = pidRotation._pidControllerRotation(centerBall[1].x / 3.5);
		pidJarakBola = pidDestination._pidControlleDestination(radiusCircle[1]);
		

		if (kondisi == -1) {
			getData(pidArahBola/3, pidJarakBola, pidJarakBola, pidArahBola, 1, 1, 1, 1);
		}
		else if (kondisi == 1) {
			getData(pidArahBola, pidJarakBola, pidJarakBola, pidArahBola/3, 1, 1, 1, 1);
		}
		else getData(0, 0, 0, 0, 0, 0, 0, 0);

		
	}
}

void kondisiMotor(int kondisiKamera) {
	/*
		kondisi
		1 = arahKiri
		-1 = arahKanan
		-2 = putarKiri
		2 = putarKanan
		3 = arahGawangTerdekatKanan
		-3 = arahgawangTerdekatKiri
	*/
	if (kondisiKamera == 1) {
		cout << "Kamera atas aktifkan" << endl;
		if (!ball /*&& kondisi == 0*/) {}
		else if (ball) {
			if (pidOmniView.rawOutput < 0 && locBola[0] > 310 && ball) kondisi = -2;
			else if (pidOmniView.rawOutput > 0 && locBola[0] < 190 && ball) kondisi = 2;
			else if (pidOmniView.rawOutput < 0 && locBola[0] > 260 && locBola[0] < 310 && ball) kondisi = 1;
			else if (pidOmniView.rawOutput > 0 && locBola[0] > 190 && locBola[0] < 260 && ball) kondisi = -1;
		}
		int _rows = 60;
		cv::line(_ori2, cv::Point(0, _ori2.rows - _rows), cv::Point(_ori2.cols, _ori2.rows - _rows), cv::Scalar(255, 255, 255), 1);
	}
	else if (kondisiKamera == 2) {
		//cout << "Kamera depan AKtifkan" << endl;
		
		if (!ball) {

		}
		else {
			if (pidRotation.rawOutput < 0) kondisi = 1;
			else if (pidRotation.rawOutput > 0) kondisi = -1;
		}
	}
	if (dataHeading < pidArahGawang.setPoint && centerBall[0].x > 480 && centerBall[0].x < 500 && centerBall[0].y > 245 && centerBall[0].y < 275) kondisi = -3;
	else if ( dataHeading > pidArahGawang.setPoint && centerBall[0].x > 480 && centerBall[0].x < 500 && centerBall[0].y > 245 && centerBall[0].y < 275) kondisi = 3;
	
	
}

char* intToChar(int data) {
	string dataStr = to_string(data);
	char* rad = new char[dataStr.length() + 1];

	std::strcpy(rad, dataStr.c_str());

	return rad;

}

void getData(int m1, int m2, int m3, int m4, int kon1, int kon2, int kon3, int kon4) {
	
	string data = to_string(m1);
	data += "," + to_string(m2);
	data += "," + to_string(m3);
	data += "," + to_string(m4);
	data += "," + to_string(kon1);
	data += "," + to_string(kon2);
	data += "," + to_string(kon3);
	data += "," + to_string(kon4);
	data += "\n";

	dir = new char[data.length()+2];
	std::strcpy(dir, data.c_str());
}


void cameraAtas() {
	//bola
	locBola[0] = processFindContour_base(
		processThreshold(_ori, inisiateScalarLowBola(), inisiateScalarHighBola()), 
		processThreshold(_ori, inisiateScalarLowBolaGloss(), inisiateScalarHighBolaGloss()), 
		_ori, 1, topContours);
	//lapangan
	 //processFindContour(processThreshold(_ori, inisiateScalarLowLapang(), inisiateScalarHighLapang()), 0);

	//garis
//	cv::Mat destImg;
	//cv::bitwise_not(processThreshold(_ori, inisiateScalarLowGaris(), inisiateScalarHighGaris()), destImg);
	//processFindContour(destImg, 3);

	//obstacle
	//processFindContour(processThreshold(_ori, inisiateScalarLowObstacle(), inisiateScalarHighObstacle()), 4);

	cv::line(_ori, cv::Point(_ori.cols / 2, _ori.rows / 2), cv::Point(_ori.cols, _ori.rows/2), cv::Scalar(255, 0, 0), 2);
	kejarBola(1);
}

void cameraDepan() {
	//bola
	cv::Mat tmp;
	processFindContour_base(
		processThreshold(_ori2, inisiateScalarLowBolaDepan(), inisiateScalarHighBolaDepan()), _ori2, _ori2, 2, frontContours);
	//Gawang
	//cv::Mat destImg;
	//cv::bitwise_not(processThreshold(_ori2, inisiateScalarLowGaris(), inisiateScalarHighGaris()), destImg);
	//processFindContour(destImg, 5);
	//cout << centerBall[1];
	kejarBola(2);
}


void circleRoi() {
	int radius = 230;

	cv::Point center(_ori.cols / 2, _ori.rows / 2);
//	cv::Rect r(center.x - radius, center.y - radius, radius * 2, radius * 2);
//cv::Mat roi(_ori, r);
//	cv::Mat mask(roi.size(), roi.type(), cv::Scalar::all(0));

	
	//cv::circle(mask, cv::Point(radius, radius), radius, cv::Scalar::all(255), -1);
	//_ori = roi & mask;;
	
}

cv::Mat processThreshold(cv::Mat _pros, cv::Scalar low, cv::Scalar high) {
	cv::Mat hsv;
	cv::cvtColor(_pros, hsv, cv::COLOR_BGR2HSV);
	

	cv::inRange(_pros, low, high, _pros);
	erodeDilate(_pros);

	//cv::medianBlur(_pros, _pros, 25);
	cv::GaussianBlur(_pros, _pros, cv::Size(9, 9), 1, 2);
	return _pros;
}

int processFindContour_base(cv::Mat _ths1, cv::Mat _ths2, cv::Mat _ths3, int id, vector<vector<cv::Point>> cekContours) {

//	cv::imshow("Process1", _ths1);
//	cv::imshow("Process2", _ths2);
//	cv::imshow("Process3", _ths3);
	if (id == 1) {
		cv::findContours(_ths1, contours[0], hierarchy[0], CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		cv::findContours(_ths2, contours[1], hierarchy[1], CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		//cv::findContours(_ths3, contours[2], hierarchy[2], CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

		cekContours.insert(cekContours.end(), contours[0].begin(), contours[0].end());
		cekContours.insert(cekContours.end(), contours[1].begin(), contours[1].end());
		//cekContours.insert(cekContours.end(), contours[2].begin(), contours[2].end());
	}
	else if (id == 2) {
		cv::findContours(_ths1, contours[0], hierarchy[0], CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		cekContours.insert(cekContours.end(), contours[0].begin(), contours[0].end());
	}
	int largestContourIndex, largestArea = 0;

	if (cekContours.size() > 0) {
		ball = true;
		for (int i = 0; i < cekContours.size(); i++) {
			
			double a = cv::contourArea(cekContours[i], false);;
			if (a > largestArea) {
				largestArea = (int) a;
				largestContourIndex = i;

			}
		}

	return implementContour(cekContours, id, largestContourIndex);
	}
	else {
		ball = false;
		//if(id == 2)
			//centerBall[1] = cv::Point(0, 0);
		//return 0;

	}
	return NULL;
}

int implementContour(vector<vector<cv::Point>> _contours, int id, int largestIdContours) {
	int rotAngle = 0;
	if (id == 1) {
		topContours = _contours;
		drawing(largestIdContours, id, _contours);
		rotAngle = getDirection();
		topContours.clear();
		return rotAngle;
	}
	else if (id == 2) {
		frontContours = _contours;
		drawing(largestIdContours, id, _contours);
		getData(100, 100, 100, 100, 1, 1, 1, 1);  
		frontContours.clear();
	}
	else if (id == 0)
		cv::drawContours(_ori, _contours, largestIdContours, cv::Scalar(12, 255, 26), CV_FILLED);
	else if (id == 3)
		cv::drawContours(_ori, _contours, largestIdContours, cv::Scalar(170, 205, 60), CV_FILLED);
	else if (id == 4)
		cv::drawContours(_ori, _contours, largestIdContours, cv::Scalar(255, 255, 250), CV_FILLED);
	else if (id == 5)
		cv::drawContours(_ori2, _contours, largestIdContours, cv::Scalar(255, 255, 250), CV_FILLED);

	contours[0].clear();
	contours[1].clear();
	contours[2].clear();
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
	/*else
		return 0;*/
}


void drawing(int largeI, int id, vector<vector<cv::Point>> cekContours) {

	int cX, cY = 0;
	
	for (int i = 0; i < cekContours.size(); i++) {

		cv::Rect r;
		r = cv::boundingRect(cekContours[largeI]);
		cX = r.x + r.width / 2;
		cY = r.y + r.height / 2;

		if (id == 1){
				centerBall[0] = cv::Point(cX, cY);
				//cout << centerBall[0] << endl;
				radiusCircle[0] = r.height / 2;
				cv::circle(_ori, centerBall[0], radiusCircle[0], cv::Scalar::all(255), 1);
				cv::drawContours(_ori, cekContours, largeI, cv::Scalar(12, 255, 26), CV_FILLED);
		}else if(id == 2){
				centerBall[1] = cv::Point(cX, cY);
				radiusCircle[1] = r.height / 2;
				cv::circle(_ori2, centerBall[1], radiusCircle[1], cv::Scalar::all(255), 1);
				cv::line(_ori2, cv::Point(_ori2.cols / 2, _ori2.rows), cv::Point(cX, cY), cv::Scalar(16, 50, 220), 5);
		}
	}

}

void clickAndDrag_Rectangle(int event, int x, int y, int flags, void* param) {
	//only if calibration mode is true will we use the mouse to change HSV values
	if (calibrationMode == true) {
		//get handle to video feed passed in as "param" and cast as Mat pointer
		cv::Mat* videoFeed = (cv::Mat*)param;

		if (event == CV_EVENT_LBUTTONDOWN && mouseIsDragging == false)
		{
			//keep track of initial point clicked
			initialClickPoint = cv::Point(x, y);
			//user has begun dragging the mouse
			mouseIsDragging = true;
		}
		/* user is dragging the mouse */
		if (event == CV_EVENT_MOUSEMOVE && mouseIsDragging == true)
		{
			//keep track of current mouse point
			currentMousePoint = cv::Point(x, y);
			//user has moved the mouse while clicking and dragging
			mouseMove = true;
		}
		/* user has released left button */
		if (event == CV_EVENT_LBUTTONUP && mouseIsDragging == true)
		{
			//set rectangle ROI to the rectangle that the user has selected
			rectangleROI = cv::Rect(initialClickPoint, currentMousePoint);

			//reset boolean variables
			mouseIsDragging = false;
			mouseMove = false;
			rectangleSelected = true;
		}

		if (event == CV_EVENT_RBUTTONDOWN) {
			//user has clicked right mouse button
			//Reset HSV Values
			LOW[0] = 0;
			LOW[1] = 0;
			LOW[2] = 0;
			HIGH[1] = 255;
			HIGH[2] = 255;
			HIGH[3] = 255;

		}
		if (event == CV_EVENT_MBUTTONDOWN) {

			//user has clicked middle mouse button
			//enter code here if needed.
		}
	}

}
void recordHSV_Values(cv::Mat frame, cv::Mat hsv_frame) {

	//save HSV values for ROI that user selected to a vector
	if (mouseMove == false && rectangleSelected == true) {

		//clear previous vector values
		if (H_ROI.size()>0) H_ROI.clear();
		if (S_ROI.size()>0) S_ROI.clear();
		if (V_ROI.size()>0)V_ROI.clear();
		//if the rectangle has no width or height (user has only dragged a line) then we don't try to iterate over the width or height
		if (rectangleROI.width<1 || rectangleROI.height<1) cout << "Please drag a rectangle, not a line" << endl;
		else {
			for (int i = rectangleROI.x; i<rectangleROI.x + rectangleROI.width; i++) {
				//iterate through both x and y direction and save HSV values at each and every point
				for (int j = rectangleROI.y; j<rectangleROI.y + rectangleROI.height; j++) {
					//save HSV value at this point
					H_ROI.push_back((int)hsv_frame.at<cv::Vec3b>(j, i)[0]);
					S_ROI.push_back((int)hsv_frame.at<cv::Vec3b>(j, i)[1]);
					V_ROI.push_back((int)hsv_frame.at<cv::Vec3b>(j, i)[2]);
				}
			}
		}
		//reset rectangleSelected so user can select another region if necessary
		rectangleSelected = false;
		//set min and max HSV values from min and max elements of each array

		if (H_ROI.size()>0) {
			//NOTE: min_element and max_element return iterators so we must dereference them with "*"
			LOW[0] = *std::min_element(H_ROI.begin(), H_ROI.end());
			HIGH[0] = *std::max_element(H_ROI.begin(), H_ROI.end());
			cout << "MIN 'H' VALUE: " << LOW[0]<< endl;
			cout << "MAX 'H' VALUE: " << HIGH[0] << endl;
		}
		if (S_ROI.size()>0) {
			LOW[1]= *std::min_element(S_ROI.begin(), S_ROI.end());
			HIGH[1] = *std::max_element(S_ROI.begin(), S_ROI.end());
			cout << "MIN 'S' VALUE: " << LOW[1] << endl;
			cout << "MAX 'S' VALUE: " << HIGH[1] << endl;
		}
		if (V_ROI.size()>0) {
			LOW[2]= *std::min_element(V_ROI.begin(), V_ROI.end());
			HIGH[2] = *std::max_element(V_ROI.begin(), V_ROI.end());
			cout << "MIN 'V' VALUE: " << LOW[2]<< endl;
			cout << "MAX 'V' VALUE: " << HIGH[2] << endl;
		}

	}

	if (mouseMove == true) {
		//if the mouse is held down, we will draw the click and dragged rectangle to the screen
		rectangle(frame, initialClickPoint, cv::Point(currentMousePoint.x, currentMousePoint.y), cv::Scalar(0, 255, 0), 1, 8, 0);
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

	cv::Scalar LOWBola = cv::Scalar(57, 70, 179);
	//cv::Scalar LOWBola = cv::Scalar(LOW[0], LOW[1], LOW[2]);
	//cv::Scalar LOWBola = cv::Scalar(32, 62, 159);
	return LOWBola;
}
cv::Scalar inisiateScalarHighBola() {

	cv::Scalar HIGHBola = cv::Scalar(160, 192, 255);
	//cv::Scalar HIGHBola = cv::Scalar(86, 102, 256);
	//cv::Scalar HIGHBola = cv::Scalar(HIGH[0], HIGH[1], HIGH[2]);
	
	return HIGHBola;
}
cv::Scalar inisiateScalarLowBolaDepan() {
	//cv::Scalar LOWBola = cv::Scalar(LOW[0], LOW[1], LOW[2]);
	cv::Scalar LOWBola = cv::Scalar(0, 0, 167);
	return LOWBola;
}
cv::Scalar inisiateScalarHighBolaDepan() {

	cv::Scalar HIGHBola = cv::Scalar(82, 115, 256);
	//cv::Scalar HIGHBola = cv::Scalar(HIGH[0], HIGH[1], HIGH[2]);

	return HIGHBola;
}

cv::Scalar inisiateScalarLowBolaGloss() {
	//cv::Scalar LOWBola = cv::Scalar(LOW[0], LOW[1], LOW[2]);
	cv::Scalar LOWBola = cv::Scalar(80, 137, 247);
	return LOWBola;
}
cv::Scalar inisiateScalarHighBolaGloss() {

	cv::Scalar HIGHBola = cv::Scalar(180, 255, 256);
	//cv::Scalar HIGHBola = cv::Scalar(HIGH[0], HIGH[1], HIGH[2]);

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