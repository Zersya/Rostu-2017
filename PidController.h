#pragma once

class PidControllerRotation {
public:
	double _pidControllerRotation(double input);
	double rawOutput;

private:
	double kp = 4;
	double kd = 2;
	double limits = 1;

	double setPoint = 90;
	double lastError = 0;

	double maxValue = 150;


};

class PidControllerDestionation {
public:
	double _pidControlleDestination(double input);
	double rawOutput;
	double setPoint = 225;
private:
	double kp = 4;
	double kd = 7.2;
	double limits = 2;

	double lastError = 0;

	double maxValue = 100;
};

class PidControllerArahGawang {
public:
	double _pidControllerArahGawang(int input);
	double setPoint = 50;
private:
	double kp = .7;
	double kd = .5;
	double limits = 2;

	double lastError = 0;

	double maxValue = 150;
};

class PidControllerOmniCamera {
public:
	double _pidControllerOmniCamera(double input);
	double rawOutput;
	double setPoint = 260;
private:
	double kp = 1.8;
	double ki = 1;
	double kd = .55;

	double limits = 1;

	double lastError = 0;

	double maxValue = 200;
};
