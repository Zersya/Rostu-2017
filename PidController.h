#pragma once

class PidControllerRotation {
public:
	double _pidControllerRotation(double input);
	double rawOutput;

private:
	double kp = 4;
	double kd =  2;
	double limits = 4;

	double setPoint = 90;
	double lastError = 0;

	double maxValue = 100;


};

class PidControllerDestionation {
public:
	double _pidControlleDestination(double input);
	double rawOutput;
	double setPoint = 200;
private:
	double kp = 4;
	double kd = 6;
	double limits = 2;

	double lastError = 0;

	double maxValue = 150;
};

class PidControllerArahGawang {
public:
	double _pidControllerArahGawang(double input);
	double setPoint = 200;
private:
	double kp = 4;
	double kd = 6;
	double limits = 2;

	double lastError = 0;

	double maxValue = 80;
};