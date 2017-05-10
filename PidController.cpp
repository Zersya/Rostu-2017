#include "PidController.h"
#include <iostream>

double PidControllerRotation::_pidControllerRotation(double input) {
	double error;
	if (input < 0) error = setPoint + input;
	else error = setPoint - input;

	double rate = input - lastError;
	double output = ((kp * error) + (kd * rate)) / limits;

	lastError = input;
	rawOutput = output;
	if (output > 0) {
		if (output > maxValue) {
			output = maxValue;
		}
	}
	else if (output < 0) {
		output *= -1;
		if (output > maxValue) {
			output = maxValue;
		}

	}
	//std::cout << output << std::endl;
	return output;
}

double PidControllerDestionation::_pidControlleDestination(double input) {
	double error;
	if (input < 0) error = setPoint + input;
	else error = setPoint - input;

	double rate = input - lastError;
	double output = ((kp * error) + (kd * rate)) / limits;

	lastError = input;
	rawOutput = output;
	if (output > 0) {
		if (output > maxValue) {
			output = maxValue;
		}
	}
	else if (output < 0) {
		output *= -1;
		if (output > maxValue) {
			output = maxValue;
		}

	}
	return output;	
}
double PidControllerArahGawang::_pidControllerArahGawang(int input) {
	double error;
	if (input < 0) error = setPoint + input;
	else error = setPoint - input;

	double rate = input - lastError;
	double output = ((kp * error) + (kd * rate)) / limits;
	lastError = input;
	if (output > 0) {
		if (output > maxValue) {
			output = maxValue;
		}
	}
	else if (output < 0) {
		output *= -1;
		if (output > maxValue) {
			output = maxValue;
		}

	}
	return output;
}
double PidControllerOmniCamera::_pidControllerOmniCamera(double input) {
	double error;
	if (input < 0) error = setPoint + input;
	else error = setPoint - input;

	double rate = input - lastError;
	double output = ((kp * error) + (kd * rate));
	lastError = input;
	rawOutput = output;
	if (output > 0) {
		if (output > maxValue) {
			output = maxValue;
		}
	}
	else if (output < 0) {
		output *= -1;
		if (output > maxValue) {
			output = maxValue;
		}

	}
	return output;
}