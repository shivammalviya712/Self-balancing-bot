/*
	Encoders.h - A Library for Encoders
	Created by Shivam Malviya, Jan 30, 2020.
*/
#include "Arduino.h"
#include "Encoders.h"

Encoders::Encoders(byte A=0, byte B=0) {
	this->A = A;
	this->B = B;

	init();
}

void Encoders::init() {
	pinMode(A, INPUT_PULLUP);
	pinMode(B, INPUT_PULLUP);

	//To let RC filter get charged
	delayMicroseconds(2000);
	rotation = 0;
	lastRotation = 0;
}

double Encoders::getPhi() {
	return (2.0 * M_PI * rotation)/(double) RESOLUTION;
}

long Encoders::getRotation(){
	return rotation;
}

double Encoders::getPhiDot() {
	return phiDot;
}

double Encoders::getX() {
	return getPhi() * RADIUS;
}

double Encoders::getXDot() {
	return getPhiDot() * RADIUS;
}