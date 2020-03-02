// /**
//  *	Written by: Raj Shah 
//  **/

#ifndef _MPULib_h_
#define _MPULib_h_

#include "Arduino.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// #include "Wire.h"
MPU6050 mpu;
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	unsigned long timeStamp;
	double accFx, accFy, accFz, gyFx, accP, gyP, gxVal, gxPrev;
	double dT, alphaLPF, alphaHPF, Tau;
	double pitch, pitchRate, pitchPrev;
	const double fCut = 5.0;
	const double alpha = 0.02;
	const byte sensorInterruptPin;

// class MPULib{
// public:
// 	MPULib(MPU6050 mpuRef, const byte sensorInterruptPinVal, 
// 		double fCutVal = 5.0, double alphaVal = 0.02);
// 	// void initMPU(void);
// 	// void readMPUData(void);
// 	// void iterate(void);
// 	// double getTheta(void);
// 	// double getThetaDot(void);
// 	// void printStates(void);
// 	MPU6050 mpu;
// 	int16_t ax, ay, az;
// 	int16_t gx, gy, gz;
// 	unsigned long timeStamp;
// 	double accFx, accFy, accFz, gyFx, accP, gyP, gxVal, gxPrev;
// 	double dT, alphaLPF, alphaHPF, Tau;
// 	double pitch, pitchRate, pitchPrev;
// 	const double fCut = 5.0;
// 	const double alpha = 0.02;
// 	const byte sensorInterruptPin;
// };

extern volatile bool isMPUReady;
void sensorISR(void);
#endif
