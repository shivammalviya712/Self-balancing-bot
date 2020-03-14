/*
  Encoders.h - A Library for Encoders
  Created by Shivam Malviya, Jan 30, 2020.
*/
#include "Motor.h"

Motor::Motor(byte M1, byte M2, byte EN, byte A, byte B, unsigned int motorOffset = 0) {
    this->M1 = M1;
    this->M2 = M2;
    this->EN = EN;
    this->encoder.A = A;
    this->encoder.B = B;
    this->MotorOffset = motorOffset;
    init();
}

void Motor::init() {
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(EN, OUTPUT);

    stop(); 
}

void Motor::stop() {
    digitalWrite(M1, LOW);
    digitalWrite(M2, LOW);
    analogWrite(EN, 0);
}

void Motor::forward(int speed) {
    digitalWrite(M1, HIGH);
    digitalWrite(M2, LOW);
    analogWrite(EN, speed);
}

void Motor::reverse(int speed) {
    digitalWrite(M1, LOW);
    digitalWrite(M2, HIGH);
    analogWrite(EN, speed);
}

void Motor::generate(int analogVal) {
    if (analogVal > 0) {
        analogVal += MotorOffset;
        if(analogVal > 255)  { analogVal = 255; }
        forward(analogVal);    
    }
    else {
        analogVal -= MotorOffset;
        if(analogVal < -255) {analogVal = -255; }
        reverse(-analogVal);
    }
} 

void leftEncoderInterruptA() {
    if (digitalReadFast(LEFT_ENCODER_A) != digitalReadFast(LEFT_ENCODER_B)) {
        leftMotor.encoder.rotation--;
    } 
    else {
        leftMotor.encoder.rotation++;
    }
    leftMotor.encoder.phiDot = (0.02243994753 * 1000000/ ENCODING) / ((double)(micros() - deltaT));
    deltaT = micros();
}

void rightEncoderInterruptA() { 
    
    if (digitalReadFast(RIGHT_ENCODER_A) != digitalReadFast(RIGHT_ENCODER_B)) {
        rightMotor.encoder.rotation++;
    } 
    else {
        rightMotor.encoder.rotation--;
    }
}

void attachMotorInterrupts() {
  attachInterrupt(
    digitalPinToInterrupt(leftMotor.encoder.A), 
    leftEncoderInterruptA,
    RISING
    );

  attachInterrupt(
    digitalPinToInterrupt(rightMotor.encoder.A), 
    rightEncoderInterruptA,
    RISING
    );   
}

//    void driveMotors (double voltage) {
//    int analogVal = (int) ((double)voltage*255.0/12.0 + 0.5);
//    int offset = OFFSET_FACTOR*(rightMotor.encoder.rotation - leftMotor.encoder.rotation); 
//    leftMotor.generate((int) (analogVal + offset));
//    rightMotor.generate((int) (analogVal - offset));
//}