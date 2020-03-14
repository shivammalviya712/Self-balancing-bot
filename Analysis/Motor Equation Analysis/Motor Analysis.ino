#include <TimerInterrupts.h>
#include <Motor.h>

const byte lM1 = 22;  //M1 - OUT1 - IN1 - 22
const byte lM2 = 23;  //M2 - OUT2 - IN2 - 23
const byte lME = 11;  //ENA - 11
const byte lMA = 18;  //A - 18
const byte lMB = 17;  //B - 17

const byte rM1 = 30;  //M1 - OUT4 - IN4 - 30
const byte rM2 = 31;  //M2 - OUT3 - IN3 - 31
const byte rME = 12;  //ENB - 12
const byte rMA = 19;   //A - 19
const byte rMB = 16;   //B - 16

double torque, voltage;
unsigned long deltaT = 0;

Motor leftMotor(lM1, lM2, lME, lMA, lMB);
Motor rightMotor(rM1, rM2, rME, rMA, rMB);

void setup() {
  Serial.begin(115200);
  attachMotorInterrupts();
//  initTimer3();
  sei();  
}

void loop() {
  Serial.println();
  int analogVol;
  for (voltage = 0; voltage <= 12; voltage += 0.01)
  {
    analogVol = (int) (voltage*255.0/12.0 + 0.5);
    leftMotor.generate(analogVol);
    Serial.print("Voltage = ");
    Serial.println(voltage);
    Serial.print("Omega = ");
    Serial.println(leftMotor.encoder.phiDot);
    delay(100);
  }
  Serial.println();
}

//ISR(TIMER3_COMPA_vect) {
//    leftMotor.encoder.phiDot = (0.02243994753 / ENCODING) * (leftMotor.encoder.rotation - leftMotor.encoder.lastRotation) / SAMPLING_TIME; //0.02243994753 = 2 * M_P1 / 280.0
//    rightMotor.encoder.phiDot = (0.02243994753 / ENCODING) * (rightMotor.encoder.rotation - rightMotor.encoder.lastRotation) / SAMPLING_TIME; //0.02243994753 = 2 * M_P1 / 280.0
//    leftMotor.encoder.lastRotation = leftMotor.encoder.rotation;
//    rightMotor.encoder.lastRotation = rightMotor.encoder.rotation;        
//}

double x() {
  return leftMotor.encoder.getX();
}

double xDot() {
  return leftMotor.encoder.getXDot();
}
