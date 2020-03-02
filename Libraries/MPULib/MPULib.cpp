#include "MPULib.h"

// MPULib(MPU6050 mpuRef, const byte sensorInterruptPinVal, 
// double fCutVal = 5.0, double alphaVal = 0.02): mpu(mpuRef), sensorInterruptPin(sensorInterruptPinVal),
// fCut(fCutVal), alpha(alphaVal) {	
// 	Tau = 1/(2*PI*fCut);
//   	accP = gyP = pitch = accFx = accFy = accFz = gxPrev = gyFx = 0;
// }


void readMPUData(MPULib sensor){
    dT = (double)(micros()-timeStamp)/1e6;
    timeStamp = micros();
    gxPrev = gxVal;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gxVal = (double)gx/131.0;
    alphaHPF = alphaLPF = Tau/(Tau+dT);
    gyFx = (1-alphaHPF)*(gxVal-gxPrev)+(1-alphaHPF)*gyFx;
    
    accFx = alphaLPF*accFx+(1-alphaLPF)*ax;
    accFy = alphaLPF*accFy+(1-alphaLPF)*ay;
    accFz = alphaLPF*accFz+(1-alphaLPF)*az;
    accP = atan(accFy/abs(accFz))*180/PI;
    gyP = -dT*gyFx;
    pitchPrev = pitch;
	pitch = (1-alpha)*(gyP+pitch)+alpha*accP;
	pitchRate = (pitch-pitchPrev)/dT;
}

void sensorISR(MPULib sensor){
	isMPUReady = true;
}

// double getTheta(MPULib sensor){ return pitch; }
// double getThetaDot(MPULib sensor){ return pitchRate; }
void printStates(MPULib sensor) {
  Serial.print("Theta: "); Serial.println(pitch);
  Serial.print(" ThetaDot: "); Serial.println(pitchRate);
}

void initMPU(MPULib sensor){
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(sensorInterruptPin, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available()); // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  int devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-901);
  mpu.setYGyroOffset(44);
  mpu.setZGyroOffset(19);
  
  mpu.setXAccelOffset(-3991);
  mpu.setYAccelOffset(-1309);
  mpu.setZAccelOffset(4911);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0){
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(sensorInterruptPin));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(sensorInterruptPin), sensorISR, RISING);
    int mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
  }
  else{
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMPz configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void iterate(MPULib sensor){
  if(isMPUReady){
    readMPUData(sensor);
    isMPUReady = false;
  }
}