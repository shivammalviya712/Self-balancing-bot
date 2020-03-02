#include "RCModule.h"

RCModule:: RCModule(int InL1Val, int InL2Val, int InR1Val, int InR2Val, int MagFVal):InL1(InL1Val), InL2(InL2Val), InR1(InR1Val), InR2(InR2Val), MagF(MagFVal){}

void RCModule::iterate(){
  bool check = readXBee();

  getJoystickData();

  controlMotors();
  controlMagnet();
  if(check) // Debugging message
    Serial.println("Success");
}


bool RCModule::readXBee(){
  // Init temporary array to store the data frame
  unsigned char tmpData[15], tmpByte;
  int checkSum = 0x83;
  // Discard all the bytes until the occurance of first 0x7E  
  while(Serial.available() && Serial.read() != 0x7E);
  // The full data frame length must be available
  if (Serial.available() < 19)  return false;
  // Check the length of the data frame, it must be 16 bytes long
  tmpByte = Serial.read();
  if (tmpByte != 0x00)  return false;
  tmpByte = Serial.read();
  if (tmpByte != 0x10)  return false;
  // Check the frame type
  tmpByte = Serial.read();
  if (tmpByte != 0x83)  return false;
  // Read 15 bytes and store it in the array
  for (int i = 0; i < 15; ++i){
    tmpData[i] = Serial.read();
    checkSum += (int)tmpData[i];
  }
  // Read checkSum byte
  tmpByte = Serial.read();
  
  // Flush the serial
  while(Serial.available())
    Serial.read();

  // Check whether the checkSum obtained by our calculation (stored as 8byte int) 
  // is same as the last byte on the same frame
  if (tmpByte == 0xFF - (0xFF & (unsigned char)checkSum)){
    // Copy tmpData to data array as the reading was successful
    for (int i = 0; i < 15; ++i){
      data[i] = tmpData[i];
    }
    return true;
  }
  else return false;
}

void RCModule::controlMotors(){
  if(control[0] == 1){ // Forward
    motorForwardL();
    motorForwardR();
  }
  else if(control[0] == -1){ // Backward
    motorBackwardL();
    motorBackwardR();
  }
  else if(control[1] == -1){ // Right
    motorForwardL();
    motorBackwardR();
  }
  else if(control[1] == 1){ // Left
    motorBackwardL();
    motorForwardR();
  }
  else{ //Stop
    motorStopL();
    motorStopR(); 
  }
}

void RCModule::controlMagnet(){
  // Use bit-masking to get the value of DO3 pin on XBee
  int eMState = data[8] & 8 ;
  if(eMState == 0)
    MagPick();
  else
    MagDrop();
}

void RCModule:: getJoystickData(){
  // Calculate the Analog value at AO1 using two bytes data[11] (LSB) and data[12] (MSB)
  int AD1 = data[11]*255+data[12];
  // Limiting AD1
  if(AD1 > 1023) AD1 = 1023;
  
  // Calculate the Analog value at AO2 using two bytes data[13] (LSB) and data[14] (MSB)
  int AD2 = data[13]*255+data[14];
  // Limiting AD2
  if(AD2 > 1023) AD2 = 1023;

  // Thresolding 
  if(AD1 >= 400 && AD1 <= 750) control[0] = 0;
  else if(AD1 < 400) control[0] = 1;
  else if(AD1 > 750) control[0] = -1;

  // Debugging statements
  Serial.print("C1 = ");   
  Serial.print(control[0]);
  Serial.print(" ");
  Serial.print("AD1 = ");    
  Serial.println(AD1);

  // Thresolding
  if(AD2 >= 400 && AD2 <= 750) control[1] = 0;
  else if(AD2 < 400) control[1] = 1;
  else if(AD2 > 750) control[1] = -1;
  
  // Debugging statements
  Serial.print("C2 = ");    
  Serial.print(control[1]);
  Serial.print(" ");
  Serial.print("AD2 = ");    
  Serial.println(AD2);
}

// ==================== HELPER FUNCTIONS ===========================

// Initialize the pin-modes of the motor-pins (InL1, InL2, InR1, InR2)
void RCModule:: motorInit(){
    pinMode(InL1, OUTPUT);
    pinMode(InL2, OUTPUT);
    
    pinMode(InR1, OUTPUT);
    pinMode(InR2, OUTPUT);
}

// Set the left motor to move forward
void RCModule::motorForwardL(){
    digitalWrite(InL1, LOW);
    digitalWrite(InL2, HIGH);
}

// Set the right motor to move forward
void RCModule::motorForwardR(){
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, HIGH);
}

// Set the left motor to move backward
void RCModule::motorBackwardL(){
    digitalWrite(InL1, HIGH);
    digitalWrite(InL2, LOW);
}

// Set the right motor to move backward
void RCModule::motorBackwardR(){
    digitalWrite(InR1, HIGH);
    digitalWrite(InR2, LOW);
}

// Stop left motor
void RCModule::motorStopL(){
    digitalWrite(InL1, LOW);
    digitalWrite(InL2, LOW);
}

// Stop right motor
void RCModule::motorStopR(){
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, LOW);
}

// Initialize the pin-mode of electromagnet pin MagF and initialize it to LOW(0V)
void RCModule::MagInit(){
  pinMode(MagF, OUTPUT);
  digitalWrite(MagF, LOW);
}

// switch on the electromagnet
void RCModule::MagPick(void)  {
  digitalWrite(MagF, HIGH);
}

// switch off the electromagnet
void RCModule::MagDrop(void)  {
  digitalWrite(MagF, LOW);
}
