#ifndef _RCModule_h
#define _RCModule_h

#include "Arduino.h"

class RCModule{
    public:
        RCModule(int InL1Val, int InL2Val, int InR1Val, int InR2Val, int MagFVal);
        bool readXBee(void);
        void iterate();
        void controlMotors(void);
        void controlMagnet(void);
        void getJoystickData(void);
        void motorInit(void);
        void MagInit(void);
        void MagPick(void);
        void MagDrop(void);

    private:
        unsigned char data[15];
        int control[2];
        unsigned char tmpData[15], tmpByte;
        int checkSum = 0x83;
        int eMState;
        int MagF, InL1, InL2, InR1, InR2;

        void motorForwardL(void);
        void motorForwardR(void);
        void motorBackwardL(void);
        void motorBackwardR(void);
        void motorStopL(void);
        void motorStopR(void);
        
};
    
#endif