/*
	Encoders.h - A Library for Encoders
	Created by Shivam Malviya, Jan 30, 2020.
*/
#ifndef Encoders_h
#define Encoders_h

#include "Arduino.h"
#include "digitalWriteFast.h"

#define RADIUS 0.03
#define RESOLUTION 540

class Encoders {

    public:
        byte A;
        byte B;
        volatile long rotation;
        volatile double lastRotation;
        volatile double phiDot;
        
    public:
        Encoders(byte A=0, byte B=0);
        void init();
        double getPhi();
        double getPhiDot();
        double getX();
        double getXDot();
        long getRotation();
};

#endif