#ifndef PID_h
#define PID_h
#define LIBRARY_VERSION	1.0
#include "PID_def.h"
/*-----------------------------------------------------------------//
  This is the library for the standard PID (no anti-windup) 
//-----------------------------------------------------------------*/

class PID: public PID_def
{
	public:

    PID(double* Output,double Kp,double Ki,double Kd,double N,uint32_t T):
    PID_def( Output,Kp,Ki,Kd,N,T,0,0){}   // constructor with initial tuning parameters (Output,Kp,Ki,Kd,N,T)

    void Compute(double);                       //  calculates the output 
    void Reset();											//restarts the PID                                    
};
#endif

