#ifndef PID_h
#define PID_h
#define LIBRARY_VERSION	1.0
#include "PID_def.h"
/*-----------------------------------------------------------------//
  This is the library for the standard PID (no anti-windup) 
//-----------------------------------------------------------------*/

class PID: public PID_def {
	public:
    PID(float* Output, float Kp, float Ki, float Kd, float N, uint32_t T):
    PID_def(Output, Kp, Ki, Kd, N, T, 0, 0){}   // constructor with initial tuning parameters (Output,Kp,Ki,Kd,N,T), with T in ms
    void compute(float e);                       //  calculates the output 
    void reset();											//restarts the PID                                    
};
#endif

