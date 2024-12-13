#ifndef PID_IC_h
#define PID_IC_h
#define LIBRARY_VERSION	1.0
#include "PID_def.h"
/*-----------------------------------------------------------------//
  This library if for the PID with the Integrator Clamping Anti-Windup Technique
//-----------------------------------------------------------------*/

class PID_IC: public PID_def {
	public:
    PID_IC(float* Output, float Kp, float Ki, float Kd, float N, uint32_t T):
    PID_def(Output, Kp, Ki, Kd, N, T, 2, 0){}   //constructor with initial tuning parameters (Output,Kp,Ki,Kd,N,T), with T in ms
    void compute(float e);                       //calculates the output 
    void reset();											//restarts the PID
};
#endif
