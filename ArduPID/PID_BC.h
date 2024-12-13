#ifndef PID_BC_h
#define PID_BC_h
#define LIBRARY_VERSION	1.0
#include "PID_def.h"
/*-----------------------------------------------------------------//
  This library if for the PID with the Back-Calculation Anti-Windup Technique
//-----------------------------------------------------------------*/

class PID_BC: public PID_def {
	public:
    PID_BC(float* Output, float Kp, float Ki, float Kd, float N, uint32_t T, float Kb):
    PID_def(Output, Kp, Ki, Kd, N, T, 1, Kb){}  //constructor with initial tuning parameters (Output,Kp,Ki,Kd,N,T,Kb), with T in ms
    void compute(float e);                       //calculates the output 
	  void reset();											//restarts the PID                       //stores the back calculation contribution of the previous step         
};
#endif
