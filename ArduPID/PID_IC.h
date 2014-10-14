#ifndef PID_IC_h
#define PID_IC_h
#define LIBRARY_VERSION	1.0
#include "PID_def.h"
/*-----------------------------------------------------------------//
  This library if for the PID with the Integrator Clamping Anti-Windup Technique
//-----------------------------------------------------------------*/

class PID_IC: public PID_def
{
	 public:

    PID_IC(double* Output,double Kp,double Ki,double Kd,double N,uint32_t T):
    PID_def( Output,Kp,Ki,Kd,N,T,2,0){_e_old_i=0;}   //constructor with initial tuning parameters (Output,Kp,Ki,Kd,N,T)

    void Compute(double);                       //calculates the output 
    void Reset();											//restarts the PID
	
    private:
    
    double _e_old_i;                					//stores the error of the previous step, used to calculate the integral contribution     
};
#endif
