#ifndef PID_BC_h
#define PID_BC_h
#define LIBRARY_VERSION	1.0
#include "PID_def.h"
/*-----------------------------------------------------------------//
  This library if for the PID with the Back-Calculation Anti-Windup Technique
//-----------------------------------------------------------------*/

class PID_BC: public PID_def
{
	public:

    PID_BC(double* Output,double Kp,double Ki,double Kd,double N,uint32_t T,double Kb):
    PID_def( Output,Kp,Ki,Kd,N,T,1,Kb){_aw_bc_old=0;}  // constructor with initial tuning parameters (Output,Kp,Ki,Kd,N,T,Kb)
    
    void Compute(double);                       //  calculates the output 
     
     private: 
     
     double _aw_bc_old;                          // stores the back calculation contribution at the previous step         
};
#endif
