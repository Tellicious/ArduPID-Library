/*-----------------------------------------------------------------------------------//
  Arduino PID Library (Back-Calculation Anti-Windup)
  by Andrea Vivani <tellicious@icloud.com> 
  This Library is licensed under GPLv3 License
//-----------------------------------------------------------------------------------*/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <PID_BC.h>
 
/*--------------------------------Calculate Output-------------------------------//
This function should be called inside an If statement that checks if the time 
elapsed since the last step is greater or equal than the step time
 //---------------------------------------------------------------------------------*/
void PID_BC::Compute(double e)
{
	double Du_p=_kp*e;
	_Du_i+=_ki*(e+_e_old);
	_Du_d=_kf*_Du_d+_kd*(e-_e_old);
	double u=Du_p+_Du_i+_Du_d;
	double aw_bc=constrain2(u,_outMin,_outMax)-u;
	_Du_i+=_kb*(aw_bc+_aw_bc_old);
    _e_old=e;
    _aw_bc_old=aw_bc;
    *_Output=constrain2((Du_p+_Du_i+_Du_d),_outMin,_outMax);
}

/*-------------------------------------Reset---------------------------------------//
This function can be called everywhere in order to re-initialize the PID
 //---------------------------------------------------------------------------------*/
void PID_BC::Reset()
{
PID_def:Reset_def();
_aw_bc_old=0;
}