/*-----------------------------------------------------------------------------------//
  Arduino PID Library (Integral Clamping Anti-Windup)
  by Andrea Vivani <tellicious@icloud.com> 
  This Library is licensed under GPLv3 License
//-----------------------------------------------------------------------------------*/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <PID_IC.h>
 
/*--------------------------------Calculate Output-------------------------------//
This function should be called inside an If statement that checks if the time 
elapsed since the last step is greater or equal than the step time
 //---------------------------------------------------------------------------------*/
void PID_IC::Compute(double e)
{
	double Du_p=_kp*e;
	double Delta_i=_ki*(e+_e_old_i);
	_Du_d=_kf*_Du_d+_kd*(e-_e_old);
	double u=Du_p+_Du_i+_Du_d+Delta_i;
	if ((((e*u)>0)&&((u>_outMax)||(u<_outMin)))){
	_Du_i+=_ki*_e_old_i;
	_e_old_i=0;
	}
	else{
	_Du_i+=Delta_i;
	_e_old_i=e;
	}
    _e_old=e;
    *_Output=constrain2((Du_p+_Du_i+_Du_d),_outMin,_outMax);
}

/*-------------------------------------Reset---------------------------------------//
This function can be called everywhere in order to re-initialize the PID
 //---------------------------------------------------------------------------------*/
void PID_IC::Reset()
{
PID_def:Reset_def();
_e_old_i=0;
}