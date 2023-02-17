/*-----------------------------------------------------------------------------------//
  Arduino Standard PID Library
  by Andrea Vivani <andrea.vivani@gmail.com> 
  This Library is licensed under GPLv3 License
//-----------------------------------------------------------------------------------*/

#include <PID.h>
 
/*--------------------------------Calculate Output-------------------------------//
This function should be called inside an If statement that checks if the time 
elapsed since the last step is greater or equal than the step time
 //---------------------------------------------------------------------------------*/
void PID::compute(float e) {
	float Du_p = _kp * e;
	_Du_i += _ki * (e + _e_old);
	_Du_d = _kf * _Du_d + _kd * (e - _e_old);
    _e_old = e;
    *_Output = constrain2((Du_p + _Du_i + _Du_d), _outMin, _outMax);
}

/*-------------------------------------Reset---------------------------------------//
This function can be called everywhere in order to re-initialize the PID
 //---------------------------------------------------------------------------------*/
void PID::reset() {
PID_def:reset_def();
}