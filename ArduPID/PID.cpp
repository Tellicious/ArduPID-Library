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
  _Du_i += _ki * e;
  _Du_d += _kd * e;
  *_Output = constrain2(_kp * e + _Du_i + _Du_d, _outMin, _outMax);
  _Du_i += _ki * e;
  _Du_d = _kf * _Du_d - _kd * e;
}

/*-------------------------------------Reset---------------------------------------//
This function can be called everywhere in order to re-initialize the PID
 //---------------------------------------------------------------------------------*/
void PID::reset() {
  PID_def:reset_def();
}