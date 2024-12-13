/*-----------------------------------------------------------------------------------//
  Arduino PID Library (Integral Clamping Anti-Windup)
  by Andrea Vivani <tellicious@icloud.com> 
  This Library is licensed under GPLv3 License
//-----------------------------------------------------------------------------------*/

#include <PID_IC.h>
 
/*--------------------------------Calculate Output-------------------------------//
This function should be called inside an If statement that checks if the time 
elapsed since the last step is greater or equal than the step time
 //---------------------------------------------------------------------------------*/
void PID_IC::compute(float e) {
    _Du_i += _ki * e;
    _Du_d += _kd * e;
    float tmpOutput = _kp * e + _Du_i + _Du_d;
    if (((e * tmpOutput) > 0) && ((tmpOutput < _outMin) || (tmpOutput > _outMax))) {
        _Du_i -= _ki * e;
        tmpOutput -= _ki * e;
    } else {
        _Du_i += _ki * e;
    }
    *_Output = constrain2(tmpOutput, _outMin, _outMax);
    /* Prepare variables for next step */
    _Du_d = _kf * _Du_d - _kd * e;
}

/*-------------------------------------Reset---------------------------------------//
This function can be called everywhere in order to re-initialize the PID
 //---------------------------------------------------------------------------------*/
void PID_IC::reset() {
	PID_def:reset_def();
}