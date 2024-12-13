/*-----------------------------------------------------------------------------------//
  Arduino PID Library (Back-Calculation Anti-Windup)
  by Andrea Vivani <tellicious@icloud.com> 
  This Library is licensed under GPLv3 License
//-----------------------------------------------------------------------------------*/

#include <PID_BC.h>
 
/*--------------------------------Calculate Output-------------------------------//
This function should be called inside an If statement that checks if the time 
elapsed since the last step is greater or equal than the step time
 //---------------------------------------------------------------------------------*/
void PID_BC::compute(float e) {
	float bcVal;
    _Du_d += _kd * e;
    float tmpOutput = _kp * e + _Du_i + _Du_d;
    if (tmpOutput > _outMax) {
        bcVal = _outMax - tmpOutput;
    } else if (tmpOutput < _outMin) {
        bcVal = _outMin - tmpOutput;
    } else {
        bcVal = 0;
    }
    _Du_i += _ki * e + _kb * bcVal;
    tmpOutput = _kp * e + _Du_i + _Du_d;
    *_Output = constrain2(tmpOutput, _outMin, _outMax);
    /* Prepare variables for next step */
    _Du_i += _ki * e + _kb * bcVal;
    _Du_d = _kf * _Du_d - _kd * e;
}

/*-------------------------------------Reset---------------------------------------//
This function can be called everywhere in order to re-initialize the PID
 //---------------------------------------------------------------------------------*/
void PID_BC::reset() {
	PID_def:reset_def();
}