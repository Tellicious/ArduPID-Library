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
void PID_IC::Compute(float e) {
	float Du_p = _kp * e;
	_Du_d = _kf * _Du_d + _kd * (e - _e_old);
	/*
	float Delta_i = _ki * (e + _e_old_i);
	float u = Du_p + _Du_i + _Du_d + Delta_i;
	if ((((e * u) > 0) && ((u > _outMax) || (u < _outMin)))){
		_Du_i += _ki * _e_old_i;
		_e_old_i = 0;
	}
	else{
		_Du_i += Delta_i;
		_e_old_i = e;
	}
    _e_old = e;
    *_Output = constrain2((Du_p + _Du_i + _Du_d), _outMin, _outMax);
	*/
	_Du_i += _ki * (e + _e_old_i);
	float u = _kp * e + _Du_i + _Du_d;
	if ((((e * u) > 0) && ((u > _outMax) || (u < _outMin)))){
		float d_int = _ki * e;
		_Du_i -= d_int;
		u -= d_int;
		_e_old_i = 0;
	}
	else{
		_e_old_i = e;
	}
	_e_old = e;
	*_Output = constrain2(u, _outMin, _outMax);
}

/*-------------------------------------Reset---------------------------------------//
This function can be called everywhere in order to re-initialize the PID
 //---------------------------------------------------------------------------------*/
void PID_IC::Reset() {
PID_def:Reset_def();
_e_old_i = 0;
}