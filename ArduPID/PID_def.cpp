/*-----------------------------------------------------------------------------------/
  Arduino PID Library Parent Class
  by Andrea Vivani <tellicious@icloud.com> 
  This Library is licensed under GPLv3 License
/-----------------------------------------------------------------------------------*/

#include <PID_def.h>

/*------------------------------------Constructor------------------------------------//
The constructor is called by the child classes to set all the starting parameters,
including the sampling time in us and the back-calculation gain
 //---------------------------------------------------------------------------------*/
PID_def::PID_def(float* Output,float Kp,float Ki,float Kd,float N,uint32_t T,uint8_t Aw,float Kb) {	
	_aw = Aw;
	_N_d = N;
	_T = T;
	_T_sec = (float) _T / 1e3;
	_kf = (2 - _N_d * _T_sec) / (2 + _N_d * _T_sec);
	_kp = Kp;
	_kd = (2 * Kd * _N_d) / (2 + _N_d * _T_sec);
	_ki = 0.5 * Ki * _T_sec;
	if((_aw == 1) && (_ki != 0)) {
		_kb = 0.5 * Kb * _T_sec;
	}
	else {
	_kb = 0;
	}
	PID_def::setSaturation(OUTMIN, OUTMAX);				//default output limit are big enough not to limit anything(I hope)
    _lastTime = millis();
    _e_old = 0;
    _Du_d = 0;
    _Du_i = 0;
    _Output = Output;			
}

/*--------------------------------AutoCalculate Output------------------------------//
This function can be called inside the loop() environment without the need for
a loop-time check. It should be called as the condition of an if statement so that
if it returns false, the program will stop, otherwise it can continue
 //---------------------------------------------------------------------------------*/
bool PID_def::autoCompute(float e) {
	uint32_t now = millis();
	if((now - _lastTime) >= _T){
		_lastTime = now;
		this->compute(e);
		if ((millis() - _lastTime) > _T){
			*_Output = 0;
			return false;
		}
		else{return true;}
    }
    else if ((now - _lastTime) < 0){_lastTime = 0L;}
}

/*------------------------------------SetTunings------------------------------------//
This function can be called everywhere to change the PID gains after initialization
 //---------------------------------------------------------------------------------*/
void PID_def::setTunings(float Kp, float Ki, float Kd, float N) {
   if (Kp < 0 || Ki < 0 || Kd < 0 || N < 0) return;
  	_N_d = N;
	_kp = Kp;
	_kf = (2 - _N_d * _T_sec) / (2 + _N_d * _T_sec);
	_kd = (2 * Kd * _N_d) / (2 + _N_d * _T_sec);
	_ki = 0.5 * Ki * _T_sec;
}

/*------------------------------------SetBackCalc-----------------------------------//
This function can be called everywhere to change the Back-calculation gain after
initialization. The gain is set only if an integral action is present and if the 
anti-windup method is set to "Back-Calculation"
 //---------------------------------------------------------------------------------*/
 void PID_def::setBackCalc(float Kb) {
	if((_aw == 1) && (_ki != 0)) {
		_kb = 0.5 * Kb * _T_sec;
	}
	else {
		_kb = 0;
	}
}
	
/*-----------------------------------SetSaturation----------------------------------//
This function can be called to set upper and lower limits for the output. It is 
advisable to call this function ONLY if the chosen PID has an anti-windup logic,
in order to avoid the integral to windup causing undesired output behaviours
 //---------------------------------------------------------------------------------*/
void PID_def::setSaturation(float Min, float Max) {
   if(Min >= Max) return;
   _outMin = Min;
   _outMax = Max;
}

/*------------------------------------Reset_def-------------------------------------//
This function is called by the reset() function in the child classes to reset the
support variables to 0, that means to re-initialize the PID to starting conditions.
 //---------------------------------------------------------------------------------*/
void PID_def::reset_def() {
	_e_old = 0;
	_Du_i = 0;
	_Du_d = 0;
}


/*---------------------------------Display Functions--------------------------------//
The following functions are used to know what the gains are, for example to know what
gain has been set if changed dynamically with a potentiometer or to know if the
previous functions worked correctly in setting the gains
 //---------------------------------------------------------------------------------*/
float PID_def::getKp(){ return  _kp; }
float PID_def::getKi(){ return  (2 * _ki / _T_sec);}
float PID_def::getKd(){ return  (0.5 * (2 + _N_d * _T_sec) * _kd / _N_d);}
float PID_def::getKb(){ return  (2 * _kb / _T_sec);}
