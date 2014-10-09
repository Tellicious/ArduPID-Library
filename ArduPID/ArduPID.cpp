/**********************************************************************************************
 * Arduino PID Library
 * by Andrea Vivani <andrea.vivani@gmail.com> 
 *
 * This Library is licensed under a GPLv2 License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <ArduPID.h>
// Auxiliary Functions
template <typename T>  T constrain2(T in, T inf, T sup)
{
if (in>sup){ return sup;}
else if (in<inf){ return inf;}
else{ return in;}
}

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Output,double Kp,double Kd,double Ki,double N,uint16_t T,int Direction,int Aw,double Kb)
{
	_dir=Direction;
	_N_d=N;
	_T=T;
	_T_sec=(double) _T/1e6;
	_kf=(2-_N_d*_T_sec)/(2+_N_d*_T_sec);
	
	if(_dir>0) {
	_kp=Kp;
	_kd=(2*Kd*_N_d)/(2+_N_d*_T_sec);
	_ki=0.5*Ki*_T_sec;
	}
	else{
	_kp=-Kp;
	_kd=-(2*Kd*_N_d)/(2+_N_d*_T_sec);
	_ki=-0.5*Ki*_T_sec;
	}
	
	if((Aw>0)&&(_ki!=0)) {
	_kb=0.5*Kb*_T_sec;
	}
	else {
	_kb=0;
	}
	PID::SetOutputLimits(-1e6, 1e6);				//default output limit are big enough not to limit anything(I hope)
	
    _lastTime = micros();
    
    _e_old=0;
    _u_old_d=0;
    _daw=0;
    _Du_i=0;
    _Output = Output;
    _daw_old=0;				
}
 
 
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
void PID::Compute(double e)
{
	double Du_p=_kp*e;
	_Du_i+=_ki*(e+_e_old);
	double Du_d=_kf*_u_old_d+_kd*(e-_e_old);
	//double u=Du_p+_Du_i+Du_d+_daw;
	double u=Du_p+_Du_i+Du_d;
	//_daw+=_kb*(constrain2(u,_outMin,_outMax)-u);
	double daw=constrain2(u,_outMin,_outMax)-u;
	_Du_i+=0.5*_T_sec*(daw+_daw_old);
	//_Du_i+=_daw;
    _u_old_d=Du_d;
    _e_old=e;
    _daw_old=daw;
    *_Output=constrain2((Du_p+_Du_i+Du_d),_outMin,_outMax);
    
}

bool PID::Compute_auto(double e)
{
	uint16_t now = micros();
   if((now-_lastTime)>=_T){
   _lastTime=now;
	double Du_p=_kp*e;
	_Du_i+=_ki*(e+_e_old);
	double Du_d=_kf*_u_old_d+_kd*(e-_e_old);
	double u=Du_p+_Du_i+Du_d;
	_daw+=_kb*(constrain2(u,_outMin,_outMax)-u);
	_Du_i+=_daw;
    _u_old_d=Du_d;
    _e_old=e;
    if ((micros()-_lastTime)<_T){
    *_Output=constrain2((u+_daw),_outMin,_outMax);
    return true;
    }
    else{*_Output=0;return false;}
    
    }
    else if ((now-_lastTime)<0){ _lastTime=0;}
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void PID::SetTunings(double Kp, double Kd, double Ki,double N)
{
   if (Kp<0 || Ki<0 || Kd<0 || N<0) return;
   _N_d=N;
 if(_dir>0) {
	_kp=Kp;
	_kd=(2*Kd*_N_d)/(2+_N_d*_T_sec);
	_ki=0.5*Ki*_T_sec;
	}
	else{
	_kp=-Kp;
	_kd=-(2*Kd*_N_d)/(2+_N_d*_T_sec);
	_ki=-0.5*Ki*_T_sec;
	}
   
}
  
/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   _outMin = Min;
   _outMax = Max;
}


/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp(){ return  _kp; }
double PID::GetKi(){ return  (2*_ki/_T_sec);}
double PID::GetKd(){ return  (0.5*(2+_N_d*_T_sec)*_kd/_N_d);}
