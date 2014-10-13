/*-----------------------------------------------------------------------------------/
  Arduino PID Library Parent Class
  by Andrea Vivani <andrea.vivani@gmail.com> 
  This Library is licensed under GPLv3 License
/-----------------------------------------------------------------------------------*/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <PID_def.h>
//-----------------------------Auxiliary Functions----------------------------------//
template <typename T, typename T2, typename T3>  T constrain2(T in, T2 inf, T3 sup)
{
if (in>sup){ return (T) sup;}
else if (in<inf){ return (T) inf;}
else{ return in;}
}

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID_def::PID_def(double* Output,double Kp,double Ki,double Kd,double N,uint32_t T,int Aw,double Kb)
{	
	_aw=Aw;
	_N_d=N;
	_T=T;
	_T_sec=(double) _T/1e6;
	_kf=(2-_N_d*_T_sec)/(2+_N_d*_T_sec);
	
	_kp=Kp;
	_kd=(2*Kd*_N_d)/(2+_N_d*_T_sec);
	_ki=0.5*Ki*_T_sec;

	if((_aw==1)&&(_ki!=0)) {
	_kb=0.5*Kb*_T_sec;
	}
	else {
	_kb=0;
	}
	PID_def::SetSaturation(OUTMIN, OUTMAX);				//default output limit are big enough not to limit anything(I hope)
	
    _lastTime = micros();
    
    _e_old=0;
    _Du_d=0;
    _Du_i=0;
    _Output = Output;			
}

 
 
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   PID_def Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
bool PID_def::AutoCompute(double e)
{
	uint32_t now = micros();
   if((now-_lastTime)>=_T){
   _lastTime=now;
	this->Compute(e);
    if ((micros()-_lastTime)>=_T){
    *_Output=0;
    return false;
    }
    else{return true;}
    
    }
    else if ((now-_lastTime)<0){ _lastTime=0L;}
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void PID_def::SetTunings(double Kp, double Ki, double Kd,double N)
{
   if (Kp<0 || Ki<0 || Kd<0 || N<0) return;
   _N_d=N;
	_kp=Kp;
	_kd=(2*Kd*_N_d)/(2+_N_d*_T_sec);
	_ki=0.5*Ki*_T_sec;
   
}
 void PID_def::SetBackCalc(double Kb){
  if((_aw==1)&&(_ki!=0)) {
	_kb=0.5*Kb*_T_sec;
	}
	else {
	_kb=0;
	}
	}
/* SetSaturation(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID_def::SetSaturation(double Min, double Max)
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
double PID_def::GetKp(){ return  _kp; }
double PID_def::GetKi(){ return  (2*_ki/_T_sec);}
double PID_def::GetKd(){ return  (0.5*(2+_N_d*_T_sec)*_kd/_N_d);}
