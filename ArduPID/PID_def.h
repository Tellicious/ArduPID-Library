#ifndef PID_def_h
#define PID_def_h
#define LIBRARY_VERSION	1.0

class PID_def
{


  public:
  //Parameters
  #define OUTMIN -1e6
  #define OUTMAX 1e6

  //commonly used functions **************************************************************************
    PID_def(double*,double,double,double,double,uint32_t,int,double);      // * constructor  Initial tuning parameters (Output,Kp,Ki,Kd,N,T,ANTIWINDUP,Kb)

    virtual void Compute(double)=0;                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively
                                          
    bool AutoCompute(double);
                                          

    void SetSaturation(double, double); //clamps the output to a specific range. 0-255 by default, but
										  //it's likely the user will want to change this depending on
										  //the application
	

  //available but not commonly used functions ********************************************************
    void SetTunings(double, double,       // * While most users will set the tunings once in the 
                    double, double);         	  //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
										
	void SetBackCalc(double);  
										  
  //Display functions ****************************************************************
	double GetKp();						  // These functions query the pid for interal values.
	double GetKi();
	double GetKd();						  

  protected:
	      
	      
	       
	double _kp;                  // * (P)roportional Tuning Parameter
    double _ki;                  // * (I)ntegral Tuning Parameter
    double _kd;                  // * (D)erivative Tuning Parameter
    double _N_d;					// * Derivative Filter Constant
    uint32_t _T;						// * Loop Time in Microseconds
    double _T_sec;					// * Loop Time in Seconds
	double _kb;						// * Back-Calculation AntiWindup Parameter
	double _kf;						// * Derivative Filter Multiplication Constant
	
 	double *_Output; 
 	
	uint32_t _lastTime;
	double _outMin, _outMax;
	int _aw;
	
	//support variables
	double _e_old;
	double _Du_i,_Du_d;
	
	
	
};
#endif

