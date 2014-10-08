#ifndef ArduPID_h
#define ArduPID_h
#define LIBRARY_VERSION	1.0

class PID
{


  public:

  //Constants used in some of the functions below
  #define DIRECT  1
  #define REVERSE 0 
  #define AW_ON 1
  #define AW_OFF 0

  //commonly used functions **************************************************************************
    PID(double*,double,double,double,double,uint16_t,int,int,double);      // * constructor  Initial tuning parameters (Output,Kp,Kd,Ki,N,T,DIRECTION/ANTIWINDUP/Kb)

    void Compute(double);                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively
                                          
    bool Compute_auto(double);
                                          

    void SetOutputLimits(double, double); //clamps the output to a specific range. 0-255 by default, but
										  //it's likely the user will want to change this depending on
										  //the application
	


  //available but not commonly used functions ********************************************************
    void SetTunings(double, double,       // * While most users will set the tunings once in the 
                    double, double);         	  //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
										  
										  
  //Display functions ****************************************************************
	double GetKp();						  // These functions query the pid for interal values.
	double GetKi();
	double GetKd();						  

  private:
	      
	      
	       
	double _kp;                  // * (P)roportional Tuning Parameter
    double _ki;                  // * (I)ntegral Tuning Parameter
    double _kd;                  // * (D)erivative Tuning Parameter
    double _N_d;					// * Derivative Filter Constant
    uint16_t _T;						// * Loop Time in Microseconds
    double _T_sec;					// * Loop Time in Seconds
	double _kb;						// * Back-Calculation AntiWindup Parameter
	double _kf;						// * Derivative Filter Multiplication Constant
	
 	double *_Output; 
 	
	uint16_t _lastTime;
	double _outMin, _outMax;
	int _dir;
	
	//support variables
	double _e_old ;
	double _u_old_d ;
	double _daw;
	double _Du_i;
	double _daw_old;
	
	
	
};
#endif

