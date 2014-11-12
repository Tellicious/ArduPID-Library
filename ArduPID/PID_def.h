#ifndef PID_def_h
#define PID_def_h
#define LIBRARY_VERSION	1.0

class PID_def
{

 	protected:
 	
  	//Parameters
  	#define OUTMIN -1e6     //default minimum saturation value
  	#define OUTMAX 1e6	  //default maximum saturation value
	#define constrain2(in,inf,sup) (in<inf?inf:(in>sup?sup:in))
  
  	public:
  	
    PID_def(double*,double,double,double,double,uint32_t,int,double);      //constructor  Initial tuning parameters (Output,Kp,Ki,Kd,N,T,Antiwindup,Kb)
																		   //with sampling time in ms

    virtual void Compute(double)=0;       //this is used to calculate the output, receives the error as input
                                          
    bool AutoCompute(double);			  //this is used to calculate the output without the need for a loop-time check, receives the error as input
                                         
    void SetTunings(double,double,double,double);		//this is used to change the gains after the initialization like to dynamically test their effect
										
	void SetBackCalc(double);  		//this is used to change the Back-Calculation gain after the initialization, only effective if the PID is defined as PID_BC
							
	void SetSaturation(double,double);   //this is used to set the output limits: it is advisable to select an anti-windup PID if it is necessary to limit
										  						//output
										  						
	virtual void Reset()=0;							//restarts the PID   
							
	double GetKp();						  //these are used to check what are the gains currently used by the PID
	double GetKi();
	double GetKd();		
	double GetKb();

  	protected:
  
	void Reset_def();		 	 //this is called by the Reset() function of the child classes to re-initialize the PID
	double _kp;                  //proportional gain
    double _ki;                  //integral gain
    double _kd;                  //derivative gain
    double _N_d;				 //derivative filter constant N: derivative in Laplace=s/(1+s/N)
    uint32_t _T;				 //loop time in milliseconds
    double _T_sec;				 //loop time in seconds
	double _kb;					 //back calculation anti-windup gain
	double _kf;					 //derivative filter coefficient (depends only on N) and multiplies the derivative output at the previous step
 	double *_Output; 			 //stores the pointer to the output variable
	uint32_t _lastTime;			 //stores the last time the cycle has been executed
	double _outMin, _outMax;	 //store the saturation limits
	int _aw;					 //stores the chosen anti-windup technique (0=off, 1=Back Calculation, 2=Integrator Clamping)
	
	//support variables
	double _e_old;				 //stores the error of the previous step
	double _Du_i,_Du_d;			 //store the integral and derivative contributions to the control action
	
	
	
};
#endif

