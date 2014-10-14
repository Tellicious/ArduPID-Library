#include "ArduPID.h"
double kp = 10; //proportional gain
double ki = 4; //integral gain
double kd = 5; //derivative gain
double N = 100; //derivative filter constant D(s)=s/(1+s/N)
//a good rule is: N>10*Kd/Kp (also avoid too large values)
double kb = 1; //back-calculation constant (README.md) to be
//set only if PID_BC used
double U1 = 0; //output variable 1
double U2 = 0; //output variable 2
double U3 = 0; //output variable 3
uint32_t T = 20; //20ms => 50Hz cycle frequency
PID PID1(&U1, kp, ki, kd, N, T); //standard PID, no anti-windup
PID_IC PID2(&U2, kp, ki, kd, N, T);  //PID with integrator clamping anti-windup
PID_BC PID3(&U3, kp, ki, kd, N, T, kb); //PID with back-calculation anti-windup

void setup() {
  PID2.SetSaturation(-1000, 1000); //sets lower and upper limit to the PID output;
  //advisable to use an an anti-windup PID if sat needed
  PID3.SetSaturation(-1000, 1000);

}

void loop() {
  double e1 = analogRead(A1); //this is an example: the Compute function receives as input the error
  double e2 = analogRead(A2);
  double e3 = analogRead(A3);
  PID1.AutoCompute(e1); //if possible use the standard PID: this way you have more control on what is going on
  PID2.AutoCompute(e2); 
  PID3.AutoCompute(e3);
  analogWrite(4, U1); //this is just an example, using an example pin
  analogWrite(5, U2);
  analogWrite(6, U3);
}

