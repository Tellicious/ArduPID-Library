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
uint32_t T = 20e3; //20e3us => 50Hz cycle frequency
uint32_t lastTime = 0;
PID PID1(&U1, kp, ki, kd, N, T); //standard PID, no anti-windup
PID_IC PID2(&U2, kp, ki, kd, N, T);  //PID with integrator clamping anti-windup
PID_BC PID3(&U3, kp, ki, kd, N, T, kb); //PID with back-calculation anti-windup

void setup() {
  PID2.SetSaturation(-1000, 1000); //sets lower and upper limit to the PID output;
  //advisable to use an an anti-windup PID if sat needed
  PID3.SetSaturation(-1000, 1000);

}

void loop() {
  uint32_t now = micros();
  if ((now - lastTime) >= T) {
    lastTime = now;
    double e1 = analogRead(A1); //this is an example: the Compute function receives as input the error
    double e2 = analogRead(A2);
    double e3 = analogRead(A3);
    PID1.Compute(e1);
    PID2.Compute(e2);
    PID3.Compute(e3);

    if ((micros() - lastTime) < T) {
      analogWrite(4, U1); //this is just an example, using an example pin
      analogWrite(5, U2);
      analogWrite(6, U3);
    }
  }
  else if ((now - lastTime) < 0) {
    lastTime = 0L;
  }
  if (digitalRead(2) == 1) { //example: if this pin is set to high, the PID are re-initialized
    PID1.Reset();
    PID2.Reset();
    PID3.Reset();
  }
}

/* Other functions are:
PID.SetTunings(kp,ki,kd,N); //changes the gains
PID.SetBackCalc(kb); //changes the back-calculation gain
double Kp=PID.GetKp(); //gives the actual value of Kp
double Ki=PID.GetKi(); //gives the actual value of Ki
double Kd=PID.GetKd(); //gives the actual value of Kd
double Kb=PID.GetKb(); //gives the actual value of Kb
*/
