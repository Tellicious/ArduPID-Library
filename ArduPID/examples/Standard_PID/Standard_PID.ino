#include "ArduPID.h"
float kp = 10; //proportional gain
float ki = 4; //integral gain
float kd = 5; //derivative gain
float N = 100; //derivative filter constant D(s)=s/(1+s/N)
//a good rule is: N > (10 * Kd / Kp) (also avoid too large values)
float kb = 1; //back-calculation constant (README.md) to be set only if PID_BC used
float U1 = 0; //output variable 1
float U2 = 0; //output variable 2
float U3 = 0; //output variable 3
uint32_t T = 20; //20ms => 50Hz cycle frequency
uint32_t lastTime = 0;
PID PID1(&U1, kp, ki, kd, N, T); //standard PID, no anti-windup
PID_IC PID2(&U2, kp, ki, kd, N, T);  //PID with integrator clamping anti-windup
PID_BC PID3(&U3, kp, ki, kd, N, T, kb); //PID with back-calculation anti-windup

void setup() {
  PID2.setSaturation(-1000, 1000); //sets lower and upper limit to the PID output;
  //advisable to use an an anti-windup PID if sat needed
  PID3.setSaturation(-1000, 1000);

}

void loop() {
  uint32_t now = millis();
  if ((now - lastTime) >= T) {
    lastTime = now;
	float reference = 1000;
    float actual1 = analogRead(A1);
    float actual2 = analogRead(A2);
    float actual3 = analogRead(A3);
	float e1 = reference - actual1; //first error
	float e2 = reference - actual2; //second error
	float e3 = reference - actual3; //third error
    PID1.compute(e1); //The Compute function receives the error as input
    PID2.compute(e2);
    PID3.compute(e3);
    if ((millis() - lastTime) < T) { //write output only if step took less than step time
      analogWrite(4, U1); //this is just an example, using an example pin
      analogWrite(5, U2);
      analogWrite(6, U3);
    }
  }
  else if ((now - lastTime) < 0) {
    lastTime = 0L;
  }
  if (digitalRead(2) == 1) { //example: if this pin is set to high, the PID are re-initialized
    PID1.reset();
    PID2.reset();
    PID3.reset();
  }
}

/* Other functions are:
PID.setTunings(kp, ki, kd, N); //changes the gains
PID.setBackCalc(kb); //changes the back-calculation gain
float Kp = PID.getKp(); //gives the actual value of Kp
float Ki = PID.getKi(); //gives the actual value of Ki
float Kd = PID.getKd(); //gives the actual value of Kd
float Kb = PID.getKb(); //gives the actual value of Kb
*/
