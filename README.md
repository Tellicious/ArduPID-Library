ArduPID
=======

A PID Library for Arduino digitalized with the Tustin's method with Anti-Windup by Andrea Vivani (tellicious@icloud.com)

To install drag the ArduPID folder to your Arduino=>libraries folder and include "ArduPID.h" in your script

This libray has some major improvements over the already existing libraries:
  - The PID has been digitalized with Tustin's method, in order to maintain the avoid the risk of become unstable because of the digitalization itself (this can happen using forward Euler, mainly). The Tustin's method numerically computes the integral using the trapezoidal rule and preserves the continuous time stability. To have more informations about Tustin's method refer to http://en.wikipedia.org/wiki/Bilinear_transform;
  - The derivative is low-pass filtered, using this approximation D(s)=s/(1+s/N), this in order to have a causal transfer function and to avoid enormous/infinite values, that can happen using D(s)=s. As an example, using D(s)=s and forward Euler, the digital derivative is approximated to D(k)=(u(k)-u(k-1))/T, where T is the sampling time. As T gets smaller, the D(k) value tends to be too much dependant on numeric issues and it is likely to become infinite and to vary almost instantly;
  - There are three different classes: PID, PID_BC, PID_IC; the first one does not have any anti-windup method, the second uses the back-calculation algorithm while the third uses the integral clamping algorithm. Integral windup can happen when the output of the control hits a saturation: if when saturated the error decreases too slowly, the integrator still increases the control output, even if not effective, and then when the error gets towards zero the integral is still over-charged and needs some time to decharge, causing the output to have an undesired behaviour. Usually the integral clamping method performs better. To have more infos about the integral windup and anti-windup methods refer to:
    - http://en.wikipedia.org/wiki/Integral_windup
    - http://www.mathworks.com/help/simulink/slref/pidcontroller.html
    - http://www.mathworks.com/help/simulink/examples/anti-windup-control-using-a-pid-controller.html


THE AUTOCOMPUTE FUNCTION HAS NOT BEEN TESTED YET, ALL THE OTHER ARE WORKING PERFECTLY FINE.

Feel free to report any issue you encounter.

This library is licensed under GPL v3.

Contact me if you need any help.
