//This is the PID struct used for the PID_Controller in the original source
/*
struct PIDInfo {
  float r; //reference (set point)
  float y; //current measured position (process variable)
  float Kp; 
  float Kd;
  float Ki;
  float imax; //maximum contribution of the integrator term
  float Dd; //derivative filter bandwidth (P)
  float Ad; //exp(-P*Ts)
  float Bd; //(Ad - 1)*P
  float Xd; //derivative filter state
  float Xi; //integral filter state
};
//This is the function in the original source
float PIDController (PIDInfo p, float r, floay y) {
  int  result;
  //update pid info and compute error
  p->r = r;
  p->y = y;
  float e = r - y;
  
  result = (p->Kp * e + p->Kd * (p->Xd + p->Dd * e) + p->Ki * p->Xi);
  
  p->Xd = p->Ad * p->Xd + p->Bd * e; //update derivative filter
  p->Xi += e; //update integrator value
  if (p->Xi > p->Imax) p->Xi = p->Imax;
  if (p->Xi < -p->Imax) p->Xi = p->Imax;
  
  return result;
  
*/
//Here is my attempt at working the PID arduino library:
#include <PID_v3.h>
//important to note: the PID arduino library runs on doubles not floats
//consider conversion problems (in relation to other parts of control)
float PWM_LIMIT = 256;
int sampleTime; //SET THIS SHIT YO.
float steerRef, steerState leanRef, leanState, steerValveCmd, leanValveCmd;
//structure: myPID(&input, &output, &setpoint, Kp, Ki, Kd, direction)
// steerState is the current position of the steer - I assume.
// steerRef is the position of the joystick in relation to the steer - I assume.
// similarly with leanState and leanRef
PID steerPID(&steerState, &steerValveCmd, &steerRef, 25.5, 0, 0.02, DIRECT);
PID leanPID(&leanState, &leanValveCmd, &leanRef, 40.5, 0, 0, DIRECT);

void setup() {
  steerPID.SetOutputLimits(-PWM_LIMIT, PWM_LIMIT);
  leanPID.SetOutputLimits(-PWM_LIMIT, PWM_LIMIT);
  //AUTOMATIC is on, MANUAL is off (in regards to auto-computation)
  steerPID.SetMode(AUTOMATIC);
  leanPID.SetMode(AUTOMATIC);
  steerPID.SetSampleTime(sampleTime);
  leanPID.SetSampleTime(sampleTime);
}

void loop() {
  //do your stuff setting up the steerRef, steerState, leanRef, leanState
  //... blah blah
  // then you want to turn on the PID
  steerPID.SetMode(AUTOMATIC);
  leanPID.SetMode(AUTOMATIC);
  steerPID.Compute();
  //do stuff with your steerValveCmd
  leanPID.Compute();
  //do stuff with your leanValveCmd
}
