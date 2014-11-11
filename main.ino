/*
 *  Dagne Micro-Controller Main Control Loop 
 *
 *  This file contains the main control system that runs on the Arduino for the
 *  Dagne Electric Vehicle.
 *
 *  Created By: The Dream Team
 */

const int PWMone = 10;
const int PWMtwo = 9;

int outputJoy = 0;      
int outputSte = 0;  

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Serial.write("Dagne control loop begin");
}


//Used to indicate direction of car revving (a.k.a. FwdRevState)
typedef enum {PARK, DIRECTION_CHOOSE, FORWARD, REVERSE} revState_t;
revState_t revState = PARK;

// the loop routine runs over and over again forever:
void loop() {
  // TODO add print status
  sampleSensors(); 
  speedSteeringControlMap();
  adjustForLeanMode();
  steerControl();
  leanControl();
  brakeControl();
  hydraulicControl();
  tractionMotorCommandProcessing();
  setThrottle();
  setRevValues();

  int joystickVal = readJoystick(A0);
  int steerVal = readSteer(A1);
  PWMOutput1(joystickVal, steerVal, 9);
  PWMOutput2(joystickVal, steerVal, 10);
  delay(30);        // delay in between reads for stability
}

/* Main Control Functions */
void sampleSensors() {

}

void speedSteeringControlMap() {

}

void adjustForLeanMode() {

}

void steerControl() {

}

void leanControl() {

}

void brakeControl() {

}

void hydraulicControl() {

}

void tractionMotorCommandProcessing() {

}

//These values address the saturation limits of throttle circuit in setThrottle
const float THROTTLE_FWD_GAIN = 0.040;
const float THROTTLE_REV_GAIN = 0.0990;
const float THROTTLE_OFFSET = 280.0;

const float FWD_LIMIT = 4095.0; //Full throttle
const float REV_LIMIT = 600.0; //Limit throttle when reversing

float accelRef = 0.0; //Acceleration amount as requested by driver

//Sets controlVal based on user throttle input and saturation limits
void setThrottle() {
  float controlVal;     //TODO: Make ControlVal actually do something PID related

  switch (revState){
    case FORWARD:
      controlVal = FWD_LIMIT * THROTTLE_FWD_GAIN * accelRef + THROTTLE_OFFSET;
      if (controlVal > FWD_LIMIT){ controlVal = FWD_LIMIT; } //Saturation control
      break;
    case REVERSE:
      controlVal = REV_LIMIT * THROTTLE_REV_GAIN * accelRef + THROTTLE_OFFSET;
      if (controlVal > REV_LIMIT){ controlVal = REV_LIMIT; } //Saturation control
      break;
    default: //For park and directional choose state, clear controlVal
      controlVal = 0.0;
      break;
  }
  if (accelRef == 0.0){ controlVal = 0.0; } //If no acceleration requested, clear
}

//Changes revState or accelRef/brakeRef based on input
// TODO: Fill in switch statements 
void setRevValues(){
    switch (revState){
    case DIRECTION_CHOOSE: //Set Rev Direction based on joystick input
      break;
    case FORWARD:
      break;
    case REVERSE:
      controlVal = 0.0;
      break;
    case PARK:
      break;
  }
}


/* IO Functions */

int readJoystick(int JoystickPin) {
  int sensorValue = analogRead(JoystickPin);
  int outputJoy = map(sensorValue, 0, 1023, 0, 255);
  return outputJoy;
}

int readSteer(int SteerPin) {
  int sensorValue = analogRead(SteerPin);
  int outputJoy = map(sensorValue, 0, 1023, 0, 255);
  return outputJoy;
}

void PWMOutput1(int joyval, int sasval, int OutputPin) {
  int voodoo = joyval - sasval;
  int out = 64;
  if(voodoo > 0)
    out = map(voodoo, 0, 255, 64, 191);
  analogWrite(OutputPin, out);  
}

void PWMOutput2(int joyval, int sasval, int OutputPin) {
  int voodoo = joyval - sasval;
  int out = 64;
  if(voodoo < 0)
    out = map(voodoo, -255, 0, 64, 191);
  analogWrite(OutputPin, out);  
}
