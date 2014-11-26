/*
 *  Dagne Micro-Controller Main Control Loop 
 *
 *  This file contains the main control system that runs on the Arduino for the
 *  Dagne Electric Vehicle.
 *
 *  Created By: The Dream Team
 */

//The Arduino Uno does not have enough pins to output all the PWM outputs.
const int PWMone = 10; //Valve Coil FET. For Brake Pressure, Steer Angle, Lean Angle control
const int PWMtwo = 9;  //Valve Coil FET. (Same as above)

// Sensor Input Pins for Hydralic Valve(0-12v on/off) [see powerpoint]
const int pLeanSenseIn = 0; 
const int pSteerSenseIn = 1;
const int pSpeedSenseIn = 2;

// Reverse Switch, Hydraulic Pump, and Keyswitch Relay pins
const int pRevSw1 = 3;  //  Reversing contactor number 1
const int pRevSw2 = 4;  //  Reversing contactor number 2
const int pKeySwitch = 5;  // Throttle key switch to enable motor controller
const int HydPumpEn = 6;  		//  Hydraulic Pump Control Pin

// Reverse Switch Input
const int ParkBrakeEn = 7;

const int Aux_Pin = 8;


const int JoystickBtn1 = 12;
const int JoystickBtn2 = 13;

int outputJoy = 0;      
int outputSte = 0;  

int buttonState1 = 0;   
int buttonState2 = 0;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Serial.write("Dagne control loop begin");
  initDevice();
  pinMode(pSpeedSenseIn, OUTPUT);
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
  //analog inputs
  int joystickValy = readAnalog(A0);
  int joystickValx = readAnalog(A1);
  int PumpVal = readAnalog(A2);
  int BrakeSensor = readAnalog(A3);
   //PWM inputs
  int SteerSensor = readAnalog(A4);
  int LeanSensor = readAnalog(A4);
  int SpeedSensor = readAnalog(A4);
  
  PWMOutput2(joystickValx, SteerSensor, 10);
  button_test(); 
  led_test();
  digitalWrite(pSpeedSenseIn, HIGH);
  delay(30);        // delay in between reads for stability
}

//Initialization function for device
void initDevice(){
	pinMode(JoystickBtn1, INPUT);
    pinMode(JoystickBtn2, INPUT);
    pinMode(pSteerSenseIn, OUTPUT);
	pinMode(pLeanSenseIn, OUTPUT);
	//pinMode(pSpeedSenseIn, OUTPUT);
	pinMode(pRevSw1, OUTPUT);
	pinMode(pRevSw2, OUTPUT);
	pinMode(pKeySwitch, OUTPUT);
	pinMode(HydPumpEn, OUTPUT);
	pinMode(ParkBrakeEn, OUTPUT);
	pinMode(Aux_Pin, OUTPUT);
  
}

void button_test(){
  buttonState1 = digitalRead(JoystickBtn1);
  buttonState2 = digitalRead(JoystickBtn2);
  if (buttonState1 == HIGH) {
    Serial.println("btn1: on");
  }else{
    Serial.println("btn1: off"); 
  }
  if (buttonState2 == HIGH) {
    Serial.println("btn2: on");
  }else{
    Serial.println("btn: off"); 
  }
}

void led_test(){
  digitalWrite(pSteerSenseIn, HIGH);
  digitalWrite(pLeanSenseIn, HIGH);
  digitalWrite(pSpeedSenseIn, HIGH);
  digitalWrite(pRevSw1, HIGH);
  digitalWrite(pRevSw2, HIGH);
  digitalWrite(pKeySwitch, HIGH);
  digitalWrite(HydPumpEn, HIGH);
  digitalWrite(HydPumpEn, HIGH);
  digitalWrite(Aux_Pin, HIGH);
}

/* Main Control Functions */
void sampleSensors() {

}

void speedSteeringControlMap() {

}

void adjustForLeanMode() {

}

//Amount of lean/steer/break/accel as requested by driver input
float leanRef = 0.0;
float steerRef = 0.0;
float brakeRef = 0.0;
float accelRef = 0.0;
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
//TODO: Fill in switch statements 
void setRevValues(){
    switch (revState){
    case DIRECTION_CHOOSE:
      setRevDir();
      break;
    case FORWARD:
      break;
    case REVERSE:
      break;
    case PARK:
      break;
  }
}

// If in direction choose mode, set rev direction based on joystick input
void setRevDir(){

}


/* IO Functions */

int readAnalog(int AnalogPin) {
  int sensorValue = analogRead(AnalogPin);
  int output = map(sensorValue, 0, 1023, 0, 255);
  return output;
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
