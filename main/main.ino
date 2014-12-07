/*
 *  Dagne Micro-Controller Main Control Loop 
 *
 *  This file contains the main control system that runs on the Arduino for the
 *  Dagne Electric Vehicle.
 *
 *  Created By: The Dream Team
 */

#define SPEED_SENS_MAX      30.0   //  Speed [mph] above which speed sensitive steering/leaning saturates
#define SPEED_SENS_MIN      4.0    //  Speed [mph] below which speed sensitive steering/leaning saturates


#define STEER_MAX_ANGLE     22.0
#define STEER_MIN_ANGLE     15.0
#define LEAN_MAX_ANGLE      29.0
#define LEAN_MIN_ANGLE      2.0

//range of speed values where speed sensative steering/leaning activates
#define SPEED_SENS_MAX      30.0
#define SPEED_SENS_MIN      4.0
//calculations for the amount of degrees you can change per unit of speed/lean
#define SPEED_LIMIT_STEER_SLOPE   (STEER_MIN_ANGLE - STEER_MAX_ANGLE)/(SPEED_SENS_MAX - SPEED_SENS_MIN)
#define SPEED_LIMIT_LEAN_SLOPE    (LEAN_MAX_ANGLE - LEAN_MIN_ANGLE)/(SPEED_SENS_MAX - SPEED_SENS_MIN)

//dead zones for the joystick values (to scale with 1)
#define LR_DEADZONE         0.02
#define FB_DEADZONE         0.08
//used to adjust left right values such that you have dead zones
//where -1 will still mean max left, and 1 will mean max right
#define LR_SCALE           1/(1-LR_DEADZONE)
#define FB_SCALE           1/(1-FB_DEADZONE)


#define LEAN_OFFSET         30      //  Dead zone offset value

//TODO: Is our PWM resolution different from theirs?
#define PWM_LIMIT           256    //  Practical upper limit, = (PWM_RESOLUTION - #_OFFSET) because OFFSET will push duty cycle to limit


//The Arduino Uno does not have enough pins to output all the PWM outputs.
const int PWMone = 10; //Valve Coil FET. For Brake Pressure, Steer Angle, Lean Angle control
const int PWMtwo = 9;  //Valve Coil FET. (Same as above)

// Joystick sensor pins
const int joystickLRSensor = A9;
const int joystickFBSensor = A3;

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

const int leanModeTogglePin = 8;

//Lean Valve Coil FET output pins
const int leanValvePinOne = 9;
const int leanValvePinTwo = 10;

const int JoystickBtn1 = 12;
const int JoystickBtn2 = 13;

int outputJoy = 0;      
int outputSte = 0;  

int buttonState1 = 0;
int buttonState2 = 0;

//limit calculated depending on speed
float steerAngleLimit = STEER_MAX_ANGLE;
//Current speed in MPH
float SpeedState = 0.0;

int LRanalogVal, FBanalogVal;
float joystick_LR, joystick_FB;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Serial.write("Dagne control loop begin");
  initDevice();
  pinMode(pSpeedSenseIn, OUTPUT);
  pinMode(HydPumpEn, OUTPUT);
  pinMode(leanValvePinOne, OUTPUT);
  pinMode(leanValvePinTwo, OUTPUT);
  pinMode(leanModeTogglePin, INPUT);
  pinMode(pLeanSenseIn, INPUT);
  pinMode(joystickLRSensor, INPUT);
  pinMode(joystickFBSensor, INPUT);
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
  FBanalogVal = readAnalog(joystickFBSensor);
  LRanalogVal = readAnalog(joystickLRSensor);
  //Print joystick input values for testing purposes
  Serial.print("Left-right analog value: ");
  Serial.println(LRanalogVal);
  Serial.print("Forward-brake analog value: ");
  Serial.println(FBanalogVal);
  int PumpVal = readAnalog(A2);
  int BrakeSensor = readAnalog(A3);
  //PWM inputs
  int SteerSensor = readAnalog(A4);
  int LeanSensor = readAnalog(A4);
  int SpeedSensor = readAnalog(A4);

  //convert to [-1,1] value
  joystick_LR = (LRanalogVal / 512.0) - 1.0;
  joystick_FB = (FBanalogVal / 512.0) - 1.0;

  setJoystickDeadzones();
  setSpeedSensitivity();

  PWMOutput2(joystick_LR, SteerSensor, 10);
  button_test(); 
  led_test();
  digitalWrite(pSpeedSenseIn, HIGH);
  delay(30);        // delay in between reads for stability
}

void setJoystickDeadzones(){
  //set the deadzones for the left right ranges of the joystick
  if (joystick_LR <= -LR_DEADZONE){
    joystick_LR = (joystick_LR + LR_DEADZONE) * LR_SCALE;
  }else if (joystick_LR > LR_DEADZONE){
    joystick_LR = (joystick_LR + LR_DEADZONE) * LR_SCALE;
  }else {
    joystick_LR = 0.0;
  }
  //set the deadzones for the foward brake ranges of the joystick
  if (joystick_FB <= -FB_DEADZONE) joystick_FB = (joystick_FB + FB_DEADZONE) * FB_SCALE;
  else if (joystick_LR > FB_DEADZONE) joystick_FB = (joystick_FB + FB_DEADZONE) * FB_SCALE;
  else joystick_LR = 0.0;
}
void setSpeedSensitivity(){
  //you want it so that the faster you go, the less sensative your steer
  steerAngleLimit = SPEED_LIMIT_STEER_SLOPE * SpeedState + STEER_MAX_ANGLE;
  if (steerAngleLimit > STEER_MAX_ANGLE) steerAngleLimit = STEER_MAX_ANGLE;
  if (steerAngleLimit < STEER_MIN_ANGLE) steerAngleLimit = STEER_MIN_ANGLE;
  //this is not in their code - but I'm wondering why it isn't.
  //it's supposed to account for bottom of the range where your
  //steering becomes speed sensative.
  //(delete if unnecessary, or put into an if-else loop if appropriate).
  if (SpeedState <= 4.0) {
    steerAngleLimit = STEER_MAX_ANGLE;
  }
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
	pinMode(leanModeTogglePin, OUTPUT);
  
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
  digitalWrite(leanModeTogglePin, HIGH);
}

class LeanController{
  public:
    void update(){
      leanOn = readLeanMode();
      leanAngleLimit = calcLeanAngleLimit();
      leanAngleState = calcLeanAngle();
      leanRef = calcLeanRef();
    }
    //Accessors
    void setSensorVal(int sensorVal);
    int getSensorVal();
    float getLeanRef();
    float getLeanAngleState();
    LeanController();
  private:
    int pLeanSenseInVal;
    bool leanOn;
    float leanAngleLimit; // Speed sensitive lean limits
    float leanAngleState;
    float leanRef;
    float leanError;
    int leanValveCmd;

    bool readLeanMode();
    float calcLeanAngleLimit();
    float calcLeanAngle();
    float calcLeanRef();
    void calcLeanValveOutputs();
};
LeanController *lc;

void LeanController::setSensorVal(int sensorVal){
  this->pLeanSenseInVal = pLeanSenseInVal;
}

int LeanController::getSensorVal(){
  return this->pLeanSenseInVal;
}

float LeanController::getLeanRef(){
  return this->leanRef;
}

float LeanController::getLeanAngleState(){
  return this->leanAngleState;
}
// True == car leans (like bike), False == lean mode off (like traditional car)
bool LeanController::readLeanMode(){
  return ((digitalRead(leanModeTogglePin) == HIGH) ? true : false);
}

// This limit differs from the hardcoded limits in that it changes based on speed
float LeanController::calcLeanAngleLimit(){
  float newLimit = SPEED_LIMIT_LEAN_SLOPE * SpeedState + LEAN_MIN_ANGLE;
  // Clamp limits to their maxima
  if (newLimit > LEAN_MAX_ANGLE){ newLimit = LEAN_MAX_ANGLE;}
  if (newLimit < LEAN_MIN_ANGLE){ newLimit = LEAN_MIN_ANGLE;}
  return newLimit;
}

float LeanController::calcLeanAngle(){
  //TODO: Probably calculated wrong
  //convert to [-1, 1] value
  float sensedLeanAngle = (pLeanSenseInVal / 512.0) - 1.0;
  return (360.0 * sensedLeanAngle - 1.80);
}

// In the original code, this also calculates steerRef & countersteering
float LeanController::calcLeanRef(){
  if (!leanOn){ //If lean mode off, no lean needed
    return 0.0;
  }
  float newRef = -joystick_LR * leanAngleLimit; //TODO: May be wrong
  this->leanError = newRef - leanAngleState;
  return newRef;
}

//Stolen from the brake valve outputs, thanks Leland
void LeanController::calcLeanValveOutputs(){
  float ControlVal = 0.0; //TODO: No idea how to compute w/o ComputePWMOutputs()
  if (ControlVal > PWM_LIMIT) {
    leanValveCmd = PWM_LIMIT;
  } else if (ControlVal < -PWM_LIMIT) {
    leanValveCmd = -PWM_LIMIT;
  } else {
    leanValveCmd = ((int) ControlVal);
  }

  if (leanValveCmd >= 0) {
    analogWrite(leanValvePinOne, leanValveCmd + LEAN_OFFSET);
    digitalWrite(leanValvePinTwo, 0);
  } else {
    analogWrite(leanValvePinTwo, -leanValveCmd + LEAN_OFFSET);
    digitalWrite(leanValvePinOne, 0);
  }
}

//Constructor -- set default values
LeanController::LeanController(){
    leanOn = true;
    leanAngleLimit = LEAN_MIN_ANGLE; // Speed sensitive lean limits
}



/* Main Control Functions */
void sampleSensors() {
  lc->setSensorVal(analogRead(pLeanSenseIn));
}

void speedSteeringControlMap() {

}

void adjustForLeanMode() {

}

//Amount of lean/steer/break/accel as requested by driver input
float steerRef = 0.0;
float brakeRef = 0.0;
float accelRef = 0.0;
void steerControl() {

}

void leanControl() {
    lc->update();
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
