#define SPEED_SENS_MAX      30.0   //  Speed [mph] above which speed sensitive steering/leaning saturates
#define SPEED_SENS_MIN      4.0    //  Speed [mph] below which speed sensitive steering/leaning saturates

#define LEAN_MAX_ANGLE      29.0
#define LEAN_MIN_ANGLE      2.0
#define SPEED_LIMIT_LEAN_SLOPE  (LEAN_MAX_ANGLE - LEAN_MIN_ANGLE)/(SPEED_SENS_MAX - SPEED_SENS_MIN)

#define LEAN_VALVE_1_PIN    11 //Valve Coil FET
#define LEAN_VALVE_2_PIN    12

#define LEAN_OFFSET         30      //  Dead zone offset value

//TODO: Is our PWM resolution different from theirs?
#define PWM_LIMIT           256    //  Practical upper limit, = (PWM_RESOLUTION - #_OFFSET) because OFFSET will push duty cycle to limit


// Set up pins
int leanModeTogglePin = 2;
int leanSensor = A0;
int joystickLRSensor = A4;
int joystickFBSensor = A5;
int hydraulicValve = 13;


// These eventually will need to be calculated and adjusted properly
float SpeedState = 0.0;
float Joystick_LR_Ref = 0.0;
float Joystick_FB_Ref = 0.0;


// the setup routine runs once when you press reset:
void setup() {
  // initialize the digital pin as an output.
  pinMode(hydraulicValve, OUTPUT);
  pinMode(LEAN_VALVE_1_PIN, OUTPUT);
  pinMode(LEAN_VALVE_2_PIN, OUTPUT);
  pinMode(leanModeTogglePin, INPUT);
  pinMode(leanSensor, INPUT);
  pinMode(joystickLRSensor, INPUT);
  pinMode(joystickFBSensor, INPUT);
}


//TODO: Everything regarding reading PWM input of the lean sensor
class LeanSensorController {
  public:
    volatile int sensorState;
    void initInterrupts();
    float getDutyCycle();
    LeanSensorController();
  private:
    void uptickInterrupt();
    void downtickInterrupt();
};

LeanSensorController::LeanSensorController(){
  sensorState = LOW;
  initInterrupts();
}

void LeanSensorController::initInterrupts(){}
void LeanSensorController::uptickInterrupt(){}
void LeanSensorController::downtickInterrupt(){}
float LeanSensorController::getDutyCycle(){return 0.0;}



class LeanController{
  public:
    LeanSensorController *lsc;
    void update(){
      leanOn = readLeanMode();
      leanAngleLimit = calcLeanAngleLimit();
      leanAngleState = calcLeanAngle();
      leanRef = calcLeanRef();
    }
    void updateSensors();
    //Accessors
    void setSensorVal(float sensorVal);
    float getSensorVal();
    float getLeanRef();
    float getLeanAngleState();
    LeanController();
  private:
    float leanSensorVal;
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

void LeanController::updateSensors(){
  setSensorVal(lsc->getDutyCycle());
}

void LeanController::setSensorVal(float sensorVal){
  this->leanSensorVal = leanSensorVal;
}

float LeanController::getSensorVal(){
  return this->leanSensorVal;
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
  //return (360.0*LeanDuty/LeanPeriod - 1.80);  // Correct for a 1.8deg offset 
  //TODO: May be calculated wrong
  return (360.0 * leanSensorVal - 1.80); //1023 is the max analog input val in Arduino
}

// In the original code, this also calculates steerRef & countersteering
float LeanController::calcLeanRef(){
  if (!leanOn){ //If lean mode off, no lean needed
    return 0.0;
  }
  float newRef = -Joystick_LR_Ref * leanAngleLimit; //TODO: May be wrong
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
    analogWrite(LEAN_VALVE_1_PIN, leanValveCmd + LEAN_OFFSET);
    digitalWrite(LEAN_VALVE_2_PIN, 0);
  } else {
    analogWrite(LEAN_VALVE_2_PIN, -leanValveCmd + LEAN_OFFSET);
    digitalWrite(LEAN_VALVE_1_PIN, 0);
  }
}

//Constructor -- set default values
LeanController::LeanController(){
    leanOn = true;
    leanAngleLimit = LEAN_MIN_ANGLE; // Speed sensitive lean limits
}

LeanController lc;
void sampleSensors(){
  lc.setSensorVal(analogRead(leanSensor));
  //These are temp -- to be handled by joystick section
  Joystick_LR_Ref = analogRead(joystickLRSensor);
  Joystick_FB_Ref = analogRead(joystickFBSensor);
}

void loop() {
  sampleSensors();
  lc.update();
}