#define SPEED_SENS_MAX      30.0   //  Speed [mph] above which speed sensitive steering/leaning saturates
#define SPEED_SENS_MIN      4.0    //  Speed [mph] below which speed sensitive steering/leaning saturates

#define LEAN_MAX_ANGLE      29.0
#define LEAN_MIN_ANGLE      2.0
#define SPEED_LIMIT_LEAN_SLOPE  (LEAN_MAX_ANGLE - LEAN_MIN_ANGLE)/(SPEED_SENS_MAX - SPEED_SENS_MIN)

// Set up pins
int leanModeTogglePin = 2;
int leanSensor = A0;
int joystickLRSensor = A4;
int joystickFBSensor = A5;
int hydraulicValve = 13;
int valveCoilFET_1 = 11; //Valve Coil FET
int valveCoilFET_2 = 12;

// These eventually will need to be calculated and adjusted properly
float SpeedState = 0.0;
float Joystick_LR_Ref = 0.0;
float Joystick_FB_Ref = 0.0;


// the setup routine runs once when you press reset:
void setup() {
  // initialize the digital pin as an output.
  pinMode(hydraulicValve, OUTPUT);
  pinMode(valveCoilFET_1, OUTPUT);
  pinMode(valveCoilFET_2, OUTPUT);
  pinMode(leanModeTogglePin, INPUT);
}


class LeanController{
  public:
    void update(){
      leanOn = readLeanMode();
      leanAngleLimit = calcLeanAngleLimit();
      leanAngleState = calcLeanAngle();
      leanRef = calcLeanRef();
    }
    void setSensorVal(int sensorVal);
    int getSensorVal();
    LeanController();
  private:
    int leanSensorVal;
    bool leanOn;
    float leanAngleLimit; // Speed sensitive lean limits
    float leanAngleState;
    float leanRef;
    float leanError;

    bool readLeanMode();
    float calcLeanAngleLimit();
    float calcLeanAngle();
    float calcLeanRef();
};

void LeanController::setSensorVal(int sensorVal){
  this->leanSensorVal = leanSensorVal;
}

int LeanController::getSensorVal(){
  return this->leanSensorVal;
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

//TODO: Find out how to read PWM inputs & calculate duty cycle in Arduino
float LeanController::calcLeanAngle(){
  //return (360.0*LeanDuty/LeanPeriod - 1.80);  // Correct for a 1.8deg offset 
  return ((360.0 / 1023.0) * leanSensorVal); //1023 is the max analog input val in Arduino
}

// In the original code, this also calculates steerRef & countersteering
float LeanController::calcLeanRef(){
  if (!leanOn){ //If lean mode off, no lean needed
    return 0.0;
  }
  float newRef = (Joystick_LR_Ref / 1023.0) * leanAngleLimit;
  leanError = newRef - leanAngleState;
  return newRef;
}

//Constructor -- set default values
LeanController::LeanController(){
    leanOn = true;
    leanAngleLimit = LEAN_MIN_ANGLE; // Speed sensitive lean limits
    leanAngleState = 0.0;
    leanRef = 0.0;
    leanError = 0.0;
}

LeanController lc;
//TODO: Properly calculate deadzones etc. for joysticks
void sampleSensors(){
  lc.setSensorVal(analogRead(leanSensor));
  Joystick_LR_Ref = analogRead(joystickLRSensor);
  Joystick_FB_Ref = analogRead(joystickFBSensor);
}

void loop() {
  sampleSensors();
  lc.update();
}