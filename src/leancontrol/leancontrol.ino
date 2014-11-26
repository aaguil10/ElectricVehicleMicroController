#define SPEED_SENS_MAX      30.0   //  Speed [mph] above which speed sensitive steering/leaning saturates
#define SPEED_SENS_MIN      4.0    //  Speed [mph] below which speed sensitive steering/leaning saturates

#define LEAN_MAX_ANGLE      29.0
#define LEAN_MIN_ANGLE      2.0
#define SPEED_LIMIT_LEAN_SLOPE  (LEAN_MAX_ANGLE - LEAN_MIN_ANGLE)/(SPEED_SENS_MAX - SPEED_SENS_MIN)

#define LEAN_OFFSET         30      //  Dead zone offset value

//TODO: Is our PWM resolution different from theirs?
#define PWM_LIMIT           256    //  Practical upper limit, = (PWM_RESOLUTION - #_OFFSET) because OFFSET will push duty cycle to limit

//Lean Valve output pins
const int leanValvePinOne = 11;
const int leanValvePinTwo = 11;


// Set up pins
const int joystickFBSensor = A0;
const int joystickLRSensor = A1;
const int leanModeTogglePin = 2;
const int pLeanSenseIn = A0;
const int HydPumpEn = 14;


// These eventually will need to be calculated and adjusted properly
float SpeedState = 0.0;
float joystickValx = 0.0;
float joystickValy = 0.0;

// the setup routine runs once when you press reset:
void setup() {
  // initialize the digital pin as an output.
  pinMode(HydPumpEn, OUTPUT);
  pinMode(leanValvePinOne, OUTPUT);
  pinMode(leanValvePinTwo, OUTPUT);
  pinMode(leanModeTogglePin, INPUT);
  pinMode(pLeanSenseIn, INPUT);
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
    float pLeanSenseInVal;
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

LeanController lc; //Global LeanController object

void LeanController::updateSensors(){
  setSensorVal(lsc->getDutyCycle());
}

void LeanController::setSensorVal(float sensorVal){
  this->pLeanSenseInVal = pLeanSenseInVal;
}

float LeanController::getSensorVal(){
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
  //return (360.0*LeanDuty/LeanPeriod - 1.80);  // Correct for a 1.8deg offset 
  //TODO: May be calculated wrong
  return (360.0 * pLeanSenseInVal - 1.80); //1023 is the max analog input val in Arduino
}

// In the original code, this also calculates steerRef & countersteering
float LeanController::calcLeanRef(){
  if (!leanOn){ //If lean mode off, no lean needed
    return 0.0;
  }
  float newRef = -joystickValx * leanAngleLimit; //TODO: May be wrong
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

void sampleSensors(){
  lc.setSensorVal(analogRead(pLeanSenseIn));
  //These are temp -- to be handled by joystick section
  joystickValx = analogRead(joystickLRSensor);
  joystickValy = analogRead(joystickFBSensor);
}

void loop() {
  sampleSensors();
  lc.update();
}