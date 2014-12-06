#define SPEED_SENS_MAX      30.0   //  Speed [mph] above which speed sensitive steering/leaning saturates
#define SPEED_SENS_MIN      4.0    //  Speed [mph] below which speed sensitive steering/leaning saturates

#define STEER_MAX_ANGLE     22.0; // Max steer angle at lowest speed
#define STEER_MIN_ANGLE     15.0; // Max steer angle at top speed
#define SPEED_LIMIT_STEER_SLOPE  (STEER_MIN_ANGLE - STEER_MAX_ANGLE)/(SPEED_SENS_MAX - SPEED_SENS_MIN)

#define STEER_OFFSET        30     //  Dead zone offset value

//TODO: Is our PWM resolution different from theirs?
#define PWM_LIMIT           256    //  Practical upper limit, = (PWM_RESOLUTION - #_OFFSET) because OFFSET will push duty cycle to limit

//Steer Valve output pins
const int steerValvePinOne = 11; // TODO: Plug in actual pin numbers
const int steerValvePinTwo = 11;

// Set up pins TODO: Plug in actual pin numbers
const int joystickFBSensor = A0;
const int joystickLRSensor = A1;
const int pSteerSenseIn = A0;
const int HydPumpEn = 14;

// These eventually will need to be calculated and adjusted properly
float SpeedState = 0.0;
float joystickValx = 0.0;
float joystickValy = 0.0;

// the setup routine runs once when you press reset:
void setup() {
  // initialize the digital pin as an output.
  pinMode(HydPumpEn, OUTPUT);
  pinMode(steerValvePinOne, OUTPUT);
  pinMode(steerValvePinTwo, OUTPUT);
  pinMode(pSteerSenseIn, INPUT);
  pinMode(joystickLRSensor, INPUT);
  pinMode(joystickFBSensor, INPUT);
  
  Serial.begin(9600)
}

//TODO: Everything regarding reading PWM input of the lean sensor
class SteerSensorController {
  public:
    volatile int sensorState;
    void initInterrupts();
    float getDutyCycle();
    SteerSensorController();
  private:
    void uptickInterrupt();
    void downtickInterrupt();
};

SteerSensorController::SteerSensorController(){
  sensorState = LOW;
  initInterrupts();
}

void SteerSensorController::initInterrupts(){}
void SteerSensorController::uptickInterrupt(){}
void SteerSensorController::downtickInterrupt(){}
float SteerSensorController::getDutyCycle(){return 0.0;}

class SteerController{
  public:
    SteerSensorController *ssc;
    void update(){
      steerAngleLimit = 0;// calcSteerAngleLimit();
      steerAngleState = calcSteerAngle();
      steerRef = calcSteerRef();
    }
    void updateSensors();
    //Accessors
    void setSensorVal(float sensorVal);
    float getSensorVal();
    float getSteerRef();
    float getSteerAngleState();
    SteerController();
  private:
    float pSteerSenseInVal;
    bool steerOn;
    float steerAngleLimit; // Speed sensitive lean limits
    float steerAngleState;
    float steerRef;
    float steerError;
    int steerValveCmd;

    //float calcSteerAngleLimit();
    float calcSteerAngle();
    float calcSteerRef();
    void calcSteerValveOutputs();
};

SteerController sc; //Global SteerController object

void SteerController::updateSensors(){
  setSensorVal(ssc->getDutyCycle());
}

void SteerController::setSensorVal(float sensorVal){
  this->pSteerSenseInVal = pSteerSenseInVal;
}

float SteerController::getSensorVal(){
  return this->pSteerSenseInVal;
}

float SteerController::getSteerRef(){
  return this->steerRef;
}

float SteerController::getSteerAngleState(){
  return this->steerAngleState;
}
// Calculate speed-sensitive steer angle limit
//float SteerController::calcSteerAngleLimit(){
//  float newLimit = SPEED_LIMIT_STEER_SLOPE * SpeedState + STEER_MAX_ANGLE; // Shouldn't this be STEER_MIN_ANGLE?
//  // Clamp limits to their maxima
//  if (newLimit > STEER_MAX_ANGLE){ newLimit = STEER_MAX_ANGLE;}
//  if (newLimit < STEER_MIN_ANGLE){ newLimit = STEER_MIN_ANGLE;}
//  return newLimit;
//}

float SteerController::calcSteerAngle(){
  //return (360.0*SteerDuty/SteerPeriod - 1.80);  // Correct for a 1.8deg offset 
  //TODO: May be calculated wrong
  return (360.0 * pSteerSenseInVal - 1.80); //1023 is the max analog input val in Arduino
}

// In the original code, this also calculates steerRef & countersteering
float SteerController::calcSteerRef(){
  float newRef = -joystickValx * steerAngleLimit; //TODO: May be wrong
  this->steerError = newRef - steerAngleState;
  return newRef;
}

//Stolen from the brake valve outputs, thanks Leland
void SteerController::calcSteerValveOutputs(){
  float ControlVal = 0.0; //TODO: No idea how to compute w/o ComputePWMOutputs()
  if (ControlVal > PWM_LIMIT) {
    steerValveCmd = PWM_LIMIT;
  } else if (ControlVal < -PWM_LIMIT) {
    steerValveCmd = -PWM_LIMIT;
  } else {
    steerValveCmd = ((int) ControlVal);
  }

  if (steerValveCmd >= 0) {
    analogWrite(steerValvePinOne, steerValveCmd + STEER_OFFSET);
    digitalWrite(steerValvePinTwo, 0);
  } else {
    analogWrite(steerValvePinTwo, -steerValveCmd + STEER_OFFSET);
    digitalWrite(steerValvePinOne, 0);
  }
}

//Constructor -- set default values
SteerController::SteerController(){
    steerAngleLimit = STEER_MIN_ANGLE; // Speed sensitive lean limits
}

void sampleSensors(){
  sc.setSensorVal(analogRead(pSteerSenseIn));
  //These are temp -- to be handled by joystick section
  joystickValx = analogRead(joystickLRSensor);
  joystickValy = analogRead(joystickFBSensor);
}

void loop() {
  sampleSensors();
  sc.update();
  
  Serial.println("SteerRef: " + sc.getSteerRef());
  delay(1);
}
