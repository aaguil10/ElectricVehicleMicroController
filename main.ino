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

// the loop routine runs over and over again forever:
void loop() {
  // TODO add print status

  SampleSensors(); 
  SpeedSteeringControlMap();
  AdjustForLeanMode();
  SteerControl();
  LeanControl();
  BrakeControl();
  HydraulicControl();
  TractionMotorCommandProcessing();
  SetThrottle();

  int joystickVal = readJoystick(A0);
  int steerVal = readSteer(A1);
  PWMOutput1(joystickVal, steerVal, 9);
  PWMOutput2(joystickVal, steerVal, 10);
  delay(30);        // delay in between reads for stability
}

/* Main Control Functions */
void SampleSensors() {

}

void SpeedSteeringControlMap() {

}

void AdjustForLeanMode() {

}

void SteerControl() {

}

void LeanControl() {

}

void BrakeControl() {

}

void HydraulicControl() {

}

void TractionMotorCommandProcessing() {

}

void SetThrottle() {

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
