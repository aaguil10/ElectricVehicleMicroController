//This Controler simulates the PWM inputs for the steer, lean, and rear wheel speed.

// the setup routine runs once when you press reset:
void setup() {

}

// the loop routine runs over and over again forever:
void loop() {
  int steer = readAnalog(A5);
  int lean = readAnalog(A4);
  int speed = readAnalog(A3);
  chooseInput(steer, 1); //Unfortunately there are enough pins so you can only choose one
}

//Simulates one of the following PWM inputs
//Steer = 1, Lean = 2, Speed = 3
void chooseInput(int val, int choice){
  if(choice == 1){
    SteerAngleSimulator(val,10);
  }
  if(choice == 2){
    LeanAngleSimulator(val,10);
  }
  if(choice == 3){
  	RearWheelSpeedSimulator(val,10);
  }
}

int readAnalog(int AnalogPin) {
  int sensorValue = analogRead(AnalogPin);
  int output = map(sensorValue, 0, 1023, 0, 255);
  return output;
}


void SteerAngleSimulator(int input, int OutputPin) {
  //Voodoo is a magic varalble that simulates the
  //steer input perfecttly
  int voodoo = input/2;
  int out = 64;
  if(voodoo > 0)
    out = map(voodoo, 0, 255, 64, 191);
  analogWrite(OutputPin, out);  
}

void LeanAngleSimulator(int input, int OutputPin) {
  //Voodoo is a magic varalble that simulates the
  //lean input perfecttly
  int voodoo = input/2;
  int out = 64;
  if(voodoo > 0)
    out = map(voodoo, 0, 255, 64, 191);
  analogWrite(OutputPin, out);  
}
void RearWheelSpeedSimulator(int input, int OutputPin) {
  //Voodoo is a magic varalble that simulates the
  //speed input perfecttly
  int voodoo = input/2;
  int out = 64;
  if(voodoo > 0)
    out = map(voodoo, 0, 255, 64, 191);
  analogWrite(OutputPin, out);  
}
