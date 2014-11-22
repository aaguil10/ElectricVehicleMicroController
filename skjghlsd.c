#include <PID_v1.h>

#define BRAKE_GAIN			10			//  Roughly 1500psi error yields full open valve

#define BRAKE_Kp			10
#define BRAKE_Kd			0.0
#define BRAKE_Ki			0.0
#define BRAKE_Imax			10
#define BRAKE_Ad			0.0432139  // exp(-P*Ts)
#define BRAKE_Bd		   -300.583  // (exp(-P*Ts) - 1)*P
#define BRAKE_Dd			314.159  // P
//#PID brakePID(&Input, &Output, &Setpoint,2,5,1, DIRECT);
#define BRAKE_DITHER        170       //  approximately 10%
#define BRAKE_OFFSET        30       //  Dead zone offset value

const int pBrakeValveS1 = 8;
const int pBrakeValveS2 = 7;
  
const int Btn1 = 13; //the left button on the diagram
const int Btn2 = 12;

//global variables
int driveEn = 0; //check to see whether drive or reverse are engaged
int curState = 1; //1 = decision state; 2= forward; 3= reverse

void setup() {
  // initialize the digital pin as an output.
  pinMode(Btn1, INPUT);
  pinMode(Btn2, INPUT);
  pinMode(pBrakeValveS1, OUTPUT);
  pinMode(pBrakeValveS2, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  int joystickValy = readAnalog(A0);
  int joystickValx = readAnalog(A1);
  ledtest
  ledinput(pBrakeValveS1, pBrakeValveS2);
  print_joystick_values(joystickValx, joystickValy);
  ComputePWMOutputs();
  delay(1000);               // wait for a second
}

void ledtest(int pBrakeValveS1, int pBrakeValveS2){
  digitalWrite(pBrakeValveS1, HIGH);   // sets the LED on
  digitalWrite(pBrakeValveS2, LOW);
  delay(1000);                  // waits for a second
  digitalWrite(pBrakeValveS1, LOW);    // sets the LED off
  digitalWrite(pBrakeValveS2, HIGH);
}
void ledinput(int pBrakeValveS1, int pBrakeValveS2){
  digitalWrite(pBrakeValveS1, digitalRead(pBrakeValveS1));   // sets the LED on
  digitalWrite(pBrakeValveS2, digitalRead(pBrakeValveS2));
}

void print_joystick_values(int x,int y){
  Serial.print("joyValy: ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print("joyValx: ");
  Serial.print(x);
  Serial.println();
}

int readAnalog(int AnalogPin) {
  int sensorValue = analogRead(AnalogPin);
  int output = map(sensorValue, 0, 1023, 0, 255);
  return output;
}

void brake_control(){
  
}

void PID_Controller(){
  
}

void ComputePWMOutputs(){}



//disregard kewSwState for now
void tractionMotorCP(int joystickValx, int joystickValy){
  if(pBrakeValveS1 == 1 || pBrakeValveS2 == 1){ driveEn = 1;} //if selected engage
  
  if(curState == 1){ //decison state
    if(joystickValy>0 && pBrakeValveS1 ==1 ){
    }
      	
  }
  
  
  //once engaged check only joystickValy 
  //predefined range use 0-126 for accel and 127-255 where 127 is 0 no braking
  
 
  
  
}