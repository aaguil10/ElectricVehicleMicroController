

#define BRAKE_GAIN			10			//  Roughly 1500psi error yields full open valve
#define MAX_BRAKE_PRESSURE  700.0      //  Maximum cylinder pressure in PSI

#define BRAKE_Kp			10
#define BRAKE_Kd			0.0
#define BRAKE_Ki			0.0
#define BRAKE_Imax			10
#define BRAKE_Ad			0.0432139  // exp(-P*Ts)
#define BRAKE_Bd		   -300.583  // (exp(-P*Ts) - 1)*P
#define BRAKE_Dd			314.159  // P
#define BRAKE_DITHER        170       //  approximately 10%
#define BRAKE_OFFSET        30       //  Dead zone offset value

#define PWM_RESOLUTION      256.0  //  8 bit PWM resolution
#define PWM_RES_HALF        128.0  //  Half scale (0-point) of PWM range
#define PWM_LIMIT           256    //  Practical upper limit, = (PWM_RESOLUTION - #_OFFSET) because OFFSET will push duty cycle to limit

#define Code2VoltageCnst    0.0006070774730338503 // derived from measured values (0.0005860805860805861 ideal)
#define PRESS_VOFFSET		-869.01     //  Pressure Voltage sensor characteristic y-offset
#define PRESS_VSLOPE		1780.5      //  Pressure Voltage sensor characteristic slope
float BrakeRef = 0.0;

//------------------------------------------------------------------------------------
// Global VARIABLES
//------------------------------------------------------------------------------------
double BrakePressState = 0;
int BrakeValveCmd = 0;
float ControlVal = 0; //modified in setThrottle()

const int pBrakeValveS1 = 8;
const int pBrakeValveS2 = 7;

int joystickValy = 0;
int joystickValx = 0;

const int Btn1 = 13; //the left button on the diagram
const int Btn2 = 12;

const int PWMone = 11; //Valve Coil FET.
const int PWMtwo = 10;  //Valve Coil FET.

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
  //ledtest(pBrakeValveS1, pBrakeValveS2);
  sample_sensors();
  ledinput(pBrakeValveS1, pBrakeValveS2);
  print_joystick_values();
  //tractionmotorCP(joystickValx, joystickValy); // will only utilize Y-value
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
  digitalWrite(pBrakeValveS1, digitalRead(Btn1));   // sets the LED on
  digitalWrite(pBrakeValveS2, digitalRead(Btn2));
  Serial.print(digitalRead(Btn1));
}

void print_joystick_values(){
  Serial.print("joyValy: ");
  Serial.print(joystickValy);
  Serial.print(" ");
  Serial.print("joyValx: ");
  Serial.print(joystickValx);
  Serial.println();
}

int readAnalog(int AnalogPin) {
  int sensorValue = analogRead(AnalogPin);
  int output = map(sensorValue, 0, 1023, 0, 255);
  return output;
}

void sample_sensors(){
  joystickValy = readAnalog(A0);
  joystickValx = readAnalog(A1);
}

void brake_control(){
  BrakePressState  = ((float)joystickValy) * Code2VoltageCnst * PRESS_VSLOPE + PRESS_VOFFSET;  //  Sample Brake Pressure
  ControlVal = BRAKE_GAIN*(BrakeRef - BrakePressState);
		
  if (ControlVal > PWM_LIMIT){BrakeValveCmd = PWM_LIMIT; } 
  else if (ControlVal < -PWM_LIMIT){BrakeValveCmd = -PWM_LIMIT; } 
  else {BrakeValveCmd = ((int) ControlVal); }
}


void ComputePWMOutputs(){
  int out = 64;
  out = map(joystickValy, 0, 255, 64, 191);
  analogWrite(out, PWMone);
  out = map(joystickValx, 0, 255, 64, 191);
  analogWrite(out, PWMtwo);
}



//disregard kewSwState for now
//still need to account for parking brake engagement
void tractionMotorCP(int joystickValx, int joystickValy){
  int drive = digitalRead(Btn1); 
  int reverse  = digitalRead(Btn2);
  if(drive== 1 || reverse == 1){//if selected engage
    driveEn = 1;
  }
  else {
    return; 
  }
  
  if(curState == 1 && driveEn == 1){ //decison state
    if(joystickValy>127 && drive ==1 && reverse ==0 ){ //forward acceleration
      Serial.print("need to move forward");
      curState =2 ; //set to accelerate
    }
    else if (joystickValy<127 && reverse == 1 && drive ==0){ //reverse acceleartion
      Serial.print("need to move backword");
      curState = 3; 
    }
    else { // default back to unknown
      	curState = 1; // do nothing since joystick is neutral
      	ControlVal = 0; //set acceleration to 0; 
    }  
  }
  else if (curState ==2 && driveEn ==1 ){//forward acceleration
   //send output signal for acceleration
   Serial.print("set acccelref");
  }
  else if ( curState == 3 && driveEn ==1 ){//reverse acceleration
  Serial.print("set acccelref for reverse");
  }
  else{
    driveEn = 0;
    Serial.print("set parking brake");
  }
  
  
  //once engaged check only joystickValy 
  //predefined range use 0-126 for braking and 127-255 where 127 baseline and >127 is acceleration and <127 is braking
  
 
  
  
}