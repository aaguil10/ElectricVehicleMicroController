// TODO Bring parking brake in
// needs to export BrakeRef (MAX_BRAKE_PRESSURE?)
// Depends on Code2VoltageCnst, PRESS_VSLOPE, PRESS_VOFFSET (used in hydraulic sensor)

#define BRAKE_GAIN			10			//  Roughly 1500psi error yields full open valve

#define MAX_BRAKE_PRESSURE  700.0      //  Maximum cylinder pressure in PSI

#define BRAKE_DITHER        170       //  approximately 10%
#define BRAKE_OFFSET        30       //  Dead zone offset value

//#define Code2VoltageCnst 0.0006070774730338503  
#define PRESS_VOFFSET	-869.01 // Pressure Voltage sensor characteristic y-offset
#define PRESS_VSLOPE	1780.5 
// I/O Pins
#define BRAKE_VALVE_1_PIN 45 // Brake valve 1 output pin (DIGITAL)
#define BRAKE_VALVE_2_PIN 44 // Brake valve 2 output pin (DIGITAL)
#define BRAKE_CONTROL_PIN A1 // Brake control input (ANALOG)


#define PWM_LIMIT 256
// Valve control pins
// sbit pBrakeValveS1 = P6^4;
// sbit pBrakeValveS2 = P6^6;  

// Reverse Switch Input
int ParkBrakeEn = 5; // Was: RevSW_Input = P2^5;

float BrakeRef = 0.0;
//xdata float BrakePressState = 0.0;
int BrakeValveCmd = 0;

class BrakeController {
    public:
        int update() {
            //  Brake Control
            // float BrakePressState  = ((float)SampleVoltage(6)) * Code2VoltageCnst * PRESS_VSLOPE + PRESS_VOFFSET;  //  Sample Brake Pressure
            // NOTE: removed Code2VoltageCnst
            float BrakePressState = analogRead(BRAKE_CONTROL_PIN) *
                                    PRESS_VSLOPE + PRESS_VOFFSET;
            Serial.print("BrakePressState: ");
            Serial.println(BrakePressState);
            float ControlVal = BRAKE_GAIN * (BrakeRef - BrakePressState);
            Serial.print("ControlVal: ");
            Serial.println(ControlVal);
            if (ControlVal > PWM_LIMIT) {
                BrakeValveCmd = PWM_LIMIT;
            } else if (ControlVal < -PWM_LIMIT) {
                BrakeValveCmd = -PWM_LIMIT;
            } else {
                BrakeValveCmd = ((int) ControlVal);
            }
            //ComputePWMOutputs();// Instead just write brake signal
            // Set outputs
            /*
            if (BrakeValveCmd >= 0) // S1 active, S2 inactive
            {	
                if (PWMCounter < BrakeValveCmd + BRAKE_OFFSET) { pBrakeValveS1 = 1; }
                else  {	pBrakeValveS1 = 0; }
                pBrakeValveS2 = 0;
            }
            else  // S1 inactive, S2 active
            {
                if (PWMCounter < -BrakeValveCmd + BRAKE_OFFSET) { pBrakeValveS2 = 1; }
                else  {	pBrakeValveS2 = 0; }
                pBrakeValveS1 = 0;
            }
             */
            if (BrakeValveCmd >= 0) {
               analogWrite(BRAKE_VALVE_1_PIN, BrakeValveCmd + BRAKE_OFFSET); 
               digitalWrite(BRAKE_VALVE_2_PIN, 0);
            } else {
               analogWrite(BRAKE_VALVE_2_PIN, -BrakeValveCmd + BRAKE_OFFSET); 
               digitalWrite(BRAKE_VALVE_1_PIN, 0);
            }
        };
};

BrakeController *b;

void setup(){
  Serial.begin(9600);
}

void loop(){
  (*b).update();
  //Serial.println("Aloha!");
}
