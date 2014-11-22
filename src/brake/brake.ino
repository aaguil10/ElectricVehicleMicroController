// TODO Bring parking brake in

// needs to export BrakeRef (MAX_BRAKE_PRESSURE?)

// Depends on Code2VoltageCnst, PRESS_VSLOPE, PRESS_VOFFSET (used in hydraulic sensor)

#define BRAKE_GAIN			10			//  Roughly 1500psi error yields full open valve

#define MAX_BRAKE_PRESSURE  700.0      //  Maximum cylinder pressure in PSI

#define BRAKE_DITHER        170       //  approximately 10%
#define BRAKE_OFFSET        30       //  Dead zone offset value

// I/O Pins
#define BRAKE_VALVE_1_PIN 1 // Brake valve 1 output pin (DIGITAL)
#define BRAKE_VALVE_2_PIN 2 // Brake valve 2 output pin (DIGITAL)
#define BRAKE_CONTROL_PIN 3 // Brake control input (ANALOG)

// Valve control pins
// sbit pBrakeValveS1 = P6^4;
// sbit pBrakeValveS2 = P6^6;  

// Reverse Switch Input
sbit ParkBrakeEn = P2^5; // Was: RevSW_Input = P2^5;

xdata float BrakeRef = 0.0;
//xdata float BrakePressState = 0.0;
xdata int BrakeValveCmd = 0;



class BrakeController {
    public:
        int update() {
            //  Brake Control

            // float BrakePressState  = ((float)SampleVoltage(6)) * Code2VoltageCnst * PRESS_VSLOPE + PRESS_VOFFSET;  //  Sample Brake Pressure
            // NOTE: removed Code2VoltageCnst

            float BrakePressState = analogRead(BRAKE_CONTROL_PIN) *
                Code2VoltageCnst * PRESS_VSLOPE + PRESS_VOFFSET;

            float ControlVal = BRAKE_GAIN * (BrakeRef - BrakePressState);

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

