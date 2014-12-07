// TODO Bring parking brake in

// needs to export BrakeRef (MAX_BRAKE_PRESSURE?)

// Depends on PWM_LIMIT, Code2VoltageCnst, PRESS_VSLOPE, PRESS_VOFFSET (used in hydraulic sensor)

// Old
//xdata float BrakePressState = 0.0;
//#define BRAKE_DITHER        170       //  approximately 10%

// BRAKE DEFINES
#define BRAKE_GAIN			10			//  Roughly 1500psi error yields full open valve
#define MAX_BRAKE_PRESSURE  700.0      //  Maximum cylinder pressure in PSI
#define BRAKE_OFFSET        30       //  Dead zone offset value
// I/O Pins
#define BRAKE_VALVE_1_PIN 1 // Brake valve 1 output pin (DIGITAL)
#define BRAKE_VALVE_2_PIN 2 // Brake valve 2 output pin (DIGITAL)
#define BRAKE_SENSOR_PIN 3 // Brake control input (ANALOG)

// BRAKE VARS
xdata float BrakeRef = 0.0;

class BrakeController {
    public:
        int update() {
            //  Brake Control
            int BrakeValveCmd = 0;

            float BrakePressState = analogRead(BRAKE_SENSOR_PIN) *
                Code2VoltageCnst * PRESS_VSLOPE + PRESS_VOFFSET;

            float ControlVal = BRAKE_GAIN * (BrakeRef - BrakePressState);

            if (ControlVal > PWM_LIMIT) {
                BrakeValveCmd = PWM_LIMIT;
            } else if (ControlVal < -PWM_LIMIT) {
                BrakeValveCmd = -PWM_LIMIT;
            } else {
                BrakeValveCmd = ((int) ControlVal);
            }

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
