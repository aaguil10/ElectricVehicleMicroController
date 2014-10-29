//------------------------------------------------------------------------------------
// DagneControl.c
//------------------------------------------------------------------------------------
// Copyright (C) 2008 Multimode Technologies LLC
//
// AUTH: Eric Sandoz
// DATE: 17 October 2008
//
// This program controls the sustainable supercar Dagne.
//

//------------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------------
#include <c8051f120.h>                    // SFR declarations
#include <stdio.h>

//-----------------------------------------------------------------------------
// 16-bit SFR Definitions for 'F12x
//-----------------------------------------------------------------------------

//sfr16 RCAP3    = 0xCA;                 // Timer3 reload value
sfr16 TMR3     = 0xCC;                 // Timer3 counter value
sfr16 TMR4	   = 0xCC;				   // Timer4 counter value
sfr16 TMR2     = 0xcc;                 // Timer2
sfr16 RCAP2    = 0xca;                 // Timer2 capture/reload
sfr16 ADC0     = 0xbe;                 // ADC0 data

//------------------------------------------------------------------------------------
// Global CONSTANTS
//------------------------------------------------------------------------------------

#define SYSCLK                (22118400L * 9 / 4) //  49.7664MHz

#define BAUDRATE              57600
#define UARTBUFSIZE           300

#define LOWRATE_CLKDIVIDE     20           // divide factor to get low rate sample clock
#define NUM_AVG_SAMPLES       16           // number of redundant samples over which the useful sample is averaged
										   // Must be a power of 2
#define NUM_AVG_BITS          4            // number of bits matching NUM_AVG_SAMPLES										   

#define VREF_VOLTAGE          2.40         // ADC0 reference voltage
#define AINPUT_GAIN			  0.5          // Prescaling of the ADC signal set by ADC0CF

#define SYSPRESS_LOW_THRESHOLD  1300.0
#define SYSPRESS_HIGH_THRESHOLD 2100.0

#define STEER_Kp			25.5
#define STEER_Kd			0.002
#define STEER_Ki			0.0 // 1.3 worked, but adds oscillations
#define STEER_Imax			0
#define STEER_Ad			0.0432139  // exp(-P*Ts)
#define STEER_Bd		   -300.583  // (exp(-P*Ts) - 1)*P
#define STEER_Dd			314.159  // P

#define LEAN_Kp				40.5
#define LEAN_Kd				0.0
#define LEAN_Ki				0.0
#define LEAN_Imax			50
#define LEAN_Ad				0.0432139  // exp(-P*Ts)
#define LEAN_Bd		   	   -300.583  // (exp(-P*Ts) - 1)*P
#define LEAN_Dd				314.159  // P

#define BRAKE_GAIN			10			//  Roughly 1500psi error yields full open valve

#define BRAKE_Kp			10
#define BRAKE_Kd			0.0
#define BRAKE_Ki			0.0
#define BRAKE_Imax			10
#define BRAKE_Ad			0.0432139  // exp(-P*Ts)
#define BRAKE_Bd		   -300.583  // (exp(-P*Ts) - 1)*P
#define BRAKE_Dd			314.159  // P

#define SPEED_SENS_MAX      30.0   //  Speed [mph] above which speed sensitive steering/leaning saturates
#define SPEED_SENS_MIN      4.0    //  Speed [mph] below which speed sensitive steering/leaning saturates

// RPP - Rev per Pulse (60), eta - belt drive ratio = 11.25"/3.25", r - Tire Rad = 0.3 m,
// Meters in a mile = 1609.344, DT - Time of one PCA cycle SYSCLK/4 = 8.03755e-008 
#define SPEED2MPH_CNST      252586.7479 // RPP*eta*2*pi*r*3600/1609.344/DT
#define STEER_MAX_ANGLE     22.0
#define STEER_MIN_ANGLE		15.0
#define SPEED_LIMIT_STEER_SLOPE  (STEER_MIN_ANGLE - STEER_MAX_ANGLE)/(SPEED_SENS_MAX - SPEED_SENS_MIN)

#define LEAN_MAX_ANGLE      29.0
#define LEAN_MIN_ANGLE		2.0
#define SPEED_LIMIT_LEAN_SLOPE  (LEAN_MAX_ANGLE - LEAN_MIN_ANGLE)/(SPEED_SENS_MAX - SPEED_SENS_MIN)

#define Code2VoltageCnst    0.0006070774730338503 // derived from measured values (0.0005860805860805861 ideal)
#define PRESS_VOFFSET		-869.01     //  Pressure Voltage sensor characteristic y-offset
#define PRESS_VSLOPE		1780.5      //  Pressure Voltage sensor characteristic slope

#define MAX_SPEED			30.0		//  Maximum speed in mph
#define MAX_BRAKE_PRESSURE  700.0      //  Maximum cylinder pressure in PSI

#define AB_DEADZONE			0.08		//  Accel/Brake deadzone in [0,1]
#define AB_SCALE 		   	1.08696     // AB_SCALE = 1/(1-AB_DEADZONE);

#define LR_DEADZONE			0.02		//  Left/Right deadzone in [0,1]
#define LR_SCALE 		   	1.05263157894737 // LR_SCALE = 1/(1-LR_DEADZONE);

#define PWM_RESOLUTION      256.0  //  8 bit PWM resolution
#define PWM_RES_HALF        128.0  //  Half scale (0-point) of PWM range
#define PWM_LIMIT           256    //  Practical upper limit, = (PWM_RESOLUTION - #_OFFSET) because OFFSET will push duty cycle to limit

#define STEER_DITHER        170      //  approximately 10%
#define STEER_OFFSET        30     //  Dead zone offset value

#define LEAN_DITHER         170       //  approximately 10%
#define LEAN_OFFSET         30      //  Dead zone offset value

#define BRAKE_DITHER        170       //  approximately 10%
#define BRAKE_OFFSET        30       //  Dead zone offset value

#define THROTTLE_FWD_GAIN   0.040 //  Address sat limits of throttle circuit
#define THROTTLE_REV_GAIN   0.0990
#define THTOTTLE_OFFSET     280.0

#define STEER_REF_GAIN      0.05    //  Gain for carving!
#define LAMBDA_GAIN			0.05  //  Map a lean error of 10 deg to max tollerable level
#define LAMBDA_SPEED_GAIN   1/60  //  Map our counter steer only point to 20mph

#define PARK_STATE_SPEED    0.18   // Speed below which we throw the brakes in park state

										   				
sbit  LED = P1^6;                          // green LED: '1' = ON; '0' = OFF

// Sensor Input Pins
sbit pLeanSenseIn = P1^1;
sbit pSteerSenseIn = P1^0;
sbit pSpeedSenseIn = P1^2;

// Valve control pins
sbit pSteerValveS1 = P6^0;
sbit pSteerValveS2 = P6^1;

sbit pLeanValveS1 = P6^2;
sbit pLeanValveS2 = P6^3;

sbit pBrakeValveS1 = P6^4;
sbit pBrakeValveS2 = P6^6;  

// Reverse Switch, Hydraulic Pump, and Keyswitch Relay pins
sbit pRevSw1 = P3^2;  //  Reversing contactor number 1
sbit pRevSw2 = P3^0;  //  Reversing contactor number 2
sbit pKeySwitch = P3^6;  // Throttle key switch to enable motor controller
sbit HydPumpEn = P3^4;  		//  Hydraulic Pump Control Pin

// Reverse Switch Input
sbit ParkBrakeEn = P2^5; // Was: RevSW_Input = P2^5;
sbit JoystickBtn1 = P2^6;
sbit JoystickBtn2 = P2^7;

sbit Aux_Pin = P5^0;

//-----------------------------------------------------------------------------
// typedefs
//-----------------------------------------------------------------------------
/*
typedef union                          // union used for writing to TL0 & TH0
    {
        struct
        {
            unsigned char hi;
            unsigned char lo;
        } b;
        unsigned int w;
    }udblbyte;
*/

//  PID controller data type
typedef struct
{
    float r; // reference
    float y; // Current measured position
    float Kp;
    float Kd;
    //float P;  // Derivative Filter Bandwidth
    float Ki;
    float Imax; // Maximum contribution of the integrator term

	//  Discrete time derivative filter parameters where P is the filter bandwidth
	float Ad; // exp(-P*Ts)
	float Bd; // (exp(-P*Ts) - 1)*P
	float Dd; // P
	float Xd; // Derivative filter state
	float Xi; // Integral filter state    
} PIDInfo;

//  Lead/Lag Compensator data type (pole/zero)
/*
typedef struct
{
	//  Discretized version of (b1*s + b0)/(s + a0), with sample period Ts in seconds
	//  b1 is the numerator gain, -b0/b1 is the zero location, -a0 is the Pole location

	//  Discrete Time stuff
	float Ad; // exp(-a0*Ts)
	float Bd; // (-1/a0)(exp(-a0*Ts) - 1)(b0 -a0*b1)
	float Dd; // b1
	float Xd;  //  State variable
}LeadLagInfo;
*/

//------------------------------------------------------------------------------------
// Function PROTOTYPES
//------------------------------------------------------------------------------------
void  Port_IO_Init(void);
void  Timer_Init(void);
void  PCA_Init(void);
void  ADC_Init(void);
void  DAC_Init(void);
void  UART_Init(void);
void  Interrupts_Init(void);
void  Oscillator_Init(void);
void  Voltage_Reference_Init(void);
void  Init_Device(void);

unsigned int  SampleVoltage(unsigned char ACh);
void  SpeedSteeringControlMap(void);
void  SampleSensors(void);
void  ComputePWMOutputs(void);

float PID_Controller(PIDInfo *p, float r, float y); 
//float LeadLagComp(LeadLagInfo *p, float u);
int   SendUartBuf(void);

void  Timer3_ISR (void);
void  PCA_ISR (void);
void  UART0_ISR (void);

//------------------------------------------------------------------------------------
// Global VARIABLES
//------------------------------------------------------------------------------------

bit SampleFlagHighRate = 0;
bit SampleFlag = 0;
bit NewSpeedSampleFlag = 0;
bit LFCR_Flag = 0;
bit ControlMode = 0;

xdata float LeanRef = 0.0;
xdata float SteerRef = 0.0;
xdata float BrakeRef = 0.0;
xdata float AccelRef = 0.0;
xdata float Lambda = 0.0;
xdata float LeanError = 0.0;
xdata float LambdaSens = 0.0;

xdata float Joystick_LR_Ref = 0.0; 
xdata float Joystick_FB_Ref = 0.0;

xdata float LeanAngleLimit = LEAN_MIN_ANGLE;  //  Speed sensitive lean limits
xdata float SteerAngleLimit = STEER_MAX_ANGLE;  //  Speed sensitive steer limits

xdata float SteerState = 0.0;

//float Code2VoltageCnst;

bit PumpState = 0;
bit NextPumpState = 0;
bit KeySwState = 0;
bit NextKeySwState = 0;
unsigned char FwdRevState = 0; // Start in park state
unsigned char DriveEn = 0;

bit Dither = 0;
xdata unsigned int PWMCounter = 0;


xdata float SteerAngleState = 0.0;
xdata float LeanAngleState = 0.0;

xdata float BrakePressState = 0.0;
xdata float SysHydPressState = 0.0;

xdata float SpeedState = 0.0;
xdata long SpeedDT = 0;

//udblbyte SteerValveCmd;
xdata int SteerValveCmd = 0;
xdata int LeanValveCmd = 0;
xdata int BrakeValveCmd = 0;

//  Sensor PWM counter variables
xdata unsigned char CntOvrFlow_Steer = 0;
xdata unsigned char CntOvrFlow_Lean = 0;
xdata unsigned char CntOvrFlow_Speed = 0;

xdata unsigned int SteerSensCntVal0 = 0;
xdata unsigned int SteerSensCntVal1 = 0;
xdata long SteerDuty = 0;
xdata long SteerPeriod = 0;
xdata long SteerDiff1 = 0;
xdata long SteerDiff2 = 0;


xdata unsigned int LeanSensCntVal0 = 0;
xdata unsigned int LeanSensCntVal1 = 0;
xdata long LeanDuty = 0;
xdata long LeanPeriod = 0;
xdata long LeanDiff1 = 0;
xdata long LeanDiff2 = 0;

xdata long SpeedSensCntVal0 = 0;
xdata long SpeedSensCntVal1 = 0;
xdata long SpeedSensCntVal2 = 0;

xdata unsigned char SteerSensOvrFlow1 = 0;
xdata unsigned char SteerSensOvrFlow2 = 0;
xdata unsigned char LeanSensOvrFlow1 = 0;
xdata unsigned char LeanSensOvrFlow2 = 0;
xdata unsigned char SpeedSensOvrFlow1 = 0;
xdata unsigned char SpeedSensOvrFlow2 = 0;

xdata unsigned char SteerSensFlag = 0;
xdata unsigned char LeanSensFlag = 0;
xdata unsigned char SpeedSensFlag = 0;

//  UART0 Variables
xdata unsigned char SendBuf[UARTBUFSIZE];
xdata unsigned char CommCounter = 0;
xdata int NumChars = 0;
xdata unsigned int  SendCounter = 0;

// Control Variables
//float ControlVal;


//------------------------------------------------------------------------------------
// MAIN Routine
//------------------------------------------------------------------------------------
void main (void) {

	//unsigned int TmpMeasuredVar;
	float ControlVal; //, MeasuredVar; 
	unsigned int AccelCmd = 0;
	//unsigned int PWMCounter = 0;
	//char Dither = 1;
	xdata PIDInfo SteerPID, LeanPID; //, BrakePID;

   // disable watchdog timer
   WDTCN = 0xde;
   WDTCN = 0xad;

   SFRPAGE = CONFIG_PAGE;                 // Switch to configuration page
   Init_Device ();
                                     
   EA = 1;											// enable global interrupts

   //SFRPAGE = LEGACY_PAGE;                 // Page to sit in for now
   SFRPAGE = CONFIG_PAGE;

	//  Initilize Steering PID controller
	SteerPID.Kp = STEER_Kp; SteerPID.Kd = STEER_Kd; SteerPID.Ki = STEER_Ki; SteerPID.Imax = STEER_Imax;
	SteerPID.Ad = STEER_Ad; SteerPID.Bd = STEER_Bd; SteerPID.Dd = STEER_Dd;

	LeanPID.Kp = LEAN_Kp; LeanPID.Kd = LEAN_Kd; LeanPID.Ki = LEAN_Ki; LeanPID.Imax = LEAN_Imax;
	LeanPID.Ad = LEAN_Ad; LeanPID.Bd = LEAN_Bd; LeanPID.Dd = LEAN_Dd;

	//BrakePID.Kp = STEER_Kp; BrakePID.Kd = STEER_Kd; BrakePID.Ki = STEER_Ki; BrakePID.Imax = BRAKE_Imax;
	//BrakePID.Ad = STEER_Ad; BrakePID.Bd = STEER_Bd; BrakePID.Dd = STEER_Dd;


	//==========================================================
	//     Let's start the program shall we
	//==========================================================

	
	//printf("Hello Dagne\n");
	
 while (1) {                            // spin forever
	

	ComputePWMOutputs();

	if (SampleFlag == 1) 
	{	
		// Analog ADC channels
		// P1.0 - Joystick Left/Right
		// P1.1 - Joystick Front/Back
		// P1.2 - Main Hydraulic Pressure
		// P1.3 - Brake Hydraulic Pressure
		SFRPAGE = CONFIG_PAGE;
		SampleFlag = 0;

		CommCounter++;
		if (CommCounter >= 20){
			CommCounter = 0;
			/*
			NumChars = sprintf(SendBuf,"%f,%f,%f,%f,%d,%f,%f,%f,%d,%f,%f,%f,%d,%f,%u,%u,%u,%f,%f",
				SpeedState, AccelRef, SteerAngleState, SteerRef, SteerValveCmd, SteerAngleLimit, 
				LeanAngleState, LeanRef, LeanValveCmd, LeanAngleLimit, BrakePressState, BrakeRef,
				BrakeValveCmd, SysHydPressState, (unsigned int)PumpState, (unsigned int)KeySwState, 
				(unsigned int)FwdRevState, Joystick_LR_Ref, Joystick_FB_Ref);
			*/
			NumChars = sprintf(SendBuf,"FR= %d, JS %f\n", 
				(int) FwdRevState, Joystick_FB_Ref);
			//NumChars = sprintf(SendBuf,"Th = %f\n", AccelRef);
			SendUartBuf();				
		}
		
		//  Hydraulic valve Dither signal, toggle at lower sampling rate
		Dither = ~Dither;		

		SampleSensors();  // Check for valid samples
		//========= Open Loop Control Mapping ==============

		SpeedSteeringControlMap();  //  Map control commands to lean and steer commands, speed dependent


		//========= Steer, Lean, Brake Control Loops ==============

		ControlMode = ~Aux_Pin; //  Later connect this to a switch pin

		// Reference signal generation

		if (ControlMode == 0){  //  Old school, no lean mode.
			LeanRef = 0.0 * LeanAngleLimit; 
			SteerRef = -Joystick_LR_Ref * SteerAngleLimit;  //  For now map joystick to steer only
		}
		else {  //  Carving mode
			LeanRef = -Joystick_LR_Ref * LeanAngleLimit; 
			//  Steer reference signal will now become a feedback signal wrt lean
			LeanError = LeanRef - LeanAngleState;
			LambdaSens = (SpeedState - 2.0)/20.0;
			if (LambdaSens > 1.0){LambdaSens = 1.0;}
			if (LambdaSens < 0.0){LambdaSens = 0.0;}

			//  Map Lean Error to Lambda, where Lambda is non-negative, and Lambda = 1 means countersteer
			//  Lambda is also speed sensitive, Lambda -> 1 as speed increases
			//  Lambda uniformly clamps to 1 for speed above a given threshold like 20mph
			if (LeanError < 0.0) {Lambda = (-LeanError*LambdaSens*LAMBDA_GAIN  + SpeedState*LAMBDA_SPEED_GAIN);}
			else {Lambda = LeanError*LambdaSens*LAMBDA_GAIN + SpeedState*LAMBDA_SPEED_GAIN;}
			if (Lambda > 1.0){Lambda = 1.0;}
			if (Lambda < 0.0){Lambda = 0.0;}
			

			//  Countersteer's gain is speed sensetive, gain goes down as speed increases
			SteerRef =  (-STEER_REF_GAIN*SteerAngleLimit*LeanError)*Lambda 
				+ (1-Lambda)*(-Joystick_LR_Ref * SteerAngleLimit);  //  For now map joystick to steer only

			if (SteerRef > SteerAngleLimit){SteerRef = SteerAngleLimit;}
			if (SteerRef < -SteerAngleLimit){SteerRef = -SteerAngleLimit;}
		}


		ComputePWMOutputs();

		//  Steer Control 
		ControlVal = PID_Controller(&SteerPID, SteerRef, SteerAngleState);
		
		if (ControlVal > PWM_LIMIT){SteerValveCmd = PWM_LIMIT; } 
		else if (ControlVal < -PWM_LIMIT){SteerValveCmd = -PWM_LIMIT; } 
		else {SteerValveCmd = (int) ControlVal; } 
		
        
		ComputePWMOutputs();
		
		//  Lean Control
		ControlVal = PID_Controller(&LeanPID, LeanRef, LeanAngleState);

		if (ControlVal > PWM_LIMIT){LeanValveCmd = PWM_LIMIT; } 
		else if (ControlVal < -PWM_LIMIT){LeanValveCmd = -PWM_LIMIT; } 
		else {LeanValveCmd = (int) ControlVal; }
		//LeanValveCmd = 0;

		ComputePWMOutputs();

		//  Brake Control
		BrakePressState  = ((float)SampleVoltage(6)) * Code2VoltageCnst * PRESS_VSLOPE + PRESS_VOFFSET;  //  Sample Brake Pressure
		ControlVal = BRAKE_GAIN*(BrakeRef - BrakePressState);
		
		if (ControlVal > PWM_LIMIT){BrakeValveCmd = PWM_LIMIT; } 
		else if (ControlVal < -PWM_LIMIT){BrakeValveCmd = -PWM_LIMIT; } 
		else {BrakeValveCmd = ((int) ControlVal); }


		ComputePWMOutputs();
		//========= Hydraulic System Control Loop ==============
		SysHydPressState = ((float)SampleVoltage(7)) * Code2VoltageCnst * PRESS_VSLOPE + PRESS_VOFFSET;  //  Sample Main Hydrualic Pressure
		
		SFRPAGE = CONFIG_PAGE;
		if (PumpState == 0)
		{
			// Pump is off, transition if system pressure is below our lower threshold
			if (SysHydPressState < SYSPRESS_LOW_THRESHOLD)	
			{
				NextPumpState = 1;
			}
			HydPumpEn = 0;
		}
		else if (PumpState == 1)
		{
			// Pump is on, transition if system pressure is above our upper threshold
			if (SysHydPressState > SYSPRESS_HIGH_THRESHOLD)	
			{
				NextPumpState = 0;
			}
			HydPumpEn = 1;
		}
		PumpState = NextPumpState; // State Update
		

		//========= Traction Motor Command Processing ==============
		//  Keyswitch Control - Use joystick deadzone for keyswitch signal
		SFRPAGE = CONFIG_PAGE;
		if (KeySwState == 0)
		{
			// Motor Controller is off, transition if Accelerate is commanded
			if (AccelRef > 0.0)	
			{
				NextKeySwState = 1;
			}
			pKeySwitch = 0; // Should be 0
		}
		else if (KeySwState == 1)
		{
			// Motor Controller is on, transition if brake is applied
			if (BrakeRef > 0.0 || AccelRef == 0.0)	
			{
				NextKeySwState = 0;
			}
			pKeySwitch = 1; //  Should be 1
		}
		KeySwState = NextKeySwState; // State Update




		DriveEn = ~JoystickBtn1 || ~JoystickBtn2;

		//  Set Throttle
		if (FwdRevState == 3)
		{
			ControlVal = 600.0* THROTTLE_REV_GAIN*AccelRef + THTOTTLE_OFFSET; //1000.0*AccelRef; //  Limit Throttle when in reverse
			if (ControlVal > 600.0){ ControlVal = 600.0; } //  Saturate Control
		}
		else if (FwdRevState == 2)
		{
			ControlVal = 4095.0* THROTTLE_FWD_GAIN*AccelRef + THTOTTLE_OFFSET; //4095.0*AccelRef;  //  Full Throttle
			if (ControlVal > 4095.0){ ControlVal = 4095.0; } //  Saturate Control
		}
		else
		{
			ControlVal = 0.0;
		}
		if (AccelRef == 0.0)
		{
			ControlVal = 0.0;
		}

		ComputePWMOutputs();
		//  Clamp command to allowable range
		if (ControlVal > 4095.0) { ControlVal = 4095.0;}
		if (ControlVal < 0.0) { ControlVal = 0.0;}

		SFRPAGE   = DAC0_PAGE;
		AccelCmd = 4095 - ((unsigned int) ControlVal); // Must invert to cancel inverting OpAmp output stage
		DAC0L = (0xFF & AccelCmd); //  Write low byte
		DAC0H = (0xFF & (AccelCmd>>8)); //  Write high byte
		SFRPAGE = CONFIG_PAGE;

		
		if (FwdRevState == 1) // Direction Choose State
		{
			if (DriveEn == 1 && Joystick_FB_Ref > 0.0)
			{ 
				FwdRevState = 2;
			}
			else if (DriveEn == 1 && Joystick_FB_Ref < 0.0) 
			{
				FwdRevState = 3;
			}
			else if (DriveEn == 0) 
			{ 
				FwdRevState = 0; 
			}
			AccelRef = 0.0;
			if (ParkBrakeEn == 0){
				BrakeRef = 0.0; 
			}
			else {
				BrakeRef = 0.8*MAX_BRAKE_PRESSURE;
			}
		}
		else if (FwdRevState == 2)  // Forward State
		{
			if (SpeedState <= PARK_STATE_SPEED && DriveEn == 0) { FwdRevState = 0;}

			// Outputs
			pRevSw1 = 1;
			pRevSw2 = 0;
			if (Joystick_FB_Ref > 0.0){
				AccelRef = Joystick_FB_Ref;
				BrakeRef = 0.0;
			}
			else if (Joystick_FB_Ref == 0.0){
				AccelRef = 0.0;
				BrakeRef = 0.0;
			}
			else {
				AccelRef = 0.0;
				BrakeRef = -Joystick_FB_Ref*MAX_BRAKE_PRESSURE;
			}
		}
		else if (FwdRevState == 3)  //  Reverse State
		{
			if (SpeedState <= PARK_STATE_SPEED && DriveEn == 0) { FwdRevState = 0;}

			//  Outputs
			pRevSw1 = 0;
			pRevSw2 = 1;
			if (Joystick_FB_Ref > 0.0){
				AccelRef = 0.0;
				BrakeRef = Joystick_FB_Ref*MAX_BRAKE_PRESSURE;
			}
			else if (Joystick_FB_Ref == 0.0){
				AccelRef = 0.0;
				BrakeRef = 0.0;
			}
			else {
				AccelRef = -Joystick_FB_Ref;
				BrakeRef = 0.0;
			}
		}
		else //(FwdRevState == 0) : Park State
		{
			if (DriveEn == 1 && Joystick_FB_Ref == 0.0)
			{
				FwdRevState = 1;
			} // Go to direction choose state
			AccelRef = 0.0;
			if (ParkBrakeEn == 0){
				BrakeRef = 0.0; 
			}
			else {
				BrakeRef = 0.8*MAX_BRAKE_PRESSURE;
			}
		}

		
		
		//============== End Control Loops ========================
	} // End: low sample rate operations
   } // End: main while loop
}

//------------------------------------------------------------------------------------
// General Purpose Functions
//------------------------------------------------------------------------------------

float PID_Controller(PIDInfo *p, float r, float y)
{
	int output;
	float E;
	
	p->r = r;  p->y = y;  //  Store reference and measured output
	E = r - y;  //  Compute tracking error
	
	//dE = Xd + Dd*E;  //  Compute error derivative
	
	output = (p->Kp*E + p->Kd*(p->Xd + p->Dd*E) + p->Ki*p->Xi);
	
	//  Compute state updates
	p->Xd = p->Ad*p->Xd + p->Bd*E;  //  Derivative filter update
	
	p->Xi += E;  // Compute integrator state update
	if (p->Xi > p->Imax) { p->Xi = p->Imax;}
	if (p->Xi < -p->Imax) { p->Xi = -p->Imax;}  //  Clamp integrator value to avoid windup
       
    return output;
}

/*
float LeadLagComp(LeadLagInfo *p, float u)
{
	float output;

	output = p->Xd + p->Dd*u;
	
	//  Compute state updates
	p->Xd = p->Ad*p->Xd + p->Bd*u;  //  state update

	return output;
}
*/

void  ComputePWMOutputs(void)
{
	
if (SampleFlagHighRate == 1)
	{

		SFRPAGE = CONFIG_PAGE;
		SampleFlagHighRate = 0; //  Reset sample flag

		PWMCounter += 1;
		if (PWMCounter >= PWM_RESOLUTION)
		{
			PWMCounter = 0; //  Reset counter
			SampleFlag = 1; //  Set low rate sample flag
		}

		//============== Set Software PWM outputs =====================
		
		//  Steer Valve outputs, SteerValveCmd is the duty cycle, for 8bit PWM
		if (SteerValveCmd >= 0) // S1 active, S2 dither only
		{	
			if (Dither == 1)
			{
				if (PWMCounter <= SteerValveCmd + STEER_OFFSET + STEER_DITHER) { pSteerValveS1 = 1; }
				else  {	pSteerValveS1 = 0; }
				pSteerValveS2 = 0;
			}
			else
			{
				if (PWMCounter <= SteerValveCmd + STEER_OFFSET) { pSteerValveS1 = 1; }
				else  {	pSteerValveS1 = 0; }

				if (PWMCounter <= STEER_OFFSET + STEER_DITHER) { pSteerValveS2 = 1; }
				else  {	pSteerValveS2 = 0; }
			}
		}
		else  // S1 dither only, S2 active
		{
			if (Dither == 1)
			{
				if (PWMCounter <= STEER_OFFSET + STEER_DITHER) { pSteerValveS1 = 1; }
				else  {	pSteerValveS1 = 0; }

				if (PWMCounter <= -SteerValveCmd + STEER_OFFSET) { pSteerValveS2 = 1; }
				else  {	pSteerValveS2 = 0; }
			}
			else
			{
				if (PWMCounter <= -SteerValveCmd + STEER_OFFSET + STEER_DITHER) { pSteerValveS2 = 1; }
				else  {	pSteerValveS2 = 0; }
				pSteerValveS1 = 0;
			}
		}
		

		//  Lean Valve outputs, LeanValveCmd is the duty cycle, for 10bit PWM
		if (LeanValveCmd >= 0) // S1 active, S2 inactive
		{	
			if (PWMCounter <= LeanValveCmd + LEAN_OFFSET) { pLeanValveS1 = 1; }
			else  {	pLeanValveS1 = 0; }
			pLeanValveS2 = 0;
		}
		else  // S1 inactive, S2 active
		{
			if (PWMCounter <= -LeanValveCmd + LEAN_OFFSET) { pLeanValveS2 = 1; }
			else  {	pLeanValveS2 = 0; }
			pLeanValveS1 = 0;
		}

		//  Brake Valve outputs, BrakeValveCmd is the duty cycle, for 10bit PWM
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

	}


}


void SpeedSteeringControlMap(void)
{
	
	int tmpV;

	SFRPAGE = CONFIG_PAGE;

	tmpV = 1729 - (int) SampleVoltage(0);
	Joystick_FB_Ref = ((float) tmpV)/1642.0;  //  Sample JoystickFB

	ComputePWMOutputs();
	tmpV = 1672 - (int) SampleVoltage(1);
	Joystick_LR_Ref = ((float) tmpV)/1623.0;  //  Sample JoystickLR

	ComputePWMOutputs();
	if (Joystick_LR_Ref <= - LR_DEADZONE)
	{
		Joystick_LR_Ref = (Joystick_LR_Ref + LR_DEADZONE) * LR_SCALE;
	}
	else if (Joystick_LR_Ref > -LR_DEADZONE && Joystick_LR_Ref < LR_DEADZONE)
	{
		Joystick_LR_Ref = 0.0;  // Insert steering deadzone
	}
	else
	{
		Joystick_LR_Ref = (Joystick_LR_Ref - LR_DEADZONE) * LR_SCALE;
	}

	//  Compute Speed sensitive lean and steering angle limits
	SteerAngleLimit = SPEED_LIMIT_STEER_SLOPE * SpeedState + STEER_MAX_ANGLE;
	//  Clamp limits to their maxima
	if (SteerAngleLimit > STEER_MAX_ANGLE){ SteerAngleLimit = STEER_MAX_ANGLE;}
	if (SteerAngleLimit < STEER_MIN_ANGLE){ SteerAngleLimit = STEER_MIN_ANGLE;}

	LeanAngleLimit = SPEED_LIMIT_LEAN_SLOPE * SpeedState + LEAN_MIN_ANGLE;
	//  Clamp limits to their maxima
	if (LeanAngleLimit > LEAN_MAX_ANGLE){ LeanAngleLimit = LEAN_MAX_ANGLE;}
	if (LeanAngleLimit < LEAN_MIN_ANGLE){ LeanAngleLimit = LEAN_MIN_ANGLE;}

	//  Map Joystick to have a deadzone defined by AB_DEADZONE
	if (Joystick_FB_Ref <= - AB_DEADZONE)
	{
		Joystick_FB_Ref = (Joystick_FB_Ref + AB_DEADZONE) * AB_SCALE;
	}
	else if (Joystick_FB_Ref > -AB_DEADZONE && Joystick_FB_Ref < AB_DEADZONE)
	{
		Joystick_FB_Ref = 0.0;  // Insert steering deadzone
	}
	else
	{
		Joystick_FB_Ref = (Joystick_FB_Ref - AB_DEADZONE) * AB_SCALE;
	}
}


unsigned int SampleVoltage(unsigned char ACh)
{
  	char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
	unsigned char i;
	unsigned int sum;
	unsigned int result = 0;

   	SFRPAGE = ADC0_PAGE;

	AMX0SL = ACh;  //  Set sample channel

	sum = 0;
    for (i = 4; i != 0; i--)           // repeat 8 times
    {
		AD0INT = 0;                         // clear ADC0 end-of-conversion
   		AD0BUSY = 1;                        // initiate conversion
   		while (!AD0INT);
       	sum += ADC0;                // read ADC and add to sum
    }
    result = sum>>2;   // divide by 16 and cast to uchar

	//result = ADC0; // * (float) Code2VoltageCnst;

	SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
	
	return (result);
}


void SampleSensors(void)
{

	if (SteerSensFlag == 1)
	{
		SteerAngleState = 360.0*SteerDuty/SteerPeriod;	
		//SteerDuty = 0;
		//SteerPeriod = 0;	
	}

	

	//======= Compute Lean Angle ==================
	if (LeanSensFlag == 1)
	{
		LeanAngleState = 360.0*LeanDuty/LeanPeriod - 1.80; //  Correct for a 1.8deg offset	
		//LeanDuty = 0;
		//LeanPeriod = 0;	
	}


	//======= Compute Speed ==================

	if (CntOvrFlow_Speed >= 100 && NewSpeedSampleFlag == 0)
	{
		SpeedState = 0.0; //  We haven't had an edge in 0.5 seconds, effectively zero speed
	}
	else if (NewSpeedSampleFlag == 1)
	{
		if (SpeedDT == 0){
			SpeedState = 0.0;
		}
		else {
			//  SpeedState = 2916000.0/(float)SpeedDT;  compute RPM for a 256cpr optical encoder (testing)
			SpeedState = SPEED2MPH_CNST/(float)SpeedDT;
		}
		NewSpeedSampleFlag = 0;  //  Indicate that we have logged the latest sample
	}
	//  else, SpeedState remains the same

}

int SendUartBuf(void)
{
	if (SendCounter == 0){	

		SFRPAGE = UART0_PAGE;
	
		if (SendBuf[0] == '\n'){
			LFCR_Flag = 1;
		}
		SBUF0 = SendBuf[0];  // Start sending first character to start things off, the rest will be sent via interrupts

		if (NumChars >= UARTBUFSIZE - 1){  //  This is an error, because the buffer overran, just send the valid part
			NumChars = UARTBUFSIZE - 1;
		}
		SFRPAGE = CONFIG_PAGE;
		return(1);  //  Send initiated successfully
	}
	else {
		return(0);  // Data not sent, because previous send is not yet finished.
	}
}

//------------------------------------------------------------------------------------
// Interrupt Service Routines
//------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------
// Timer3_ISR
//------------------------------------------------------------------------------------
// This routine divides the clock further for sampling whenever Timer2 overflows.
//
// NOTE: The SFRPAGE register will automatically be switched to the Timer 3 Page
// When an interrupt occurs.  SFRPAGE will return to its previous setting on exit
// from this routine.
//


void Timer3_ISR (void) interrupt 14
{
	EA = 0;  //  Disable global interrupts
	EA = 0;  //  Dummy code to ensure glitch can't occur

    TMR3CN = 0x00;                      // Stop Timer3; Clear TF2;
	
	SampleFlagHighRate = 1;
	

	TR3 = 1;  //  Restart timer
	EA = 1; //  Enable global interrupts
}

//------------------------------------------------------------------------------------
// UART0_ISR
//------------------------------------------------------------------------------------
// This routine preforms UART functions 
void UART0_ISR (void) interrupt 4
{
   	SFRPAGE = UART0_PAGE;
   	if (RI0==1){ //do receive stuff
   		RI0 = 0;
   	}
 	if(TI0 == 1){ //do transmit stuff
		TI0 = 0;
		if (LFCR_Flag == 1){  // Check to see if we need to send additional CR character
			LFCR_Flag = 0; // Reset flag
			SBUF0 = 0x0D;           // Output CR 
			//SBUF0 = 0x0A; 			// Output LF
		}
		else {
			SendCounter++;
			if (SendCounter < NumChars){
				if (SendBuf[SendCounter] == '\n'){LFCR_Flag = 1;} //  Indicate that we need to send both CR and LF   
				SBUF0 = SendBuf[SendCounter]; //  Send Next Character
			}
			else {
				SendCounter = 0;  // Indicate that we are done
			}
		}
 	}
}

//------------------------------------------------------------------------------------
// PCA_ISR
//------------------------------------------------------------------------------------
// This routine handles interrupts generated by the programmable counter array. 
// Interrupts are caused by PWM sensor signals. Routine detects which sensor triggered
// the interrupt and stores the counter value to be calculated later.
//

void  PCA_ISR (void) interrupt 9 //using 1
{
	unsigned char PCA0H_check1, PCA0H_check2, PCA0L_check1, PCA0L_check2;
	unsigned char tmp = 0;
	//char SFRPAGE_SAVE = SFRPAGE; 

	//  Perhaps disable interrupts and then re-enable them at end
	EA = 0;  // Disable global interrupts
	EA = 0;  //  Dummy code to ensure glitch can't occur

	SFRPAGE   = PCA0_PAGE;

	PCA0L_check1 = PCA0L;
	PCA0H_check1 = PCA0H;
/*
Step 1. Disable global interrupts.
Step 2. Read PCA0L. This will latch the value of PCA0H.
Step 3. Read PCA0H, saving the value.
Step 4. Execute the bit-wise operation on CCFn (for example, CLR CCF0, or CCF0 = 0;).
Step 5. Read PCA0L.
Step 6. Read PCA0H, saving the value.
Step 7. If the value of PCA0H read in Step 3 is 0xFF and the value for PCA0H read in Step 6 is
0x00, then manually set the CF bit in software (for example, SETB CF, or CF = 1;).
Step 8. Re-enable interrupts.
*/

	// Determine if interrupt is caused by counter overflow, or by sensor input edge
	if (CF == 1) // Check CF bit 7 of PCA0CN for counter overflow
	{
		// Set flags for each sensor indicating that counter has overflowed for bookeeping
		CF = 0;
		CntOvrFlow_Steer ++;
		CntOvrFlow_Lean ++;

		CntOvrFlow_Speed ++;
		if (CntOvrFlow_Speed >= 255) {CntOvrFlow_Speed = 0;} //  Limit to two seconds without an edge

		SFRPAGE = CONFIG_PAGE;
		SFRPAGE   = PCA0_PAGE;
	}

	if (CCF0) // Check CCF0 bit 0 of PCA0CN, Steer sensor
	{
		// Should get multiple interrupts between overflows
		CCF0 = 0; // Clear interrupt flag
		SFRPAGE = CONFIG_PAGE;
		tmp = pSteerSenseIn;
		SFRPAGE = PCA0_PAGE;

		if (tmp == 1) //  Rising Edge
		{ 
			// Val0 is both the last data point, and next time's first data point
			SteerSensCntVal0 = (long)((PCA0CPH0 << 8) + PCA0CPL0);
			SteerDiff2 = SteerSensCntVal0 + ((long)CntOvrFlow_Steer << 16) - SteerSensCntVal1; 
			
			CntOvrFlow_Steer = 0; // Reset overflow counter		
			SteerSensFlag = 1;  // Indicate valid data ready
			
			SteerPeriod = SteerDiff1 + SteerDiff2;
			if (SteerDiff1 <= SteerDiff2)
			{
				SteerDuty = SteerDiff1 - (SteerPeriod >> 1);
			}
			else
			{
				SteerDuty = (SteerPeriod >> 1) - SteerDiff2;
			}
			
		}
		else  // Falling edge
		{
			SteerSensCntVal1 = (long)((PCA0CPH0 << 8) + PCA0CPL0);
			SteerDiff1 = SteerSensCntVal1 + ((long)CntOvrFlow_Steer << 16) - SteerSensCntVal0;

			CntOvrFlow_Steer = 0; // Rest overflow counter
			SteerSensFlag = 0;  // Indicate valid data not ready
		}

		SFRPAGE = PCA0_PAGE;
		
		
	}

	if (CCF1) // Check CCF1 bit 1 of PCA0CN, Lean sensor
	{
		// Should get multiple interrupts between overflows
		CCF1 = 0; // Clear interrupt flag

		SFRPAGE = CONFIG_PAGE;
		tmp = pLeanSenseIn;
		SFRPAGE = PCA0_PAGE;

		if (tmp == 1) //  Rising Edge
		{ 
			// Val0 is both the last data point, and next time's first data point
			LeanSensCntVal0 = (long)((PCA0CPH1 << 8) + PCA0CPL1);
			LeanDiff2 = LeanSensCntVal0 + ((long)CntOvrFlow_Lean << 16) - LeanSensCntVal1; 
			
			CntOvrFlow_Lean = 0; // Reset overflow counter		
			LeanSensFlag = 1;  // Indicate valid data ready
			
			LeanPeriod = LeanDiff1 + LeanDiff2;
			if (LeanDiff1 <= LeanDiff2)
			{
				LeanDuty = LeanDiff1 - (LeanPeriod >> 1);
			}
			else
			{
				LeanDuty = (LeanPeriod >> 1) - LeanDiff2;
			}
			
		}
		else  // Falling edge
		{
			LeanSensCntVal1 = (long)((PCA0CPH1 << 8) + PCA0CPL1);
			LeanDiff1 = LeanSensCntVal1 + ((long)CntOvrFlow_Lean << 16) - LeanSensCntVal0;

			CntOvrFlow_Lean = 0; // Rest overflow counter
			LeanSensFlag = 0;  // Indicate valid data not ready
		}

		SFRPAGE = PCA0_PAGE;
		
	}

	if (CCF2) // Check CCF2 bit 2 of PCA0CN, Speed sensor
	{
		// Possible to have many overflows between interrupts (at low to zero speed)
		CCF2 = 0; // Clear interrupt flag
		
		SpeedSensCntVal0 = (PCA0CPH2 << 8) + PCA0CPL2;
		SpeedDT = SpeedSensCntVal0 + ((long)CntOvrFlow_Speed << 16) - SpeedSensCntVal1; 
		SpeedSensCntVal1 = SpeedSensCntVal0;	//  Set to previous value	
		CntOvrFlow_Speed = 0;  //  Reset overflow counter
		NewSpeedSampleFlag = 1;  //  Indicate new data available
		
	}
	if (CCF3)
	{
		CCF3 = 0;
		//PCA0CPL3 = (0xFF & SteerValveCmd); //  Write low byte
		//PCA0CPH3 = (0xFF & (SteerValveCmd>>8)); //  Write high byte
		//PCA0CPM3 = 0xCA;
	}
	if (CCF4) 
	{
		CCF4 = 0;
		//PCA0CPL4 = (0xFF & LeanValveCmd); //  Write low byte
		//PCA0CPH4 = (0xFF & (LeanValveCmd>>8)); //  Write high byte
		//PCA0CPM4 = 0xCA;
	}
	if (CCF5) 
	{
		CCF5 = 0;
		//PCA0CPL5 = (0xFF & BrakeValveCmd); //  Write low byte
		//PCA0CPH5 = (0xFF & (BrakeValveCmd>>8)); //  Write high byte
		//PCA0CPM5 = 0xCA;
	}
		

	PCA0L_check2 = PCA0L;
	PCA0H_check2 = PCA0H;

	if (PCA0H_check1 == 0xFF && PCA0H_check2 == 0x00)
	{
		CF = 1;  // Overflow occured while in the ISR, manually set overflow flag	
	}

	EA = 1;  // Enable global interrupts

	//SFRPAGE = SFRPAGE_SAVE; 
}


/////////////////////////////////////
// Config2 Code Configuration File //
/////////////////////////////////////

// Peripheral specific initialization functions,
// Called from the Init_Device() function
void Timer_Init()
{
	SFRPAGE   = TMR2_PAGE;
    TMR2CN    = 0x04;
    TMR2CF    = 0x08;
    //RCAP2L    = 0xC1;
    //RCAP2H    = 0xFE;
	
	RCAP2 = - ((long) SYSCLK/BAUDRATE/16);
	TMR2 = RCAP2;
  	TR2= 1; 


    SFRPAGE   = TMR3_PAGE;
	TMR3CN    = 0x04;
	TMR3CF    = 0x08;
    RCAP3L    = 0x3A;  //  Set for 25.026kHz (same as before to avoid screwing up controllers)
    RCAP3H    = 0xF8;

}

void PCA_Init()
{
	SFRPAGE   = PCA0_PAGE;
    PCA0CN    = 0x40;
    PCA0MD    = 0x03;
    PCA0CPM0  = 0x31;
    PCA0CPM1  = 0x31;
    PCA0CPM2  = 0x21;
    PCA0CPH2  = 0x80;
    PCA0CPH3  = 0x80;
    PCA0CPH4  = 0x80;
    PCA0CPH5  = 0x80;

/*
	// Code for PCA 16bit PWM, not used presently, but note config problem
	// PCA0CPMn = CA is necessary for 16bit PWM, the config utility is wrong
    PCA0CPM3  = 0xCA; 
    PCA0CPM4  = 0xCA;
    PCA0CPM5  = 0xCA;
    PCA0CPH2  = 0x80;
    PCA0CPH3  = 0x80;
    PCA0CPH4  = 0x80;
    PCA0CPH5  = 0x80;
*/
}

void UART_Init()
{
   SFRPAGE = UART0_PAGE;
   SCON0 = 0x40;                       
   // 8-bit variable baud rate;
   // 9th bit ignored; RX disabled, use 0x50 to enable
   // clear all flags
   SSTA0 = 0x05;  //  use timer 2 for baud rate generation   
   //TI0     = 1;   // Indicate TX0 ready
  
//   ES0 = 1;  
  // IP |= 0x10;
}

void ADC_Init()
{
    SFRPAGE   = ADC0_PAGE;
    ADC0CF    = 0x28;
    ADC0CN    = 0x80;
}

void DAC_Init()
{
    SFRPAGE   = DAC0_PAGE;
    DAC0CN    = 0x80;
}

void Voltage_Reference_Init()
{
    SFRPAGE   = ADC0_PAGE;
    REF0CN    = 0x03;
}

void Port_IO_Init()
{
    // P0.0  -  TX0 (UART0), Push-Pull,  Digital
    // P0.1  -  RX0 (UART0), Open-Drain, Digital
    // P0.2  -  SCK  (SPI0), Open-Drain, Digital
    // P0.3  -  MISO (SPI0), Open-Drain, Digital
    // P0.4  -  MOSI (SPI0), Open-Drain, Digital
    // P0.5  -  NSS  (SPI0), Open-Drain, Digital
    // P0.6  -  SDA (SMBus), Open-Drain, Digital
    // P0.7  -  SCL (SMBus), Open-Drain, Digital

    // P1.0  -  CEX0 (PCA),  Open-Drain, Digital
    // P1.1  -  CEX1 (PCA),  Open-Drain, Digital
    // P1.2  -  CEX2 (PCA),  Open-Drain, Digital
    // P1.3  -  Unassigned,  Open-Drain, Digital
    // P1.4  -  Unassigned,  Open-Drain, Digital
    // P1.5  -  Unassigned,  Open-Drain, Digital
    // P1.6  -  Unassigned,  Push-Pull,  Digital
    // P1.7  -  Unassigned,  Open-Drain, Digital

    // P2.0  -  Unassigned,  Open-Drain, Digital
    // P2.1  -  Unassigned,  Open-Drain, Digital
    // P2.2  -  Unassigned,  Open-Drain, Digital
    // P2.3  -  Unassigned,  Open-Drain, Digital
    // P2.4  -  Unassigned,  Open-Drain, Digital
    // P2.5  -  Unassigned,  Open-Drain, Digital
    // P2.6  -  Unassigned,  Open-Drain, Digital
    // P2.7  -  Unassigned,  Open-Drain, Digital

    // P3.0  -  Unassigned,  Push-Pull,  Digital
    // P3.1  -  Unassigned,  Push-Pull,  Digital
    // P3.2  -  Unassigned,  Push-Pull,  Digital
    // P3.3  -  Unassigned,  Push-Pull,  Digital
    // P3.4  -  Unassigned,  Push-Pull,  Digital
    // P3.5  -  Unassigned,  Push-Pull,  Digital
    // P3.6  -  Unassigned,  Push-Pull,  Digital
    // P3.7  -  Unassigned,  Push-Pull,  Digital

    SFRPAGE   = CONFIG_PAGE;
    P0MDOUT   = 0x01;
    P1MDOUT   = 0x40;
    P3MDOUT   = 0xFF;
	//P5MDOUT   = 0xFF;
    P6MDOUT   = 0xFF;
    XBR0      = 0x1F;
    XBR2      = 0x40;


}

void Oscillator_Init()
{
	//  Configure crystal as sysclk source for UART timing
	int i;                              // Software timer

   char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

   SFRPAGE = CONFIG_PAGE;              // Set SFR page

   OSCICN = 0x83;                      // Set internal oscillator to run
                                       // at its slowest frequency

   CLKSEL = 0x00;                      // Select the internal osc. as
                                       // the SYSCLK source

   // Initialize external crystal oscillator to use 22.1184 MHz crystal

   OSCXCN = 0x67;                      // Enable external crystal osc.
   for (i=0; i < 256; i++);            // Wait at least 1ms

   while (!(OSCXCN & 0x83));           // Wait for crystal osc to settle

   SFRPAGE = LEGACY_PAGE;
   FLSCL |=  0x30;                     // Initially set FLASH read timing for
                                       // 100MHz SYSCLK (most conservative
                                       // setting)
   if (SYSCLK <= 25000000) {           // Set FLASH read timing for <=25MHz
      FLSCL &= ~0x30;
   } else if (SYSCLK <= 50000000) {    // Set FLASH read timing for <=50MHz
      FLSCL &= ~0x20;
   } else if (SYSCLK <= 75000000) {    // Set FLASH read timing for <=75MHz
      FLSCL &= ~0x10;
   } else {                            // set FLASH read timing for <=100MHz
      FLSCL &= ~0x00;
   }

   // Start PLL for 50MHz operation
   SFRPAGE = PLL0_PAGE;
   PLL0CN = 0x04;                      // Select EXTOSC as clk source
   PLL0CN |= 0x01;                     // Enable PLL power
   PLL0DIV = 0x04;                     // Divide by 4
   PLL0FLT &= ~0x0f;
   PLL0FLT |=  0x0f;                   // Set Loop Filt for (22/4)MHz input clock
   PLL0FLT &= ~0x30;                   // Set ICO for 30-60MHz
   PLL0FLT |=  0x10;

   PLL0MUL = 0x09;                     // Multiply by 9

   // wait at least 5us
   for (i = 0; i < 256; i++) ;

   PLL0CN |= 0x02;                     // Enable PLL

   while (PLL0CN & 0x10 == 0x00);      // Wait for PLL to lock

   SFRPAGE = CONFIG_PAGE;

   CLKSEL = 0x02;                      // Select PLL as SYSCLK source

   SFRPAGE = SFRPAGE_SAVE;             // Restore SFRPAGE

    //SFRPAGE   = CONFIG_PAGE;
    //OSCICN    = 0x83;
}

void Interrupts_Init()
{
	//  Enable UART0, Timer3 and PCA interrupts
	IE        = 0x90;  // 0x90 to enable UART0 interrupt, 0x80 to disable
    EIE1      = 0x08;
    EIE2      = 0x01;

}

// Initialization function for device,
// Call Init_Device() from your main program
void Init_Device(void)
{
    Timer_Init();
    PCA_Init();
    ADC_Init();
    DAC_Init();
    Voltage_Reference_Init();
    Port_IO_Init();
    Oscillator_Init();
    Interrupts_Init();
	UART_Init();
}


/*
		//  Reverse Switch Control & Throttle Control
		if (RevSW_Input == 0) // Reverse State
		{
			pRevSw1 = 0;
			pRevSw2 = 1;
			if (AccelRef == 0.0){
				ControlVal = 0.0;
			}
			else {
				ControlVal = 600.0* THROTTLE_REV_GAIN*AccelRef + THTOTTLE_OFFSET; //1000.0*AccelRef; //  Limit Throttle when in reverse
				if (ControlVal > 600.0){ ControlVal = 600.0; } //  Saturate Control
			}
			FwdRevState = 0;
		}
		else
		{
			pRevSw1 = 1;
			pRevSw2 = 0;
			if (AccelRef == 0.0){
				ControlVal = 0.0;
			}
			else {
				ControlVal = 4095.0* THROTTLE_FWD_GAIN*AccelRef + THTOTTLE_OFFSET; //4095.0*AccelRef;  //  Full Throttle
				if (ControlVal > 4095.0){ ControlVal = 4095.0; } //  Saturate Control
			}
			FwdRevState = 1;
		}
*/

/*
	// Fwd-Reverse (For reverse, make a backward pull on joystick be rev accel)
	if (FwdRevState == 0)
	{
		Joystick_FB_Ref = -Joystick_FB_Ref; // Switch brake/accel mapping when in reverse
	}


	if (Joystick_FB_Ref > 1.0)
	{
		AccelRef = 1.0;
		BrakeRef = 0.0;
	}
	else if (Joystick_FB_Ref >= AB_DEADZONE && Joystick_FB_Ref <= 1.0)
	{
		AccelRef = (Joystick_FB_Ref - AB_DEADZONE) * AB_SCALE;
		BrakeRef = 0.0;
	}
	else if (Joystick_FB_Ref < AB_DEADZONE && Joystick_FB_Ref > -AB_DEADZONE)
	{
		AccelRef = 0.0;
    	BrakeRef = 0.0;
	}
	else if (Joystick_FB_Ref <= -AB_DEADZONE && Joystick_FB_Ref >= -1.0)
	{
		AccelRef = 0.0;
		BrakeRef = (-AB_DEADZONE - Joystick_FB_Ref)*AB_SCALE * MAX_BRAKE_PRESSURE;
	}
	else 
	{
		AccelRef = 0.0;
		BrakeRef = MAX_BRAKE_PRESSURE;
	}

	//BrakeRef = Joystick_FB_Ref; // For no feedback control on brakes
	*/
