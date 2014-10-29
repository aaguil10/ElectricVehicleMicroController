//------------------------------------------------------------------------------------
// DagneControl.c
//------------------------------------------------------------------------------------
// Copyright (C) 2008 Multimode Technologies LLC
//
// AUTH: Eric Sandoz
// DATE: 8 June 2008
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
sfr16 RCAP2    = 0xca;                 // Timer2 capture/reload
sfr16 ADC0     = 0xbe;                 // ADC0 data

//------------------------------------------------------------------------------------
// Global CONSTANTS
//------------------------------------------------------------------------------------

#define SYSCLK                24500000     // approximate SYSCLK frequency in Hz
#define LOWRATE_CLKDIVIDE     20          // divide factor to get low rate sample clock
#define STEP_ANGLE			  1.80         // angle of one step in degrees
#define MICRO_STEP_ANGLE      0.2250       // angle of smallest microstep (1/8 of a full step)
#define NUM_AVG_SAMPLES       16            // number of redundant samples over which the useful sample is averaged
										   // Must be a power of 2
#define NUM_AVG_BITS          4            // number of bits matching NUM_AVG_SAMPLES										   

#define VREF_VOLTAGE          2.40         // ADC0 reference voltage
#define AINPUT_GAIN			  0.5          // Prescaling of the ADC signal set by ADC0CF

#define SYSPRESS_LOW_THRESHOLD  1300.0
#define SYSPRESS_HIGH_THRESHOLD 2100.0

#define STEER_GAIN			22.1			//  Roughly a 25deg error maps to 255.0, or full open valve
#define LEAN_GAIN			25.0
#define BRAKE_GAIN			10			//  Roughly 1500psi error yields full open valve

#define STEER_MAX_ANGLE     25.0
#define LEAN_MAX_ANGLE      45.0

#define Code2VoltageCnst    0.0006070774730338503 // derived from measured values (0.0005860805860805861 ideal)
#define PRESS_VOFFSET		-869.01     //  Pressure Voltage sensor characteristic y-offset
#define PRESS_VSLOPE		1780.5      //  Pressure Voltage sensor characteristic slope

#define MAX_SPEED			20.0		//  Maximum speed in mph
#define MAX_BRAKE_PRESSURE  700.0      //  Maximum cylinder pressure in PSI

#define AB_DEADZONE			0.08		//  Accel/Brake deadzone in [0,1]
#define AB_SCALE 		   	1.08695652173913 // AB_SCALE = 1/(1-AB_DEADZONE);

#define LR_DEADZONE			0.05		//  Left/Right deadzone in [0,1]
#define LR_SCALE 		   	1.05263157894737 // LR_SCALE = 1/(1-LR_DEADZONE);

#define PWM_RESOLUTION      256.0  //  10 bit PWM resolution
#define PWM_RES_HALF        128.0  //  Half scale (0-point) of PWM range
#define PWM_LIMIT           245    //  Practical upper limit, = (PWM_RESOLUTION - #_OFFSET) because OFFSET will push duty cycle to limit

#define STEER_DITHER        80      //  approximately 10%
#define STEER_OFFSET        50      //  Dead zone offset value

#define LEAN_DITHER         5       //  approximately 10%
#define LEAN_OFFSET         2       //  Dead zone offset value

#define BRAKE_DITHER        5       //  approximately 10%
#define BRAKE_OFFSET        2       //  Dead zone offset value

#define SFAd                10
#define SFBd                10
										   				
sbit  LED = P1^6;                          // green LED: '1' = ON; '0' = OFF

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
sbit RevSW_Input = P2^7;

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
//------------------------------------------------------------------------------------
// Function PROTOTYPES
//------------------------------------------------------------------------------------
void  Port_IO_Init(void);
void  Timer_Init(void);
void  PCA_Init(void);
void  ADC_Init(void);
void  DAC_Init(void);
void  Interrupts_Init(void);
void  Oscillator_Init(void);
void  Voltage_Reference_Init(void);
void  Init_Device(void);

unsigned int  SampleVoltage(unsigned char ACh);
void  SpeedSteeringControlMap(void);
void  SampleEncoders(void);
void  ComputePWMOutputs(void);

void  InitGlobalVariables(void);

void  Timer2_ISR (void);
void  PCA_ISR (void);

//------------------------------------------------------------------------------------
// Global VARIABLES
//------------------------------------------------------------------------------------

bit SampleFlagHighRate;
bit SampleFlag;

float LeanRef;
float SteerRef;
float BrakeRef;
float AccelRef;

float SteerState;

//float Code2VoltageCnst;

bit PumpState;
bit NextPumpState;
bit KeySwState;
bit NextKeySwState;
bit FwdRevState;

bit Dither;
unsigned int PWMCounter;

//float MeasuredVar;

float SteerAngleAccum;
unsigned char SteerAngleAccumCnt;

float LeanAngleAccum;
unsigned char LeanAngleAccumCnt;

float SpeedAccum;
unsigned int SpeedAccumCnt;

//udblbyte SteerValveCmd;
int SteerValveCmd;
int LeanValveCmd;
int BrakeValveCmd;

//  Sensor PWM counter variables
unsigned char CntOvrFlow_Steer;
unsigned char CntOvrFlow_Lean;
unsigned char CntOvrFlow_Speed;

unsigned int SteerSensCntVal0;
unsigned int SteerSensCntVal1;
unsigned int SteerSensCntVal2;

unsigned int LeanSensCntVal0;
unsigned int LeanSensCntVal1;
unsigned int LeanSensCntVal2;

unsigned int SpeedSensCntVal0;
unsigned int SpeedSensCntVal1;
unsigned int SpeedSensCntVal2;

unsigned char SteerSensOvrFlow1;
unsigned char SteerSensOvrFlow2;
unsigned char LeanSensOvrFlow1;
unsigned char LeanSensOvrFlow2;
unsigned char SpeedSensOvrFlow1;
unsigned char SpeedSensOvrFlow2;

unsigned char SteerSensFlag;
unsigned char LeanSensFlag;
unsigned char SpeedSensFlag;

// Control Variables
//float ControlVal;


//------------------------------------------------------------------------------------
// MAIN Routine
//------------------------------------------------------------------------------------
void main (void) {

	//unsigned int TmpMeasuredVar;
	float ControlVal, MeasuredVar; 
	unsigned int AccelCmd = 0;
	//unsigned int PWMCounter = 0;
	//char Dither = 1;

   // disable watchdog timer
   WDTCN = 0xde;
   WDTCN = 0xad;

   SFRPAGE = CONFIG_PAGE;                 // Switch to configuration page
   Init_Device ();
                                     
   EA = 1;											// enable global interrupts

   //SFRPAGE = LEGACY_PAGE;                 // Page to sit in for now
   SFRPAGE = CONFIG_PAGE;
   InitGlobalVariables();

	//==========================================================
	//     Let's start the program shall we
	//==========================================================
	
   while (1) {                            // spin forever
	
	SampleEncoders();  // Check for valid samples
	

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
		//LED = ~LED;
		
		//  Measure Analog values from Joystick and pressure transducers
		
		//========= Open Loop Control Mapping ==============

		SpeedSteeringControlMap();  //  Map control commands to lean and steer commands, speed dependent

		//  Hydraulic valve Dither signal, toggle at lower sampling rate
		Dither = ~Dither;		

		
		//========= Steer, Lean, Brake Control Loops ==============

		ComputePWMOutputs();
		//  Steer Control 
		MeasuredVar = -SteerAngleAccum / ((float) SteerAngleAccumCnt);
		SteerAngleAccum = 0;
		SteerAngleAccumCnt = 0;

		//SteerInputState = SFAd*SteerInputState + SFBd*(SteerRef - MeasuredVar);
		ControlVal = -STEER_GAIN*(SteerRef - MeasuredVar);
		SteerState = ControlVal;
		//ControlVal = PWM_RESOLUTION*SteerRef/STEER_MAX_ANGLE  + Dither*STEER_DITHER;
		
		if (ControlVal > PWM_LIMIT){SteerValveCmd = PWM_LIMIT; } 
		else if (ControlVal < -PWM_LIMIT){SteerValveCmd = -PWM_LIMIT; } 
		else {SteerValveCmd = (int) ControlVal; } 
		
        
		ComputePWMOutputs();
		//  Lean Control
		MeasuredVar = LeanAngleAccum/((float)LeanAngleAccumCnt);
		LeanAngleAccum = 0;
		LeanAngleAccumCnt = 0;
		ControlVal = LEAN_GAIN*(LeanRef - MeasuredVar);

		if (ControlVal > PWM_LIMIT){LeanValveCmd = PWM_LIMIT; } 
		else if (ControlVal < -PWM_LIMIT){LeanValveCmd = -PWM_LIMIT; } 
		else {LeanValveCmd = (int) ControlVal; }
		LeanValveCmd = 0;


		ComputePWMOutputs();
		//  Brake Control
		MeasuredVar  = ((float)SampleVoltage(6)) * Code2VoltageCnst * PRESS_VSLOPE + PRESS_VOFFSET;  //  Sample Brake Pressure
		ControlVal = BRAKE_GAIN*(BrakeRef - MeasuredVar);

		//ControlVal = PWM_RESOLUTION*BrakeRef/MAX_BRAKE_PRESSURE  + 0*Dither*BRAKE_DITHER;
		
		
		if (ControlVal > PWM_LIMIT){BrakeValveCmd = PWM_LIMIT; } 
		else if (ControlVal < -PWM_LIMIT){BrakeValveCmd = -PWM_LIMIT; } 
		else {BrakeValveCmd = ((int) ControlVal); }


		ComputePWMOutputs();
		//========= Hydraulic System Control Loop ==============
		MeasuredVar = ((float)SampleVoltage(7)) * Code2VoltageCnst * PRESS_VSLOPE + PRESS_VOFFSET;  //  Sample Main Hydrualic Pressure
		
		SFRPAGE = CONFIG_PAGE;
		if (PumpState == 0)
		{
			// Pump is off, transition if system pressure is below our lower threshold
			if (MeasuredVar < SYSPRESS_LOW_THRESHOLD)	
			{
				NextPumpState = 1;
			}
			HydPumpEn = 0;
		}
		else if (PumpState == 1)
		{
			// Pump is on, transition if system pressure is above our upper threshold
			if (MeasuredVar > SYSPRESS_HIGH_THRESHOLD)	
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
			pKeySwitch = 0;
		}
		else if (KeySwState == 1)
		{
			// Motor Controller is on, transition if brake is applied
			if (BrakeRef > 0.0)	
			{
				NextKeySwState = 0;
			}
			pKeySwitch = 1;
		}
		KeySwState = NextKeySwState; // State Update


		//  Reverse Switch Control
		if (RevSW_Input == 0) // Reverse State
		{
			pRevSw1 = 0;
			pRevSw2 = 1;
			ControlVal = 2000.0*AccelRef; //1000.0*AccelRef; //  Limit Throttle when in reverse
			FwdRevState = 0;
		}
		else
		{
			pRevSw1 = 1;
			pRevSw2 = 0;
			ControlVal = 4095.0*AccelRef; //4095.0*AccelRef;  //  Full Throttle
			FwdRevState = 1;
		}
		
		//  Throttle Control
		/*
		MeasuredVar = SpeedAccum/SpeedAccumCnt;
		SpeedAccum = 0;
		SpeedAccumCnt = 0;
		
		if (MeasuredVar >= MAX_SPEED)
		{
			ControlVal = 0.1 * AccelRef;  // Cut the throttle back big time 
		}
		else
		{
			ControlVal = 4095.0*AccelRef;
		}
		*/

		ComputePWMOutputs();
		//  Clamp command to allowable range
		if (ControlVal > 4095.0) { ControlVal = 4095.0;}
		if (ControlVal < 0.0) { ControlVal = 0.0;}

		SFRPAGE   = DAC0_PAGE;
		AccelCmd = 4095 - ((unsigned int) ControlVal); // Must invert to cancel inverting OpAmp output stage
		DAC0L = (0xFF & AccelCmd); //  Write low byte
		DAC0H = (0xFF & (AccelCmd>>8)); //  Write high byte
		SFRPAGE = CONFIG_PAGE;

		//============== End Control Loops ========================

		

	} // End: low sample rate operations



   } // End: main while loop
}

//------------------------------------------------------------------------------------
// General Purpose Functions
//------------------------------------------------------------------------------------

void InitGlobalVariables(void)
{


SampleFlag = 0;
SampleFlagHighRate = 0;


LeanRef = 0.0;
SteerRef = 0.0;
BrakeRef = 0.0;
AccelRef = 0.0;

SteerState = 0.0;

PWMCounter = 0;
Dither = 0;

//MeasuredVar = 0.0;

PumpState = 0;
NextPumpState = 0;

KeySwState = 0;
NextKeySwState = 0;

FwdRevState = 1;

SteerAngleAccum = 0.0;
SteerAngleAccumCnt = 0;

LeanAngleAccum = 0.0;
LeanAngleAccumCnt = 0;

SpeedAccum = 0.0;
SpeedAccumCnt = 0;


//Code2VoltageCnst = VREF_VOLTAGE/4095;

//  Sensor PWM counter variables
CntOvrFlow_Steer = 0;
CntOvrFlow_Lean = 0;
CntOvrFlow_Speed = 0;

SteerSensCntVal0 = 0;
SteerSensCntVal1 = 0;
SteerSensCntVal2 = 0;

LeanSensCntVal0 = 0;
LeanSensCntVal1 = 0;
LeanSensCntVal2 = 0;

SpeedSensCntVal0 = 0;
SpeedSensCntVal1 = 0;
SpeedSensCntVal2 = 0;

SteerSensOvrFlow1 = 0;
SteerSensOvrFlow2 = 0;
LeanSensOvrFlow1 = 0;
LeanSensOvrFlow2 = 0;
SpeedSensOvrFlow1 = 0;
SpeedSensOvrFlow2 = 0;

SteerSensFlag = 0;
LeanSensFlag = 0;
SpeedSensFlag = 0;

// Control Variables
//ControlVal = 128.0;

SteerValveCmd = 0;
LeanValveCmd = 0;
BrakeValveCmd = 0;

}

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
	float Joystick_LR_Ref = 0, Joystick_FB_Ref = 0;
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


	LeanRef = 0.0*LEAN_MAX_ANGLE; 
	SteerRef = Joystick_LR_Ref * STEER_MAX_ANGLE;  //  For now map joystick to steer only

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


void SampleEncoders(void)
{
	long int Diff1 = 0; 
	long int Diff2 = 0;
	float Duty = 0.0;
	// Poll for valid data on all three sensors

	//======= Compute Steer Angle ==================
	if (SteerSensFlag >= 3)
	{
		// if SteerSensFlag > 3 then this is an error condition, but not sure what to do about it
		//  Compute On time	
		Diff1 = (long int)SteerSensCntVal1 + (long int)SteerSensOvrFlow1*65535 - (long int)SteerSensCntVal0; // Should always be positive
		//  Compute period
		Diff2 = (long int)SteerSensCntVal2 + (long int)SteerSensOvrFlow2*65535 - (long int)SteerSensCntVal0;
			
		SteerSensOvrFlow1 = 0;
		SteerSensOvrFlow2 = 0;
		CntOvrFlow_Steer = 0;  // Reset overflow counter
		SteerSensFlag = 0; // Reset flag so that more data can be taken
		
		//SteerSensCntVal0 = SteerSensCntVal2;
		// Set up sensor such that, in our operational range, we are always < 50% duty cycle
		// Duty should have a range of 25%
		if (Diff1 <= Diff2 && Diff1 >= 0 && Diff2 > 0) //Diff2 >= 24000 && Diff2 <= 26000)
		{
			Duty = (float) Diff1/ ((float) Diff2);
			if (Duty > 0.5)
			{
				Duty = 1 - Duty;
			}
			SteerAngleAccum += (Duty - 0.25)*360.0;
			SteerAngleAccumCnt += 1;
		}
	}

	//======= Compute Lean Angle ==================
	if (LeanSensFlag >= 3)
	{
		// if LeanSensFlag > 3 then this is an error condition, but not sure what to do about it
		//  Compute On time	
		Diff1 = (long int)LeanSensCntVal1 + (long int)LeanSensOvrFlow1*65535 - (long int)LeanSensCntVal0; // Should always be positive
		//  Compute period
		Diff2 = (long int)LeanSensCntVal2 + (long int)LeanSensOvrFlow2*65535 - (long int)LeanSensCntVal0;
			
		LeanSensOvrFlow1 = 0;
		LeanSensOvrFlow2 = 0;
		CntOvrFlow_Lean = 0;  // Reset overflow counter
		LeanSensFlag = 0; // Reset flag so that more data can be taken
		
		// Set up sensor such that, in our operational range, we are always < 50% duty cycle
		// Duty should have a range of 25%
		if (Diff1 <= Diff2 && Diff1 >= 0 && Diff2 >= 0) //24000 && Diff2 <= 26000)
		{
			Duty = (float) Diff1/ ((float) Diff2);
			if (Duty > 0.5)
			{
				Duty = 1 - Duty;
			}
			LeanAngleAccum += (Duty - 0.25)*360.0;
			LeanAngleAccumCnt += 1;
		}
	}

	//======= Compute Speed ==================
	if (SpeedSensFlag >= 2)
	{
		Diff1 = (long int)SpeedSensCntVal1 + (long int)SpeedSensOvrFlow1*65535 - (long int)SpeedSensCntVal0; 
		
		if (Diff1 == 0)
		{
			SpeedAccum += 0;
			SpeedAccumCnt += 1;
		}
		else
		{
			// Cnst = 3600 c[mi]/(#teeth Ts), c[mi] is tire circumfrance in miles
			// Ts is PCA sample rate 4/SYSCLK, #teeth = 60
			SpeedAccum += 430436.9475/ ((float)Diff1);
			SpeedAccumCnt += 1;
		}
	}


}


//------------------------------------------------------------------------------------
// Interrupt Service Routines
//------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------
// Timer2_ISR
//------------------------------------------------------------------------------------
// This routine divides the clock further for sampling whenever Timer2 overflows.
//
// NOTE: The SFRPAGE register will automatically be switched to the Timer 3 Page
// When an interrupt occurs.  SFRPAGE will return to its previous setting on exit
// from this routine.
//


void Timer2_ISR (void) interrupt 5
{
	EA = 0;  //  Disable global interrupts
	EA = 0;  //  Dummy code to ensure glitch can't occur

    TMR2CN = 0x00;                      // Stop Timer2; Clear TF2;
	
	SampleFlagHighRate = 1;

	TR2 = 1;  //  Restart timer
	EA = 1; //  Enable global interrupts
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
	//char SFRPAGE_SAVE = SFRPAGE; 

	//  Perhaps disable interrupts and then re-enable them at end
	EA = 0;  // Disable global interrupts
	EA = 0;  //  Dummy code to ensure glitch can't occur

	//SFRPAGE   = PCA0_PAGE;

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
		CntOvrFlow_Steer += 1;
		CntOvrFlow_Lean += 1;
		CntOvrFlow_Speed += 1;
		//LED = ~LED;
		
	}

	if (CCF0) // Check CCF0 bit 0 of PCA0CN, Steer sensor
	{
		// Should get multiple interrupts between overflows
		CCF0 = 0; // Clear interrupt flag
		if (SteerSensFlag == 0)
		{
			SteerSensCntVal0 = (PCA0CPH0 << 8) + PCA0CPL0; // Store captured value
			CntOvrFlow_Steer = 0;
		}
		else if (SteerSensFlag == 1)
		{
			SteerSensCntVal1 = (PCA0CPH0 << 8) + PCA0CPL0; // Store captured value
			SteerSensOvrFlow1 = CntOvrFlow_Steer;
		}
		else if (SteerSensFlag == 2)
		{
			SteerSensCntVal2 = (PCA0CPH0 << 8) + PCA0CPL0; // Store captured value
			SteerSensOvrFlow2 = CntOvrFlow_Steer;
		}
		//  Else is an error, because the samples should have been processed and Flag set to 0
		SteerSensFlag += 1; // Log number of interrupts
	}

	if (CCF1) // Check CCF1 bit 1 of PCA0CN, Lean sensor
	{
		// Should get multiple interrupts between overflows
		CCF1 = 0; // Clear interrupt flag
		if (LeanSensFlag == 0)
		{
			LeanSensCntVal0 = (PCA0CPH1 << 8) + PCA0CPL1; // Store captured value
			CntOvrFlow_Lean = 0;
		}
		else if (LeanSensFlag == 1)
		{
			LeanSensCntVal1 = (PCA0CPH1 << 8) + PCA0CPL1; // Store captured value
			LeanSensOvrFlow1 = CntOvrFlow_Lean;
		}
		else if (LeanSensFlag == 2)
		{
			LeanSensCntVal2 = (PCA0CPH1 << 8) + PCA0CPL1; // Store captured value
			LeanSensOvrFlow2 = CntOvrFlow_Lean;
		}
		//  Else is an error, because the samples should have been processed and Flag set to 0
		LeanSensFlag += 1; // Log number of interrupts
	}

	if (CCF2) // Check CCF2 bit 2 of PCA0CN, Speed sensor
	{
		// Possible to have many overflows between interrupts (at low to zero speed)
		CCF2 = 0; // Clear interrupt flag
		if (SpeedSensFlag == 0)
		{
			SpeedSensCntVal0 = (PCA0CPH2 << 8) + PCA0CPL2; // Store captured value
			SpeedSensCntVal1 = SpeedSensCntVal0;
			CntOvrFlow_Speed = 0;
		}
		else if (SteerSensFlag == 1)
		{
			SpeedSensCntVal1 = (PCA0CPH2 << 8) + PCA0CPL2; // Store captured value
			SpeedSensOvrFlow1 = CntOvrFlow_Speed;
		}
		
		//  Else is an error, because the samples should have been processed and Flag set to 0
		SpeedSensFlag += 1; // Log number of interrupts
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
    TMR2CF    = 0x0A;
    RCAP2L    = 0x2C;
    RCAP2H    = 0xFC;
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
    // P0.0  -  CEX0 (PCA),  Open-Drain, Digital
    // P0.1  -  CEX1 (PCA),  Open-Drain, Digital
    // P0.2  -  CEX2 (PCA),  Open-Drain, Digital
    // P0.3  -  Unassigned,  Open-Drain, Digital
    // P0.4  -  Unassigned,  Open-Drain, Digital
    // P0.5  -  Unassigned,  Open-Drain, Digital
    // P0.6  -  Unassigned,  Open-Drain, Digital
    // P0.7  -  Unassigned,  Open-Drain, Digital

    // P1.0  -  Skipped,     Open-Drain, Analog
    // P1.1  -  Skipped,     Open-Drain, Analog
    // P1.2  -  Skipped,     Open-Drain, Analog
    // P1.3  -  Skipped,     Open-Drain, Analog
    // P1.4  -  Skipped,     Open-Drain, Analog
    // P1.5  -  Skipped,     Open-Drain, Analog
    // P1.6  -  Unassigned,  Push-Pull,  Digital
    // P1.7  -  Skipped,     Open-Drain, Analog

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
    P1MDIN    = 0x40;
    P1MDOUT   = 0x40;
    P3MDOUT   = 0xFF;
    P6MDOUT   = 0xFF;
    XBR0      = 0x18;
    XBR2      = 0x40;
}

void Oscillator_Init()
{
    SFRPAGE   = CONFIG_PAGE;
    OSCICN    = 0x83;
}

void Interrupts_Init()
{
    IE        = 0xA0;
    EIE1      = 0x08;
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
}
