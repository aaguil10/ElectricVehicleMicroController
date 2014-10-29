//------------------------------------------------------------------------------------
// DagneControl.c
//------------------------------------------------------------------------------------
// Copyright (C) 2008 Multimode Technologies LLC
//
// AUTH: Eric Sandoz
// DATE: 20 August 2008
//
// This program controls the sustainable supercar Dagne.
// Specifically this moves valve control to the PCA, employs the crystal oscillators
// for precise timing, and uses both the UART for data logging and the SPI for reading
// in accelerometer data.

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

//#define SYSCLK                24500000     // approximate SYSCLK frequency in Hz
#define SYSCLK       (22118400L * 9 / 4)
#define LOWRATE_CLKDIVIDE     20           // divide factor to get low rate sample clock
#define STEP_ANGLE			  1.80         // angle of one step in degrees
#define MICRO_STEP_ANGLE      0.2250       // angle of smallest microstep (1/8 of a full step)
#define NUM_AVG_SAMPLES       16           // number of redundant samples over which the useful sample is averaged
										   // Must be a power of 2
#define NUM_AVG_BITS          4            // number of bits matching NUM_AVG_SAMPLES										   

#define VREF_VOLTAGE          2.40         // ADC0 reference voltage
#define AINPUT_GAIN			  0.5          // Prescaling of the ADC signal set by ADC0CF

#define SYSPRESS_LOW_THRESHOLD  1300.0
#define SYSPRESS_HIGH_THRESHOLD 2100.0

#define STEER_GAIN			13.1			//  Roughly a 25deg error maps to 255.0, or full open valve
#define STEER_Kp			13.1
#define STEER_Kd			0
#define STEER_Ki			0
#define STEER_Imax			20
#define STEER_Ad			0.0432139  // exp(-P*Ts)
#define STEER_Bd		   -300.583  // (exp(-P*Ts) - 1)*P
#define STEER_Dd			314.159  // P

#define LEAN_GAIN			5.0
#define BRAKE_GAIN			10			//  Roughly 1500psi error yields full open valve

#define STEER_MAX_ANGLE     25.0
#define LEAN_MAX_ANGLE      10.0

//#define TIRE_RADIUS         0.295;  // Tire radius in meters
//#define N_TEETH             60;  // Number of gear teeth on speed sensor
//#define TIRE_CIRC_MI        0.00115281; // Tire circumfrance in miles (2*pi*R*0.0006213712)
//#define PCA_TS              4/SYSCLK;  // Ts is PCA sample rate 4/SYSCLK
#define SPEED_SENSOR_CNST   423263.0411 //3600*TIRE_CIRC_MI/(N_TEETH*PCA_TS);

#define Code2VoltageCnst    0.0006070774730338503 // derived from measured values (0.0005860805860805861 ideal)
#define PRESS_VOFFSET		-869.01     //  Pressure Voltage sensor characteristic y-offset
#define PRESS_VSLOPE		1780.5      //  Pressure Voltage sensor characteristic slope

#define MAX_SPEED			20.0		//  Maximum speed in mph
#define MAX_BRAKE_PRESSURE  700.0      //  Maximum cylinder pressure in PSI

#define AB_DEADZONE			0.03		//  Accel/Brake deadzone in [0,1]
#define AB_SCALE 		   	1.08695652173913 // AB_SCALE = 1/(1-AB_DEADZONE);

#define LR_DEADZONE			0.02		//  Left/Right deadzone in [0,1]
#define LR_SCALE 		   	1.05263157894737 // LR_SCALE = 1/(1-LR_DEADZONE);

#define PWM_RESOLUTION      256.0  //  10 bit PWM resolution
#define PWM_RES_HALF        128.0  //  Half scale (0-point) of PWM range
#define PWM_LIMIT           136    //  Practical upper limit, = (PWM_RESOLUTION - #_OFFSET) because OFFSET will push duty cycle to limit

#define STEER_DITHER        30      //  approximately 10%
#define STEER_OFFSET        120     //  Dead zone offset value

#define LEAN_DITHER         30       //  approximately 10%
#define LEAN_OFFSET         120      //  Dead zone offset value

#define BRAKE_DITHER        30       //  approximately 10%
#define BRAKE_OFFSET        80       //  Dead zone offset value

#define SFAd                10
#define SFBd                10
										   				
sbit  LED = P1^6;                          // green LED: '1' = ON; '0' = OFF

// Sensor Input Pins
sbit pLeanSenseIn = P0^1;
sbit pSteerSenseIn = P0^0;
sbit pSpeedSenseIn = P0^2;

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
sbit RevSW_Input = P2^5;

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
void  UART_Init(void);
void  Voltage_Reference_Init(void);
void  Init_Device(void);

unsigned int  SampleVoltage(unsigned char ACh);
void  SpeedSteeringControlMap(void);
void  SampleSensors(void);

float PID_Controller(PIDInfo *p, float r, float y); 

void  Timer2_ISR (void);
void  PCA_ISR (void);

//------------------------------------------------------------------------------------
// Global VARIABLES
//------------------------------------------------------------------------------------

bit SampleFlagHighRate = 0;
bit SampleFlag = 0;

xdata float LeanRef = 0.0;
xdata float SteerRef = 0.0;
xdata float BrakeRef = 0.0;
xdata float AccelRef = 0.0;

//float Code2VoltageCnst;

bit PumpState = 0;
bit NextPumpState = 0;
bit KeySwState = 0;
bit NextKeySwState = 0;
bit FwdRevState = 1;

bit Dither = 0;
unsigned int PWMCounter = 0;

//float MeasuredVar;

float SteerAngle = 0.0;
float LeanAngle = 0.0;
float Speed_mph = 0.0;
float BrakePressure = 0.0;
float SystemPressure = 0.0;

//udblbyte SteerValveCmd;
int SteerValveCmd = 0;
int LeanValveCmd = 0;
int BrakeValveCmd = 0;

//  Sensor PWM counter variables
xdata unsigned char CntOvrFlow_Steer = 0;
xdata unsigned char CntOvrFlow_Lean = 0;
xdata unsigned char CntOvrFlow_Speed = 0;

xdata unsigned int SteerSenseCntVal0 = 0;
xdata unsigned int SteerSenseCntVal1 = 0;
xdata unsigned int SteerSenseCntVal2 = 0;

xdata unsigned int SteerSenseLatch2 = 0;
xdata unsigned int SteerSenseLatch1 = 0;
xdata unsigned int SteerSenseLatch0 = 0;
xdata unsigned int SteerCntOvrFlowLatch = 0;

xdata unsigned int LeanSenseCntVal0 = 0;
xdata unsigned int LeanSenseCntVal1 = 0;
xdata unsigned int LeanSenseCntVal2 = 0;

xdata unsigned int LeanSenseLatch2 = 0;
xdata unsigned int LeanSenseLatch1 = 0;
xdata unsigned int LeanSenseLatch0 = 0;
xdata unsigned int LeanCntOvrFlowLatch = 0;

xdata unsigned int SpeedSenseCntVal0 = 0;
xdata unsigned int SpeedSenseCntVal1 = 0;

xdata unsigned int SpeedSenseLatch1 = 0;
xdata unsigned int SpeedSenseLatch0 = 0;
xdata unsigned int SpeedCntOvrFlowLatch = 0;

unsigned char SteerSenseFlag = 0;
unsigned char LeanSenseFlag = 0;
unsigned char SpeedSenseFlag = 0;

// Control Variables
//float ControlVal;


//------------------------------------------------------------------------------------
// MAIN Routine
//------------------------------------------------------------------------------------
void main (void) {

	//unsigned int TmpMeasuredVar;
	float ControlVal = 0.0; 
	unsigned int AccelCmd = 0;
	//unsigned int PWMCounter = 0;
	//char Dither = 1;

	xdata PIDInfo *SteerPID; //, LeanPID, BrakePID;

	//  Initilize Steering PID controller
	SteerPID->Kp = STEER_Kp; SteerPID->Kd = STEER_Kd; SteerPID->Ki = STEER_Ki; SteerPID->Imax = STEER_Imax;
	SteerPID->Ad = STEER_Ad; SteerPID->Bd = STEER_Bd; SteerPID->Dd = STEER_Dd;

   // disable watchdog timer
   WDTCN = 0xde;
   WDTCN = 0xad;

   SFRPAGE = CONFIG_PAGE;                 // Switch to configuration page
   Init_Device ();
                                     
   EA = 1;											// enable global interrupts

   //SFRPAGE = LEGACY_PAGE;                 // Page to sit in for now
   SFRPAGE = CONFIG_PAGE;

	//==========================================================
	//     Let's start the program shall we
	//==========================================================
	
   while (1) {                            // spin forever



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

		SampleSensors();  // Check for valid samples
		
		//  Measure Analog values from Joystick and pressure transducers
		
		//========= Open Loop Control Mapping ==============

		SpeedSteeringControlMap();  //  Map control commands to lean and steer commands, speed dependent

		//  Hydraulic valve Dither signal, toggle at lower sampling rate
		Dither = ~Dither;		

		
		//========= Steer, Lean, Brake Control Loops ==============

		//  Steer Control 

		//SteerInputState = SFAd*SteerInputState + SFBd*(SteerRef - MeasuredVar);
		//ControlVal = STEER_GAIN*(SteerRef - SteerAngle);
		
		ControlVal = PID_Controller(SteerPID, SteerRef, SteerAngle);
		
		//ControlVal = PWM_RESOLUTION*SteerRef/STEER_MAX_ANGLE;
		
		if (ControlVal > PWM_LIMIT){SteerValveCmd = PWM_LIMIT; } 
		else if (ControlVal < -PWM_LIMIT){SteerValveCmd = -PWM_LIMIT; } 
		else {SteerValveCmd = (int) ControlVal; } 
		
        
		//  Lean Control
		ControlVal = LEAN_GAIN*(LeanRef - LeanAngle);

		if (ControlVal > PWM_LIMIT){LeanValveCmd = PWM_LIMIT; } 
		else if (ControlVal < -PWM_LIMIT){LeanValveCmd = -PWM_LIMIT; } 
		else {LeanValveCmd = (int) ControlVal; }
		LeanValveCmd = 0; // Hold for now until steering is tuned in

		//  Brake Control
		
		ControlVal = BRAKE_GAIN*(BrakeRef - BrakePressure);

		//ControlVal = PWM_RESOLUTION*BrakeRef/MAX_BRAKE_PRESSURE  + 0*Dither*BRAKE_DITHER;
		
		
		if (ControlVal > PWM_LIMIT){BrakeValveCmd = PWM_LIMIT; } 
		else if (ControlVal < -PWM_LIMIT){BrakeValveCmd = -PWM_LIMIT; } 
		else {BrakeValveCmd = ((int) ControlVal); }


		//========= Hydraulic System Control Loop ==============
		
		SFRPAGE = CONFIG_PAGE;
		if (PumpState == 0)
		{
			// Pump is off, transition if system pressure is below our lower threshold
			if (SystemPressure < SYSPRESS_LOW_THRESHOLD)	
			{
				NextPumpState = 1;
			}
			HydPumpEn = 0;
		}
		else if (PumpState == 1)
		{
			// Pump is on, transition if system pressure is above our upper threshold
			if (SystemPressure > SYSPRESS_HIGH_THRESHOLD)	
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
				NextKeySwState = 1;
			
			pKeySwitch = 0;
		}
		else if (KeySwState == 1)
		{
			// Motor Controller is on, transition if brake is applied
			if (BrakeRef > 0.0)	
				NextKeySwState = 0;
			
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

void SpeedSteeringControlMap(void)
{
	float Joystick_LR_Ref = 0, Joystick_FB_Ref = 0;
	int tmpV;

	SFRPAGE = CONFIG_PAGE;

	tmpV = 1729 - (int) SampleVoltage(0);
	Joystick_FB_Ref = ((float) tmpV)/1642.0;  //  Sample JoystickFB

	tmpV = 1672 - (int) SampleVoltage(1);
	Joystick_LR_Ref = ((float) tmpV)/1623.0;  //  Sample JoystickLR

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


	LeanRef = Joystick_LR_Ref * LEAN_MAX_ANGLE; 
	SteerRef = 0.0 * STEER_MAX_ANGLE;  //  For now map joystick to steer only

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


void SampleSensors(void)
{
	long int OnTime = 0; 
	long int Period = 0;

	// Poll for valid data on all five sensors
	SystemPressure = ((float)SampleVoltage(7)) * Code2VoltageCnst * PRESS_VSLOPE + PRESS_VOFFSET;  //  Sample Main Hydrualic Pressure
		
	BrakePressure  = ((float)SampleVoltage(6)) * Code2VoltageCnst * PRESS_VSLOPE + PRESS_VOFFSET;  //  Sample Brake Pressure

	//======= Compute Steer Angle ==================
	if (SteerSenseFlag == 1)
	{
		//  Compute On time	
		OnTime = (long int)SteerSenseLatch1 + ((long int)SteerCntOvrFlowLatch << 16) - (long int)SteerSenseLatch0; // Should always be positive
		//  Compute period
		Period = (long int)SteerSenseLatch2 + ((long int)SteerCntOvrFlowLatch << 16) - (long int)SteerSenseLatch0;
			
		SteerSenseFlag = 0; // Reset flag so that more data can be taken
		
		SteerAngle = -((float) OnTime/ ((float) Period) - 0.5)*360.0;
	}

	//======= Compute Lean Angle ==================
	if (LeanSenseFlag == 1) 
	{
		//  Compute On time	
		OnTime = (long int)LeanSenseLatch1 + ((long int)LeanCntOvrFlowLatch << 16) - (long int)LeanSenseLatch0; // Should always be positive
		//  Compute period
		Period = (long int)LeanSenseLatch2 + ((long int)LeanCntOvrFlowLatch << 16) - (long int)LeanSenseLatch0;
			
		LeanSenseFlag = 0; // Reset flag so that more data can be taken
		
		LeanAngle = ((float) OnTime/ ((float) Period) - 0.5)*360.0;
	}

	//======= Compute Speed ==================
	//  If Sample Encoders is called, and no new valid data exists, the Speed_mph will hold its last value
	if (SpeedSenseFlag == 1) 
	{
		SpeedSenseFlag = 0;  // Reset flag so that we know when next valid sample is available
		Period = (long int)SpeedSenseLatch1 + ((long int)SpeedCntOvrFlowLatch << 16) - (long int)SpeedSenseLatch0; 

		if (Period == 0)
			Speed_mph = 0.0;

		else 
		{
			// Cnst = 3600 c[mi]/(#teeth Ts), c[mi] is tire circumfrance in miles
			// Ts is PCA sample rate 4/SYSCLK, #teeth = 60  
			//SPEED_SENSOR_CNST
			Speed_mph = SPEED_SENSOR_CNST / ((float)Period);
		}
	}


}


//------------------------------------------------------------------------------------
// Interrupt Service Routines
//------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------
// Timer3_ISR
//------------------------------------------------------------------------------------
// This routine changes the state of the LED whenever Timer3 overflows.
void Timer3_ISR (void) interrupt 14
{
   SFRPAGE = TMR3_PAGE;
   TF3 = 0;                               // clear TF3
   TR3 = 1; 
   SampleFlag = 1;
}

//------------------------------------------------------------------------------------
// UART0_ISR
//------------------------------------------------------------------------------------
// This routine sends bytes from the data buffer until it is empty 
void UART0_ISR (void) interrupt 4
{
	
   SFRPAGE = UART0_PAGE;
   
   if (RI0==1){ //do receive stuff
   		RI0 = 0;
		mrc = SBUF0;
   		
		if ((c[pns] == mrc) && (pns < 7))
			pns++;
			 
   		else if ((pns >= 7) && (pns < 17)){
		
			currentTime[pns - 7] = mrc;
			pns++;
			}

		else if (pns >= 17){
			gotTimeStamp = 1;
			pns = 0;
			currentTime[10]= '\0';
			} 

		else{
			pns = 0;		
		}
   }
 	if(TI0 == 1){ //do transmit stuff
		TI0 = 0;
 	}
}


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
		
		if (CntOvrFlow_Speed >= 255) 
		{   // No sensor events in 255 overflows of PCA, we are at zero speed 
			CntOvrFlow_Speed = 0;  // Reset counter
			SpeedSenseCntVal0 = (PCA0H << 8) + PCA0L; // Store captured value

			//  Use special case of Period = 0 implies Speed = 0;
			SpeedSenseLatch1 = 0;
			SpeedSenseLatch0 = 0;
			SpeedCntOvrFlowLatch = 0;
			SpeedSenseFlag = 1;  //  Indicate valid sample of zero speed
		}
		
	}

	if (CCF0) // Check CCF0 bit 0 of PCA0CN, Steer sensor
	{
		// Should get multiple interrupts between overflows
		CCF0 = 0; // Clear interrupt flag
			
		if (pSteerSenseIn == 1) {
			// Rising edge, 
			SteerSenseCntVal2 = (PCA0CPH0 << 8) + PCA0CPL0; // Store captured value

			// Latch in values for angle computation, set flag indicating valid sample is ready
			SteerSenseLatch2 = SteerSenseCntVal2;
			SteerSenseLatch1 = SteerSenseCntVal1;
			SteerSenseLatch0 = SteerSenseCntVal0;
			SteerCntOvrFlowLatch = CntOvrFlow_Steer;
			SteerSenseFlag = 1;

			SteerSenseCntVal0 = SteerSenseCntVal2;
			CntOvrFlow_Steer = 0; // reset overflow counter
		}
		else if (pSteerSenseIn == 0) // Falling edge
			SteerSenseCntVal1 = (PCA0CPH0 << 8) + PCA0CPL0;
	}

	if (CCF1) // Check CCF1 bit 1 of PCA0CN, Lean sensor
	{
		// Should get multiple interrupts between overflows
		CCF1 = 0; // Clear interrupt flag
		if (pLeanSenseIn == 1) {
			// Rising edge, 
			LeanSenseCntVal2 = (PCA0CPH1 << 8) + PCA0CPL1; // Store captured value

			// Latch in values for angle computation, set flag indicating valid sample is ready
			LeanSenseLatch2 = LeanSenseCntVal2;
			LeanSenseLatch1 = LeanSenseCntVal1;
			LeanSenseLatch0 = LeanSenseCntVal0;
			LeanCntOvrFlowLatch = CntOvrFlow_Lean;
			LeanSenseFlag = 1;

			LeanSenseCntVal0 = LeanSenseCntVal2;
			CntOvrFlow_Lean = 0; // reset overflow counter
		}
		else if (pLeanSenseIn == 0) // Falling edge
			LeanSenseCntVal1 = (PCA0CPH1 << 8) + PCA0CPL1;
	}

	if (CCF2) // Check CCF2 bit 2 of PCA0CN, Speed sensor
	{
		// Possible to have many overflows between interrupts (at low to zero speed)
		CCF2 = 0; // Clear interrupt flag
		
		SpeedSenseCntVal1 = (PCA0CPH2 << 8) + PCA0CPL2; // Store captured value

		// Latch in values for speed computation, set flag indicating valid sample is ready
		SpeedSenseLatch1 = SpeedSenseCntVal1;
		SpeedSenseLatch0 = SpeedSenseCntVal0;
		SpeedCntOvrFlowLatch = CntOvrFlow_Speed;
		SpeedSenseFlag = 1;

		SpeedSenseCntVal0 = SpeedSenseCntVal1;
		CntOvrFlow_Speed = 0;
	}
	if (CCF3)
	{
		CCF3 = 0;
		PCA0CPL3 = (0xFF & SteerValveCmd); //  Write low byte
		PCA0CPH3 = (0xFF & (SteerValveCmd>>8)); //  Write high byte
		PCA0CPM3 = 0xCA;
	}
	if (CCF4) 
	{
		CCF4 = 0;
		PCA0CPL4 = (0xFF & LeanValveCmd); //  Write low byte
		PCA0CPH4 = (0xFF & (LeanValveCmd>>8)); //  Write high byte
		PCA0CPM4 = 0xCA;
	}
	if (CCF5) 
	{
		CCF5 = 0;
		PCA0CPL5 = (0xFF & BrakeValveCmd); //  Write low byte
		PCA0CPH5 = (0xFF & (BrakeValveCmd>>8)); //  Write high byte
		PCA0CPM5 = 0xCA;
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


	// Code for PCA 16bit PWM, not used presently, but note config problem
	// PCA0CPMn = CA is necessary for 16bit PWM, the config utility is wrong
    PCA0CPM3  = 0xCA; 
    PCA0CPM4  = 0xCA;
    PCA0CPM5  = 0xCA;
    PCA0CPH2  = 0x80;
    PCA0CPH3  = 0x80;
    PCA0CPH4  = 0x80;
    PCA0CPH5  = 0x80;

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

void UART_Init()
{
   SFRPAGE = UART0_PAGE;
   SCON0 = 0x50;                       // 8-bit variable baud rate;
                                       // 9th bit ignored; RX enabled
                                       // clear all flags
   SSTA0 = 0x05;   
  //TI0     = 1;                        // Indicate TX0 ready
  
//   ES0 = 1;  
  // IP |= 0x10;
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
	UART_Init();
    Port_IO_Init();
    Oscillator_Init();
    Interrupts_Init();
}
