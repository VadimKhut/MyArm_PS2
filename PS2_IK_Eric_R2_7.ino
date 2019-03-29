/*************************************************************************************
*   Inverse Kinematics code to control a (modified) 
*   LynxMotion AL5D robot arm using a PS2 controller.
*
*   Original IK code by Oleg Mazurov:
*       www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
*
*   Great intro to IK, with illustrations:
*       github.com/EricGoldsmith/AL5D-BotBoarduino-PS2/blob/master/Robot_Arm_IK.pdf
*
*   Revamped to use BotBoarduino microcontroller:
*       www.lynxmotion.com/c-153-botboarduino.aspx
*   Arduino Servo library:
*       arduino.cc/en/Reference/Servo
*   and PS2X controller library:
*       github.com/madsci1016/Arduino-PS2X
*
*   Eric Goldsmith
*   www.ericgoldsmith.com
*
*   Current Version:
*       https://github.com/EricGoldsmith/AL5D-BotBoarduino-PS2
*   Version history
*       0.1 Initial port of code to use Arduino Server Library
*       0.2 Added PS2 controls
*       0.3 Added constraint logic & 2D kinematics
*       0.4 Added control to modify speed of movement during program run
*       0.5 Write to servos directly in microseconds to improve resolution
*           Should be accurate to ~1/2 a degree
*
*    To Do
*    - Improve arm parking logic to gently move to park position
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* <http://www.gnu.org/licenses/>
*
*
*
**************************************************************************************
*
*   PS2 Controls:
*
*       Start Button        On/Off
*       Right Joystick L/R:  X position (side to side)
*       Right Joystick U/D:  Y position (distance out from base center)
*       R1/R2 Buttons:       Z position (height from surface)
*       Left  Joystick L/R:  Wrist rotate (if installed)
*       Left  Joystick U/D:  Gripper/Wrist Angle
*       L1/L2 Buttons:       Gripper close/open
*       L2+Pad L/R:          Gripper adjusting the torque
*       /\ TRIANGLE:         Gripper fully open
*       Digital Pad U/D:     Arm Speed increase/decrease
*
*        Select mode:
*       Digital Pad U/D:     Play speed increase/decrease
*       Digital Pad L/R:     Playback Program increase/decrease
*
*        Record mode:
*       O-Circle             Record (Hold 5sec)
*       O-Circle             Capture all servo positions 
*       Digital Pad U/D:     Interpalate Time increase/decrease
*       Digital Pad L/R:     Delay Time increase/decrease
*
*        Play mode:
*       []-Square            Play (Hold 4sec)
*       []-Square            Play/Pause
*       Digital Pad U/D:     Play speed increase/decrease       
*
*       X - Cross            Stop (Hold 5sec)
*
**************************************************************************************
* The Arduino connections are as follows:
* 
* D24 - LCD CLK
* D25 - LCD DIO
* D53 - SD card CS
* D51 - SD card MOSI
* D50 - SD card MISO
* D52 - SD card CK
* 
*************************************************************************************/




#include <ServoEx.h>
#include <PS2X_lib.h>
#include <SdFat.h>              //  https://github.com/greiman/SdFat
#include <TM1637Display.h>      //  https://github.com/avishorp/TM1637





int dummy;                      // Defining this dummy variable to work around a bug in the
                                // IDE (1.0.3) pre-processor that messes up #ifdefs
                                // More info: http://code.google.com/p/arduino/issues/detail?id=906
                                //            http://code.google.com/p/arduino/issues/detail?id=987
                                //            http://arduino.cc/forum/index.php/topic,125769.0.html

#define DEBUG                   // Uncomment to turn on debugging output

#define CYL_IK                  // Apply only 2D, or cylindrical, kinematics. The X-axis component is
                                // removed from the equations by fixing it at 0. The arm position is
                                // calculated in the Y and Z planes, and simply rotates around the base.

                                // Arm dimensions (mm). Standard AL5D arm, but with longer arm segments
#define BASE_HGT	90.00       // Base height to X/Y plane 3.1875"
#define HUMERUS		135.00      // Shoulder-to-elbow "bone" 10.375"
#define ULNA		155.00      // Elbow-to-wrist "bone" 12.8125"
#define GRIPPER		35.00       // Gripper length, to middle of grip surface 2.875" (3.375" - 0.5")

// Arduino pin numbers for Servo connections
#define BAS_SERVO_PIN	2       // Base Servo 
#define SHL_SERVO_PIN	3       // Shoulder Servo
#define SHL_SERVO1_PIN	23      // Shoulder Servo1
#define ELB_SERVO_PIN	4       // Elbow Servo
#define WRI_SERVO_PIN	10      // Wrist Servo
#define WRO_SERVO_PIN	11      // Wrist rotate Servo HS-485HB
#define GRI_SERVO_PIN	12      // Gripper Servo

#define BAS_SERVO_ANG_PIN	0
#define SHL_SERVO_ANG_PIN	1
#define ELB_SERVO_ANG_PIN	2
#define WRI_SERVO_ANG_PIN	3
#define WRO_SERVO_ANG_PIN	4
#define GRI_SERVO_ANG_PIN	5
#define FSR_ANG_PIN			6

#define SERVO_ANALOG_MG996_MAX_MV     455
#define SERVO_ANALOG_MG996_MIN_MV     132
#define SERVO_ANALOG_DS3218_MAX_MV    620
#define SERVO_ANALOG_DS3218_MIN_MV    48

// Arduino pin numbers for PS2 controller connections
#define PS2_DAT_PIN		6       // Data           [green] 
#define PS2_CMD_PIN		7       // Command        [blue]
#define PS2_ATT_PIN		8       // Attention      [purple]
#define PS2_CLK_PIN		9       // Clock          [gray]

// Arduino pin number of on-board speaker
#define SPK_PIN			5
#define Buzz_p          5      // OUTPUT* 

volatile uint8_t *pin_port;
volatile uint8_t pin_mask; 

// Define generic range limits for servos, in microseconds (us) and degrees (deg)
// Used to map range of 180 deg to 1800 us (native Servo units).
// Specific per-Servo/joint limits are defined below
//
// MIN     MID    MAX
// 700     1500   2300
// 600     1500   2400
// 500     1500   2500
// 400     1500   2600
//
//#define SERVO_MIN_US	600
//#define SERVO_MID_US	1450
//#define SERVO_MAX_US	2300

#define SERVO_MG996_MAX_US		2300
#define SERVO_MG996_MIN_US		700
#define SERVO_DS3218_MAX_US		2500
#define SERVO_DS3218_MIN_US		500

#define SERVO_MIN_DEG			0.0
#define SERVO_MID_DEG			90.0
#define SERVO_MAX_DEG			180.0

// Set physical limits (in degrees) per Servo/joint.
// Will vary for each Servo/joint, depending on mechanical range of motion.
// The MID setting is the required Servo input needed to achieve a 
// 90 degree joint angle, to allow compensation for horn misalignment

// Base
#define BAS_MIN		0.0			// Fully CCW
#define BAS_MID		90.0
#define BAS_MAX		180.0		// Fully CW
#define BAS_OFF		90.0		// Centre

// Shoulder
#define SHL_MIN		0.0			// Max forward motion
#define SHL_MID		90.0
#define SHL_MAX		180.0		// Max rearward motion
#define SHL_OFF		108.0

// Elbow
#define ELB_MIN		50.0		// Max upward motion
#define ELB_MID		90.0
#define ELB_MAX		165.0		// Max downward motion
#define ELB_OFF		148.45

// Wrist
#define WRI_MIN		0.0			// Max downward motion
#define WRI_MID		90.0
#define WRI_MAX		180.0		// Max upward motion
#define WRI_OFF		83.2

// Wrist rotate
#define WRO_MIN		0.0			// 
#define WRO_MID		90.0
#define WRO_MAX		180.0
#define WRO_OFF		90.0

// Gripper
//#define GRI_MIN		25.0		// Fully open
//#define GRI_MID		90.0
//#define GRI_MAX		165.0		// Fully closed
//#define GRI_OFF		90.0

// Speed adjustment parameters
// Percentages (1.0 = 100%) - applied to all arm movements
#define SPEED_MIN			0.25
#define SPEED_MAX			1.5
#define SPEED_DEFAULT		0.5
#define SPEED_INCREMENT		0.25

// Play Speed adjustment parameters
#define PLAY_SPEED_MIN					-2000
#define PLAY_SPEED_MAX					2000
#define PLAY_SPEED_DEFAULT				0
#define PLAY_SPEED_INCREMENT			50

#define DELAY_TIME_R_MIN				0
#define DELAY_TIME_R_MAX				3000
#define DELAY_TIME_R_DEFAULT			0
#define DELAY_TIME_R_INCREMENT			50

#define INTERPOLATE_TIME_R_MIN			100
#define INTERPOLATE_TIME_R_MAX			3000
#define INTERPOLATE_TIME_R_DEFAULT		300
#define INTERPOLATE_TIME_R_INCREMENT	50

#define INTERPOLATE_P_MIN		70          // the amount of time to complete a pose 
#define TIME_DELAY_P_MIN		70          // delay between transitions 

// Practical navigation limit.
// Enforced on controller input, and used for CLV calculation 
// for base rotation in 2D mode. 
#define Y_MIN 100.0				// (mm)

// PS2 controller characteristics
#define JS_MIDPOINT		128		// Numeric value for joystick midpoint
#define JS_DEADBAND		4		// Ignore movement this close to the center position
#define JS_IK_SCALE		50.0	// Divisor for scaling JS output for IK control
#define JS_SCALE		100.0	// Divisor for scaling JS output for raw Servo control
#define Z_INCREMENT		2.0		// Change in Z axis (mm) per button press
#define GR_INCREMENT	2.0		// Change in Gripper jaw opening (Servo angle) per button press

// IK function return values
#define IK_SUCCESS		0
#define IK_ERROR		1		// Desired position not possible

// Arm parking positions
#define PARK_MIDPOINT	1		// Servos at midpoints
#define PARK_READY		2		// Arm at Ready-To-Run position
#define PARK_OFF		3		// Arm at For OFF position

// Ready-To-Run arm position. See descriptions below
// NOTE: Have the arm near this position before turning on the 
// Servo power to prevent whiplash
#ifdef CYL_IK   // 2D kinematics
 #define READY_BA	BAS_MID		// deg
#else           // 3D kinematics
 #define READY_X		0.0			// (mm)
#endif

#define READY_Y		133.76		// (mm)
#define READY_Z		101.0		// (mm)
#define READY_GRA	-47.33		// (deg) Wrist angle
#define READY_WRO	WRO_MID		// (deg)

#ifdef CYL_IK   // 2D kinematics
 #define OFF_BA		BAS_MID		// deg
#else           // 3D kinematics
 #define OFF_X		0.0			// (mm)
#endif
#define OFF_Y		105.00		// (mm)
#define OFF_Z		113.00		// (mm)
#define OFF_GRA		17.18		// (deg) Wrist angle
#define OFF_WRO		WRO_MID		// (deg)
 
#ifdef CYL_IK   // 2D kinematics
 #define MID_BA		BAS_MID		// deg
#else           // 3D kinematics
 #define MID_X		0.0			// (mm)
#endif
#define MID_Y		155.5		// (mm)
#define MID_Z		171.0		// (mm)
#define MID_GRA		23.42		// (deg) Wrist angle
#define MID_WRO		WRO_MID		// (deg)


// Global variables for arm position, and initial settings
#ifdef CYL_IK   // 2D kinematics
 float BA_pos2D = READY_BA;          // Base angle. Servo (degrees) - 0 is fully CCW
 float BA_pos_s;                     // Base position save (2D)
#else           // 3D kinematics
 float BA_pos3D;                    // distance (mm)
 float X = READY_X;                 // Left/right distance (mm) from base centerline - 0 is straight
 float x_tmp;
#endif

float Y = READY_Y;                 // Distance (mm) out from base center
float Z = READY_Z;                 // Height (mm) from surface (i.e. X/Y plane)
float GA_pos = READY_GRA;          // Gripper angle Servo (degrees), relative to X/Y plane - 0 is horizontal
//float Gr_pos = READY_GR;           // Gripper jaw opening Servo (degrees) - midpoint is halfway open
//float Gr_pos_s;                    // Gripper position save

float y_tmp, z_tmp, ga_tmp;	       // temp. variables
float WRro_pos = READY_WRO;        // Wrist Rotate. Servo degrees - midpoint is horizontal
float WRro_pos_s;                  // Wrist Rotate position save

float Speed = SPEED_DEFAULT;

// Pre-calculations
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;

int Bas_fb, Shl_fb, Shl1_fb, Elb_fb, Wri_fb, Wro_fb, Gri_fb;
float Bas_AngFb, Shl_AngFb, Elb_AngFb, Wri_AngFb, Wro_AngFb, Gri_AngFb;
int ly_trans, lx_trans, ry_trans, rx_trans;

static short	PS2ErrorCnt;
#define	MAXPS2ERRORCNT		5           // How many times through the loop will we go before shutting off robot?

#define	T_PARK_ON			2000
#define	T_PARK_OFF			2000
#define	T_PARK_MID			2000
#define THRESHOLD_REC		10          // threshold for Record to SD card changing the servo position   

// GripperFSR -----
//FSR connected to analog input: A2
//Use a 10k resistor between GND and ADIN. Connect FSR between ADIN and VCC(+5v)
#define GRIPPER_FSR_MAX_TORQUE  750     //Set the upper limit for how hard the grippers can pinch
#define GRIPPER_FSR_MIN_TORQUE  370     //Minimum torque level
#define GRIPPER_CONTACT       	350     //readings lower than this means that the Grippers probably don't touch anything
#define TORQUE_MULTIFACTOR     ((GRIPPER_FSR_MAX_TORQUE - GRIPPER_FSR_MIN_TORQUE) * 100 / 255) // (=176)

// [POT_POS = 2 * ( 1450 - GRIP_US) / 5] - Calculate POT_POS
#define GRIP_CLOSED_US		1495
#define GRIPPER_OPEN_US		890
#define GRIP_READY_US		1370
#define GRIP_MID_US			1190
#define GRIP_OFF_US			1450
#define POT_READY_POS		50
#define POT_MID_POS			122
#define POT_OFF_POS			0

// Audible feedback sounds
// #define TONE_READY		1000	// Hz
// #define TONE_IK_ERROR	200		// Hz
// #define TONE_DURATION	100		// ms

float shl_pos, elb_pos, wri_pos;
int BA_pos_us, shl_pos_us, shl1_pos_us, elb_pos_us, wri_pos_us, WRro_pos_us;;
int old_BA_pos_us, old_shl_pos_us, old_elb_pos_us, old_wri_pos_us, old_WRro_pos_us;

bool fServosAttached = false;      // remember we are not attached. Could simply ask one of our servos...
bool fArmOn = false;
bool fArmOn_prev = false;
bool arm_moveIK = false;           // input occurred that can move the arm IK (BA_pos3D, shl_pos, elb_pos, wri_pos)
bool arm_moveNoIK = false;         // input occurred that can move the arm (BA_pos2D, WRro_pos, Gr_pos)
bool fButtonPlay1 = false;
bool fButtonPlay2 = false;
bool fButtonRec = false;
bool fButtonStop = false;
bool fRecStart_10s = false;
bool fRecStop_10s = false;
bool fPSB_SQUARE_4s = false;
bool fCapture_pos = false;
bool fButtonPause = false;

unsigned long ulTimePSB_CIRCLE;
unsigned long ulTimePSB_SQUARE;
unsigned long ulTimePSB_CROSS;
//unsigned long cTimePrev;
int PlaySpeed = PLAY_SPEED_DEFAULT;

int InterpTime_R =  INTERPOLATE_TIME_R_DEFAULT;
int DelayTime_R = DELAY_TIME_R_DEFAULT; 

byte MultIndex = 0;
byte MultIncr = 1;

float mServoAngleOffset[6];
const byte SERVO_ANG_PIN[6] = {BAS_SERVO_ANG_PIN, SHL_SERVO_ANG_PIN, ELB_SERVO_ANG_PIN, 
	                             WRI_SERVO_ANG_PIN, GRI_SERVO_ANG_PIN, WRO_SERVO_ANG_PIN};

const word SERVO_MAX_US[2] = {SERVO_MG996_MAX_US, SERVO_DS3218_MAX_US};
const word SERVO_MIN_US[2] = {SERVO_MG996_MIN_US, SERVO_DS3218_MIN_US};

const word SERVO_ANALOG_MIN_MV[2] = {SERVO_ANALOG_MG996_MIN_MV, SERVO_ANALOG_DS3218_MIN_MV};
const word SERVO_ANALOG_MAX_MV[2] = {SERVO_ANALOG_MG996_MAX_MV, SERVO_ANALOG_DS3218_MAX_MV};
static const byte INCREMENT_MUL[] PROGMEM = {1,2,4};


word	GripperFSRInput;
word	Gr_pos_us = GRIP_CLOSED_US;                     // PWM=1495
word	old_Gr_pos_us;
bool	GripperFSR_Activated = false;
byte	POTSavePos;                                     // save
byte	POTCtrlPos = POT_READY_POS;                     // Controlled indirectly by the L1 and L2 button, inc/dec variable
byte	POTCtrlPos_s;                                   // save
word	TorqueLevel;
byte	TorqueIndex = 2;
static const byte TorqueSelect[] PROGMEM = {0,64,128,192,255};


void writeCommand(void);
int deg_to_us(float value, int model);
void InitPs2(void);
void TurnArmOff(void);
void TurnArmOn(void);
void startRecord(void);
void startPlayback(int in_playbackProgram);
void startSelect(void);
void setDisplay(char inMode);
void TM1637Display_Off(void);
void setName(int i);
int doArmIK(float x, float y, float z, float grip_angle_d);
void servo_park(int park_type);
float map_float(float x, float in_min, float in_max, float out_min, float out_max);
void AttachServos(void);
void FreeServos(void);
void InitPs2(void);
void Control_PS2_Input(void);
void DegToUsServoIK(void);
void GroupServoUpdate(unsigned int DeltaTime, int BA_pos_us, int shl_pos_us, int shl1_pos_us, int elb_pos_us, int wri_pos_us, int WRro_pos_us, int Gr_pos_us);
void MSound(byte cNotes, ...);
void SoundNoTimer(unsigned long duration,  unsigned int frequency);
void Control_PS2_Input_S(void);
void delay_ms( unsigned long delayTime_ms);
void SaveOldPos_us(void);
void GetFeetbackAllServo(void);
void Display_d(int dig_i);
void ResetAllFlags(void);
float readFbServoAngle(byte servoNum, int model, boolean withOffset=false);
unsigned int getServoAnalogData(byte servoNum);
int getAnalogPinValue(unsigned int pin);
static void _sort(unsigned int array[], unsigned int len);
double analogToAngle(byte servoNum, int inputAnalog);
//void readLinearOffset(byte servoNum, double& interceptVal, double& slopeVal);
void GripperControl(void);
void WristRotControl(void);
#ifdef CYL_IK   // 2D kinematics
 void BaseControl(void);
#endif






// PS2 Controller object
PS2X    Ps2x;


// ServoEx objects 
ServoEx   Bas_Servo;
ServoEx   Shl_Servo;
ServoEx   Shl_Servo1;
ServoEx   Elb_Servo;
ServoEx   Wri_Servo;
ServoEx   Wro_Servo;
ServoEx   Gri_Servo;


// LED module connection pins (Digital Pins)
#define CLK 24
#define DIO 25


TM1637Display display(CLK, DIO);

 
int k;
uint8_t data[] = { 0xff, 0xff, 0xff, 0xff };


// SD shield
const int chipSelect = 53;          // Ard_MEGA: CS =53


SdFat sd;                           // Ard_MINI: CS/MOSI/MISO/CK = 10/11/12/13 [ Ard_MEGA: CS/MOSI/MISO/CK = 53/51/50/52]
SdFile myFile;
char name[] = "ARM000.CSV";         // Will be incremented to create a new file each run.

// R = Recording
// S = Select playback program
// P = Playback
// N = None
char mode = 'N'; 
//unsigned long recStart;
int playbackProgram = -1;
int maxProgram = 0;


 
 
 
 
void TurnArmOff(void){
    
  #ifdef DEBUG
	Serial.println(F("Arm OFF!"));
  #endif

	servo_park(PARK_OFF);

#ifdef DEBUG
	Serial.print(F(" PARK_ms_st = "));
	Serial.println(millis());
#endif

	delay_ms(T_PARK_OFF + 1000);

#ifdef DEBUG
	Serial.print(F(" PARK_ms_end = "));
	Serial.println(millis());

#endif
	FreeServos();

  	if (mode != 'N') { 

		mode = 'N'; 
		TM1637Display_Off();
		ResetAllFlags();
	}

	//// Sound tone to indicate it's safe to turn on Servo power
	//tone(SPK_PIN, TONE_READY, TONE_DURATION);
	//delay(TONE_DURATION * 2);
	//tone(SPK_PIN, TONE_READY, TONE_DURATION);

	MSound(3, 100, 2500, 80, 2250, 60, 2000);
}



void TurnArmOn(void){

	int Shl1_fbk;

  #ifdef DEBUG
	Serial.println(F("Arm ON!"));
  #endif

	Bas_AngFb = readFbServoAngle(BAS_SERVO_ANG_PIN, 0);
	Shl_AngFb = readFbServoAngle(SHL_SERVO_ANG_PIN, 1);
	Elb_AngFb = readFbServoAngle(ELB_SERVO_ANG_PIN, 1);
	Wri_AngFb = readFbServoAngle(WRI_SERVO_ANG_PIN, 0);
	Wro_AngFb = readFbServoAngle(WRO_SERVO_ANG_PIN, 0);
	Gri_AngFb = readFbServoAngle(GRI_SERVO_ANG_PIN, 0);

	Shl1_fbk = int(182-Shl_AngFb);


	Serial.print("  Bas_AngFb: ");
	Serial.println(Bas_AngFb);

	Serial.print("  Shl_AngFb: ");
	Serial.print(Shl_AngFb);

	Serial.print("  Shl1_Fbk: ");
	Serial.println(Shl1_fbk);

	Serial.print("  Elb_AngFb: ");
	Serial.println(Elb_AngFb);

	Serial.print("  Wri_AngFb: ");
	Serial.println(Wri_AngFb);

	Serial.print("  WrRO_AngFb: ");
	Serial.println(Wro_AngFb);

	Serial.print("  Gri_AngFb: ");
	Serial.println(Gri_AngFb);

	//delay_ms(2000);

	// Position the servos
	Bas_Servo.write(int(Bas_AngFb));
	Shl_Servo.write(int(Shl_AngFb));
	Shl_Servo1.write(Shl1_fbk);
	Elb_Servo.write(int(Elb_AngFb));
	Wri_Servo.write(int(Wri_AngFb));
	Wro_Servo.write(int(Wro_AngFb));
	Gri_Servo.write(int(Gri_AngFb));

	Serial.println("Servo.write");


	//delay_ms(2000);

	AttachServos();

	GetFeetbackAllServo();

	//delay_ms(3000);

	
	// NOTE: Ensure arm is close to the desired park position before turning on Servo power!
	servo_park(PARK_READY);

#ifdef DEBUG
	Serial.print(F(" PARK_ms_st = "));
	Serial.println(millis());
#endif

	delay_ms(T_PARK_ON + 1000);

#ifdef DEBUG
	Serial.print(F(" PARK_ms_end = "));
	Serial.println(millis());
#endif

	GetFeetbackAllServo();

	//// Sound tone to indicate it's safe to turn on Servo power
	//tone(SPK_PIN, TONE_READY, TONE_DURATION);
	//delay(TONE_DURATION * 2);
	//tone(SPK_PIN, TONE_READY, TONE_DURATION);

	MSound(3, 60, 2000, 80, 2250, 100, 2500); 

}



void TM1637Display_Off(void){

	#ifdef DEBUG
	 // Turn off display
	 Serial.println(F("Display Off"));
	#endif

	for(int k = 0; k < 4; k++) 
		data[k] = 0;

	display.setSegments(data);
}


 
void InitPs2(void){

	// Setup PS2 controller. Loop until ready.
	byte ps2_stat;

	do {
		ps2_stat = Ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_ATT_PIN, PS2_DAT_PIN);
    //#ifdef DEBUG
		if (ps2_stat == 1)
			Serial.println(F("No controller found. Re-trying ..."));
    //#endif
	} while (ps2_stat == 1);

    
#ifdef DEBUG
	switch (ps2_stat) {
		case 0:
			Serial.println(F("Found Controller, configured successfully."));
			break;
		case 2:
			Serial.println(F("Controller found but not accepting commands."));
			break;
		case 3:
			Serial.println(F("Controller refusing to enter 'Pressures' mode, may not support it. "));      
			break;
	}
#endif
}



void Control_PS2_Input(void){
    
	Ps2x.read_gamepad();                            //read controller

	if ((Ps2x.Analog(1) & 0xf0) == 0x70) {

		PS2ErrorCnt = 0;                            // clear error counter

		if (Ps2x.ButtonPressed(PSB_START)) {		// Start Button Test 

			if (fArmOn) {			                // Turn off
				fArmOn = false;
			} 
			else {					                // Turn on
				fArmOn = true;
			}
		}

		if (fArmOn) {

			// Read the left and right joysticks and translate the 
			// normal range of values (0-255) to zero-centered values (-128 - 128)
			ly_trans = JS_MIDPOINT - Ps2x.Analog(PSS_LY);            // Gripper/Wrist Angle
			lx_trans = Ps2x.Analog(PSS_LX) - JS_MIDPOINT;            // Wrist rotate (if installed)
			ry_trans = JS_MIDPOINT - Ps2x.Analog(PSS_RY);            // Y position (distance out from base center)
			rx_trans = Ps2x.Analog(PSS_RX) - JS_MIDPOINT;            // X position or Base Position (side to side)



		#ifdef CYL_IK   // 2D kinematics
			// Base Position (in degrees)
			// Restrict to MIN/MAX range of Servo
			if (abs(rx_trans) > JS_DEADBAND) {            

				// Multiplying by the ratio (Y_MIN/Y) is to ensure constant linear velocity
				// of the gripper as its distance from the base increases
				BA_pos2D += ((float)rx_trans / JS_SCALE * Speed * (Y_MIN/Y));   // Base  Position  (side to side)   
				BA_pos2D = constrain(BA_pos2D, BAS_MIN, BAS_MAX);

				if (BA_pos2D != BA_pos_s) {

					arm_moveNoIK = true;

				}

				// Provide audible feedback of reaching limit
				if (BA_pos2D == BAS_MIN || BA_pos2D == BAS_MAX) {

					// Provide audible feedback of reaching limit
					//tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
					MSound (2, 40, 2500, 40, 2500);
				}
			}
		#else    // 3D kinematics
			// X Position (in mm)
			// Can be positive or negative. Servo range checking in IK code
			if (abs(rx_trans) > JS_DEADBAND) {                                // X Position  (side to side)

				x_tmp += ((float)rx_trans / JS_IK_SCALE * Speed);
				arm_moveIK = true;
			}
		#endif



			// Y Position (in mm)
			// Must be > Y_MIN. Servo range checking in IK code
			if (abs(ry_trans) > JS_DEADBAND) {                                // Y position (distance out from base center)

				y_tmp += ((float)ry_trans / JS_IK_SCALE * Speed);
				y_tmp = max(y_tmp, Y_MIN);
				arm_moveIK = true;

				if (y_tmp == Y_MIN) {

					// Provide audible feedback of reaching limit
					//tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
					MSound (2, 40, 2500, 40, 2500);
				}
			}



			// Z Position (in mm)
			// Must be positive Servo range checking in IK code
			if (Ps2x.Button(PSB_R1) || Ps2x.Button(PSB_R2)) {                 // PSB_R1 OR PSB_R2 Test

				if (Ps2x.Button(PSB_R1)) {
					z_tmp += Z_INCREMENT * Speed;   // up
				} else {
					z_tmp -= Z_INCREMENT * Speed;   // down
				}
				z_tmp = max(z_tmp, 0);
				arm_moveIK = true;
			}



			// Gripper/Wrist angle (in degrees) relative to horizontal
			// Can be positive or negative Servo range checking in IK code
			if (abs(ly_trans) > JS_DEADBAND) {                                // Gripper/Wrist Angle

				ga_tmp -= ((float)ly_trans / JS_SCALE * Speed);
				arm_moveIK = true;
			}



			// Wrist rotate (in degrees)
			// Restrict to MIN/MAX range of Servo
			if (abs(lx_trans) > JS_DEADBAND) {                                // Wrist rotate

				WRro_pos += ((float)lx_trans / JS_SCALE * Speed);
				WRro_pos = constrain(WRro_pos, WRO_MIN, WRO_MAX);

				if (WRro_pos != WRro_pos_s) {

					arm_moveNoIK = true;
				}

				if (WRro_pos == WRO_MIN || WRro_pos == WRO_MAX) {

					// Provide audible feedback of reaching limit
					//tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
					MSound (2, 40, 2500, 40, 2500);
				}
			}



			// Gripper jaw position (in degrees - determines width of jaw opening)
			//-Gripper Open---------------------------
			if (Ps2x.Button(PSB_L1)) {                                        // L1 Button Test

				POTCtrlPos = min((POTCtrlPos+5), 255);

				if (POTCtrlPos == 255) {                                      // Open

					// Provide audible feedback of reaching limit
					//tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
					MSound (2, 40, 2500, 40, 2500);
				}
			}

			//-Gripper Close---------------------------
			if (Ps2x.Button(PSB_L2)) {                                        // L2 Button Test

				if (!GripperFSR_Activated) {                                  // Don't decrease this value if FSR is activated

					POTCtrlPos = max((POTCtrlPos-5), 0);

					if (POTCtrlPos == 0) {                                    // Close

						// Provide audible feedback of reaching limit
						//tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
						MSound (2, 40, 2500, 40, 2500);
					}
				}
				//adjusting the torque using the left/right D-Pad:
				//-Increase torque-----------
				if (Ps2x.ButtonPressed(PSB_PAD_RIGHT)) {                      // (L2 + D-Right), D-Right - Button Test

					if (TorqueIndex < 4) {
						TorqueIndex ++;
						if (TorqueIndex == 4) {

							MSound(1, 40, 3000);
						}  
						else {

							MSound(1, 50, 6000);
						}
						
						TorqueLevel = (GRIPPER_FSR_MIN_TORQUE + (pgm_read_byte(&TorqueSelect[TorqueIndex]) * TORQUE_MULTIFACTOR) / 100);
					}
				}

				//-Decrease torque----------
				if (Ps2x.ButtonPressed(PSB_PAD_LEFT)) {                       // (L2 + D-Left), D-Left - Button Test

					if (TorqueIndex > 0) {
						TorqueIndex --;
						if (!TorqueIndex) {

							MSound(1, 40, 3000);
						}  
						else {

							MSound(1, 50, 6000);
						}

						TorqueLevel = (GRIPPER_FSR_MIN_TORQUE + (pgm_read_byte(&TorqueSelect[TorqueIndex]) * TORQUE_MULTIFACTOR) / 100);
					}
				}  
			}    



			// Fully open gripper--------------------------
			if (Ps2x.ButtonPressed(PSB_TRIANGLE)) {                           // PSB_TRIANGLE Test

				if (POTCtrlPos != POTCtrlPos_s) {

					POTCtrlPos = 255;

				#ifdef DEBUG
					Serial.println(F("Gripper Open"));
				#endif
				}
				else{

					// Provide audible feedback of reaching limit
					//tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
					MSound (2, 40, 2500, 40, 2500);
				}
			}

			// Increment Multiplier - control--------------------------
			if (Ps2x.ButtonPressed(PSB_SELECT)) {                          // PSB_SELECT Test 
			//-Increase torque-----------
						
				MultIndex %= 3;
				MultIncr = pgm_read_byte(&INCREMENT_MUL[MultIndex]);
				MultIndex ++;

			#ifdef DEBUG
				Serial.print(F("MultIncr = "));
				Serial.print(MultIncr);
				Serial.print(F("  MultInd = "));
				Serial.println(MultIndex);

			#endif

				Display_d(MultIncr);

				if (MultIncr == 3) {

					MSound(1, 40, 3000);
				}  
				else {

					MSound(1, 50, 6000);
				}
			}

			// Speed ARM increase/decrease--------------------------
			if(mode != 'S' && mode != 'R') {

				if (Ps2x.ButtonPressed(PSB_PAD_UP) || Ps2x.ButtonPressed(PSB_PAD_DOWN)) {   // PSB_PAD_UP OR PSB_PAD_DOWN Test

					if (Ps2x.ButtonPressed(PSB_PAD_UP)) {                     // PSB_PAD_UP Test

						Speed += SPEED_INCREMENT;                             // increase speed
					}

					if (Ps2x.ButtonPressed(PSB_PAD_DOWN)) {                   // PSB_PAD_DOWN Test

							Speed -= SPEED_INCREMENT;                         // decrease speed
					}

					// Constrain to limits
					Speed = constrain(Speed, SPEED_MIN, SPEED_MAX);

				#ifdef DEBUG	
					Serial.print(F(" Speed = "));
					Serial.println(Speed);
				#endif

					//tone(SPK_PIN, (TONE_READY * Speed), TONE_DURATION);
					MSound(1, 50, 6000);

					// Provide audible feedback of reaching limit
					if (Speed == SPEED_MIN || Speed == SPEED_MAX) {

						//tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
						MSound (2, 40, 2500, 40, 2500);
					}
				}
			} // (mode != 'S') || (mode != 'R')

			else if (mode == 'S') {

				// PlaybackProgram increase/decrease--------------------------
				if (Ps2x.ButtonPressed(PSB_PAD_RIGHT)) {                      // PSAB_PAD_RIGHT Test

						if (playbackProgram < maxProgram) {

							playbackProgram++;
							setName(playbackProgram);
							setDisplay('S');

							//tone(SPK_PIN, TONE_READY, TONE_DURATION);
							MSound(1, 50, 6000);

						#ifdef DEBUG	
							Serial.print(F(" PlProgr = "));
							Serial.println(playbackProgram);
 						#endif
						}
						else {

							//tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
							MSound (2, 40, 2500, 40, 2500);
						}
				}

				else if(Ps2x.ButtonPressed(PSB_PAD_LEFT)) {                   // PSAB_PAD_LEFT Test

					if (playbackProgram >= 1) {
						playbackProgram--;
						setName(playbackProgram);
						setDisplay('S');

						//tone(SPK_PIN, TONE_READY, TONE_DURATION);
						MSound(1, 50, 6000);

					#ifdef DEBUG	
						Serial.print(F(" PlProgr = "));
						Serial.println(playbackProgram);
					#endif

					}
					else {
						//tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
						MSound (2, 40, 2500, 40, 2500);
					}
				}


				// Playback - Speed control--------------------------
				if (Ps2x.ButtonPressed(PSB_PAD_UP) || Ps2x.ButtonPressed(PSB_PAD_DOWN)) {   // PSB_PAD_UP OR PSB_PAD_DOWN Test PSB_SELECT

					//Increase Play speed with -50mS   -->
					if(Ps2x.ButtonPressed(PSB_PAD_UP)) {                      // PSB_PAD_UP Test

						PlaySpeed -= PLAY_SPEED_INCREMENT * MultIncr;         // Default 50 * 1
					}
					
					//-Decrease Play speed +50mS   <--
					if (Ps2x.ButtonPressed(PSB_PAD_DOWN)) {                   // PSB_PAD_DOWN Test

						PlaySpeed += PLAY_SPEED_INCREMENT * MultIncr;         // Default 50 * 1
					}

					// Constrain to limits
					PlaySpeed = constrain(PlaySpeed, PLAY_SPEED_MIN, PLAY_SPEED_MAX);

					//tone(SPK_PIN, TONE_READY, TONE_DURATION);
					MSound(1, 50, 6000);

				#ifdef DEBUG	
					Serial.print(F(" PlaySpeed = "));
					Serial.println(PlaySpeed);
				#endif

					Display_d(PlaySpeed);

					// Provide audible feedback of reaching limit
					if (PlaySpeed == PLAY_SPEED_MIN || PlaySpeed == PLAY_SPEED_MAX) {

						//tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
						MSound (2, 40, 2500, 40, 2500);
					}
				}
			} // mode == 'S'
	

			// Delay and Interpolate Time Record control---------------
			else if (mode == 'R') {

				if (Ps2x.ButtonPressed(PSB_PAD_UP) || Ps2x.ButtonPressed(PSB_PAD_DOWN)) {   // PSB_PAD_UP OR PSB_PAD_DOWN Test

					//Increase Interpalate Time with -100mS   -->
					if(Ps2x.ButtonPressed(PSB_PAD_DOWN)) {                                  // PSB_PAD_UP Test

						InterpTime_R -= INTERPOLATE_TIME_R_INCREMENT * MultIncr;            // Default 50 * 1;
					}
					
					//-Decrease Interpalate Time +100mS   <--
					if (Ps2x.ButtonPressed(PSB_PAD_UP)) {                                   // PSB_PAD_DOWN Test

						InterpTime_R += INTERPOLATE_TIME_R_INCREMENT * MultIncr;            // Default 50 * 1;
					}

					// Constrain to limits
					InterpTime_R = constrain(InterpTime_R, INTERPOLATE_TIME_R_MIN, INTERPOLATE_TIME_R_MAX);

					//tone(SPK_PIN, TONE_READY, TONE_DURATION);
					MSound(1, 50, 6000);

				#ifdef DEBUG	
					Serial.print(F(" InterpTime_R = "));
					Serial.println(InterpTime_R);
				#endif

					Display_d(InterpTime_R);

					// Provide audible feedback of reaching limit
					if (InterpTime_R == INTERPOLATE_TIME_R_MIN || InterpTime_R == INTERPOLATE_TIME_R_MAX) {

						//tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
						MSound (2, 40, 2500, 40, 2500);
					}
				}

				if (Ps2x.ButtonPressed(PSB_PAD_LEFT) || Ps2x.ButtonPressed(PSB_PAD_RIGHT)) {   // PSB_PAD_UP OR PSB_PAD_DOWN Test

					//Increase Delay Time with -100mS   -->
					if(Ps2x.ButtonPressed(PSB_PAD_LEFT)) {                                     // PSB_PAD_UP Test

						DelayTime_R -= DELAY_TIME_R_INCREMENT * MultIncr;                      // Default 50 * 1;
					}
					
					//-Decrease Delay Time +100mS   <--
					if (Ps2x.ButtonPressed(PSB_PAD_RIGHT)) {                                   // PSB_PAD_DOWN Test

						DelayTime_R += DELAY_TIME_R_INCREMENT * MultIncr;                      // Default 50 * 1;
					}

					// Constrain to limits
					DelayTime_R = constrain(DelayTime_R, DELAY_TIME_R_MIN, DELAY_TIME_R_MAX);

					//tone(SPK_PIN, TONE_READY, TONE_DURATION);
					MSound(1, 50, 6000);

				#ifdef DEBUG	
					Serial.print(F(" DelayTime_R = "));
					Serial.println(DelayTime_R);
				#endif

					Display_d(DelayTime_R);

					// Provide audible feedback of reaching limit
					if (DelayTime_R == DELAY_TIME_R_MIN || DelayTime_R == DELAY_TIME_R_MAX) {

						//tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
						MSound (2, 40, 2500, 40, 2500);
					}
				}
			} // mode == 'R'



			// Record--------------------------
			if (!Ps2x.Button(PSB_CIRCLE)) {		                              // PSB_CIRCLE not pressed

				ulTimePSB_CIRCLE = millis();                                  // reset ulTimePSB_CIRCLE

			}  
			else if ((Ps2x.Button(PSB_CIRCLE)) && (mode != 'R') && !fRecStart_10s) {  // PSB_CIRCLE - (O) hold pressed 5sec

				if ((millis() - ulTimePSB_CIRCLE) > 5000) {

				#ifdef DEBUG
					Serial.println(F("Record button press 5s"));
				#endif

					//tone(SPK_PIN, TONE_READY , TONE_DURATION);
					MSound(1, 50, 2000); 

					fRecStart_10s = true;
				}  
			}

			if ((Ps2x.ButtonPressed(PSB_CIRCLE)) && (mode == 'R')) {

				//MSound(1, 50, 6000);
				fCapture_pos = true;
			}



			if (!Ps2x.Button(PSB_CROSS)) {		                              // PSB_CROSS not pressed

				ulTimePSB_CROSS = millis();                                   // reset ulTimePSB_CROSS

			}  
			else if (Ps2x.Button(PSB_CROSS) && !fRecStop_10s) {	              // PSB_CROSS - (X) hold pressed 5sec

				if ((millis() - ulTimePSB_CROSS) > 5000) {

				#ifdef DEBUG
					Serial.println(F("Stop button press 5s"));
				#endif

					//tone(SPK_PIN, TONE_READY , TONE_DURATION);
					MSound(1, 50, 2000); 

					fRecStop_10s = true;
				}
			}  


			if (fRecStart_10s){

				// First check to see if we need to record or stop.
				if (Ps2x.ButtonPressed(PSB_CIRCLE)) {		                  // PSB_CIRCLE Button Test

					MSound(1, 50, 6000);

					fButtonRec = true; 
					fRecStart_10s = false;
				}
			} // fRecStart_10s



			if (fRecStop_10s){
				// First check to see if we need to record or stop.
				if (Ps2x.ButtonPressed(PSB_CROSS)) {		                  // PSB_CROSS Button Test

					MSound(1, 50, 6000);

					if (mode != 'N')
							mode = 'N'; 
						
					TM1637Display_Off();
					ResetAllFlags();
				}
			} // fRecStop_10s



			// Play
			if (!Ps2x.Button(PSB_SQUARE)) {		                              // PSB_SQUARE not pressed

				ulTimePSB_SQUARE = millis();                                  // reset ulTimePSB_SQUARE

			}  
			else if (Ps2x.Button(PSB_SQUARE) && !fPSB_SQUARE_4s) {	                              // PSB_SQUARE hold pressed 4sec

				if ((millis() - ulTimePSB_SQUARE) > 4000) {

				#ifdef DEBUG
					Serial.println(F("Play button press 4s"));
				#endif

					//tone(SPK_PIN, TONE_READY , TONE_DURATION);
					MSound(1, 50, 2000); 
					fPSB_SQUARE_4s = true;
				}  
			}  


			// Now check if we need to playback...
			if(fPSB_SQUARE_4s){
				
				if (Ps2x.ButtonPressed(PSB_SQUARE) && mode != 'S') {		  // PSB_SQUARE Button Test

					//tone(SPK_PIN, TONE_READY , TONE_DURATION);
					MSound(1, 50, 6000);

					fButtonPlay1 = true;
				}
				if (Ps2x.ButtonPressed(PSB_SQUARE) && mode == 'S') {		  // PSB_SQUARE Button Test

					//tone(SPK_PIN, TONE_READY , TONE_DURATION);
					MSound(1, 50, 6000);

					fButtonPlay2 = true; 
					fPSB_SQUARE_4s = false;
					
				}
			} // fPSB_SQUARE_4s
		} // fArmOn


	} // end, if ((Ps2x.Analog(1) & 0xf0) == 0x70), read PS2 controller 
	else {
		if (PS2ErrorCnt < MAXPS2ERRORCNT)
			
			PS2ErrorCnt++;						// Increment the error count and if to many errors, turn OFF the Arm.

		else if (fArmOn){

			fArmOn == false;	
		}  

		Ps2x.reconfig_gamepad();
	}

 }





void writeCommand(void) {

	int DelayTime, InterpTime;

	if (mode == 'R') {

		//myFile.println("InterpTime,Delay_Time,BA_pos_us,shl_pos_us,shl1_pos_us,elb_pos_us,wri_pos_us,WRro_pos_us,Gr_pos_us");
		if (myFile.open(name, O_RDWR | O_CREAT | O_AT_END)) {

			InterpTime = InterpTime_R;
			DelayTime = DelayTime_R;

			myFile.print(InterpTime);    
			myFile.print(",");
			myFile.print(DelayTime);    
			myFile.print(",");
			myFile.print(BA_pos_us);
			myFile.print(",");
			myFile.print(shl_pos_us);
			myFile.print(",");
			myFile.print(shl1_pos_us);
			myFile.print(",");
			myFile.print(elb_pos_us);
			myFile.print(",");
			myFile.print(wri_pos_us);
			myFile.print(",");
			myFile.print(WRro_pos_us);
			myFile.print(",");
			myFile.println(Gr_pos_us);
			myFile.close();

		#ifdef DEBUG
			Serial.print(F("Write to SD: "));
			Serial.print(InterpTime);
			Serial.print(F(","));
			Serial.print(DelayTime);
			Serial.print(F(","));
			Serial.print(BA_pos_us);
			Serial.print(F(","));
			Serial.print(shl_pos_us);
			Serial.print(",");
			Serial.print(shl1_pos_us);
			Serial.print(",");
			Serial.print(elb_pos_us);
			Serial.print(",");
			Serial.print(wri_pos_us);
			Serial.print(",");
			Serial.print(WRro_pos_us);
			Serial.print(",");
			Serial.println(Gr_pos_us);
		#endif

			SaveOldPos_us();                       // old_BA_pos_us = BA_pos_us ....

		}
	}
}



void startRecord(void) {


	// Look at the SD card and figure out the new file name to use.
	// Then initialize the file.
	if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
		Serial.println (F("Cannot access local SD card"));
	}
	else {
		// Set the output file name
		for (uint8_t i = 0; i < 1000; i++) {
			setName(i);
			if (sd.exists(name)) continue;
			playbackProgram = i;
			break;
		}

	#ifdef DEBUG
		Serial.print(F("SD card output file is: "));
		Serial.println(name);
	#endif

	}
	if (!myFile.open(name, O_RDWR | O_CREAT | O_AT_END)) {

		Serial.println (F("Opening of local SD card FAILED"));
	}
	else {

	 #ifdef DEBUG
		Serial.println(F("Start Record!"));
	 #endif	

		mode = 'R';

		//cTimePrev = millis();

	 //#ifdef DEBUG
		//Serial.print(F("RecStart = "));
		//Serial.println(cTimePrev);
	 //#endif

		delay_ms(1000);
		servo_park(PARK_OFF);
		delay_ms(T_PARK_OFF + 1000);

		// myFile.println("InterpTime=0,DelayTime=0,BA_pos_us,shl_pos_us,shl1_pos_us,elb_pos_us,wri_pos_us,WRro_pos_us,Gr_pos_us");   
		// Need to first write the initial (current) position
		// of the arm so it can be initialized upon playback.

        myFile.print("0,");
		myFile.print("0,"); myFile.print(BA_pos_us);
		myFile.print(","); myFile.print(shl_pos_us);
		myFile.print(","); myFile.print(shl1_pos_us);
		myFile.print(","); myFile.print(elb_pos_us);
		myFile.print(","); myFile.print(wri_pos_us);
		myFile.print(","); myFile.print(WRro_pos_us);
		myFile.print(","); myFile.println(Gr_pos_us);
		myFile.close();

	 #ifdef DEBUG
		Serial.print(F("SD: "));
		Serial.print("0,");
		Serial.print("0,"); Serial.print(BA_pos_us);
		Serial.print(","); Serial.print(shl_pos_us);
		Serial.print(","); Serial.print(shl1_pos_us);
		Serial.print(","); Serial.print(elb_pos_us);
		Serial.print(","); Serial.print(wri_pos_us);
		Serial.print(","); Serial.print(WRro_pos_us);
		Serial.print(","); Serial.println(Gr_pos_us);
	 #endif

		setDisplay('R');

		SaveOldPos_us();                // old_BA_pos_us = BA_pos_us ....

	}
}



void startSelect() {

	if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {

		Serial.println (F("Cannot access local SD card"));
	}

	mode = 'S';

	// Find maximum program number
	for (uint8_t i = 0; i < 1000; i++) {
		setName(i);
		if (sd.exists(name))
			continue;
		maxProgram = i-1;
		break;
	}

	// Get program to playback
	if (playbackProgram == -1 ) {

		// Find latest recorded program and use that.
		for (uint8_t i = 0; i < 1000; i++) {
			setName(i);
			if (sd.exists(name)) 
				continue;
			playbackProgram = i-1;
			break;
		}
	}

	setName(playbackProgram);
	setDisplay('S');

	delay(200);
}



void setName(int i){

	name[3] = i/100 + '0';
	name[4] = (i%100)/10 + '0';
	name[5] = i%10 + '0';
}



void setDisplay(char inMode) {

	display.setBrightness(7); // Turn on

	switch (inMode) {

		case 'R':
			data[0] = B01010000;
			break;
		case 'P':
			data[0] = B01110011;
			break;
		case 'S':
			//data[0] = B01101101;
			// Changed this from S to C since S looks just like % on LED.
			data[0] = B00111001;
			break;
	}

	data[1] = display.encodeDigit((int)name[3]);
	data[2] = display.encodeDigit((int)name[4]);
	data[3] = display.encodeDigit((int)name[5]);
	display.setSegments(data);

#ifdef DEBUG
	Serial.print(F("Display:  "));
	Serial.print(inMode);
	Serial.print(name[3]);
	Serial.print(name[4]);
	Serial.println(name[5]);
	Serial.println();

	Serial.print(F("Display BIN:  "));
	Serial.println(data[0], BIN);
	Serial.println(data[1], BIN);
	Serial.println(data[2], BIN);
	Serial.println(data[3], BIN);
	Serial.println();
#endif
}




void Display_d(int dig_i) {

	unsigned int dot;


	display.setBrightness(7);          // Turn on

	if(dig_i < 0) {                    // if the number is negative, display dots

		dot = 64;
		dig_i = abs (dig_i);
		display.showNumberDecEx(dig_i,dot);
	}
	else {

		dot = 0;
		display.showNumberDecEx(dig_i,dot);
	}

#ifdef DEBUG
	Serial.print(F("Display_d = "));
	Serial.print(dig_i);
	Serial.print(F(", Dot = "));
	Serial.println(dot, BIN);
#endif

}




void startPlayback(int in_playbackProgram) {

	unsigned int InterpTime;
	unsigned int DelayTime;
	int wServoMoveTime, TempServoMoveTime;
	int wTimeDelay;
	int BA, shl, shl1, elb, wri, WRro, Gr;
	char c1, c2, c3, c4, c5, c6, c7, c8;   // commas
	unsigned long fileLine = 0;
	
	setName(in_playbackProgram);
	setDisplay('P');

	mode = 'P';

	delay(50);
	
#ifdef DEBUG
	Serial.print(F("Playing file: "));
	Serial.println(name);
#endif

	fButtonStop = false;
	fButtonPause = false;

	ifstream sdin(name);

	if (sdin.is_open()) {

		while (sdin >> InterpTime >> c1 >> DelayTime >> c2 >> BA >> c3 >> shl >> c4 >> shl1 >> c5 >> elb >> c6 >> wri >> c7 >> WRro >> c8 >> Gr) {
			
			if (c1 != ',' || c2 != ',' || c3 != ',' || c4 != ',' || c5 != ',' || c6 != ','|| c7 != ','|| c8 != ',') 
				continue;
			
		#ifdef DEBUG
			Serial.print(F("FileLine = "));
			Serial.println(fileLine);
			Serial.print(F("  "));
			Serial.print(InterpTime);
			Serial.print(c1);
			Serial.print(F("  "));
			Serial.print(DelayTime);
			Serial.print(c2);
			Serial.print(" "); Serial.print(BA);
			Serial.print(c3);
			Serial.print(" "); Serial.print(shl);
			Serial.print(c4);
			Serial.print(" "); Serial.print(shl1);
			Serial.print(c5);
			Serial.print(" "); Serial.print(elb);
			Serial.print(c6);
			Serial.print(" "); Serial.print(wri);
			Serial.print(c7);
			Serial.print(" "); Serial.print(WRro);
			Serial.print(c8);
			Serial.print(" "); Serial.println(Gr);
			Serial.println();
		#endif

			fileLine++;

			if (InterpTime == 0) {

				wServoMoveTime = 2000;
			}
			else{

				TempServoMoveTime = InterpTime + PlaySpeed;                         // default 300 + 0
				wServoMoveTime = max(TempServoMoveTime, INTERPOLATE_P_MIN);
			}

			GroupServoUpdate(wServoMoveTime, BA, shl, shl1, elb, wri, WRro, Gr);

			if (InterpTime == 0 && !fButtonStop && !fButtonPause) {

				delay_ms(2000);
			}
			else if(InterpTime != 0 && !fButtonStop && !fButtonPause) {

				wTimeDelay = DelayTime + TempServoMoveTime;                         // default 0 + 300 + 0
				wTimeDelay = max(wTimeDelay, TIME_DELAY_P_MIN);

				if(wTimeDelay < wServoMoveTime)
					wTimeDelay = wServoMoveTime;


				if(!fButtonStop && !fButtonPause)

					delay_ms(wTimeDelay);
			}

			if(!fButtonStop && !fButtonPause)

				Control_PS2_Input_S();

			else if(fButtonStop){

				Serial.println(F("Stop_butt"));
				fButtonStop = false;
				break;
			}
			else if(fButtonPause){

				Serial.println(F("Pause_butt"));

				data[0] = B01000000;        // SEG_G
				data[1] = B01000000;
				data[2] = B01000000;
				data[3] = B01000000;
				display.setSegments(data);  // display raw segment

				DoPause();
			}

		} // end while
	}

	mode = 'N';
	TM1637Display_Off();
	
	delay_ms(1000);
	servo_park(PARK_READY);
	delay_ms(T_PARK_ON + 1000);

	ResetAllFlags();
}




void DoPause(void){   // do pause for Playback

	while(1){

		Control_PS2_Input_S();

		if(fButtonStop)
			break;

		if(!fButtonPause) {

		#ifdef DEBUG
			Serial.println(F("Play_butt"));
		#endif

			setDisplay('P');
			break;
		}

		delay(20);
	}
}










void setup(){


	// initialise Arduino ports
	pinMode(Buzz_p, OUTPUT);
	pin_port = portOutputRegister(digitalPinToPort(Buzz_p));
	pin_mask = digitalPinToBitMask(Buzz_p);
  

#ifdef DEBUG
	Serial.begin(115200);
#endif

	InitPs2();
	
	TorqueLevel = (GRIPPER_FSR_MIN_TORQUE + (pgm_read_byte(&TorqueSelect[TorqueIndex]) * TORQUE_MULTIFACTOR) / 100); //=228


#ifdef DEBUG
	Serial.println(F("Start ARM!!!"));
#endif

	// Initialize the display
	display.setBrightness(7); // Turn on
	for(int k = 0; k < 4; k++) data[k] = 0;
	display.setSegments(data);

	delay(500);

}








void loop() {


	// Store desired position in tmp variables until confirmed by doArmIK() logic
#ifndef CYL_IK   // 3D kinematics
	x_tmp = X;
#endif

	y_tmp = Y;
	z_tmp = Z;
	ga_tmp = GA_pos;

	Control_PS2_Input();

	if(fArmOn == true) {

		if(fArmOn == true && fArmOn_prev == false){      // Turn Arm On

			TurnArmOn();
		}


		if(fButtonRec == true){

			startRecord();
			fButtonRec = false;
		}


		if(fButtonPlay2 == true) {

			if (mode == 'S') {
				
				#ifdef DEBUG
					Serial.println(F("StartPlayback"));
				#endif
				
				// Selection has been made. Start playback.
				startPlayback(playbackProgram);
				fButtonPlay2 = false;
			}
		}

		if(fButtonPlay1 == true) {

				#ifdef DEBUG
					Serial.println(F("StartSelect"));
				#endif

				startSelect();
				fButtonPlay1 = false;
		}


		// Only perform IK calculations if arm motion is needed.
		if (arm_moveIK) {

		#ifdef CYL_IK   // 2D kinematics 
			if (doArmIK(0, y_tmp, z_tmp, ga_tmp) == IK_SUCCESS) {

				DegToUsServoIK();

				ServoUpdateIK(BA_pos_us, shl_pos_us, shl1_pos_us, elb_pos_us, wri_pos_us);

				// If the arm was positioned successfully, record
				// the new vales. Otherwise, ignore them.
				Y = y_tmp;
				Z = z_tmp;
				GA_pos = ga_tmp;

			}
			else {
				// Sound tone for audible feedback of error
				//tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
				MSound (2, 40, 2500, 40, 2500);
			}

		#else           // 3D kinematics
			if (doArmIK(x_tmp, y_tmp, z_tmp, ga_tmp) == IK_SUCCESS) {

				DegToUsServoIK();

				ServoUpdateIK(BA_pos_us, shl_pos_us, shl1_pos_us, elb_pos_us, wri_pos_us);

				// If the arm was positioned successfully, record
				// the new vales. Otherwise, ignore them.
				X = x_tmp;
				Y = y_tmp;
				Z = z_tmp;
				GA_pos = ga_tmp;
			}
			else {
				// Sound tone for audible feedback of error
				//tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
				MSound (2, 40, 2500, 40, 2500);
			}
		#endif

			arm_moveIK = false;
		
		} // arm_moveIK 

		GripperControl();                         // Gr_pos_us

		if(arm_moveNoIK){

			WristRotControl();                    // WRro_pos

		#ifdef CYL_IK   // 2D kinematics
			BaseControl();                        // BA_pos2D
		#endif

			arm_moveNoIK = false;
		}

		if (fCapture_pos) {

			DegToUsServoIK();
			fCapture_pos = false;
		}	
	
	} // end fArmOn == true} 

	else {  

		if(fArmOn_prev == true) {                 // Turn the Arm OFF

			TurnArmOff();
		}
	}

	if(fArmOn)
		fArmOn_prev = true;
	else
		fArmOn_prev = false;

	delay(10);

} // end loop







// Arm positioning routine utilizing Inverse Kinematics (IK).
// Input X, Y, Z (mm)
// Z is height, Y is distance from base center out, X is side to side. Y, Z can only be positive.
// X=0 for 2D kinematics;
// Input dimensions are for the gripper, just short of its tip, where it grabs things.
// return -  BA_pos3D; shl_pos; elb_pos; wri_pos (in degrees)
// If resulting arm position is physically unreachable, return error code.

int doArmIK(float x, float y, float z, float grip_angle_d) {

	// grip angle in radians for use in calculations
	float grip_angle_r = radians(grip_angle_d);    

	// Base angle and radial distance from x,y coordinates
	float bas_angle_r = atan2(x, y);

 #ifdef CYL_IK   // 2D kinematics
	// We are in cylindrical mode, probably simply set y` to the y we passed in...
	// y = y;
 #else           // 3D kinematics
	// rdist is y coordinate for the arm
	float rdist = sqrt((x * x) + (y * y));
	y = rdist;
 #endif

	// Grip offsets calculated based on grip angle
	float grip_off_z = (sin(grip_angle_r)) * GRIPPER;
	float grip_off_y = (cos(grip_angle_r)) * GRIPPER;

	// Wrist position
	float wrist_z = (z - grip_off_z) - BASE_HGT;
	float wrist_y = y - grip_off_y;

	// Shoulder to wrist distance (AKA sw)
	float s_w = (wrist_z * wrist_z) + (wrist_y * wrist_y);
	float s_w_sqrt = sqrt(s_w);

	// s_w angle to ground
	float a1 = atan2(wrist_z, wrist_y);

	// s_w angle to humerus
	float a2 = acos(((hum_sq - uln_sq) + s_w) / (2 * HUMERUS * s_w_sqrt));

	// Shoulder angle
	float shl_angle_r = a1 + a2;
	float shl_angle_d = degrees(shl_angle_r);

	// If result is NAN or Infinity, the desired arm position is not possible
	if (isnan(shl_angle_r) || isinf(shl_angle_r)){
		Serial.println(F(" Shl result is NAN or Infinity! "));
		return IK_ERROR;
	}


	// Elbow angle
	float elb_angle_r = acos((hum_sq + uln_sq - s_w) / (2 * HUMERUS * ULNA));

	// If result is NAN or Infinity, the desired arm position is not possible
	if (isnan(elb_angle_r) || isinf(elb_angle_r)){
		Serial.println(F(" Elb result is NAN or Infinity! "));
		return IK_ERROR;
	}

	float elb_angle_d = degrees(elb_angle_r);
	float elb_angle_dn = -(180.0 - elb_angle_d);

	// Gripper/Wrist angle
	float wri_angle_d = (grip_angle_d - elb_angle_dn) - shl_angle_d;


	// Calculate Servo angles (degrees)
	// Calc relative to Servo midpoint to allow compensation for Servo alignment
#ifndef CYL_IK   // 3D kinematics
	BA_pos3D  = BAS_MID + degrees(bas_angle_r);
#endif
	shl_pos = SHL_MID + (shl_angle_d - 90.0);
	elb_pos = ELB_MID - (elb_angle_d - 90.0);
	wri_pos = WRI_MID + wri_angle_d;

#ifdef DEBUG
	Serial.println(F("Fn_doArmIK"));
 #ifndef CYL_IK   // 3D kinematics
	Serial.print(F("  X: "));
	Serial.print(x);
 #endif
	Serial.print(F("  Y: "));
	Serial.print(y);
	Serial.print(F("  Z: "));
	Serial.print(z);
	Serial.print(F("  GA_pos: "));
	Serial.println(grip_angle_d);

 #ifndef CYL_IK   // 3D kinematics
	Serial.print(F("  Base Pos 3D: "));
	Serial.print(BA_pos3D);
 #endif
	Serial.print(F("  Shld Pos: "));
	Serial.print(shl_pos);
	Serial.print(F("  Elbw Pos: "));
	Serial.print(elb_pos);
	Serial.print(F("  Wrst Pos: "));
	Serial.println(wri_pos);

 #ifndef CYL_IK   // 3D kinematics
	Serial.print(F("  bas_angle_d: "));
	Serial.print(degrees(bas_angle_r));  
 #endif
	Serial.print(F("  shl_angle_d: "));
	Serial.print(shl_angle_d);  
	Serial.print(F("  elb_angle_d: "));
	Serial.print(elb_angle_d);
	Serial.print(F("  wri_angle_d: "));
	Serial.println(wri_angle_d);
	Serial.println();
#endif

	// If any Servo ranges are exceeded, return an error
#ifndef CYL_IK   // 3D kinematics
	if (BA_pos3D < BAS_MIN || BA_pos3D > BAS_MAX ){
		Serial.println(F("  Servo ranges are exceeded! "));
		return IK_ERROR;
	}
#endif

	if (shl_pos < SHL_MIN || shl_pos > SHL_MAX || elb_pos < ELB_MIN || elb_pos > ELB_MAX || wri_pos < WRI_MIN || wri_pos > WRI_MAX){
		Serial.println(F("  Servo ranges are exceeded! "));
		return IK_ERROR;
	}
	return IK_SUCCESS;
} // doArmIK()



// Calculate Servo position from degrees to us
// if mode == Record and (fCapture_pos == true) save all servo position to SD 
// input - BA_pos3D; shl_pos; elb_pos; wri_pos
// return - BA_pos_us(3D); shl_pos_us; shl1_pos_us; elb_pos_us; wri_pos_us
void DegToUsServoIK(void) {

#ifndef CYL_IK   // 3D kinematics
	BA_pos_us = deg_to_us(BA_pos3D, 0);
#endif
	shl_pos_us = deg_to_us(shl_pos, 1);
	shl1_pos_us = deg_to_us(182-shl_pos, 1);
	elb_pos_us = deg_to_us(elb_pos, 1);
	wri_pos_us = deg_to_us(wri_pos, 0);
	//WRro_pos_us = deg_to_us(WRro_pos);
	//Gr_pos_us = deg_to_us(Gr_pos);

#ifdef DEBUG
	Serial.println(F("Fn_DegToUsServoIK"));
#ifndef CYL_IK   // 3D kinematics
	Serial.print(F(" BA_pos 3D = "));
	Serial.print(BA_pos3D);
	Serial.print(F(","));
#endif
	Serial.print(F(" shl_pos = "));
	Serial.print(shl_pos);
	Serial.print(F(", elb_pos = "));
	Serial.print(elb_pos);
	Serial.print(F(", wri_pos = "));
	Serial.println(wri_pos);
	//Serial.print(F("  WRro_pos: "));
	//Serial.println(WRro_pos);
	//Serial.print(F("  Gr_pos: "));
	//Serial.println(Gr_pos);
#ifndef CYL_IK   // 3D kinematics
	Serial.print(F(" BA_pos_us = "));
	Serial.print(BA_pos_us);
	Serial.print(F(","));
#endif
	Serial.print(F(" shl_pos_us = "));
	Serial.print(shl_pos_us);
	Serial.print(F(", shl1_pos_us = "));
	Serial.print(shl1_pos_us);
	Serial.print(F(", elb_pos_us = "));
	Serial.print(elb_pos_us);
	Serial.print(F(", wri_pos_us = "));
	Serial.println(wri_pos_us);
	//Serial.print(F(" WRro_pos_us = "));
	//Serial.println(WRro_pos_us);
	//Serial.print(F(" Gr_pos_us = "));
	//Serial.println(Gr_pos_us);
	Serial.println();
#endif

	if ((mode == 'R') && (fCapture_pos == true)) {

		if (old_BA_pos_us != BA_pos_us || old_shl_pos_us != shl_pos_us || old_elb_pos_us != elb_pos_us ||
			old_wri_pos_us != wri_pos_us || old_WRro_pos_us != WRro_pos_us || old_Gr_pos_us != Gr_pos_us) {

			writeCommand();

		//#ifdef DEBUG   // Vad!
		//	Serial.println(F("Capture to SD "));
		//#endif

			//tone(SPK_PIN, TONE_READY , TONE_DURATION);
			MSound(1, 50, 6000);

			//old_BA_pos_us = BA_pos_us;
			//old_shl_pos_us = shl_pos_us;
			//old_elb_pos_us = elb_pos_us;
			//old_wri_pos_us = wri_pos_us;
			//old_WRro_pos_us = WRro_pos_us;
			//old_Gr_pos_us = Gr_pos_us;
		}
	}
}



// Writes Servo Position ant group move to position during MoveTime
void GroupServoUpdate(unsigned int MoveTime, int BA_us, int shl_us, int shl1_us, int elb_us, int wri_us, int WRro_us, int Gr_us) {

	// Starts a group move
	// The servos are not moved until the commit call
	ServoGroupMove.start();

	// Position the servos
	Bas_Servo.writeMicroseconds(BA_us);
	Shl_Servo.writeMicroseconds(shl_us);
	Shl_Servo1.writeMicroseconds(shl1_us);
	Elb_Servo.writeMicroseconds(elb_us);
	Wri_Servo.writeMicroseconds(wri_us);
	Wro_Servo.writeMicroseconds(WRro_us);
	Gri_Servo.writeMicroseconds(Gr_us);

	ServoGroupMove.commit(MoveTime);

#ifdef DEBUG
	Serial.print(F("GroupServoUpdate!  "));
	Serial.print(F("MoveTime = "));
	Serial.println(MoveTime);
	Serial.println();
#endif

}
 


// Writes Servo Position ant group move to position during MoveTime
void ServoUpdateIK(int BA_us3D, int shl_us, int shl1_us, int elb_us, int wri_us) {

	// Update position the servos
#ifndef CYL_IK   // 3D kinematics
	Bas_Servo.writeMicroseconds(BA_us3D);
 #endif
	Shl_Servo.writeMicroseconds(shl_us);
	Shl_Servo1.writeMicroseconds(shl1_us);
	Elb_Servo.writeMicroseconds(elb_us);
	Wri_Servo.writeMicroseconds(wri_us);

#ifdef DEBUG
	Serial.println(F("ServoUpdateIK!"));
#endif

}



// Move servos to parking position
void servo_park(int park_type) {

	int error;

	switch (park_type) {

		// All servos at MidPoint position
		case PARK_MIDPOINT:

		#ifdef DEBUG
			Serial.println("PARK_MIDPOINT:");
		#endif


		#ifdef CYL_IK   // 2D kinematics
			error = doArmIK(0.0, MID_Y, MID_Z, MID_GRA);              // 0, 155.5, 171, 23.42
			BA_pos2D = MID_BA;                                // 90
			BA_pos_us = deg_to_us(BA_pos2D, 0);

		#ifdef DEBUG
			Serial.print(F(" Base Pos 2D = "));
			Serial.print(BA_pos2D);
			Serial.print(F(", BA_pos_us = "));
			Serial.print(BA_pos_us);
		#endif

		#else           // 3D kinematics
			error = doArmIK(MID_X, MID_Y, MID_Z, MID_GRA);            // 0, 155.5, 171, 23.42
		#endif
			WRro_pos = MID_WRO;                               // 90 
			WRro_pos_us = deg_to_us(WRro_pos, 0);

		#ifdef DEBUG
			Serial.print(F("  WR_RO = "));
			Serial.print(WRro_pos);
			Serial.print(F(", WRro_pos_us = "));
			Serial.print(WRro_pos_us);
		#endif			

			POTCtrlPos = POT_MID_POS;                         // 122
			Gr_pos_us = GRIP_MID_US;                          // POT_MID_POS=50, GRIP_MID_US=1190

		#ifdef DEBUG
			Serial.print(F(", Gr_pos_us = "));
			Serial.println(Gr_pos_us, DEC);
		#endif

			DegToUsServoIK();
			
			if(error == IK_SUCCESS){
				GroupServoUpdate(T_PARK_MID, BA_pos_us, shl_pos_us, shl1_pos_us, elb_pos_us, wri_pos_us, WRro_pos_us, Gr_pos_us);
			}
			else{
				MSound (2, 40, 2500, 40, 2500);
			}
			Y = MID_Y;
			Z = MID_Z;
			GA_pos = MID_GRA;

		#ifdef CYL_IK   // 2D kinematics
			BA_pos_s = BA_pos2D;
		#else           // 3D kinematics
			X = MID_X;
		#endif
			WRro_pos_s = WRro_pos;
			POTCtrlPos_s = POTCtrlPos;
			break;


		// Ready-To-Run position
		case PARK_READY:
			
		#ifdef DEBUG
			Serial.println("PARK_READY:");
		#endif
			
		#ifdef CYL_IK   // 2D kinematics
			error = doArmIK(0.0, READY_Y, READY_Z, READY_GRA);        // 0; 133.76; 101; -47.33
			BA_pos2D = READY_BA;                                      // 90
			BA_pos_us = deg_to_us(BA_pos2D, 0);

		#ifdef DEBUG
			Serial.print(F(" Base Pos 2D = "));
			Serial.print(BA_pos2D);
			Serial.print(F(", BA_pos_us = "));
			Serial.print(BA_pos_us);
		#endif

		#else           // 3D kinematics
			error = doArmIK(READY_X, READY_Y, READY_Z, READY_GRA);    // 0; 170; 45; 0
		#endif
			WRro_pos = READY_WRO;                                     // 90 
			WRro_pos_us = deg_to_us(WRro_pos, 0);

		#ifdef DEBUG
			Serial.print(F("  WR_RO = "));
			Serial.print(WRro_pos);
			Serial.print(F(", WRro_pos_us = "));
			Serial.print(WRro_pos_us);
		#endif			

			POTCtrlPos = POT_READY_POS;
			Gr_pos_us = GRIP_READY_US;                                // POT_READY_POS=50, GRIP_READY_US=1370

		#ifdef DEBUG
			Serial.print(F(", Gr_pos_us = "));
			Serial.println(Gr_pos_us, DEC);
		#endif

			DegToUsServoIK();

			if(error == IK_SUCCESS){
				GroupServoUpdate(T_PARK_ON, BA_pos_us, shl_pos_us, shl1_pos_us, elb_pos_us, wri_pos_us, WRro_pos_us, Gr_pos_us);
			}
			else{

				MSound (2, 40, 2500, 40, 2500);
			}

			Y = READY_Y;
			Z = READY_Z;
			GA_pos = READY_GRA;

		#ifdef CYL_IK   // 2D kinematics
			BA_pos_s = BA_pos2D;
		#else           // 3D kinematics
			X = MID_X;
		#endif
			WRro_pos_s = WRro_pos;
			POTCtrlPos_s = POTCtrlPos;
			break;
		
		// All servos at PARK_OFF position
		case PARK_OFF:
			
		#ifdef DEBUG
			Serial.println("PARK_OFF:");
		#endif


		#ifdef CYL_IK   // 2D kinematics
			error = doArmIK(0.0, OFF_Y, OFF_Z, OFF_GRA);              // 0; 105; 113; 17.18
			BA_pos2D = OFF_BA;                                        // 90
			BA_pos_us = deg_to_us(BA_pos2D, 0);

		#ifdef DEBUG
			Serial.print(F(" Base Pos 2D = "));
			Serial.print(BA_pos2D);
			Serial.print(F(", BA_pos_us = "));
			Serial.print(BA_pos_us);
		#endif

		#else           // 3D kinematics
			error = doArmIK(OFF_X, OFF_Y, OFF_Z, OFF_GRA);            // 0; 105; 113; 17.18
		#endif
			WRro_pos = OFF_WRO;                                       // 90 
			WRro_pos_us = deg_to_us(WRro_pos, 0);

		#ifdef DEBUG
			Serial.print(F("  WR_RO = "));
			Serial.print(WRro_pos);
			Serial.print(F(", WRro_pos_us = "));
			Serial.print(WRro_pos_us);
		#endif			
		
			POTCtrlPos = POT_OFF_POS;
			Gr_pos_us = GRIP_OFF_US;                                  // POT_OFF_POS=0, GRIP_OFF_US=1450

		#ifdef DEBUG
			Serial.print(F(", Gr_pos_us = "));
			Serial.println(Gr_pos_us, DEC);
		#endif

			DegToUsServoIK();
			
			if(error == IK_SUCCESS){
				GroupServoUpdate(T_PARK_OFF, BA_pos_us, shl_pos_us, shl1_pos_us, elb_pos_us, wri_pos_us, WRro_pos_us, Gr_pos_us);
			}
			else{

				MSound (2, 40, 2500, 40, 2500);
			}

			Y = OFF_Y;
			Z = OFF_Z;
			GA_pos = OFF_GRA;

		#ifdef CYL_IK   // 2D kinematics
			BA_pos_s = BA_pos2D;
		#else           // 3D kinematics
			X = MID_X;
		#endif

			WRro_pos_s = WRro_pos;
			POTCtrlPos_s = POTCtrlPos;
			break;

	} // end switch
}



// The Arduino Servo library .write() function accepts 'int' degrees, meaning
// maximum Servo positioning resolution is whole degrees. Servos are capable 
// of roughly 2x that resolution via direct microsecond control.
// This function converts 'float' (i.e. decimal) degrees to corresponding 
// Servo microseconds to take advantage of this extra resolution.
int deg_to_us(float value, int model) {

	// Apply basic constraints
	if (value < SERVO_MIN_DEG) value = SERVO_MIN_DEG;
	if (value > SERVO_MAX_DEG) value = SERVO_MAX_DEG;

	// Map degrees to microseconds, and round the result to a whole number
	return(round(map_float(value, SERVO_MIN_DEG, SERVO_MAX_DEG, (float)SERVO_MIN_US[model], (float)SERVO_MAX_US[model])));      
}



// Same logic as native map() function, just operates on float instead of long
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {

	return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}



void AttachServos(void) {

	if (!fServosAttached) {

		// Attach to the servos and specify range limits
		Bas_Servo.attach(BAS_SERVO_PIN, SERVO_MG996_MIN_US, SERVO_MG996_MAX_US);
		Shl_Servo.attach(SHL_SERVO_PIN, SERVO_DS3218_MIN_US, SERVO_DS3218_MAX_US);
		Shl_Servo1.attach(SHL_SERVO1_PIN, SERVO_DS3218_MIN_US, SERVO_DS3218_MAX_US);
		Elb_Servo.attach(ELB_SERVO_PIN, SERVO_DS3218_MIN_US, SERVO_DS3218_MAX_US);
		Wri_Servo.attach(WRI_SERVO_PIN, SERVO_MG996_MIN_US, SERVO_MG996_MAX_US);
		Wro_Servo.attach(WRO_SERVO_PIN, SERVO_MG996_MIN_US, SERVO_MG996_MAX_US);
		Gri_Servo.attach(GRI_SERVO_PIN, SERVO_MG996_MIN_US, SERVO_MG996_MAX_US);
		

		fServosAttached = true;

	  #ifdef DEBUG
		Serial.println(F("Servos Attached"));
	  #endif
	}
}



void FreeServos(void){

	if (fServosAttached) {
		// Detach the servos 
		Bas_Servo.detach();
		Shl_Servo.detach();
		Shl_Servo1.detach();
		Elb_Servo.detach();
		Wri_Servo.detach();
		Wro_Servo.detach();
		Gri_Servo.detach();

		fServosAttached = false;

	 #ifdef DEBUG
		Serial.println(F("Servos Free"));
	 #endif
	}
}




// SoundNoTimer - Quick and dirty tone function to try to output a frequency
// to a speaker for some simple sounds.
//-------------------------------------------------------------------------------
#ifdef Buzz_p
void SoundNoTimer(unsigned long duration,  unsigned int frequency){
	long toggle_count = 0;
	long lusDelayPerHalfCycle;

	toggle_count = 2 * frequency * duration / 1000;
	lusDelayPerHalfCycle = 1000000L/(frequency * 2);

	// if we are using an 8 bit timer, scan through prescalars to find the best fit
	while (toggle_count--) {
		// toggle the pin
		*pin_port ^= pin_mask;

		// delay a half cycle
		delayMicroseconds(lusDelayPerHalfCycle);
	}    
	*pin_port &= ~(pin_mask);  // keep pin low after stop
}

void MSound(byte cNotes, ...){
	va_list ap;
	unsigned int uDur;
	unsigned int uFreq;
	va_start(ap, cNotes);

	while (cNotes > 0) {
		uDur = va_arg(ap, unsigned int);
		uFreq = va_arg(ap, unsigned int);
		SoundNoTimer(uDur, uFreq);
		cNotes--;
	}
	va_end(ap);
}
#else
void MSound(byte cNotes, ...){};
#endif //SOUND_PIN 




// Function to read inputs from the PS2 
//-------------------------------------------------------------------------------
void Control_PS2_Input_S(void){

	// try to receive a packet of information from the PS2.
	Ps2x.read_gamepad();                            // read controller
	
	PS2ErrorCnt = 0;
	
	if ((Ps2x.Analog(1) & 0xf0) == 0x70) {
		
		
		if (Ps2x.ButtonPressed(PSB_CROSS)) {		// X - Cross Button Test
			
			MSound(1, 50, 2000);                    // [50\4000]

			fButtonStop = true;
		}


		if (Ps2x.ButtonPressed(PSB_SQUARE)) {                    // PSB_SQUARE pressed

			fButtonPause = !fButtonPause;
		}

		// Increment Multiplier - control--------------------------
		if (Ps2x.ButtonPressed(PSB_SELECT) && fButtonPause) {    // PSB_SELECT Test 
		//-Increase torque-----------
						
			MultIndex %= 3;
			MultIncr = pgm_read_byte(&INCREMENT_MUL[MultIndex]);
			MultIndex ++;

		#ifdef DEBUG
			Serial.print(F("MultIncr = "));
			Serial.println(MultIncr);
		#endif

			Display_d(MultIncr);

			if (MultIncr == 3) {

				MSound(1, 40, 3000);
			}  
			else {

				MSound(1, 50, 6000);
			}
		}

		// Playback - Speed control ( mode == PLAYBACK) 
		if (mode == 'P') {
			if (Ps2x.ButtonPressed(PSB_PAD_UP) || Ps2x.ButtonPressed(PSB_PAD_DOWN)) {   // PSB_PAD_UP OR PSB_PAD_DOWN Test

				//Increase Play speed with -50mS   -->
				if(Ps2x.ButtonPressed(PSB_PAD_UP)) {                                    // PSB_PAD_UP Test

					PlaySpeed -= PLAY_SPEED_INCREMENT * MultIncr;                       // Default 50 * 1;
				}
					
				//-Decrease Play speed +50mS   <--
				if (Ps2x.ButtonPressed(PSB_PAD_DOWN)) {                                 // PSB_PAD_DOWN Test

					PlaySpeed += PLAY_SPEED_INCREMENT * MultIncr;                       // Default 50 * 1;
				}

				// Constrain to limits
				PlaySpeed = constrain(PlaySpeed, PLAY_SPEED_MIN, PLAY_SPEED_MAX);

				//tone(SPK_PIN, TONE_READY, TONE_DURATION);
				MSound(1, 50, 6000);

			#ifdef DEBUG	
				Serial.print(F(" PlaySpeed = "));
				Serial.println(PlaySpeed);
			#endif

				Display_d(PlaySpeed);

				// Provide audible feedback of reaching limit
				if (PlaySpeed == PLAY_SPEED_MIN || PlaySpeed == PLAY_SPEED_MAX) {

					//tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
					MSound (2, 40, 2500, 40, 2500);
				}
			}
		} // mode == 'P'

	}  // end, if((ps2x.Analog(1) & 0xf0) == 0x70), read PS2 controller 

	else {
		if (PS2ErrorCnt < MAXPS2ERRORCNT)
			
			PS2ErrorCnt++;							// Increment the error count and if to many errors, turn off the robot.
		
		else if (fArmOn){
			
			fArmOn == false;
		}  
		
		Ps2x.reconfig_gamepad();
	}
} // end, Control_PS2_Input_S 



void delay_ms( unsigned long delayTime_ms) {

	unsigned long carTime_ms;

#ifdef DEBUG
	Serial.print(F(" Delay_ms = "));
	Serial.println(delayTime_ms);
#endif

	carTime_ms = millis();                   // save carent time

	do{

		Control_PS2_Input_S();
		
		if( fButtonStop == true)
			break;
		
		delay(20);


	} while( millis() - carTime_ms < delayTime_ms );

#ifdef DEBUG
	Serial.println(F(" Delay_end "));
#endif
}



void SaveOldPos_us(void) {

	old_BA_pos_us = BA_pos_us;
	old_shl_pos_us = shl_pos_us;
	old_elb_pos_us = elb_pos_us;
	old_wri_pos_us = wri_pos_us;
	old_WRro_pos_us = WRro_pos_us;
	old_Gr_pos_us = Gr_pos_us;
}



void GetFeetbackAllServo(void) {

	// feetback check
	Bas_fb = Bas_Servo.read();
	Shl_fb = Shl_Servo.read();
	Shl1_fb = Shl_Servo1.read();
	Elb_fb = Elb_Servo.read();
	Wri_fb = Wri_Servo.read();
	Wro_fb = Wro_Servo.read();
	Gri_fb = Gri_Servo.read();

#ifdef DEBUG
	Serial.print("  Base Fb: ");
	Serial.print(Bas_fb);
	Serial.print("  Shld Fb: ");
	Serial.print(Shl_fb);
	Serial.print("  Shld1 Fb: ");
	Serial.print(Shl1_fb);
	Serial.print("  Elbw Fb: ");
	Serial.print(Elb_fb);
	Serial.print("  Wri Fb: ");
	Serial.print(Wri_fb);
	Serial.print("  WrRO Fb: ");
	Serial.print(Wro_fb);
	Serial.print("  Grip Fb: ");
	Serial.println(Gri_fb);
	Serial.println();
#endif
}



void ResetAllFlags(void) {

	fPSB_SQUARE_4s = false;
	fButtonPlay1 = false;
	fButtonPlay2 = false;
	fRecStart_10s = false;
	fRecStop_10s = false;
	fButtonRec = false;
	fButtonStop = false;
	fCapture_pos = false;
	fButtonPause = false;
	arm_moveIK = false;
	arm_moveNoIK = false;

	ulTimePSB_CROSS = millis();                                   // reset ulTimePSB_CROSS
	ulTimePSB_SQUARE = ulTimePSB_CROSS;                           // reset ulTimePSB_SQUARE
	ulTimePSB_CIRCLE = ulTimePSB_CROSS;                           // reset ulTimePSB_CIRCLE

}



float readFbServoAngle(byte servoNum, int model, boolean withOffset) {

	float angle;
	int AnalogVal;

	AnalogVal = getAnalogPinValue(servoNum);

#ifdef DEBUG
	Serial.print("  Ser_N = ");
	Serial.print(servoNum);
	Serial.print("  Mod = ");
	Serial.print(model);
	Serial.print(", AnVal = ");
	Serial.println(AnalogVal);
#endif

	angle = map(AnalogVal, SERVO_ANALOG_MIN_MV[model], SERVO_ANALOG_MAX_MV[model], 0, 180);

	//angle = analogToAngle(servoNum, getServoAnalogData(servoNum)); 

	angle = constrain(angle, 0.00, 180.00);

	if (withOffset) {

		angle -= mServoAngleOffset[servoNum];
	}

	return angle;
}



unsigned int getServoAnalogData(byte servoNum) {

	return getAnalogPinValue(SERVO_ANG_PIN[servoNum]);
}




// get analog value of pin
//in param pin of arduino
// return value of analog data
int getAnalogPinValue(unsigned int pin) {
 
	unsigned int dat[8];
	unsigned int result;


	for(int i = 0; i < 8; i++) {
		dat[i] = analogRead(pin);
	}

	_sort(dat, 8);

	result = (dat[2]+dat[3]+dat[4]+dat[5])/4;

	return result;    
}



static void _sort(unsigned int array[], unsigned int len) {

	unsigned char i=0,j=0;
	unsigned int temp = 0;

	for(i = 0; i < len; i++) {
		for(j = 0; i+j < (len-1); j++) {
			if(array[j] > array[j+1]) {
				temp = array[j];
				array[j] = array[j+1];
				array[j+1] = temp;
			}
		}
	}	
}



double analogToAngle(byte servoNum, int inputAnalog) {

	float intercept = 0.0f;
	float slope = 0.0f;
	float angle;

	//readLinearOffset(servoNum, intercept, slope);

	angle = intercept + slope * inputAnalog;  

	return angle;
}



//void readLinearOffset(byte servoNum, double& interceptVal, double& slopeVal) {
//
//	EEPROM.get(LINEAR_INTERCEPT_START_ADDRESS + servoNum * sizeof(interceptVal), interceptVal); // (70 + servoNum) * 4); EEPROM[70, 71, 72, 73]  
//	EEPROM.get(LINEAR_SLOPE_START_ADDRESS + servoNum * sizeof(slopeVal), slopeVal);             // (50 + servoNum) * 4); EEPROM[50, 51, 52, 53]
//}



// GripperControl
// Calculation of the value PWM with the control of compression for Gripper
// input - POTCtrlPos, TorqueLevel, 
// return - Gr_pos_us
void GripperControl(void) {

	//GripperFSRInput = analogRead(FSR_ANG_PIN);                   // Read FSR
	GripperFSRInput = 0; //Vad!

	if(GripperFSRInput > GRIPPER_CONTACT) {         
		// this is true the Gripper are touching the object; GRIPPER_CONTACT=350  
		if(!GripperFSR_Activated) {                                 // if in contact with FSR for the first time
			POTSavePos = POTCtrlPos;                                // Save the potentiometer position
		}   
		//adjusting Grippers to the correct Torque level:
		if(GripperFSRInput > TorqueLevel+100) {                     // If to much torque:
			Gr_pos_us --;                                           // open Gripper a little bit
		}
		else if (GripperFSRInput < TorqueLevel-100) {               // If to little torque:
			Gr_pos_us ++;                                           // close Gripper a little bit
		}  
		if(POTCtrlPos > (POTSavePos + 10)) {                        // open Grippers
			Gr_pos_us = GRIP_CLOSED_US - POTCtrlPos*5/2;            // 1495 - POTCtrlPos*5/2
		}
		GripperFSR_Activated = true;
	}  
	else { // Open/close Grippers

		GripperFSR_Activated = false;
		Gr_pos_us = GRIP_CLOSED_US - POTCtrlPos*5/2;                // 1495 - POTCtrlPos*5/2

		//Checks the mechanical limits of the Gripper servo
		Gr_pos_us = min(max(Gr_pos_us, GRIPPER_OPEN_US), GRIP_CLOSED_US); 
	}

	Gri_Servo.writeMicroseconds(Gr_pos_us);



	if (POTCtrlPos_s != POTCtrlPos) {

	#ifdef DEBUG
		Serial.print(F("  POTCtrlPos = "));
		Serial.print(POTCtrlPos, DEC);
		Serial.print(F(", Gr_pos_us = "));
		Serial.print(Gr_pos_us, DEC);
		Serial.print(F(", FSR = "));
		Serial.print(GripperFSRInput, DEC);
		Serial.print(F(", TorqueLevel = "));
		Serial.print(TorqueLevel, DEC);
		Serial.print(F(", TMF ="));
		Serial.println(TORQUE_MULTIFACTOR, DEC);
	#endif
	}

	POTCtrlPos_s = POTCtrlPos;
}  



void WristRotControl(void) {

	int AnalVal;

	if(WRro_pos != WRro_pos_s) {

		WRro_pos_us = deg_to_us(WRro_pos, 0);
		Wro_Servo.writeMicroseconds(WRro_pos_us);
		WRro_pos_s = WRro_pos; 

	#ifdef DEBUG
		Serial.print(F("  WR_RO = "));
		Serial.print(WRro_pos);
		Serial.print(F(", WRro_pos_us = "));
		Serial.println(WRro_pos_us);
	#endif

	//	AnalVal = getAnalogPinValue(4);

	//#ifdef DEBUG
	//	Serial.print("  AnVal = ");
	//	Serial.println(AnalVal);
	//#endif
	}
}



#ifdef CYL_IK   // 2D kinematics
void BaseControl(void) {

	if(BA_pos2D != BA_pos_s) {

		BA_pos_us = deg_to_us(BA_pos2D, 0);
		Bas_Servo.writeMicroseconds(BA_pos_us);
		BA_pos_s = BA_pos2D;

	#ifdef DEBUG
		Serial.print(F(" Base Pos 2D = "));
		Serial.print(BA_pos2D);
		Serial.print(F(", BA_pos_us = "));
		Serial.println(BA_pos_us);
	#endif 
	}
}
#endif

