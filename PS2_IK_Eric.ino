/******************************************************************
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
*
*
*   PS2 Controls
*       Start Button        On/Off
*       Right Joystick L/R: Gripper tip X position (side to side)
*       Right Joystick U/D: Gripper tip Y position (distance out from base center)
*       R1/R2 Buttons:      Gripper tip Z position (height from surface)
*       Left  Joystick L/R: Wrist rotate (if installed)
*       Left  Joystick U/D: Gripper/Wrist Angle
*       L1/L2 Buttons:      Gripper close/open
*       X Button:           Gripper fully open
*       Digital Pad U/D:    Speed increase/decrease
*
*
*
********************************************************************
* The Arduino connections are as follows:

* D2 - joystick select button
* 
* D24 - LCD CLK
* D25 - LCD DIO
* D53 - SD card CS
* D51 - SD card MOSI
* D50 - SD card MISO
* D52 - SD card CK
* A0 - record button
* A1 - playback button
* A2 - 
* 
************************************************
*
* Commands Type:
*
* B - BA2D_pos         Base rotation (2D)
* B - bas3D_pos        Base rotation (3D)
********************************************
* S - shl_pos          Shoulder position
* E - elb_pos          Elbow position
* W - WRro_pos         Wrist rotate
* w - wri_pos          Wrist position
* G - Gr_pos           Gripper
* 
* 
* 
*
*
*
* 
******************************************************************/

#include <Servo.h>
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


#define WRIST_ROTATE            // Uncomment if wrist rotate hardware is installed


// Arm dimensions (mm). Standard AL5D arm, but with longer arm segments
#define BASE_HGT	90.00       // Base height to X/Y plane 3.1875"
#define HUMERUS		135.00      // Shoulder-to-elbow "bone" 10.375"
#define ULNA		155.00      // Elbow-to-wrist "bone" 12.8125"
#define GRIPPER		35.00       // Gripper length, to middle of grip surface 2.875" (3.375" - 0.5")


// Arduino pin numbers for servo connections
#define BAS_SERVO_PIN	2       // Base servo 
#define SHL_SERVO_PIN	3       // Shoulder Servo
#define SHL_SERVO1_PIN	23      // Shoulder Servo1
#define ELB_SERVO_PIN	4       // Elbow Servo
#define WRI_SERVO_PIN	10      // Wrist servo
#define GRI_SERVO_PIN	11      // Gripper servo
#define WRO_SERVO_PIN	12      // Wrist rotate servo HS-485HB


// Arduino pin numbers for PS2 controller connections
#define PS2_DAT_PIN		6       // Data           [green] 
#define PS2_CMD_PIN		7       // Command        [blue]
#define PS2_ATT_PIN		8       // Attention      [purple]
#define PS2_CLK_PIN		9       // Clock          [gray]


// Arduino pin number of on-board speaker
#define SPK_PIN 5


// Define generic range limits for servos, in microseconds (us) and degrees (deg)
// Used to map range of 180 deg to 1800 us (native servo units).
// Specific per-servo/joint limits are defined below
#define SERVO_MIN_US	600
#define SERVO_MID_US	1450
#define SERVO_MAX_US	2300
#define SERVO_MIN_DEG	0.0
#define SERVO_MID_DEG	90.0
#define SERVO_MAX_DEG	180.0


// Set physical limits (in degrees) per servo/joint.
// Will vary for each servo/joint, depending on mechanical range of motion.
// The MID setting is the required servo input needed to achieve a 
// 90 degree joint angle, to allow compensation for horn misalignment
#define BAS_MIN		0.0			// Fully CCW
#define BAS_MID		90.0
#define BAS_MAX		180.0		// Fully CW
#define BAS_OFF		90.0		// Centre


#define SHL_MIN		0.0			// Max forward motion
#define SHL_MID		90.0
#define SHL_MAX		180.0		// Max rearward motion
#define SHL_OFF		90.0


#define ELB_MIN		70.0		// Max upward motion
#define ELB_MID		90.0
#define ELB_MAX		165.0		// Max downward motion
#define ELB_OFF		90.0


#define WRI_MIN		0.0			// Max downward motion
#define WRI_MID		90.0
#define WRI_MAX		180.0		// Max upward motion
#define WRI_OFF		90.0

#define GRI_MIN		25.0		// Fully open
#define GRI_MID		90.0
#define GRI_MAX		165.0		// Fully closed
#define GRI_OFF		90.0


#define WRO_MIN	0.0
#define WRO_MID	90.0
#define WRO_MAX	180.0
#define WRO_OFF	90.0


// Speed adjustment parameters
// Percentages (1.0 = 100%) - applied to all arm movements
#define SPEED_MIN		0.25
#define SPEED_MAX		1.5
#define SPEED_DEFAULT	0.5
#define SPEED_INCREMENT 0.25

// Practical navigation limit.
// Enforced on controller input, and used for CLV calculation 
// for base rotation in 2D mode. 
#define Y_MIN 100.0	// mm


// PS2 controller characteristics
#define JS_MIDPOINT		128		// Numeric value for joystick midpoint
#define JS_DEADBAND		4		// Ignore movement this close to the center position
#define JS_IK_SCALE		50.0	// Divisor for scaling JS output for IK control
#define JS_SCALE		100.0	// Divisor for scaling JS output for raw servo control
#define Z_INCREMENT		2.0		// Change in Z axis (mm) per button press
#define GR_INCREMENT	2.0		// Change in Gripper jaw opening (servo angle) per button press


// Audible feedback sounds
#define TONE_READY		1000	// Hz
#define TONE_IK_ERROR	200		// Hz
#define TONE_DURATION	100		// ms


// IK function return values
#define IK_SUCCESS		0
#define IK_ERROR		1		// Desired position not possible


// Arm parking positions
#define PARK_MIDPOINT	1		// Servos at midpoints
#define PARK_READY		2		// Arm at Ready-To-Run position
#define PARK_OFF		3		// Arm at For OFF position


// Ready-To-Run arm position. See descriptions below
// NOTE: Have the arm near this position before turning on the 
// servo power to prevent whiplash
#ifdef CYL_IK   // 2D kinematics
 #define READY_BA	BAS_MID
#else           // 3D kinematics
#define READY_X		90.0
#endif


#define READY_Y		170.0
#define READY_Z		45.0
#define READY_GRA	0.0
#define READY_GR	GRI_MID
#define READY_WRO	WRO_MID


// Global variables for arm position, and initial settings
#ifdef CYL_IK   // 2D kinematics
 float BA2D_pos = READY_BA;          // Base angle. Servo degrees - 0 is fully CCW
 float old_BA2D_pos;
#else           // 3D kinematics
float X = READY_X;                 // Left/right distance (mm) from base centerline - 0 is straight
#endif


float Y = READY_Y;                 // Distance (mm) out from base center
float Z = READY_Z;                 // Height (mm) from surface (i.e. X/Y plane)
float GA_pos = READY_GRA;          // Gripper angle. Servo degrees, relative to X/Y plane - 0 is horizontal
float Gr_pos = READY_GR;           // Gripper jaw opening. Servo degrees - midpoint is halfway open


float y_tmp, z_tmp, ga_tmp;	       // temp. variables
#ifdef CYL_IK   // 2D kinematics
 // not used
#else           // 3D kinematics
 float x_tmp;
#endif


float WRro_pos = READY_WRO;        // Wrist Rotate. Servo degrees - midpoint is horizontal
float old_WRro_pos;


float Speed = SPEED_DEFAULT;


// Pre-calculations
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;


int Bas_fb, Shl_fb, Elb_fb, Wri_fb, Wro_fb, Gri_fb;


int ly_trans, lx_trans, ry_trans, rx_trans;


static short	PS2ErrorCnt;
#define	MAXPS2ERRORCNT		5      // How many times through the loop will we go before shutting off robot?


float bas3D_pos, shl_pos, elb_pos, wri_pos;



bool fServosAttached = false;      // remember we are not attached. Could simply ask one of our servos...
bool fArmOn = false;
bool arm_move = false;             // Used to indidate whether an input occurred that can move the arm


void setDisplay(char inMode);
void setName(int i);
void writeCommand(char inCommand, int inValue);
int deg_to_us(float value);
void servo_park(int park_type);
void InitPs2(void);
void TurnArmOff(void);
void TurnArmOn(void);
void FreeServos(void);
void AttachServos(void);
void startRecord();




// PS2 Controller object
PS2X    Ps2x;


// Servo objects 
Servo   Bas_Servo;
Servo   Shl_Servo;
Servo   Shl_Servo1;
Servo   Elb_Servo;
Servo   Wri_Servo;
Servo   Wro_Servo;
Servo   Gri_Servo;


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
long int recStart;
int playbackProgram = -1;
int maxProgram = 0;



 
 
 
 
 
void TurnArmOff(void){

    
	servo_park(PARK_OFF);
	FreeServos();
	fArmOn = true;
  #ifdef DEBUG
	Serial.println(F("Arm OFF!"));
  #endif          
}




void TurnArmOn(void){

	AttachServos();
	
	// NOTE: Ensure arm is close to the desired park position before turning on servo power!
	servo_park(PARK_READY);

	fArmOn = true;
  #ifdef DEBUG
	Serial.println(F("Arm ON!"));
  #endif          
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




void ControlInput(void){

    
	Ps2x.read_gamepad();                            //read controller

	if ((Ps2x.Analog(1) & 0xf0) == 0x70) {

		PS2ErrorCnt = 0;                            // clear error counter

		if (Ps2x.ButtonPressed(PSB_START)) {		// Start Button Test 

			if (fArmOn) {			//Turn off
				TurnArmOff();
			} 
			else {					//Turn on
				TurnArmOn();
				fArmOn = true;
			}
		}
















		// Read the left and right joysticks and translate the 
		// normal range of values (0-255) to zero-centered values (-128 - 128)
		ly_trans = JS_MIDPOINT - Ps2x.Analog(PSS_LY); // Gripper/Wrist Angle
		lx_trans = Ps2x.Analog(PSS_LX) - JS_MIDPOINT; // Wrist rotate (if installed)
		ry_trans = JS_MIDPOINT - Ps2x.Analog(PSS_RY); // Gripper tip - Y position (distance out from base center)
		rx_trans = Ps2x.Analog(PSS_RX) - JS_MIDPOINT; // Gripper tip - X position or Base Position (side to side)



	#ifdef CYL_IK   // 2D kinematics
		// Base Position (in degrees)
		// Restrict to MIN/MAX range of servo
		if (abs(rx_trans) > JS_DEADBAND) {         // Gripper tip - Base Position  (side to side)
			// Multiplying by the ratio (Y_MIN/Y) is to ensure constant linear velocity
			// of the gripper as its distance from the base increases
			BA2D_pos += ((float)rx_trans / JS_SCALE * Speed * (Y_MIN/Y));
			BA2D_pos = constrain(BA2D_pos, BAS_MIN, BAS_MAX);

			if (BA2D_pos != old_BA2D_pos) {
				Bas_Servo.writeMicroseconds(deg_to_us(BA2D_pos));

				writeCommand('B', BA2D_pos);
				old_BA2D_pos = BA2D_pos;

			#ifdef DEBUG
				Serial.print(F(" Base Pos 2D: "));
				Serial.println(BA2D_pos);
				//Serial.println(F("  "));
			#endif 

				if (BA2D_pos == BAS_MIN || BA2D_pos == BAS_MAX) {
					// Provide audible feedback of reaching limit
					tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
				}
			}
		}
	#else    // 3D kinematics
		// X Position (in mm)
		// Can be positive or negative. Servo range checking in IK code
		if (abs(rx_trans) > JS_DEADBAND) {                    // Gripper tip - X Position  (side to side)
			x_tmp += ((float)rx_trans / JS_IK_SCALE * Speed);
			arm_move = true;
		}
	#endif

		// Gripper tip Y Position (in mm)
		// Must be > Y_MIN. Servo range checking in IK code
		if (abs(ry_trans) > JS_DEADBAND) {                     // Gripper tip - Y position (distance out from base center)
			y_tmp += ((float)ry_trans / JS_IK_SCALE * Speed);
			y_tmp = max(y_tmp, Y_MIN);
			arm_move = true;

			if (y_tmp == Y_MIN) {
				// Provide audible feedback of reaching limit
				tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
			}
		}

		// Z Position (in mm)
		// Must be positive. Servo range checking in IK code
		if (Ps2x.Button(PSB_R1) || Ps2x.Button(PSB_R2)) { // PSB_R1 OR PSB_R2 Test
			if (Ps2x.Button(PSB_R1)) {
				z_tmp += Z_INCREMENT * Speed;   // up
			} else {
				z_tmp -= Z_INCREMENT * Speed;   // down
			}
			z_tmp = max(z_tmp, 0);
			arm_move = true;
		}

		// Gripper/Wrist angle (in degrees) relative to horizontal
		// Can be positive or negative. Servo range checking in IK code
		if (abs(ly_trans) > JS_DEADBAND) {                                 // Gripper/Wrist Angle
			ga_tmp -= ((float)ly_trans / JS_SCALE * Speed);
			arm_move = true;
		}

		// Gripper jaw position (in degrees - determines width of jaw opening)
		// Restrict to MIN/MAX range of servo
		if (Ps2x.Button(PSB_L1) || Ps2x.Button(PSB_L2)) { // PSB_L1 OR PSB_L2 Test
			if (Ps2x.Button(PSB_L1)) {
				Gr_pos += GR_INCREMENT;   // close
			} else {
				Gr_pos -= GR_INCREMENT;   // open
			}
			Gr_pos = constrain(Gr_pos, GRI_MIN, GRI_MAX);
			Gri_Servo.writeMicroseconds(deg_to_us(Gr_pos));

			writeCommand('G', Gr_pos);

		#ifdef DEBUG
			Serial.print(F("  Gripper: "));
			Serial.println(Gr_pos);
			//Serial.println("  ");
		#endif
			if (Gr_pos == GRI_MIN || Gr_pos == GRI_MAX) {
				// Provide audible feedback of reaching limit
				tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
			}
		}

		// Fully open gripper
		if (Ps2x.ButtonPressed(PSB_CROSS)) { // PSB_CROSS (X) Test
			Gr_pos = GRI_MIN;
			Gri_Servo.writeMicroseconds(deg_to_us(Gr_pos));

			writeCommand('G', Gr_pos);

		#ifdef DEBUG
			Serial.print(F("  GRI_MIN: "));
			Serial.println(Gr_pos);
			//Serial.println(F("  "));
		#endif
		}

		// Speed increase/decrease
		if (Ps2x.ButtonPressed(PSB_PAD_UP) || Ps2x.ButtonPressed(PSB_PAD_DOWN)) {  // PAD_UP OR PAD_DOWN Test
			if (Ps2x.ButtonPressed(PSB_PAD_UP)) {
				Speed += SPEED_INCREMENT;   // increase speed
			} else {
				Speed -= SPEED_INCREMENT;   // decrease speed
			}
			// Constrain to limits
			Speed = constrain(Speed, SPEED_MIN, SPEED_MAX);
		#ifdef DEBUG	
			Serial.print(F("  Speed: "));
			Serial.println(Speed);
			//Serial.println(F("  "));
		#endif
			// Audible feedback
			tone(SPK_PIN, (TONE_READY * Speed), TONE_DURATION);
		}

	
		// Wrist rotate (in degrees)
		// Restrict to MIN/MAX range of servo
		if (abs(lx_trans) > JS_DEADBAND) {                             // Wrist rotate (if installed)
			WRro_pos += ((float)lx_trans / JS_SCALE * Speed);
			WRro_pos = constrain(WRro_pos, WRO_MIN, WRO_MAX);

			if (WRro_pos != old_WRro_pos) {
				Wro_Servo.writeMicroseconds(deg_to_us(WRro_pos));

				writeCommand('W', WRro_pos);
				old_WRro_pos = WRro_pos;

			#ifdef DEBUG
				Serial.print(F("  WR_RO: "));
				Serial.println(WRro_pos);
				//Serial.println(F("  "));
			#endif
				if (WRro_pos == WRO_MIN || WRro_pos == WRO_MAX) {
					// Provide audible feedback of reaching limit
					tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
				}
			}
		}












	} // end, if((Ps2x.Analog(1) & 0xf0) == 0x70), read PS2 controller 
	else {
		if (PS2ErrorCnt < MAXPS2ERRORCNT)
			PS2ErrorCnt++;						// Increment the error count and if to many errors, turn OFF the Arm.
		else if (fArmOn){
			TurnArmOff();
			
		}  
		Ps2x.reconfig_gamepad();
	}

 }





 void writeCommand(char inCommand, int inValue){

	if (mode == 'R') {

	#ifdef DEBUG
		Serial.print(F("WriteCommand: "));
		Serial.print(inCommand);
		Serial.println(inValue);
	#endif

		//myFile.println("time,command,value");
		if (myFile.open(name, O_RDWR | O_CREAT | O_AT_END)) {
			myFile.print(millis() - recStart);    
			myFile.print(",");
			myFile.print(inCommand);
			myFile.print(",");
			myFile.println(inValue);
			myFile.close();
		}
	}
}



void startRecord(){

	// Look at the SD card and figure out the new file name to use.
	// Then initialize the file.
	if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
		Serial.println (F("Cannot access local SD card."));
	}
	else {
		// Set the output file name
		for (uint8_t i = 0; i < 1000; i++) {
			setName(i);
			if (sd.exists(name)) continue;
			playbackProgram = i;
			break;
		}
		Serial.print(F("SD card output file is "));
		Serial.println(name);
	}
	if (!myFile.open(name, O_RDWR | O_CREAT | O_AT_END)) {
		Serial.println (F("Opening of local SD card FAILED."));
	}
	else {
	 #ifdef DEBUG
		Serial.println(F("Lighting up display"));
	 #endif DEBUG	
		
		// myFile.println("time,command,value");    
		// Need to first write the initial (current) position
		// of the arm so it can be initialized upon playback.
		#ifdef CYL_IK   // 2D kinematics
			myFile.print("0,B,"); myFile.println(BA2D_pos);
		#else
			myFile.print("0,B,"); myFile.println(bas3D_pos);
		#endif
		myFile.print("0,S,"); myFile.println(shl_pos);
		myFile.print("0,E,"); myFile.println(elb_pos);
		myFile.print("0,w,"); myFile.println(wri_pos);
		myFile.print("0,W,"); myFile.println(WRro_pos);
		myFile.print("0,C,"); myFile.println(Gr_pos);
		myFile.close();
		
		mode = 'R';

		recStart = millis();
		setDisplay('R');
	}
}



void startSelect(){

	if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
		Serial.println (F("Cannot access local SD card."));
	}

	mode = 'S';

	// Find maximum program number
	for (uint8_t i = 0; i < 1000; i++) {
		setName(i);
		if (sd.exists(name)) continue;
		maxProgram = i-1;
		break;
	}

	// Get program to playback
	if (playbackProgram == -1 ) {
		// Find latest recorded program and use that.
		for (uint8_t i = 0; i < 1000; i++) {
			setName(i);
			if (sd.exists(name)) continue;
			playbackProgram = i-1;
			break;
		}
	}

	setName(playbackProgram);
	setDisplay('S');
	delay(500);
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
}



void startPlayback(int in_playbackProgram) {


	long int commandTime;
	char commandType;
	int commandParam;
	char c1, c2;   // commas
	long int fileLine = 0;
	long int commandTimePrev = 0;
	

	setName(in_playbackProgram);
	setDisplay('P');
	mode = 'P';

	delay(1000);
	
#ifdef DEBUG
	Serial.print(F("Playing back file "));
	Serial.println(name);
#endif


	ifstream sdin(name);

	if (sdin.is_open()) {
		while (sdin >> commandTime >> c1 >> commandType >> c2 >> commandParam) {
			
			if (c1 != ',' || c2 != ',') 
				continue;
			
			fileLine++;
			
			switch (commandType) {
				case 'B': // Base rotation
					Bas_Servo.write(commandParam);
					break;
				case 'S': // Shoulder position
					Shl_Servo.write(commandParam);
					Shl_Servo1.write(185-commandParam)
					break;
				case 'E': //Elbow position
					Elb_Servo.write(commandParam);
					break;					
				case 'W: // Wrist position
					Wri_Servo.write(commandParam);
					break;
				case 'w: // Wrist rotate
					Wro_Servo.write(commandParam);
					break;
				case 'G': // Gripper
					Gri_Servo.write(commandParam);
					break;				
				}
				if (commandTime == 0) {
					delay(500);
			}
			else {
				
				delay(commandTime - commandTimePrev);
				
				commandTimePrev = commandTime;
			}
		}
	}
	mode = 'N';
	for(int k = 0; k < 4; k++) data[k] = 0;
	display.setSegments(data);
}

































void setup(){

#ifdef DEBUG
	Serial.begin(115200);
#endif

	InitPs2();
	

#ifdef DEBUG
	Serial.println(F("Start"));
#endif

	// Initialize the display
	display.setBrightness(7); // Turn on
	for(int k = 0; k < 4; k++) data[k] = 0;
	display.setSegments(data);



	delay(500);
	// Sound tone to indicate it's safe to turn on servo power
	tone(SPK_PIN, TONE_READY, TONE_DURATION);
	delay(TONE_DURATION * 2);
	tone(SPK_PIN, TONE_READY, TONE_DURATION);

	// mTickRecorderTime = millis(); 

}








void loop() {

	// Store desired position in tmp variables until confirmed by set_arm() logic
#ifdef CYL_IK   // 2D kinematics
	// not used
#else           // 3D kinematics
	x_tmp = X;
#endif
	y_tmp = Y;
	z_tmp = Z;
	ga_tmp = GA_pos;


	ControlInput();
	
	//Ps2x.read_gamepad();        //read controller

	//// Read the left and right joysticks and translate the 
	//// normal range of values (0-255) to zero-centered values (-128 - 128)
	//int ly_trans = JS_MIDPOINT - Ps2x.Analog(PSS_LY);
	//int lx_trans = Ps2x.Analog(PSS_LX) - JS_MIDPOINT;
	//int ry_trans = JS_MIDPOINT - Ps2x.Analog(PSS_RY);
	//int rx_trans = Ps2x.Analog(PSS_RX) - JS_MIDPOINT;




	// Only perform IK calculations if arm motion is needed.
	if (arm_move) {
#ifdef CYL_IK   // 2D kinematics
		if (set_arm(0, y_tmp, z_tmp, ga_tmp) == IK_SUCCESS) {
			// If the arm was positioned successfully, record
			// the new vales. Otherwise, ignore them.
			Y = y_tmp;
			Z = z_tmp;
			GA_pos = ga_tmp;
		} else {
			// Sound tone for audible feedback of error
			tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
		}
#else           // 3D kinematics
		if (set_arm(x_tmp, y_tmp, z_tmp, ga_tmp) == IK_SUCCESS) {
			// If the arm was positioned successfully, record
			// the new vales. Otherwise, ignore them.
			X = x_tmp;
			Y = y_tmp;
			Z = z_tmp;
			GA_pos = ga_tmp;
		} else {
			// Sound tone for audible feedback of error
			tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
		}
#endif

		// Reset the flag
		arm_move = false;
	}

	delay(10);
} // end loop










// Arm positioning routine utilizing Inverse Kinematics.
// Z is height, Y is distance from base center out, X is side to side. Y, Z can only be positive.
// Input dimensions are for the gripper, just short of its tip, where it grabs things.
// return SBase, sShoulder, sElbow, sWrist
// If resulting arm position is physically unreachable, return error code.

int set_arm(float x, float y, float z, float grip_angle_d) {

	//grip angle in radians for use in calculations
	float grip_angle_r = radians(grip_angle_d);    

	// Base angle and radial distance from x,y coordinates
	float bas_angle_r = atan2(x, y);

 #ifdef CYL_IK   // 2D kinematics

	// We are in cylindrical mode, probably simply set y` to the y we passed in...
	//y = y;

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
	if (isnan(shl_angle_r) || isinf(shl_angle_r))
		return IK_ERROR;


	// Elbow angle
	float elb_angle_r = acos((hum_sq + uln_sq - s_w) / (2 * HUMERUS * ULNA));

	// If result is NAN or Infinity, the desired arm position is not possible
	if (isnan(elb_angle_r) || isinf(elb_angle_r))
		return IK_ERROR;

	float elb_angle_d = degrees(elb_angle_r);
	float elb_angle_dn = -(180.0 - elb_angle_d);

	// Gripper/Wrist angle
	float wri_angle_d = (grip_angle_d - elb_angle_dn) - shl_angle_d;


	// Calculate servo angles
	// Calc relative to servo midpoint to allow compensation for servo alignment
	bas3D_pos = BAS_MID + degrees(bas_angle_r);
	shl_pos = SHL_MID + (shl_angle_d - 90.0);
	elb_pos = ELB_MID - (elb_angle_d - 90.0);
	wri_pos = WRI_MID + wri_angle_d;

	// If any servo ranges are exceeded, return an error
	if (bas3D_pos < BAS_MIN || bas3D_pos > BAS_MAX || shl_pos < SHL_MIN || shl_pos > SHL_MAX || elb_pos < ELB_MIN || elb_pos > ELB_MAX || wri_pos < WRI_MIN || wri_pos > WRI_MAX)
		return IK_ERROR;

	// Position the servos
 #ifdef CYL_IK   // 2D kinematics
	// Do not control base servo
 #else           // 3D kinematics
	Bas_Servo.writeMicroseconds(deg_to_us(bas3D_pos));
	writeCommand('B', bas3D_pos);
 #endif

	Shl_Servo.writeMicroseconds(deg_to_us(shl_pos));
	Shl_Servo1.writeMicroseconds(deg_to_us(185-shl_pos));
	Elb_Servo.writeMicroseconds(deg_to_us(elb_pos));
	Wri_Servo.writeMicroseconds(deg_to_us(wri_pos));

	writeCommand('S', shl_pos);
	writeCommand('E', elb_pos);
	writeCommand('w', wri_pos);
	
	Shl_fb = Shl_Servo.read();
	Elb_fb = Elb_Servo.read();
	Wri_fb = Wri_Servo.read();
	//Wro_fb = Wro_Servo.read();
	//Gri_fb = Gri_Servo.read();

#ifdef DEBUG
 #ifndef CYL_IK   // 3D kinematics
	Serial.print("X: ");
	Serial.print(x);
 #endif
	Serial.print("  Y: ");
	Serial.print(y);
	Serial.print("  Z: ");
	Serial.print(z);
	Serial.print("  GA_pos: ");
	Serial.print(grip_angle_d);
	Serial.println();

 #ifndef CYL_IK   // 3D kinematics
	Bas_fb = Bas_Servo.read();

	Serial.print("  Base Pos 3D: ");
	Serial.print(bas3D_pos);
	Serial.print("  Base Fb: ");
	Serial.print(Bas_fb);
 #endif
	Serial.print("  Shld Pos: ");
	Serial.print(shl_pos);
	Serial.print("  Shld Fb: ");
	Serial.print(Shl_fb);
	Serial.print("  Elbw Pos: ");
	Serial.print(elb_pos);
	Serial.print("  Elbw Fb: ");
	Serial.print(Elb_fb);
	Serial.print("  Wrst Pos: ");
	Serial.print(wri_pos);
	Serial.print("  Wri Fb: ");
	Serial.print(Wri_fb);
//	Serial.print("  WrRO Fb: ");
//	Serial.print(Wro_fb);
//	Serial.print("  Grip Fb: ");
//	Serial.print(Gri_fb);
	Serial.println();

 #ifndef CYL_IK   // 3D kinematics
	Serial.print("bas_angle_d: ");
	Serial.print(degrees(bas_angle_r));  
 #endif
	Serial.print("  shl_angle_d: ");
	Serial.print(shl_angle_d);  
	Serial.print("  elb_angle_d: ");
	Serial.print(elb_angle_d);
	Serial.print("  wri_angle_d: ");
	Serial.println(wri_angle_d);
	Serial.println();



#endif

	return IK_SUCCESS;
} // set_arm()





// Move servos to parking position
void servo_park(int park_type) {

	switch (park_type) {

		// All servos at midpoint
		case PARK_MIDPOINT:
	#ifdef DEBUG
			Serial.println("PARK_MIDPOINT:");
	#endif
			Bas_Servo.writeMicroseconds(deg_to_us(BAS_MID));
			Shl_Servo.writeMicroseconds(deg_to_us(SHL_MID));
			Shl_Servo1.writeMicroseconds(deg_to_us(185-SHL_MID));
			Elb_Servo.writeMicroseconds(deg_to_us(ELB_MID));
			Wri_Servo.writeMicroseconds(deg_to_us(WRI_MID));
			Gri_Servo.writeMicroseconds(deg_to_us(GRI_MID));
			Wro_Servo.writeMicroseconds(deg_to_us(WRO_MID));
			break;

			// Ready-To-Run position
		case PARK_READY:
	#ifdef CYL_IK   // 2D kinematics
		#ifdef DEBUG
			Serial.println("PARK_READY:");
			Serial.print("  Base: ");
			Serial.println(READY_BA);
		#endif
			set_arm(0.0, READY_Y, READY_Z, READY_GRA); //0; 170; 45; 0
			Bas_Servo.writeMicroseconds(deg_to_us(READY_BA)); //90.0
	#else           // 3D kinematics
			set_arm(READY_X, READY_Y, READY_Z, READY_GrA); //0; 170; 45; 0
	#endif
			Gri_Servo.writeMicroseconds(deg_to_us(READY_GR)); //90
		#ifdef DEBUG
			Serial.print("  Grip: ");
			Serial.println(READY_GR);
		#endif

			Wro_Servo.writeMicroseconds(deg_to_us(READY_WRO));
		#ifdef DEBUG
			Serial.print("  WR_RO: ");
			Serial.println(READY_WRO);
		#endif
			break;
		// All servos at midpoint
		case PARK_OFF:
	#ifdef DEBUG
			Serial.println("PARK_OFF:");
	#endif
			Bas_Servo.writeMicroseconds(deg_to_us(BAS_OFF));
			Shl_Servo.writeMicroseconds(deg_to_us(SHL_OFF));
			Shl_Servo1.writeMicroseconds(deg_to_us(185-SHL_OFF));
			Elb_Servo.writeMicroseconds(deg_to_us(ELB_OFF));
			Wri_Servo.writeMicroseconds(deg_to_us(WRI_OFF));
			Gri_Servo.writeMicroseconds(deg_to_us(GRI_OFF));
			Wro_Servo.writeMicroseconds(deg_to_us(WRO_OFF));
			break;

	}

}



// The Arduino Servo library .write() function accepts 'int' degrees, meaning
// maximum servo positioning resolution is whole degrees. Servos are capable 
// of roughly 2x that resolution via direct microsecond control.
//
// This function converts 'float' (i.e. decimal) degrees to corresponding 
// servo microseconds to take advantage of this extra resolution.
int deg_to_us(float value) {

	// Apply basic constraints
	if (value < SERVO_MIN_DEG) value = SERVO_MIN_DEG;
	if (value > SERVO_MAX_DEG) value = SERVO_MAX_DEG;

	// Map degrees to microseconds, and round the result to a whole number
	return(round(map_float(value, SERVO_MIN_DEG, SERVO_MAX_DEG, (float)SERVO_MIN_US, (float)SERVO_MAX_US)));      
}



// Same logic as native map() function, just operates on float instead of long
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {

	return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}



void AttachServos(void) {

	if (!fServosAttached) {

		// Attach to the servos and specify range limits
		Bas_Servo.attach(BAS_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
		Shl_Servo.attach(SHL_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
		Shl_Servo1.attach(SHL_SERVO1_PIN, SERVO_MIN_US, SERVO_MAX_US);
		Elb_Servo.attach(ELB_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
		Wri_Servo.attach(WRI_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
		Gri_Servo.attach(GRI_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
		Wro_Servo.attach(WRO_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

		fServosAttached = true;
	  #ifdef DEBUG
		Serial.println(F("Servos Attached"));
	  #endif
	}
}



void FreeServos(void){

	if(fServosAttached) {
		// Detach the servos 
		Bas_Servo.detach();
		Shl_Servo.detach();
		Shl_Servo1.detach();
		Elb_Servo.detach();
		Wri_Servo.detach();
		Gri_Servo.detach();
		Wro_Servo.detach();

		fServosAttached = false;

	  #ifdef DEBUG
		Serial.println(F("Servos Free"));
	  #endif
	}
}









