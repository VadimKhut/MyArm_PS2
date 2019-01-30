/*
===================================================================================

Hardware setup: PS2 version

***********************************************************************************
--- A-POD -----
***********************************************************************************

[Common Controls]
- Start,                Turn On/Off Hexapod
- D-Pad Up,             Body, up 10 mm
- D-Pad Down,           Body, down 10 mm

- X-Cross,              Toggle: Walk Mode,Translate Mode, GP Player Mode, Single Leg Mode
- O-Circle,             Toggle Rotate mode
- Triangle,             Move body to 40 mm from the ground(walk position)
                         and back to the ground - Stand Up, Sit Down 
- Square,               Toggle Balance mode(Balance mode: lifts and lowers
                         body along with moving legs.


[Rotate Controls]
- O-Circle,             Toggle Rotate mode
- Select,               Switch rotate function (Head tracking, fixed head, head only)
- Left Joystick,        L/R: Y rotate; U/D: Z Translate (Shift) 
- Right Joystick,       L/R: Z rotate; U/D: X Rotate
- R1,                   Moves the Center Point of Rotation to the Head (Hold button)
- R2,                   Moves the Center Point of Rotation to the Tail (Hold button)
- D-Pad Left,           Slower but more accurate indirect control
- D-Pad Right,          Faster respons on indirect control
- L3,                   Reset body rotations, click the left gimbal for neutralizing the IndDualShock


[Walk Controls]
- X,                    Toggle: Walk Mode,Translate Mode, GP Player Mode, Single Leg Mode
- Select,               Switch gaits

Walk mode 0:
- Left Joystick,        U/D: Walk; L/R: Strafe
- Right Joystick,       L/R: Rotate

Walk mode 1:
- Left Joystick,        Disable
- Right Joystick,       U/D: Walk; L/R: Rotate 

Walk mode 3:
- Left Joystick,        U/D: Walk; L/R: Strafe
- Right Joystick,       L/R: Rotate + BodyRot.Z; U/D: + BodyRot.X 

- R1,                   Toggle Double gait travel Speed
- R2,                   Toggle Double gait travel Length
- R3,                   Switch between Walk mode 0(default), 1, 2
- D-Pad Left,           Decrease speed with 50mS
- D-Pad Right,          Increase speed with 50mS


[Single leg Controls]
- X,                    Toggle: Walk Mode,Translate Mode, GP Player Mode, Single Leg Mode
- Select,               Switch legs
- Left Joystick,        Move Leg X/Z (relative)
- Right Joystick,       Move Leg Y (absolute)
- R2,                   Hold/release leg position


[Shift Controls]
- Left Joystick         Shift body X/Z
- Right Joystick        Shift body Y and rotate body Y


[GP Player Controls]
- X,                    Toggle: Walk Mode,Translate Mode, GP Player Mode, Single Leg Mode
- Select,               Switch Sequences
- R2,                   Start Sequence
- D-Pad Up,             Increase speed multiplier (+)
- D-Pad Down,           Decrease speed multiplier (-)
- D-Pad Left,           Increase pause between steps Seq. (+)
- D-Pad Right,          Decrease pause between steps Seq. (-)
- L1,                   Set pause to default
- L2,                   Set speed multiplier to default


[Sound ISD and Attack Controls]  
- L3,                   Saund ON/OFF
                        Attack ON/OFF
===================================================================================================
LX/RX=0   - Left
LX/RX=128 - Centre
LX/RX=255 - Right

LY/RY=0   - Up
LY/RY=128 - Centre
LY/RY=255 - Down
===================================================================================================
*/
#include <PS2X_lib.h>

//--------------------------------------------------------------------
#define	DEBUG_PS2_INPUT
#define	MONITOR_FSR
#define	DEBUG_ULTRASONIC
#define	DEBUG_INDERECT_DS

//--------------------------------------------------------------------
//[CONSTANTS]
#define	WALKMODE			0
#define	TRANSLATEMODE		1
#define	ROTATEMODE			2
#define	SINGLELEGMODE		3
#define	GPPLAYERMODE		4

#define	cTravelDeadZone		4				// The deadzone for the analog input from the remote
#define	MAXPS2ERRORCNT		5				// How many times through the loop will we go before shutting off robot?


#define	SEQ_CONT_START		0				// Seq Continue Start 
#define	SEQ_CONT_END		7				// Seq Continue End 
#define	SEQ_MAX				7				// Seq Max 


#define	SEQ_CONT1		SEQ_CONT_START					// Sequence for contninue play 1

#define	L_SEQ_CONT1		SEQ_CONT_END - SEQ_CONT_START	// length Sequence contninue 0 (7)

//#define	L_SEQ_CONT1	SEQ_CONT2 - SEQ_CONT1		// length Sequence contninue 1
//#define	SEQ_CONT2	11							// Sequence for contninue play 2
//#define	L_SEQ_CONT2	SEQ_CONT3 - SEQ_CONT2		// length Sequence contninue 2
//#define	SEQ_CONT3	16							// Sequence for contninue play 3
//#define	L_SEQ_CONT3	SEQ_CONT4 - SEQ_CONT3		// length Sequence contninue 3
//#define	SEQ_CONT4	21							// Sequence for contninue play 4
//#define	L_SEQ_CONT4	SEQ_CONT5 - SEQ_CONT4		// length Sequence contninue 4
//#define	SEQ_CONT5	26							// Sequence for contninue play 5
//#define	L_SEQ_CONT5	SEQ_CONT6 - SEQ_CONT5		// length Sequence contninue 5
//#define	SEQ_CONT6	35							// Sequence for contninue play 5
//#define	L_SEQ_CONT2	SEQ_CONT_END - SEQ_CONT2	// length Sequence contninue 2

#define  GP_SM_STEP		20				// value for step to change Speed multiplier 
#define  GP_PA_STEP		20				// value for step to change Pause multiplier  
#define  GP_PA_STEP_PR	10				// value in % for step to change Pause multiplier  



#ifndef  MAX_BODY_Y
	#define MAX_BODY_Y 100
#endif


//Define Variables:
static short	g_BodyYOffset; 
static short	g_sPS2ErrorCnt;
static short	g_BodyYShift;
static byte		ControlMode;
static bool		DoubleHeightOn;
static bool		DoubleTravelOn;
static byte		WalkMethod;
byte	GPSeq;			// Number of the sequence
short	GPSMContr;		// GPSM value for calculate speed Multiplyer
word	NewGPSM;
short	GPPauseContr;	// GPPA value for calculate pause Multiplyer
short	NewGPPA;

//mandible variables:
short	CtrlMoveInp;
short	CtrlMoveOut;
byte	CtrlDivider;
short	HeadPanOut1;
short	HeadTiltOut1;
short	HeadPanInput1;

byte	RotateFunction;
boolean	FullHeadRange;		// Set to 1 for full range on head rotation
short	NeutralStick[4];	// For use with indirect gimbal control, set high for reseting the gimbal. Controlled by L3.
byte	IDSdivFactor;		// This factor determine if the indirect control is fast (low value) or accurate and slow (high value)

byte	Xbutton_Mode;
short	IndDualShock[4];
boolean	fPSB_R1_R2_10s;
byte	GPSeqFile;
byte	GPSeqFile_old;
unsigned long	ulTimePSB_R1_R2;


void	TurnRobotOff(void);
void	PrintHexOff(void);
void	IndirectDualShock(void);
void	HeadControl(short HeadPanInput1, short HeadTiltInput1);
void	BranchRotateFunction(byte rf);
short	SmoothControl (short CtrlMoveInp, short CtrlMoveOut, short CtrlDivider);

extern void DEBUGPrintMode(void);
extern boolean DAP_PlaySound(const char *name, char check_isPlAY);
extern void PWM_EyesOff(void);


//------------------------------------------------------------------------------

PS2X ps2x; // create PS2 controller

InputController g_InputController; // create Input controller


//-------------------------------------------------------------------------------
//The function to initialize the input controller, which in this case is the 
//PS2 controller process any commands.
//-------------------------------------------------------------------------------
int InputController::Init(void){
	int error = 0;
	byte type = 0;

	// Setup gamepad (clock, command, attention, data) pins
	error = ps2x.config_gamepad(PS2_CLK_p, PS2_CMD_p, PS2_SEL_p, PS2_DAT_p);

#ifdef DEBUG
	DBGSerial.print(F("PS2 : "));
	DBGSerial.println(error, DEC);
#endif

	if(error){
#ifdef DEBUG
		Serial.println(F("PS2 check ERROR!!"));
#endif   
		MSound (2, 40, 2500, 40, 2500);    //[40\5000,40\5000]
	}   

	type = ps2x.readType(); 
	switch(type) {
	case 0:
#ifdef DEBUG
		Serial.println(F("Unknown Controller type"));
#endif
		break;
	case 1:
	#ifdef DEBUG
		Serial.println(F("DualShock Controller Found"));
	#endif
		break;
	}


	// Initialization varuables
	g_BodyYOffset = 0;						// ### (65)
	g_BodyYShift = 0;
	g_sPS2ErrorCnt = 0;						// error count

	ControlMode = WALKMODE;
	Xbutton_Mode=1;
	DoubleHeightOn = false;
	DoubleTravelOn = false;
	WalkMethod = 0;
	g_InControlState.SpeedControl = 100;
	GPSMContr = 100;
	NewGPSM = 0;
	GPPauseContr = 0;
	NewGPPA = 0;
	nSound = 0;								// num. Sound to play (=0 disable)
	fPSB_R1_R2_10s = false;
	GPSeqFile =0;
	GPSeqFile_old =0;

#ifdef DEBUG
	DEBUGPrintMode();
#endif

	IDSdivFactor = 7;						// Set Default value

	return error;
}


//-------------------------------------------------------------------------------
// Function to read inputs from the PS2 and then process any commands.
//-------------------------------------------------------------------------------
void InputController::ControlInput(void){

	// Then try to receive a packet of information from the PS2.
	ps2x.read_gamepad();				// read controller and set large motor to spin at 'vibrate' speed

	if ((ps2x.Analog(1) & 0xf0) == 0x70) {

#ifdef DEBUG
	#ifdef DEBUG_PS2_INPUT
			if (g_fDebugOutput) {
				if (g_fDebugPS2Input) {
					// DEBUG PS2 INPUT
					DBGSerial.print(F("PS2 Input: "));
					DBGSerial.print(ps2x.ButtonDataByte(), HEX);
					DBGSerial.print(F(":"));
					DBGSerial.print(ps2x.Analog(PSS_LX), DEC);
					DBGSerial.print(F(" "));
					DBGSerial.print(ps2x.Analog(PSS_LY), DEC);
					DBGSerial.print(F(" "));
					DBGSerial.print(ps2x.Analog(PSS_RX), DEC);
					DBGSerial.print(F(" "));
					DBGSerial.println(ps2x.Analog(PSS_RY), DEC);
				}
			}  
	#endif
#endif
		g_sPS2ErrorCnt = 0;							// clear error counter

		if (ps2x.ButtonPressed(PSB_START)) {		// Start Button Test 
			if (g_InControlState.fHexOn) {			//Turn off
				TurnRobotOff();
				PrintHexOff();
			} 
			else {									//Turn on
				g_InControlState.fHexOn = true;
				fAdjustLegPositions = true;
				g_InControlState.fFreeServos = false;

			}
		}

		//**[GPPlayer functions]*******************************************
		if (g_InControlState.fHexOn && ControlMode == GPPLAYERMODE) {	//(g_InControlState.fHexOn == true && ControlMode == GPPLAYERMODE)
			if (g_ServoDriver.FIsGPSeqActive() ) {
				//-Increase speed multiplier (+)
				if (ps2x.Button(PSB_PAD_UP)){
					NewGPSM = NewGPSM + 2;
					if(NewGPSM > GP_SM_STEP){
						MSound(1, 50, 6000);							// [50\12000] 
						GPSMContr = GPSMContr + GP_SM_STEP;				// (GPSMContr + 20)
						GPSMContr = min(GPSMContr, 200);
						NewGPSM = 0;
						g_ServoDriver.GPSetSpeedMultiplyer(GPSMContr);
					}    
				}  
				//-Decrease speed multiplier (-)
				if (ps2x.Button(PSB_PAD_DOWN)){
					NewGPSM = NewGPSM + 2;
					if(NewGPSM > GP_SM_STEP){
						MSound(1, 50, 6000);							// [50\12000] 
						GPSMContr = GPSMContr - GP_SM_STEP;				// (GPSMContr - 20)
						GPSMContr = max(GPSMContr, -200);
						NewGPSM = 0;
						g_ServoDriver.GPSetSpeedMultiplyer(GPSMContr);
					}    
				}  
				//-Increase pause in % between steps Seq. <--(+)
				if (ps2x.Button(PSB_PAD_LEFT)){
					NewGPPA = NewGPPA + 2;
					if(NewGPPA > GP_PA_STEP){							// 20
						MSound(1, 50, 6000);							// [50\12000] 
						GPPauseContr = GPPauseContr + GP_PA_STEP_PR;	// 10%
						GPPauseContr = min(GPPauseContr, 200);
						NewGPPA = 0;
						g_ServoDriver.GPSetPauseOffset(GPPauseContr);
					}    
				}  
				//-Decrease pause in % between steps Seq. (-)--> 
				if (ps2x.Button(PSB_PAD_RIGHT)){
					NewGPPA = NewGPPA + 2;
					if(NewGPPA > GP_PA_STEP){							// 20
						MSound(1, 50, 6000);							// [50\12000] 
						GPPauseContr = GPPauseContr - GP_PA_STEP_PR;	// 10%
						GPPauseContr = max(GPPauseContr, -200);
						NewGPPA = 0;
						g_ServoDriver.GPSetPauseOffset(GPPauseContr);
					}    
				}  
				// Set pause to default
				if (ps2x.Button(PSB_L1)) {								// L1 Button Test
					MSound(1, 50, 6000);								// [50\12000] 
					GPPauseContr = 0;
					g_ServoDriver.GPSetPauseOffset(GPPauseContr);
				}

				// Set speed multiplier to default
				if (ps2x.Button(PSB_L2)) {								// L2 Button Test
					MSound(1, 50, 6000);								// [50\12000] 
					GPSMContr = 100;
					g_ServoDriver.GPSetSpeedMultiplyer(GPSMContr);
				}
			}  // (!g_ServoDriver.FIsGPSeqActive() )

			//-Switch between sequences
			if (ps2x.ButtonPressed(PSB_SELECT)) {						// Select Button Test
				if (!g_ServoDriver.FIsGPSeqActive() ) {
					MSound(1, 50, 6000);								// [50\12000] 
					GPSeq = 7;     // seq = 7
					//g_ServoDriver.iGPSeqCont = 0;
					g_ServoDriver.GPSeqContStCYC = 1;
					g_ServoDriver.NbSeqCont = L_SEQ_CONT1;				// (7)
					nSound = 41;

					//			if (GPSeq < SEQ_MAX) {						//Max sequence (8)
					//
					//				switch (GPSeq){
					//					case SEQ_CONT1:							// (5)
					//						GPSeq = SEQ_CONT1 + L_SEQ_CONT1;	// next seq 5+6=11
					//						g_ServoDriver.iGPSeqCont = 0;
					//						g_ServoDriver.GPSeqContStCYC = 1;
					//						g_ServoDriver.NbSeqCont = L_SEQ_CONT1;
					//						nSound = 41;
					//					break;
					//					case SEQ_CONT2:							// (11)
					//						GPSeq = SEQ_CONT2 + L_SEQ_CONT2;	// next seq 11+5=16
					//						g_ServoDriver.iGPSeqCont = 0;
					//						g_ServoDriver.GPSeqContStCYC = 1;
					//						g_ServoDriver.NbSeqCont = L_SEQ_CONT2;
					//						nSound = 0;
					//					break;
					////				case SEQ_CONT3:
					////					GPSeq = SEQ_CONT3 + L_SEQ_CONT3;
					////					g_ServoDriver.iGPSeqCont = 0;
					////					g_ServoDriver.GPSeqContStCYC = 1;
					////					g_ServoDriver.NbSeqCont = L_SEQ_CONT3;
					////					nSound = 0;
					////				break;
					//					default:
					//					GPSeq = GPSeq + 1;
					//					break;  
					//				}  
					//			} 
					//			else {
					//				MSound(2, 50, 2000, 50, 2250);				// [50\4000, 50\4500]
					//				GPSeq=0;
					//			}
				}
			}




			//-Start Sequence
			if (ps2x.ButtonPressed(PSB_R2)){				// R2 Button Test
				if (!g_ServoDriver.FIsGPSeqActive() ) {		// _fGPActive == false
					//		if(GPSeq >= SEQ_CONT_START && GPSeq <= SEQ_CONT_END){
					MSound(1, 50, 6000);											// [50\12000] 
					g_ServoDriver.GPSeqContSt = GPSeq - g_ServoDriver.NbSeqCont;	// 7-7=0
					g_ServoDriver.GPStartSeq(g_ServoDriver.GPSeqContSt);
					g_ServoDriver.fGPSeqCont = true;
					//		}  
					//		else{  
					//				g_ServoDriver.GPStartSeq(GPSeq);
					//				g_ServoDriver.fGPSeqCont = false;
					//			}  
					//		if (F_SoundEnable && nSound != 0)
					//				ISD_PlayFromADR(nSound, IGNORE_EOM);		// Play sound
				}
				else { // _fGPActive == true
					g_ServoDriver.GPStartSeq(0xff);				// Abort sequence
					MSound (2, 50, 2000, 50, 2000);				// [50\4000, 50\4500]
				}
			}  
		} // end (g_InControlState.fHexOn == true && ControlMode == GPPLAYERMODE)

		if (g_InControlState.fHexOn && !g_ServoDriver.FIsGPSeqActive()) {	// fHexOn == true && _fGPActive == false

			//** [SWITCH MODES]********************************************************
			//--Switch Balance mode On/Off 
			if (ps2x.ButtonPressed(PSB_SQUARE)) {				// Square Button Test
				g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
				if (g_InControlState.BalanceMode) {
					MSound(1, 250, 1500);						// [250\3000]
				} 
				else {
					MSound(2, 100, 2000, 50, 4000);				// [100\4000, 50\8000]
				}
			#ifdef DEBUG
				DEBUGPrintMode();
			#endif
			}


			//--Rotate mode
			if (ps2x.ButtonPressed(PSB_CIRCLE)) {				// O - Circle Button Test
				MSound(1, 50, 2000);							// [50\4000]
				if (ControlMode != ROTATEMODE){
					ControlMode = ROTATEMODE;
					Xbutton_Mode=0;
				}  
				else {
					if (g_InControlState.SelectedLeg == 255){ 
						ControlMode = WALKMODE;
						Xbutton_Mode=1;
					}  
					else
						ControlMode = SINGLELEGMODE;
				}
			#ifdef DEBUG
				DEBUGPrintMode();
			#endif
			}



			//-- GP Player Mode OR Translate Mode OR Single Leg Mode OR Walk Mode
			if (ps2x.ButtonPressed(PSB_CROSS)) {				// X - Cross Button Test
				MSound(1, 50, 2000);							// [50\4000]

				switch (Xbutton_Mode){
					//-Walk Mode
				case 0:
					ControlMode=WALKMODE;
					g_InControlState.SelectedLeg = 255;
					Xbutton_Mode=1;
					break;
					//-Translate mode (Shift)
				case 1:
					ControlMode = TRANSLATEMODE;
					Xbutton_Mode=2;
					break;
					//-Walk Mode
				case 2:
					ControlMode=WALKMODE;
					g_InControlState.SelectedLeg = 255;
					Xbutton_Mode=3;
					break;
					//-GP Player Mode
				case 3:
					ControlMode = GPPLAYERMODE;			// ### Vad1
					GPSeq=0;
					Xbutton_Mode=4;
					break;
					//-Walk Mode
				case 4:
					ControlMode=WALKMODE;
					g_InControlState.SelectedLeg = 255;
					Xbutton_Mode=5;
					break;
				case 5:
					//-Single Leg Mode fNO------------------------
					if (abs(g_InControlState.TravelLength.x) < cTravelDeadZone			//No movement
						&& abs(g_InControlState.TravelLength.z) < cTravelDeadZone 
						&& abs(g_InControlState.TravelLength.y*2) < cTravelDeadZone ) {

							if (g_InControlState.SelectedLeg == 255)			//Select leg if none is selected
								g_InControlState.SelectedLeg = cRF;				//Start leg

							ControlMode = SINGLELEGMODE;
							Xbutton_Mode=0;
					}
					break;

				default:
					Xbutton_Mode=0;
					break;
				}  
			#ifdef DEBUG
				DEBUGPrintMode();
			#endif
			}



			//**[Common functions]****************
			//**Stand Up, sit Down  
			if (ps2x.ButtonPressed(PSB_TRIANGLE)) {			// Triangle - Button Test
				if (g_BodyYOffset > 0) 
						g_BodyYOffset = 0;
				else
					g_BodyYOffset = 40;
				fAdjustLegPositions = true;
			}

			//**Body Up 10mm
			if (ps2x.ButtonPressed(PSB_PAD_UP)) {			// D-Pad Up - Button Test
					g_BodyYOffset += 10;

				fAdjustLegPositions = true;
				if (g_BodyYOffset > MAX_BODY_Y)
						g_BodyYOffset = MAX_BODY_Y;
			}

			//**Body Down 10mm
			if (ps2x.ButtonPressed(PSB_PAD_DOWN) && g_BodyYOffset) {// D-Pad Down - Button Test
				if (g_BodyYOffset > 10)
						g_BodyYOffset -= 10;
				else
					g_BodyYOffset = 0;								// constrain don't go less than zero.

				fAdjustLegPositions = true;
			}



			if(!fAttackEnable){

				//**[Walk Functions]**********************
				if (ControlMode == WALKMODE) {

					//-Switch gates
					if (ps2x.ButtonPressed(PSB_SELECT)								// Select Button Test
						&& abs(g_InControlState.TravelLength.x) < cTravelDeadZone	// No movement
						&& abs(g_InControlState.TravelLength.z) < cTravelDeadZone 
						&& abs(g_InControlState.TravelLength.y*2) < cTravelDeadZone  ) {
							g_InControlState.GaitType ++;							// Go to the next gait...
							if (g_InControlState.GaitType < NUM_GAITS) {			//  < 6
								MSound(1, 50, 2000);								// [50\4000]
							} 
							else {
								MSound(2, 50, 2000, 50, 2250);						// [50\4000, 50\4500]
								g_InControlState.GaitType = 0;						// =0;
							}
							GaitSelect();
					}

					//-Speed control:
					//Increase speed with -50mS   -->
					if ((ps2x.ButtonPressed(PSB_PAD_RIGHT)) && (!ps2x.Button(PSB_L2))){		// D-Right not L2 - Button Test
						if (g_InControlState.SpeedControl > 0) {
							g_InControlState.SpeedControl = g_InControlState.SpeedControl - 50;
							MSound(1, 50, 2000);											// [50\4000]
						}
					}

					//-Decrease speed +50mS   <--
					if ((ps2x.ButtonPressed(PSB_PAD_LEFT)) && (!ps2x.Button(PSB_L2))){		// D-Left not L2 - Button Test
						if (g_InControlState.SpeedControl < 2000 ) {
							g_InControlState.SpeedControl = g_InControlState.SpeedControl + 50;
							MSound(1, 50, 2000);											// [50\4000]
						}
					}

					//-Double leg lift height
					if (ps2x.ButtonPressed(PSB_R1)) {			// R1 Button Test
						MSound(1, 50, 2000);					//[50\4000]
						DoubleHeightOn = !DoubleHeightOn;
						if (DoubleHeightOn)
							g_InControlState.LegLiftHeight = 80;
						else
							g_InControlState.LegLiftHeight = 50;
					}

					//-Double Travel Length
					if (ps2x.ButtonPressed(PSB_R2)) {			// R2 Button Test
						MSound(1, 50, 2000);					// [50\4000]
						DoubleTravelOn = !DoubleTravelOn;
					}

					//-Switch between Walk method 0 - 2 Walk Methods
					if (ps2x.ButtonPressed(PSB_R3)){			// R3 Button Test
						if(WalkMethod < 2){
							MSound(1, 50, 2000);				//[50\4000]
							WalkMethod++;
						}
						else {
							MSound(2, 50, 2000, 50, 2250);		//[50\4000, 50\4500]
							WalkMethod = 0;
						}
					#ifdef DEBUG
						DBGSerial.print(F("WalkMethod : "));
						DBGSerial.println(WalkMethod, DEC);
					#endif
					}  


					//**Walking************
					if (WalkMethod == 0){													//(Walk Method == 0)
						g_InControlState.TravelLength.x = -(ps2x.Analog(PSS_LX)-128);		//Left Stick Left/Right
						g_InControlState.TravelLength.z = (ps2x.Analog(PSS_LY)-128);		//Left Stick Up/Down
					}
					else if (WalkMethod == 1){												//(Walk Method == 1) 
						g_InControlState.TravelLength.z = (ps2x.Analog(PSS_RY)-128);		//Right Stick Up/Down  
					}  
					else if (WalkMethod == 2){												//(Walk Method == 2)
						g_InControlState.TravelLength.x = -(ps2x.Analog(PSS_LX)-128)*2/3;		//Left Stick Left/Right
						g_InControlState.TravelLength.z = (ps2x.Analog(PSS_LY)-128)*2/3;		//Left Stick Up/Down
						g_InControlState.BodyRot1.z = SmoothControl(-(ps2x.Analog(PSS_RX)-128), g_InControlState.BodyRot1.z, 2);	//Right Stick Left/Right
						g_InControlState.BodyRot1.x = SmoothControl((ps2x.Analog(PSS_RY)-128), g_InControlState.BodyRot1.x, 2);		//Right Stick Up/Down

					#ifdef DEBUG
						if (g_fWalkMethod2Rot){
							if (fWalking)
								DBGSerial.print(F("Wal : "));		// ### Vad1
							else
								DBGSerial.print(F("NWal : "));
							DBGSerial.print(F(" BR.z : "));
							DBGSerial.print(g_InControlState.BodyRot1.z, DEC);
							DBGSerial.print(F(" BR.x : "));
							DBGSerial.println(g_InControlState.BodyRot1.x, DEC);
						}   
					#endif

						HeadControl((ps2x.Analog(PSS_RX)-128)/5, (ps2x.Analog(PSS_RY)-128) );	// ### Vad1 (HeadPan, HeadTilt)
						HeadPanAngle1  =  HeadPanOut1;
						HeadTiltAngle1 =  HeadTiltOut1;
					}
					g_InControlState.TravelLength.y = -(ps2x.Analog(PSS_RX)-128)/4;				//Right Stick Left/Right 


					if (!DoubleTravelOn) {					//(Double travel length)
						g_InControlState.TravelLength.x = g_InControlState.TravelLength.x/2;
						g_InControlState.TravelLength.z = g_InControlState.TravelLength.z/2;
					}
				} // end, (ControlMode == WALKMODE)


				//**[Translate functions]*************************************
				//g_BodyYShift = 0;					//#####
				if (ControlMode == TRANSLATEMODE) {
					g_InControlState.BodyPos.x = (ps2x.Analog(PSS_LX)-128)/2;
					g_InControlState.BodyPos.z = -(ps2x.Analog(PSS_LY)-128)/3;
					g_InControlState.BodyRot1.y = (ps2x.Analog(PSS_RX)-128)*2;
					g_BodyYShift = (-(ps2x.Analog(PSS_RY)-128)/2);
				}

				HeadPanAngle1 =  g_InControlState.BodyRot1.y + g_InControlState.BodyPos.x*2;
				HeadTiltAngle1 = -g_BodyYShift*2;


				//**[Rotate functions]
				if (ControlMode == ROTATEMODE) {
					if (ps2x.ButtonPressed(PSB_SELECT)){			// Select Button Test
						// Switch Rotate Function
						if(RotateFunction < 2){
							MSound(1, 50, 2000);					//[50\4000]
							RotateFunction++;
						}
						else {
							MSound(2, 50, 2000, 50, 2250);			//[50\4000, 50\4500]
							RotateFunction = 0;
						}
					}  

					if (ps2x.ButtonPressed(PSB_L3)) {				// L3 Button Test
						MSound(1, 50, 2000);						//[50\4000]
						NeutralStick[0] = 1; 
						NeutralStick[1] = 1;
						NeutralStick[2] = 1;
						NeutralStick[3] = 1;
					}

					//Set the IDSdivFactor:
					if ((ps2x.ButtonPressed(PSB_PAD_RIGHT)) && (!ps2x.Button(PSB_L2))){   // D-Right not L2 - Button Test
						if (IDSdivFactor > 4){
							IDSdivFactor = IDSdivFactor - 2;		// Decrease value means faster indirect control
							MSound(1, 50, 2000);					//[50\4000]
						}  
						else										// have already reached the limit
							MSound(1, 40, 3000);					//[50\4000]
					}
					if ((ps2x.ButtonPressed(PSB_PAD_LEFT)) && (!ps2x.Button(PSB_L2))){   // D-Left not L2 - Button Test
						if (IDSdivFactor < 14){
							IDSdivFactor = IDSdivFactor + 2;		// ;Increase value give a slower but more accurate
							MSound(1, 50, 2000);					//[50\4000]
						}  
						else										// have already reached the limit
							MSound(1, 40, 1000);					//[40\2000]
					}
					IndirectDualShock();
					BranchRotateFunction(RotateFunction);
					g_InControlState.BodyPos.z = -IndDualShock[3]/3;  //  (PSS_LY) Translate (IndDualshock(6))

					// Shift Center Point of Rotation for the body
					if (ps2x.Button(PSB_R1)) {							// R1 Button Test // #### Vad1 CHANGED
						g_InControlState.BodyRotOffset.z = -165;		// Center Point to the Head tilt joint
					}
					else if (ps2x.Button(PSB_R2)) {						// R2 Button Test // #### Vad1 CHANGED
						g_InControlState.BodyRotOffset.z = 165;			// Center Point to the Abdomen/Tail tilt joint
					}
					else
						g_InControlState.BodyRotOffset.z = 0;
				} // end, (ControlMode == ROTATEMODE)
				else {
					g_InControlState.BodyRotOffset.x = 0;
					g_InControlState.BodyRotOffset.y = 0;
					g_InControlState.BodyRotOffset.z = 0;
				}


				//**[Single Leg Functions]
				if (ControlMode == SINGLELEGMODE) {
					//Switch leg for single leg control
					if (ps2x.ButtonPressed(PSB_SELECT)) {					// Select Button Test
						MSound(1, 50, 2000);								// [50\4000]
						if (g_InControlState.SelectedLeg < 5)
							g_InControlState.SelectedLeg = g_InControlState.SelectedLeg+1;
						else
							g_InControlState.SelectedLeg=0;
					}

					g_InControlState.SLLeg.x = (ps2x.Analog(PSS_LX)-128)/2;		//Left Stick Right/Left
					g_InControlState.SLLeg.y = (ps2x.Analog(PSS_RY)-128)/10;	//Right Stick Up/Down
					g_InControlState.SLLeg.z = (ps2x.Analog(PSS_LY)-128)/2;		//Left Stick Up/Down

					// Hold single leg in place
					if (ps2x.ButtonPressed(PSB_R2)) { // R2 Button Test
						MSound(1, 50, 2000);  
						g_InControlState.fSLHold = !g_InControlState.fSLHold;
					}
				} // end, (ControlMode == SINGLELEGMODE)



				//Calculate walking time delay
				// 0 - full pressed Joystick; 128 - release Joystick
				g_InControlState.InputTimeDelay = 128 - max(max(abs(ps2x.Analog(PSS_LX)-128), abs(ps2x.Analog(PSS_LY)-128)), abs(ps2x.Analog(PSS_RX)-128));
				// 128 - full pressed Joystick; 0 - release Joystick
				byte InputTimeDelay_t = abs(g_InControlState.InputTimeDelay - 128);
				InputTimeDelay_t = max(g_InControlState.InputTimeDelay, abs(ps2x.Analog(PSS_RY)-128));
				if (InputTimeDelay_t > 100){						// play Background sound (moved fast)
					TPlay_Every = 20000;
				}
				else{												// play Background sound (moved slow)
					TPlay_Every = 30000;
				}
			} // end fAttackEnable == 0)

		} //end, fHexOn == true && _fGPActive == false
		if(!g_InControlState.fHexOn){								// fHexOn == false
			// Soun ON/OFF
			if (ps2x.ButtonPressed(PSB_L3)) {						// L3 Button Test
				MSound(1, 50, 2000);
				F_SoundEnable = !F_SoundEnable;
				if(!F_SoundEnable) fAttackEnable = false;
			}
			TPlay_Every = 60000;									// play Background sound (Hex OFF)
			if (!ps2x.Button(PSB_R1) && !ps2x.Button(PSB_R2)){		// R1 and R2 not pressed
				ulTimePSB_R1_R2 = millis();							// reset ulTimePSB_R1_R2
			}  
			else if (ps2x.Button(PSB_R1) && ps2x.Button(PSB_R2)){	// R1 and R2 hold pressed 10c
				if((millis() - ulTimePSB_R1_R2) > 10000){
					MSound(1, 50, 2000);
					fPSB_R1_R2_10s = true;
				}  
			}  
			if(fPSB_R1_R2_10s){
				//Switch between Files for sequences
				if (ps2x.ButtonPressed(PSB_SELECT)) {		// Select Button Test
					switch (GPSeqFile){
					case 0:
						if(F_SoundEnable)
							DAP_PlaySound(PSTR("Macar"), SKEEP_isPlAY);
						GPSeqFile = 1;
					case 1:
						if(F_SoundEnable)
							DAP_PlaySound(PSTR("Lamb"), SKEEP_isPlAY);
						GPSeqFile = 2;
					case 2:
						if(F_SoundEnable)
							DAP_PlaySound(PSTR("Belly"), SKEEP_isPlAY);
						GPSeqFile = 3;
					}
				}
				//Start Sequence
				if (ps2x.ButtonPressed(PSB_R2)){			// R2 Button Test
					switch (GPSeqFile){
					case 0:
						if(GPSeqFile == GPSeqFile_old) break;
						if(ReadFilecomplete(PSTR("Macar"))){
							MSound(1, 50, 2000);
						}  
						else{
							MSound(2, 100, 2000, 50, 4000);		// [100\4000, 50\8000]
						}  
						break;
					case 1:
						if(GPSeqFile == GPSeqFile_old) break;
						if(ReadFilecomplete(PSTR("Lamb"))){
							MSound(1, 50, 2000);
						}  
						else{
							MSound(2, 100, 2000, 50, 4000);              // [100\4000, 50\8000]
						}  
						break;
					case 2:
						if(GPSeqFile == GPSeqFile_old) break;
						if(ReadFilecomplete(PSTR("Belly"))){
							MSound(1, 50, 2000);
						}  
						else{
							MSound(2, 100, 2000, 50, 4000);				// [100\4000, 50\8000]
						}  
						break;
					}
					GPSeqFile_old == GPSeqFile;
					fPSB_R1_R2_10s = false;
				}  
			}  //end fPSB_R1_R2_10s
		} // end (!g_InControlState.fHexOn)

		//Calculate g_InControlState.BodyPos.y
		g_InControlState.BodyPos.y = min(max(g_BodyYOffset + g_BodyYShift,  0), MAX_BODY_Y);

		if (fAdjustLegPositions)
			AdjustLegPositionsToBodyHeight();

	}  // end, if((ps2x.Analog(1) & 0xf0) == 0x70), read PS2 controller 
	else {
		if (g_sPS2ErrorCnt < MAXPS2ERRORCNT)
			g_sPS2ErrorCnt++;						// Increment the error count and if to many errors, turn off the robot.
		else if (g_InControlState.fHexOn){
			TurnRobotOff();
			PrintHexOff();
		}  
		ps2x.reconfig_gamepad();
	}
} // end, InputController::ControlInput



//-------------------------------------------------------------------------------
// [PS2TurnRobotOff]
//-------------------------------------------------------------------------------
void TurnRobotOff(void){                      // Turn off

	g_InControlState.BodyPos.x = 0;
	g_InControlState.BodyPos.y = 0;
	g_InControlState.BodyPos.z = 0;
	g_InControlState.BodyRot1.x = 0;
	g_InControlState.BodyRot1.y = 0;
	g_InControlState.BodyRot1.z = 0;
	g_InControlState.TravelLength.x = 0;
	g_InControlState.TravelLength.z = 0;
	g_InControlState.TravelLength.y = 0;
	g_BodyYOffset = 0;
	g_BodyYShift = 0;
	g_InControlState.SelectedLeg = 255;
	AdjustLegPositionsToBodyHeight();
	g_InControlState.fHexOn = false;
	CountVolt1Shutdown=0;
	CountVolt2Shutdown=0;
	s_bLVBeepCnt = 0;                            // reset Beep counter
	fPSB_R1_R2_10s = false;
	CountSeqAttack = 0;
	PWMIntervalMs = 50;                                      // setings for PWM Eyes OFF
	FromPWM = 255;
	ToPWM = 0;
	PWM_EyesOff();
	fAttackEnable = false;
	if (g_ServoDriver.FIsGPSeqActive())
		g_ServoDriver.GPStartSeq(0xff);           // abort sequence if possible...
}

void PrintHexOff(void){
#ifdef DEBUG
	DBGSerial.println(F("Hex OFF!"));
#endif          
}







//-------------------------------------------------------------------------------
// [IndirectDualShock]
// For an indirect control of Hex in rotation mode, necessary for obtaining a 
//  smooth and precise body control when using the PS2 controller.
//
// Note:
//DualShock(3)	PSS_RX 5	Right stick horz
//DualShock(4)	PSS_RY 6	Right stick vert
//DualShock(5)	PSS_LX 7	Left stick horz
//DualShock(6)	PSS_LY 8	Left stick vert
//
//-------------------------------------------------------------------------------
void IndirectDualShock(void) {
	short  Dualshock[4];

	Dualshock[0] = ps2x.Analog(PSS_RX);
	Dualshock[1] = ps2x.Analog(PSS_RY);
	Dualshock[2] = ps2x.Analog(PSS_LX);
	Dualshock[3] = ps2x.Analog(PSS_LY);

	for(int Index=0 ; Index <= 3; Index++) {
		if(NeutralStick[Index]) {	// (NeutralStick[Index] == 1) resets the IndirectDualShock to 0 gradually
			if(IndDualShock[Index] > 30) {
				IndDualShock[Index] = IndDualShock[Index] - 30;
			}
			else if(IndDualShock[Index] < -30) {
				IndDualShock[Index] = IndDualShock[Index] + 30;
			}  
			else {
				IndDualShock[Index] = 0;
			}  
			if(!IndDualShock[Index]) {		// IndDualShock[Index] == 0
				NeutralStick[Index] = 0;	// Reset
			}  
		}
		else { // (NeutralStick[Index] == 0)
			if (abs((Dualshock[Index] - 128)) > cTravelDeadZone) {
				IndDualShock[Index] = min (max((IndDualShock[Index] + ((Dualshock[Index] - 128)/IDSdivFactor)), -128), 127); 
			}  
		}  
#ifdef DEBUG  
	#ifdef DEBUG_INDERECT_DS
		if (g_fDebugOutput) {
			if (g_fDebugIndirectDS){
				// DEBUG INDERECT CONDROL
				DBGSerial.print(F("IDS "));
				DBGSerial.print(Index, DEC);
				DBGSerial.print(F(": "));
				DBGSerial.print(IndDualShock[Index], DEC);
				DBGSerial.print(F("    IDSdivFactor "));
				DBGSerial.println(IDSdivFactor, DEC);
			}  
		}      
	#endif
#endif  
	}  
}  



//-------------------------------------------------------------------------------
// [SmoothControl]
// This function makes the body rotation and translation much smoother while walking
//-------------------------------------------------------------------------------
short SmoothControl (short CtrlMoveInp, short CtrlMoveOut, short CtrlDivider) {

	if (fWalking) { // fWalking == 1
		if (CtrlMoveOut < (CtrlMoveInp-4)) {
			CtrlMoveOut = CtrlMoveOut + abs((CtrlMoveOut - CtrlMoveInp)/CtrlDivider);

			//		DBGSerial.print(F("Wal : "));		// ### Vad1
			//		DBGSerial.print(F(" Inp : "));
			//		DBGSerial.print(CtrlMoveInp, DEC);
			//		DBGSerial.print(F(" Out<4 : "));
			//		DBGSerial.println(CtrlMoveOut, DEC);

		}  
		else if (CtrlMoveOut > (CtrlMoveInp+4)) {
			CtrlMoveOut = CtrlMoveOut - abs((CtrlMoveOut - CtrlMoveInp)/CtrlDivider);
			//		DBGSerial.print(F("Wal : "));		// ### Vad1
			//		DBGSerial.print(F(" Inp : "));
			//		DBGSerial.print(CtrlMoveInp, DEC);
			//		DBGSerial.print(F(" Out>4 : "));
			//		DBGSerial.println(CtrlMoveOut, DEC);
		}  
		else {
			CtrlMoveOut = CtrlMoveInp;
			//		DBGSerial.print(F("Wal : "));		// ### Vad1
			//		DBGSerial.print(F(" Inp : "));
			//		DBGSerial.print(CtrlMoveInp, DEC);
			//		DBGSerial.print(F(" Out=4 : "));
			//		DBGSerial.println(CtrlMoveOut, DEC);
		}
	}  
	else {  // fWalking == 0
		CtrlMoveOut = CtrlMoveInp;
		//		DBGSerial.print(F("NWal : "));		// ### Vad1
		//		DBGSerial.print(F(" Inp : "));
		//		DBGSerial.print(CtrlMoveInp, DEC);
		//		DBGSerial.print(F(" Out : "));
		//		DBGSerial.println(CtrlMoveOut, DEC);
	}  
	return CtrlMoveOut;
}


//-------------------------------------------------------------------------------
// [Branch RotateFunction]
//-------------------------------------------------------------------------------
void BranchRotateFunction(byte rf) {

	switch (rf)  {

	case 0: 	 
		//HeadTracking
		HeadControl (IndDualShock[0], IndDualShock[1]);
		g_InControlState.BodyRot1.x = IndDualShock[1];
		g_InControlState.BodyRot1.y = IndDualShock[2];
		g_InControlState.BodyRot1.z = -IndDualShock[0]*2;
		HeadPanAngle1  = -HeadPanOut1;			// ### Vad1
		HeadTiltAngle1 = HeadTiltOut1;
		break;

	case 1: 	 
		//FixedHead:
		g_InControlState.BodyRot1.x = IndDualShock[1];
		g_InControlState.BodyRot1.y = IndDualShock[2];
		g_InControlState.BodyRot1.z = -IndDualShock[0]*2;
		HeadPanAngle1  = g_InControlState.BodyRot1.y + HeadPanOut1;
		HeadTiltAngle1 = -g_InControlState.BodyRot1.x + HeadTiltOut1;
		break;
	case 2: 	 
		//HeadOnly:				
		HeadControl (IndDualShock[0], IndDualShock[1]);
		HeadPanAngle1  = g_InControlState.BodyRot1.y + HeadPanOut1;
		HeadTiltAngle1 = -g_InControlState.BodyRot1.x + HeadTiltOut1;
		break;
	}  
}



//-------------------------------------------------------------------------------
// [Head control], return: HeadPanOut1, HeadTiltOut1
//-------------------------------------------------------------------------------
void HeadControl (short HeadPanInput1, short HeadTiltInput1) {

	HeadPanOut1  = SmoothControl ((-HeadPanInput1*3), HeadPanOut1, 2);			// 
	HeadTiltOut1 = SmoothControl ((HeadTiltInput1*c2DEC/24), HeadTiltOut1, 2);	// 
}


//-------------------------------------------------------------------------------
// [DEBUGPrintMode] - Print Hex Mode
//-------------------------------------------------------------------------------
void DEBUGPrintMode(void){

	switch (ControlMode){
		case 0:
			DBGSerial.println(F("Mode=WALK"));
			break;
		case 1:
			DBGSerial.println(F("Mode=TRANSLATE"));
			break;
		case 2:
			DBGSerial.println(F("Mode=ROTATE"));
			break;
		case 3:
			DBGSerial.println(F("Mode=SINGLELEG"));
			break;
		case 4:
			DBGSerial.println(F("Mode=GPPLAYER"));
			break;
	}
}







