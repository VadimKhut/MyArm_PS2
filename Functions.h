

int shl_pos_us, shl1_pos_us, elb_pos_us, wri_pos_us, Gr_pos_us, WRro_pos_us;
int old_shl_pos_us, old_elb_pos_us, old_wri_pos_us, old_Gr_pos_us, old_WRro_pos_us;

#ifdef CYL_IK   // 2D kinematics
	int BA2D_pos_us;
#else           // 3D kinematics
	int BA3D_pos_us;
#endif


void MoveArmTo(void){

	#ifdef CYL_IK   // 2D kinematics
	 BA2D_pos_us = deg_to_us(BA2D_pos);
	#else           // 3D kinematics
	 BA3D_pos_us = deg_to_us(BA3D_pos);
	#endif
	shl_pos_us = deg_to_us(shl_pos);
	shl1_pos_us = deg_to_us(185-shl_pos);
	elb_pos_us = deg_to_us(elb_pos);
	wri_pos_us = deg_to_us(wri_pos);
	Gr_pos_us = deg_to_us(Gr_pos);
	WRro_pos_us = deg_to_us(WRro_pos);






	// Position the servos
 #ifdef CYL_IK   // 2D kinematics

	Bas_Servo.writeMicroseconds(BA2D_pos_us);
	if (abs(BA2D_pos_us - old_BA2D_pos_us) > 5) {
		writeCommand('B', BA2D_pos_us);
		old_BA2D_pos_us = BA2D_pos_us;
	}

 #else           // 3D kinematics
	Bas_Servo.writeMicroseconds(bas3D_pos_us;
	if (abs(bas3D_pos_us - old_bas3D_pos_us) > 5) {
		writeCommand('B', bas3D_pos_us);
		old_bas3D_pos_us = bas3D_pos_us;
	}
 #endif

	Shl_Servo.writeMicroseconds(shl_pos_us);
	Shl_Servo1.writeMicroseconds(shl1_pos_us);
	Elb_Servo.writeMicroseconds(elb_pos_us);
	Wri_Servo.writeMicroseconds(wri_pos_us);


	if (abs(shl_pos_us - old_shl_pos_us) > 5) {
		writeCommand('S', shl_pos_us);
		old_shl_pos_us = shl_pos_us;
		writeCommand('S', shl_pos_us);
	}

	if (abs(elb_pos_us - old_elb_pos_us) > 5) {
		writeCommand('B', elb_pos_us);
		old_elb_pos_us = elb_pos_us;
		writeCommand('E', elb_pos_us);
	}

	if (abs(wri_pos_us - old_wri_pos_us) > 5) {
		writeCommand('B', wri_pos_us);
		old_wri_pos_us = wri_pos_us;
		writeCommand('w', wri_pos_us);
	}


	Gri_Servo.writeMicroseconds(Gr_pos_us);

	if (abs(Gr_pos_us - old_Gr_pos_us) > 5) {
		writeCommand('G', Gr_pos_us);
		old_Gr_pos_us = Gr_pos_us;
	}


	Gri_Servo.writeMicroseconds(Gr_pos_us);

	writeCommand('G', Gr_pos_us);

	Wro_Servo.writeMicroseconds(WRro_pos_us);

	if (abs(WRro_pos_us - old_WRro_pos_us) > 5) {
		writeCommand('W', WRro_pos_us);
		old_WRro_pos_us = WRro_pos_us;
	}





}



			Bas_Servo.writeMicroseconds(BA2D_pos_us);
			Shl_Servo.writeMicroseconds(shl_pos_us);
			Shl_Servo1.writeMicroseconds(shl1_pos_us);
			Elb_Servo.writeMicroseconds(elb_pos_us);
			Wri_Servo.writeMicroseconds(wri_pos_us);
			Wro_Servo.writeMicroseconds(WRro_pos_us);
			Gri_Servo.writeMicroseconds(Gr_pos_us);





			DegToUsAll();

			ServoUpdate(2000,BA_pos_us, shl_pos_us, shl1_pos_us, elb_pos_us, wri_pos_us, WRro_pos_us, Gr_pos_us);


			MSound (2, 40, 2500, 40, 2500);    //[40\5000,40\5000]  Error
			MSound (2, 50, 2000, 50, 2000);	

			MSound(1, 250, 1500);
			
			MSound(2, 100, 2000, 50, 4000);

			MSound(1, 50, 2000);



//-------------------------------------------------------------------------------
// void ControlInput_S(void)
// Function to read inputs from the PS2 and then process any commands (FOR Play Sound ONLY)
//-------------------------------------------------------------------------------
void ControlInput_S(void){

	// Then try to receive a packet of information from the PS2.
	ps2x.read_gamepad();								// read controller and set large motor to spin at 'vibrate' speed

	if ((ps2x.Analog(1) & 0xf0) == 0x70) {

		if (ps2x.ButtonPressed(PSB_CROSS)) {			// X - Cross Button Test
			MSound(1, 50, 2000);						// [50\4000]
			StopPlay();									// stop play
		}

	}  // end, if((ps2x.Analog(1) & 0xf0) == 0x70), read PS2 controller 
	else {
		if (g_sPS2ErrorCnt < MAXPS2ERRORCNT)
			g_sPS2ErrorCnt++;							// Increment the error count and if to many errors, turn off the robot.
		else if (g_InControlState.fHexOn){
			TurnRobotOff();
			PrintHexOff();
		}  
		ps2x.reconfig_gamepad();
	}
} // end, InputController::ControlInput 




	if (mode == 'R') {

		if (abs(BA_pos_us - old_BA_pos_us) > THRESHOLD_REC) {
			writeCommand();
			old_BA_pos_us = BA_pos_us;
		}


		if (abs(shl_pos_us - old_shl_pos_us) > THRESHOLD_REC) {
			writeCommand();
			old_shl_pos_us = shl_pos_us;
		}

		if (abs(elb_pos_us - old_elb_pos_us) > THRESHOLD_REC) {
			writeCommand();
			old_elb_pos_us = elb_pos_us;
		}

		if (abs(wri_pos_us - old_wri_pos_us) > THRESHOLD_REC) {
			writeCommand();
			old_wri_pos_us = wri_pos_us;
		}

	
		if (abs(WRro_pos_us - old_WRro_pos_us) > THRESHOLD_REC) {
			writeCommand();
			old_WRro_pos_us = WRro_pos_us;
		}


		if (abs(Gr_pos_us - old_Gr_pos_us) > THRESHOLD_REC) {
			writeCommand();
			old_Gr_pos_us = Gr_pos_us;
		}

	} // mode == 'R'


				if(fRecStart_10s){
				// First check to see if we need to record or stop.
				if (Ps2x.ButtonPressed(PSB_CIRCLE)) {		                  // PSB_CIRCLE Button Test

					MSound(1, 50, 6000);

				#ifdef DEBUG
					Serial.println(F("Record button press!"));
				#endif

					if ((mode == 'R') && (fRecStop_10s == true)) { 
						mode = 'N'; 

						TM1637Display_Off();
						fRecStart_10s = false;
						fCapture_pos = false;
						delay(200);
					}
					else {
						startRecord();

						delay(200);
					}
				}

			} // fRecStart_10s

