

#ifndef _MYFUNCTIONS_H_
#define _MYFUNCTIONS_H_


#include "MyArmIIC.h"
#include "MyArmRecorder.h"




bool record(){
	//debugPrint("mRecordAddr = %d", mRecordAddr);

	if(mRecordAddr <= 65530){

		unsigned char data[6];	// 0: Bas_fb; 1: Shl_fb; 2: Elb_fb; 3: Wri_fb; 4: Wro_fb; 5: Gri_fb

		//debugPrint("mRecordAddr = %d", mRecordAddr);

		if((mRecordAddr != 65530) && (mSysStatus != LEARNING_MODE_STOP)){

			//double rot, left, right;
			//controller.updateAllServoAngle();
			//controller.readServoAngles(rot, left, right);
			Bas_fb=Bas_Servo.read();
			Shl_fb=Shl_Servo.read();
			Elb_fb=Elb_Servo.read();
			Wri_fb=Wri_Servo.read();
			Wro_fb=Wro_Servo.read();
			Gri_fb=Gri_Servo.read();

			data[0] = (unsigned char)Bas_fb;
			data[1] = (unsigned char)Shl_fb;
			data[2] = (unsigned char)Elb_fb;
			data[3] = (unsigned char)Wri_fb;
			data[4] = (unsigned char)Wro_fb;
			data[5] = (unsigned char)Gri_fb;

			//data[4] = getPumpStatus() > 0 ? 1 : 0;

			//debugPrint("l=%d, r=%d, r= %d", data[0], data[1], data[2]);
		}
		else{

			data[0] = 255;	//255 is the ending flag
			recorder.write(mRecordAddr, data, 6);

			return false;
		}

		recorder.write(mRecordAddr, data, 6);
		mRecordAddr += 6;

		return true;
	}
	else{

		return false;
	}

}





void runRecTick(){

	//systemRun();

	if (millis() - mTickRecorderTime >= 50){

		mTickRecorderTime= millis();
		recorderTick();
	}
}



void recorderTick(){

    // detec every 0.05s-----------------------------------------------------------------

    switch(mSysStatus){		
    
		case SINGLE_PLAY_MODE:

			if(play() == false){

					mSysStatus = NORMAL_MODE;
					mRecordAddr = 0;
			}
			break;

		case LOOP_PLAY_MODE:

			if(play() == false){

				mRecordAddr = 0;
			}
			break;

		case LEARNING_MODE:
		case LEARNING_MODE_STOP:
			if(record() == false){

					mSysStatus = NORMAL_MODE;
					mRecordAddr = 0;
           
					//controller.attachAllServo();
			}
			break;

		default: 
			break;
    }

}



bool play(){

    unsigned char data[6];	// 0: Left; 1: Right; 2: Rotation; 3: hand rotation; 4: Pump

    recorder.read(mRecordAddr, data, 6);
	//debugPrint("mRecordAddr = %d, data=%d, %d, %d", mRecordAddr, data[0], data[1], data[2]);

    if(data[0] != 255){

        //double x, y, z;
        //controller.getXYZFromAngle(x, y, z, (double)data[2], (double)data[0], (double)data[1]);
        //moveToAngle((double)data[2], (double)data[0], (double)data[1]);
    	//controller.writeServoAngle((double)data[2], (double)data[0], (double)data[1]);
        //controller.writeServoAngle(SERVO_HAND_ROT_NUM, (double)data[3]);
        //unsigned char pumpStatus = getPumpStatus() > 0 ? 1 : 0;
        //if (pumpStatus != data[4]){

        //    if (data[4])
        //    {
        //        pumpOn();
        //    }
        //    else
        //    {
        //        pumpOff();
        //    }   
        //}
    }
    else{

        //pumpOff();
         
        return false;
    }

    mRecordAddr += 6;

    return true;
}




#endif // _MYFUNCTIONS_H_
