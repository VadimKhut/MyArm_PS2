/**
  ******************************************************************************
  * @file	uArmRecorder.h
  *  
  *  
  * @date	2018-11-30
  * 
  * 
  ******************************************************************************
  */

#ifndef _MYARMRECORDER_H_
#define _MYARMRECORDER_H_

#include <Arduino.h>
//#include "uArmConfig.h"
//#include "uArmPin.h"
#include "MyArmIIC.h"

#define EXTERNAL_EEPROM_USER_ADDRESS	0xA0 



class MyArmRecorder
{
public:
  	MyArmRecorder();
	void write(unsigned int addr, unsigned char data[], int num);
	void read(unsigned int addr, unsigned char data[], int num);

private:


};

extern MyArmRecorder recorder;


#endif // _MYARMRECORDER_H_
