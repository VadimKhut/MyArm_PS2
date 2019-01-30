/**
  ******************************************************************************
  * @file	MyArmRecorder.cpp
  *  
  *  
  * @date	2018-11-30
  *  
  *  
  ******************************************************************************
  */

#include "MyArmRecorder.h" 

MyArmRecorder recorder;

//extern void reportPos();

MyArmRecorder::MyArmRecorder()
{
	
}

void MyArmRecorder::write(unsigned int addr, unsigned char data[], int num)
{
    unsigned char i=0;
    i = (addr % 128);
    // Since the eeprom's sector is 128 byte, if we want to write 5 bytes per cycle we need to care about when there's less than 6 bytes left
    if((i >= 124) && (num == 6))
    {
        i = 128 - i;
        iic_writebuf(data, EXTERNAL_EEPROM_USER_ADDRESS, addr, i);// write data
        delay(5);
        iic_writebuf(data + i, EXTERNAL_EEPROM_USER_ADDRESS, addr + i, num - i);// write data
    }
    //if the left bytes are greater than 5, just do it
    else
    {
        iic_writebuf(data, EXTERNAL_EEPROM_USER_ADDRESS, addr, num);// write data
    }
}

void MyArmRecorder::read(unsigned int addr, unsigned char data[], int num)
{
    unsigned char i=0;
    i= (addr % 128);
    // Since the eeprom's sector is 128 byte, if we want to write 5 bytes per cycle we need to care about when there's less than 6 bytes left
    if( (i >= 124) && (num == 6))
    {
        i = 128 - i;
        iic_readbuf(data, EXTERNAL_EEPROM_USER_ADDRESS, addr, i);// write data
        delay(5);
        iic_readbuf(data + i, EXTERNAL_EEPROM_USER_ADDRESS, addr + i, num - i);// write data
    }
    //if the left bytes are greater than 5, just do it
    else
    {
        iic_readbuf(data, EXTERNAL_EEPROM_USER_ADDRESS, addr, num);// write data
    }
}


