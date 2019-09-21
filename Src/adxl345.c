#include "adxl345.h"
#include "i2c.h"

extern I2C_HandleTypeDef hi2c2;
//set the register for the adxl345
void set_adxl345(uint16_t addr,uint8_t reg, uint8_t data){

	I2C_Write(&hi2c2,addr,reg,data);
}

//read multiple bytes from adxl345
I2C_Result_t read_adxl345(uint16_t addr, uint8_t reg, uint8_t* pdata){

	return I2C_ReadMulti(&hi2c2, addr, reg ,pdata, 1);

}

I2C_Result_t adxl345_init(uint8_t device_address){

	return I2C_IsDeviceConnected(&hi2c2,  device_address);

}



I2C_Result_t adxl345_config(void){
	I2C_Result_t state;

	state = I2C_Write(&hi2c2,ADXL345_WRITE_ADDRESS,ADXL345_REG_POWER_CTL,0);

	state = I2C_Write(&hi2c2,ADXL345_WRITE_ADDRESS,ADXL345_REG_POWER_CTL,16);

	state = I2C_Write(&hi2c2,ADXL345_WRITE_ADDRESS,ADXL345_REG_POWER_CTL,8);



	char tx[2];
	tx[0] = ADXL345_REG_BW_RATE;
	tx[1] = ADXL345_DATARATE_1600_HZ;
	state = I2C_Write(&hi2c2,ADXL345_WRITE_ADDRESS,ADXL345_REG_BW_RATE,tx[1]);

	//Data format (for +-16g) - This is done by setting Bit D3 of the DATA_FORMAT register (Address 0x31) and writing a value of 0x03 to the range bits (Bit D1 and Bit D0) of the DATA_FORMAT register (Address 0x31).

	 char rx[2];
	    rx[0] = ADXL345_REG_DATA_FORMAT;
	    rx[1] = 0x0B;
	     // full res and +_16g

	state = I2C_Write(&hi2c2,ADXL345_WRITE_ADDRESS,ADXL345_REG_DATA_FORMAT,rx[1]);

	 // Set Offset  - programmed into the OFSX, OFSY, and OFXZ registers, respectively, as 0xFD, 0x03 and 0xFE.
	  char x[2];
	    x[0] =0 ;
	    x[1] = 1;
		state = I2C_Write(&hi2c2,ADXL345_WRITE_ADDRESS,ADXL345_REG_OFSX,x[1]);

	  char y[2];
	    y[0] = 0 ;
	    y[1] =2;
		state = I2C_Write(&hi2c2,ADXL345_WRITE_ADDRESS,ADXL345_REG_OFSY,y[1]);

	 char z[2];
	    z[0] = 0 ;
	    z[1] = 3;
		state = I2C_Write(&hi2c2,ADXL345_WRITE_ADDRESS,ADXL345_REG_OFSZ,z[1]);


	 char pw=0xa;



}

I2C_Result_t adxl345_get_id(uint8_t* data){
	//ID=229
	return I2C_ReadMulti(&hi2c2, ADXL345_READ_ADDRESS, ADXL345_REG_DEVID ,data,1);

}


I2C_Result_t adxl345_set_range(void){


	uint8_t format;


	I2C_ReadMulti(&hi2c2, ADXL345_READ_ADDRESS, ADXL345_REG_DATA_FORMAT ,format,1);

	  /* Update the data rate */
	  format &= ~0x0F;
	  format |= ADXL345_RANGE_16_G;

	  /* Make sure that the FULL-RES bit is enabled for range scaling */
	  format |= 0x08;

	  /* Write the register back to the IC */
	  I2C_Write(&hi2c2,ADXL345_WRITE_ADDRESS,ADXL345_REG_DATA_FORMAT,format);

}


I2C_Result_t adxl345_get_offxyz(acc3d_t* data){

	uint8_t buffer[3];
	uint16_t tmpx,tmpy,tmpz;
	I2C_Result_t state;
	state =  I2C_ReadMulti(&hi2c2, ADXL345_READ_ADDRESS, ADXL345_REG_OFSX ,buffer, 3);
	data->ax = (uint16_t)buffer[0] ;
	data->ay = (uint16_t)buffer[1];
	data->az = (uint16_t)buffer[2] ;

}


I2C_Result_t adxl345_get_mode(uint8_t* data){
	I2C_Result_t state;

	return I2C_ReadMulti(&hi2c2, ADXL345_READ_ADDRESS, ADXL345_REG_BW_RATE ,data, 1);


}


I2C_Result_t adxl345_get_dataxyz(acc3d_t* data){

	uint8_t buffer[6];
	uint16_t tmpx,tmpy,tmpz;
	I2C_Result_t state;
	state =  I2C_ReadMulti(&hi2c2, ADXL345_READ_ADDRESS, ADXL345_REG_DATAX0 ,buffer, 1);

	data->ax = buffer[1];
//	data->ay = buffer[1];
//	data->az = (uint16_t)buffer[4] << 8 | (uint16_t)buffer[5];

	return state;
}
