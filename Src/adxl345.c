#include "adxl345.h"
#include "i2c.h"

extern I2C_HandleTypeDef hi2c2;
//set the register for the adxl345
void set_adxl345(uint16_t addr,uint8_t reg, uint8_t data){

	I2C_Write(&hi2c2,addr,reg,data);
}

//read multiple bytes from adxl345
void read_adxl345(uint16_t addr, uint8_t reg, uint8_t* pdata){

	I2C_ReadMulti(&hi2c2, addr, reg ,pdata, 1);

}

I2C_Result_t adxl345_init(uint8_t device_address){
	return I2C_IsDeviceConnected(&hi2c2,  device_address);
}
