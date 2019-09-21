#include "HMC5883L.h"
#include "i2c.h"



I2C_Result_t HMC5883L_init(void){


	return I2C_IsDeviceConnected(&hi2c2,  HMC5883L_WRITE_ADDRESS);

}




