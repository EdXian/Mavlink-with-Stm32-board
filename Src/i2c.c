

#include "i2c.h"



I2C_HandleTypeDef hi2c2;

void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C2)
  {

    __HAL_RCC_GPIOF_CLK_ENABLE();
    /**I2C2 GPIO Configuration    
    PF0     ------> I2C2_SDA
    PF1     ------> I2C2_SCL 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    __HAL_RCC_I2C2_CLK_ENABLE();

  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C2)
  {

    __HAL_RCC_I2C2_CLK_DISABLE();
  
    /**I2C2 GPIO Configuration    
    PF0     ------> I2C2_SDA
    PF1     ------> I2C2_SCL 
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_0|GPIO_PIN_1);


  }
} 


void i2c_writebytes(I2C_HandleTypeDef* i2c,uint16_t addr, uint8_t* pdata, uint16_t size, uint32_t timeout  ){

	HAL_I2C_Master_Transmit(i2c,addr,pdata,size,timeout);

}
void i2c_readbytes(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout){

	HAL_I2C_Master_Receive(hi2c,  DevAddress, pData,  Size, Timeout);

}

I2C_Result_t I2C_Write(I2C_HandleTypeDef* hi2c, uint8_t device_address, uint8_t register_address, uint8_t data) {
	uint8_t d[2];

	/* Format array to send */
	d[0] = register_address;
	d[1] = data;

	/* Try to transmit via I2C */
	if (HAL_I2C_Master_Transmit(hi2c, (uint16_t)device_address, (uint8_t *)d, 2, 1000) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {

		}

		/* Return error */
		return I2C_Result_Error;
	}

	/* Return OK */
	return I2C_Result_Ok;
}

I2C_Result_t I2C_ReadMulti(I2C_HandleTypeDef* hi2c, uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count) {

	if (HAL_I2C_Master_Transmit(hi2c, (uint16_t)device_address, &register_address, 1, 1000) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {

		}

		/* Return error */
		return I2C_Result_Error;
	}

	/* Receive multiple byte */
	if (HAL_I2C_Master_Receive(hi2c, device_address, data, count, 1000) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {

		}

		/* Return error */
		return I2C_Result_Error;
	}

	/* Return OK */
	return I2C_Result_Ok;

}

I2C_Result_t I2C_IsDeviceConnected(I2C_HandleTypeDef* hi2c, uint8_t device_address) {

	/* Check if device is ready for communication */
	if (HAL_I2C_IsDeviceReady(hi2c, device_address, 10, 25) != HAL_OK) {
		/* Return error */
		return I2C_Result_Error;
	}

	/* Return OK */
	return I2C_Result_Ok;
}


