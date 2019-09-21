
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "mavlink.h"
#include "usart.h"
#include "adxl345.h"
#include "HMC5883L.h"
SemaphoreHandle_t uart_tx_Semaphore;
osThreadId StartingtaskHandle;


uint8_t rx_buffer[200];
uint8_t len;

mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
mavlink_message_t  message;
mavlink_heartbeat_t heart_beat;
mavlink_att_pos_mocap_t mocap_data;
mavlink_set_position_target_local_ned_t set_point;
uint8_t len;

void task1(void const *argument);
void task2(void const *argument);
void receive_task(void const *argument);
void flush_rxbuffer(void);

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void);

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}                   


void MX_FREERTOS_Init(void) {

	//create task
	osThreadDef(Startingtask, StartDefaultTask, osPriorityNormal, 0, 128);
	StartingtaskHandle = osThreadCreate(osThread(Startingtask), NULL);

	osThreadDef(task1_, task1, osPriorityNormal, 0, 512);
	StartingtaskHandle = osThreadCreate(osThread(task1_), NULL);

	osThreadDef(receive_task_, receive_task, osPriorityNormal, 0, 512);
	StartingtaskHandle = osThreadCreate(osThread(receive_task_), NULL);

	osThreadDef(task2_, task2, osPriorityNormal, 0, 512);
	StartingtaskHandle = osThreadCreate(osThread(task2_), NULL);

	//create semaphore

	uart_tx_Semaphore = xSemaphoreCreateMutex();

	if(uart_tx_Semaphore ==NULL)  Error_Handler();



}




void StartDefaultTask(void const * argument)
{
	HAL_StatusTypeDef  state;
  for(;;)
  {

	if(uart_tx_Semaphore !=NULL ){

		if( xSemaphoreTake( uart_tx_Semaphore, ( TickType_t ) 5 ) == pdTRUE )
			{

				mavlink_msg_mission_count_pack(1,1,&msg,1,1,1);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				state = HAL_UART_Transmit_DMA(&huart1, buf, len+8 );
				while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);

				mavlink_msg_sys_status_pack(1, 200, &msg, 0, 0, 0, 500, x*1000.0, -1, -1, 0, 0, 0, 0, 0, 0);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				state = HAL_UART_Transmit_DMA(&huart1, buf, len+8 );
				while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);

				HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_13);
				xSemaphoreGive( uart_tx_Semaphore );

			}
		}

	osDelay(500);
  }
}


void task1(void const *argument){

	HAL_StatusTypeDef  state;
	uint8_t  param_send=4;
	mavlink_param_value_t param_value;

	for(;;){

		if(uart_tx_Semaphore !=NULL ){

			if( xSemaphoreTake( uart_tx_Semaphore, ( TickType_t ) 5 ) == pdTRUE ){

				mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				state = HAL_UART_Transmit_DMA(&huart1, buf, len+8 );
				while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);


				if(param_send){
					mavlink_msg_param_value_pack(1,1,&msg,"1",param_send*1.0,param_send,param_send,param_send);
					len = mavlink_msg_to_send_buffer(buf, &msg);
					state = HAL_UART_Transmit_DMA(&huart1, buf, len+8 );
					while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);
					mavlink_msg_autopilot_version_pack(1,1,&msg,1,1,1,1,1,&param_send,&param_send,&param_send,1,1,1);
					param_send--;
				}

				xSemaphoreGive( uart_tx_Semaphore );

			}
		}

		osDelay(50);

	}

}

void flush_rxbuffer(void){
	uint8_t k=200;
	while(k>0){
		rx_buffer[k] = '\0';
		k--;
	}
}

void task2(void const *argument){
	HAL_StatusTypeDef  state;
	float time;
	TickType_t ticks_;
	uint8_t test=0;
	uint8_t test2=2.0;
	acc3d_t  acc;


	for(;;){
		if(HMC5883L_init() == 0x00){
				HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_14);
			}

		if(uart_tx_Semaphore !=NULL ){

			if( xSemaphoreTake( uart_tx_Semaphore, ( TickType_t ) 5 ) == pdTRUE ){
			ticks_ = xTaskGetTickCount();
			time = ticks_/1000.0;

			mavlink_msg_local_position_ned_pack(1, 1, &msg, time,acc.ax,acc.ay ,acc.az ,3 ,4 ,5);

			len = mavlink_msg_to_send_buffer(buf, &msg);
			state = HAL_UART_Transmit_DMA(&huart1, buf, len+8 );
			while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);

			xSemaphoreGive( uart_tx_Semaphore );

			}
		}
		osDelay(50);

	}

}


void receive_task(void const *argument){

	mavlink_status_t status;
	uint32_t last_time;
	flush_rxbuffer();

	for(;;){

		if(usart1_rx_end_flag == 1){

			HAL_UART_Receive_DMA(&huart1, rx_buffer , 200);

			uint8_t a=0;
			uint8_t parse_state = 0;

			//feed chars until  parse_state return 1 or breaking the loop when timeout occured.
			while(  (parse_state !=1 )  && ((xTaskGetTickCount() - last_time )<1000)  ){
				parse_state = mavlink_parse_char(MAVLINK_COMM_0,rx_buffer[a], &message, &status);
				a++;
			}

			//decode the msg and get the information.
			switch (message.msgid){

				case MAVLINK_MSG_ID_HEARTBEAT :

					mavlink_msg_heartbeat_decode(&message , &heart_beat);
					break;

				case  MAVLINK_MSG_ID_ATT_POS_MOCAP:

					  mavlink_msg_att_pos_mocap_decode(&message, &mocap_data);
					  break;

				case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED :
					mavlink_msg_set_position_target_local_ned_decode(&message, &set_point);
					break;
			}

			//RESET THE PARAMETERS
			last_time =   xTaskGetTickCount();
			flush_rxbuffer();
			usart1_rx_len=0;
			usart1_rx_end_flag =0;
			osDelay(10);
		}
	}
}





