/**
  ******************************************************************************
  * @file    Template/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    20-September-2013
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
 
#include "main.h"
#include "uart.h"
#include "pid.h"
#include "mpu6050.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "esp8266.h"

#include "stm32f4xx_conf.h"

TaskHandle_t PID_xHandle;

//Main Function
int main(void)
{
	//Configurations
	Init_UART1();
	Init_UART6();
//	Init_Motor(PID_xHandle);
//	Init_MPU6050();

//	if (!MPU6050_Task_Creat()) {
//		UART1_puts("Initialize information task failed!\r\n");
//	}
//	if (!PID_Task_Creat(PID_xHandle)) {
//		UART1_puts("Initialize information task failed!\r\n");
//	}
//	if (!ESP8266_Task_Creat()) {
//		UART1_puts("Initialize information task failed!\r\n");
//	}

	vTaskStartScheduler();
}
