/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "adc.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "dht.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
	 SENSOR am2301a_in;
	 SENSOR am2301a_out;
	 RTC_TimeTypeDef sTime;
/* USER CODE END Variables */
/* Definitions for tempControlTask */
osThreadId_t tempControlTaskHandle;
const osThreadAttr_t tempControlTask_attributes = {
  .name = "tempControlTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for humidityControl */
osThreadId_t humidityControlHandle;
const osThreadAttr_t humidityControl_attributes = {
  .name = "humidityControl",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void turnOnLedPanel() {
	HAL_GPIO_WritePin(LED_PANEL_GPIO_Port, LED_PANEL_Pin, RESET);
}
void turnOffLedPanel() {
	HAL_GPIO_WritePin(LED_PANEL_GPIO_Port, LED_PANEL_Pin, SET);
}
void turnOnFans () {
	HAL_GPIO_WritePin(FAN_IN_GPIO_Port, FAN_IN_Pin, RESET);
	HAL_GPIO_WritePin(FAN_OUT_GPIO_Port, FAN_OUT_Pin, RESET);
}
void turnOffFans () {
	HAL_GPIO_WritePin(FAN_IN_GPIO_Port, FAN_IN_Pin, SET);
	HAL_GPIO_WritePin(FAN_OUT_GPIO_Port, FAN_OUT_Pin, SET);
}
void turnOnFanMIX() {
	HAL_GPIO_WritePin(FAN_MIX_GPIO_Port, FAN_MIX_Pin, RESET);
}
void turnOffFanMIX() {
	HAL_GPIO_WritePin(FAN_MIX_GPIO_Port, FAN_MIX_Pin, SET);
}
void turnOnPump () {
	HAL_GPIO_WritePin(PUMP_GPIO_Port,PUMP_Pin,SET);
}
void turnOffPump () {
	HAL_GPIO_WritePin(PUMP_GPIO_Port,PUMP_Pin,RESET);
}
bool updateTemp () {
	bool correctIn,correctOut;
	correctIn=AM2301A_getData(&am2301a_in,GPIOA,AM2301A_Pin);
	correctOut=AM2301A_getData(&am2301a_out,GPIOA,DHT11_Pin);
	/*if(correctIn) {
		printf("Inside sensor: Temperature =%.2f Humidity=%.2f  \n",am2301a_in.temperature,am2301a_in.humidity);
	}
	if(correctOut) {
		printf("Outside sensor: Temperature =%.2f Humidity=%.2f  \n",am2301a_out.temperature,am2301a_out.humidity);
	}*/
	if(correctIn && correctOut)
		return 1;

	return 0;
}
void shutDownDevice() {
//ToDo: standby mode;
}
/* USER CODE END FunctionPrototypes */

void Start_tempControlTask(void *argument);
void Start_humidityControlTask(void *argument);
void Start_mainTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	HAL_GPIO_WritePin(FAN_IN_GPIO_Port, FAN_IN_Pin, SET);
	  HAL_GPIO_WritePin(FAN_OUT_GPIO_Port, FAN_OUT_Pin, SET);
	  HAL_GPIO_WritePin(LED_PANEL_GPIO_Port, LED_PANEL_Pin, SET);
	  HAL_GPIO_WritePin(FAN_MIX_GPIO_Port, FAN_MIX_Pin, SET);
	  HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, RESET);
	  HAL_TIM_Base_Start(&htim1);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	  printf("################################################## \n"
			 "######### Growbox IOT - Booting device...######### \n"
	         "################################################## \n");
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of tempControlTask */
  tempControlTaskHandle = osThreadNew(Start_tempControlTask, NULL, &tempControlTask_attributes);

  /* creation of humidityControl */
  humidityControlHandle = osThreadNew(Start_humidityControlTask, NULL, &humidityControl_attributes);

  /* creation of mainTask */
  mainTaskHandle = osThreadNew(Start_mainTask, NULL, &mainTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Start_tempControlTask */
/**
  * @brief  Function implementing the tempControlTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_Start_tempControlTask */
void Start_tempControlTask(void *argument)
{
  /* USER CODE BEGIN Start_tempControlTask */
	bool correct;
	bool printMessage;
    uint8_t errorCounter=0;
    float tempIn, tempOut, humIn, humOut;
  /* Infinite loop */
  for(;;)
  {
	  if (sTime.Hours % 2==0)
	 	  turnOnFanMIX();
	  else
		  turnOffFanMIX();

	  if (sTime.Minutes % 20 == 0) {
		  printf("Actual time: %d:%d:%d ",sTime.Hours,sTime.Minutes,sTime.Seconds);
		  printf("Inside sensor: Temperature =%.2f Humidity=%.2f  \n",tempIn,humIn);
		  printf("Actual time: %d:%d:%d ",sTime.Hours,sTime.Minutes,sTime.Seconds);
		  printf("Outside sensor: Temperature =%.2f Humidity=%.2f  \n",tempOut,humOut);
	  }

	      correct = updateTemp();
	      if (correct) {
			  tempIn = am2301a_in.temperature;
			  humIn = am2301a_in.humidity;
			  tempOut = am2301a_out.temperature;
			  humOut = am2301a_out.humidity;
			  errorCounter = 0;


			  if (tempIn-tempOut >= 1 || humIn-humOut >= 5) {
				  turnOnFans();
				  if (printMessage) {
					  printf("Actual time: %d:%d:%d ",sTime.Hours,sTime.Minutes,sTime.Seconds);
					  printf("Inside sensor: Temperature =%.2f Humidity=%.2f  \n",tempIn,humIn);
					  printf("Actual time: %d:%d:%d ",sTime.Hours,sTime.Minutes,sTime.Seconds);
					  printf("Outside sensor: Temperature =%.2f Humidity=%.2f  \n",tempOut,humOut);
					  printf("Turning on the fans \n");
				  }
				  printMessage = 0;
			  }
			  else {
				  turnOffFans();
				  if (!printMessage) {
					  printf("Actual time: %d:%d:%d ",sTime.Hours,sTime.Minutes,sTime.Seconds);
					  printf("Outside sensor: Temperature =%.2f Humidity=%.2f  \n",tempOut,humOut);
					  printf("Actual time: %d:%d:%d ",sTime.Hours,sTime.Minutes,sTime.Seconds);
					  printf("Outside sensor: Temperature =%.2f Humidity=%.2f  \n",tempOut,humOut);
					  printf("Turning off the fans \n");
				  }
				  printMessage = 1;
			  }
			  vTaskDelay( (1000 * 60) / portTICK_RATE_MS ); //1min delay
	      } else {
	    	  if (errorCounter > 30) {
	    		  printf("Actual time: %d:%d:%d ",sTime.Hours,sTime.Minutes,sTime.Seconds);
	    		  printf ("BROKEN SENSORS! \n");
	    		  shutDownDevice();
	    	  }
	    	  errorCounter++;
	    	  vTaskDelay( 1000 / portTICK_RATE_MS ); //1sec delay
	      }

    osDelay(1);
  }
  /* USER CODE END Start_tempControlTask */
}

/* USER CODE BEGIN Header_Start_humidityControlTask */
/**
* @brief Function implementing the humidityControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_humidityControlTask */
void Start_humidityControlTask(void *argument)
{
  /* USER CODE BEGIN Start_humidityControlTask */
	 bool printMessage;
	 bool sensorError = 0;
	 uint8_t counter = 0;
	 uint16_t MH_sensor_val;
	 float MH_sensor_voltage;
  /* Infinite loop */
  for(;;)
  {
	  HAL_ADC_Start(&hadc1);
	  if (HAL_ADC_PollForConversion(&hadc1,10) == HAL_OK ) {
			  MH_sensor_val=HAL_ADC_GetValue(&hadc1);
			  MH_sensor_voltage=(MH_sensor_val/4096.00f)*3.3;
			  if ( MH_sensor_voltage > 2.5 && !sensorError) {
				turnOnPump();
				counter++;
				if (counter > 10) {
					printf("Actual time: %d:%d:%d ",sTime.Hours,sTime.Minutes,sTime.Seconds);
					printf("Humidity sensor ERROR! \n");
					sensorError = 1;
				}
				if(printMessage) {
					printf("Actual time: %d:%d:%d ",sTime.Hours,sTime.Minutes,sTime.Seconds);
					printf("MH_sensor = %d Voltage=%.2f V\n", MH_sensor_val, MH_sensor_voltage);
					printf("Turning ON the pump \n");
				}
				printMessage = 0;
				vTaskDelay( 1000 / portTICK_RATE_MS ); // 1s delay
			  } else {
				turnOffPump();
				if(!printMessage) {
					printf("Actual time: %d:%d:%d ",sTime.Hours,sTime.Minutes,sTime.Seconds);
					printf("MH_sensor = %d Voltage=%.2f V\n", MH_sensor_val, MH_sensor_voltage);
					printf("Turning OFF the pump \n");
				}
				printMessage = 1;
				vTaskDelay( (1000 * 60 * 60) / portTICK_RATE_MS ); //1hour delay
			  }
	  } else {
		  turnOffPump();
		  vTaskDelay( 1000 / portTICK_RATE_MS );
	  }

    osDelay(1);
  }
  /* USER CODE END Start_humidityControlTask */
}

/* USER CODE BEGIN Header_Start_mainTask */
/**
* @brief Function implementing the mainTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_mainTask */
void Start_mainTask(void *argument)
{
  /* USER CODE BEGIN Start_mainTask */
  bool printMessage;
  bool ledPanelSet = 0;
  uint8_t startLedPanelTime = 8;
  uint8_t duringGrowTime = 14;
  //ToDo: __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,1000); fan mix power control
  /* Infinite loop */
  for(;;)
  {
	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

	  if ((sTime.Hours >= startLedPanelTime) && (sTime.Hours < duringGrowTime+startLedPanelTime)) {
		  if (!ledPanelSet) {
			  turnOnLedPanel();
		  }
		  ledPanelSet = 1;

		  if (printMessage) {
			  printf("Actual time: %d:%d:%d ",sTime.Hours,sTime.Minutes,sTime.Seconds);
			  printf("Turning ON the LED Panel \n");
		  }
		  printMessage = 0;

	  } else {
		  if (ledPanelSet) {
			  turnOffLedPanel();
			  ledPanelSet = 0;
		  }
		  if (!printMessage) {
			  printf("Actual time: %d:%d:%d ",sTime.Hours,sTime.Minutes,sTime.Seconds);
			  printf("Turning OFF the LED Panel \n");
		  }
		  printMessage = 1;


	  }
	  vTaskDelay( 1000 / portTICK_RATE_MS );
	  osDelay(1);

  }
  /* USER CODE END Start_mainTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
