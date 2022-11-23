#include "Rtc.h"
#include "lightsens.h"
#include "main.h"
#include "mybuzzer.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cmsis_os.h"
extern LIGHTSENS sensor;
extern ADC_HandleTypeDef hadc1;
extern Rtc rtc;
static int sensValue = 0;
extern DateTime dateTime2;
extern TIM_HandleTypeDef htim3;
extern BUZZER buzzer;
extern int song[];
extern int length[];
int count = 0;

void buzzer_task(void *argument)
{
  /* USER CODE BEGIN buzzer_task */
	buzzer.song = song;
	buzzer.frames = length;
	__HAL_TIM_SET_AUTORELOAD(&htim3,buzzer.song[count++]/2);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,((buzzer.song[count++]/2)/2));
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  /* Infinite loop */
  for(;;)
  {
  __HAL_TIM_SET_AUTORELOAD(&htim3,buzzer.song[count++]/2);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,((buzzer.song[count++]/2)/2));
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    osDelay(buzzer.frames[count++]);
  }
  /* USER CODE END buzzer_task */
}
void RTC_func(void *argument)
{
  /* USER CODE BEGIN RTC_func */
  /* Infinite loop */
  for(;;)
  {
		rtcGetTime(&rtc, &dateTime2);

    osDelay(1000);
  }
  /* USER CODE END RTC_func */
}

 void LIGHT_sensor(void *argument)
 {
   /* USER CODE BEGIN LIGHT_sensor */
   /* Infinite loop */
   for(;;)
   {
	   HAL_ADC_Start(&hadc1);
 	 sensValue = HAL_ADC_GetValue(&hadc1);
     osDelay(1000);
   }
   /* USER CODE END LIGHT_sensor */
 }
 void printTask(void *argument)
{
  /* USER CODE BEGIN printTask */
  /* Infinite loop */
  for(;;)
  {
	  printf("%02d:%02d:%02d-%d-%02d/%02d/%02d\r\n",
	  	    					dateTime2.hours, dateTime2.min, dateTime2.sec,
	  	    					dateTime2.weekDay,
	  	    					dateTime2.day, dateTime2.month, dateTime2.year);
	  printf("value is %d \r\n",sensValue);
    osDelay(1000);
  }
  /* USER CODE END printTask */
}
