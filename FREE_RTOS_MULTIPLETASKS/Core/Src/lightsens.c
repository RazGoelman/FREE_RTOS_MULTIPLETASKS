#include "lightsens.h"
#include "main.h"

void lightsenseInit(LIGHTSENS* sensor, ADC_HandleTypeDef* ADC,uint32_t maxvalue)
{
	sensor->ADC = ADC;
	sensor->maxvalue = maxvalue;
	//sensor->minvalue = 0;
}

int getValue(LIGHTSENS* sensor)
{
	sensor->value = HAL_ADC_GetValue(sensor->ADC);
	return sensor->value;
}

