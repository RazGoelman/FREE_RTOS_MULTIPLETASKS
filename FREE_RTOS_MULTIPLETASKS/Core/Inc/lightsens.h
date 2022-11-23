#ifndef INC_LIGHTSENS_H_
#define INC_LIGHTSENS_H_
#include "main.h"

typedef struct _lightsens{
	ADC_HandleTypeDef* ADC;
	uint32_t value;
	uint32_t maxvalue;
	uint32_t minvalue;

}LIGHTSENS;

void lightsenseInit(LIGHTSENS* sensor, ADC_HandleTypeDef* ADC, uint32_t maxvalue);
int getValue(LIGHTSENS* sensor);

#endif /* INC_LIGHTSENS_H_ */
