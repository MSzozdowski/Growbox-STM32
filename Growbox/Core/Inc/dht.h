#include <stdbool.h>
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"

typedef struct {
	float temperature;
	float humidity;
}SENSOR;
float concat(uint8_t a, uint8_t b);
uint8_t DHT11_Start (GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);
uint8_t AM2301A_Start(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);
uint8_t GetByte (GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);
bool DHT11_getData(SENSOR *dht,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);
bool AM2301A_getData(SENSOR *am2301,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);
