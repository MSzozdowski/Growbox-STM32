#include "dht.h"
float concat(uint8_t a, uint8_t b){
    float c;
    c=(float)b;
    while(c>1.0f) c *= 0.1f;
    c=(float)a+c;
    return c;
}

uint8_t DHT11_Start (GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
	Set_Pin_as_Output(GPIOx,GPIO_Pin);
	HAL_GPIO_WritePin(GPIOx,GPIO_Pin,0);
	HAL_Delay(18);
    HAL_GPIO_WritePin(GPIOx,GPIO_Pin,1);
	Delay_us (40);
	Set_Pin_as_Input(GPIOx,GPIO_Pin);
	uint8_t output = 0;
	Delay_us(10);
	if ((HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)==0))
	{
		Delay_us (80);
		if ((HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)==1))
		output = 1;
		else
		output = -1;
	} else {
		output = -1;
	}
	if(output==1)
		while (HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)==1) {};

	return output;
}
uint8_t GetByte (GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
	uint8_t result=0;
	for(int j=0;j<8;j++)
		{
			while(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)==0) {}
			Delay_us(40);
			if (HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)==1) {
				result+=1<<(7-j);
				while(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)==1) {}
			}
	}
return result;
}
bool DHT11_getData (SENSOR *dht11,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin) {
	uint8_t hum_int=0;
	uint8_t hum_dec=0;
	uint8_t temp_int=0;
	uint8_t temp_dec=0;
	uint8_t check_sum=0;
	uint8_t start=DHT11_Start (GPIOx,GPIO_Pin);

	if(start==1) {
		hum_int=GetByte(GPIOx,GPIO_Pin);
		hum_dec=GetByte(GPIOx,GPIO_Pin);
		dht11->humidity=concat(hum_int,hum_dec);
		temp_int=GetByte(GPIOx,GPIO_Pin);
		temp_dec=GetByte(GPIOx,GPIO_Pin);
		dht11->temperature=concat(temp_int,temp_dec);;
		check_sum=GetByte(GPIOx,GPIO_Pin);
		uint8_t real_sum=hum_int+hum_dec+temp_int+temp_dec;
		if (check_sum==real_sum) {
			return true;
		} else {
			printf("Incorrect checksum %d + %d + %d + %d = %d \n",hum_int,hum_dec,temp_int,temp_dec,check_sum);
			return false;
		}
	} else {
		printf("Incorrect start \n");
		return false;
	}
}

uint8_t AM2301A_Start (GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
	Set_Pin_as_Output (GPIOx, GPIO_Pin);
	HAL_GPIO_WritePin (GPIOx, GPIO_Pin, 0);
	HAL_Delay(1);
    HAL_GPIO_WritePin (GPIOx, GPIO_Pin, 1);
	Delay_us (40);
	Set_Pin_as_Input(GPIOx, GPIO_Pin);
	uint8_t output = 0;
	Delay_us(10);
	if ((HAL_GPIO_ReadPin (GPIOx, GPIO_Pin)==0))
		{
		Delay_us (80);
		if ((HAL_GPIO_ReadPin (GPIOx, GPIO_Pin)==1)) output = 1;
		else  output = -1;
		} else {
			output = -1;
		}
	if(output==1)
		while ((HAL_GPIO_ReadPin (GPIOx, GPIO_Pin)==1));
	return output;
}

bool AM2301A_getData(SENSOR *am2301,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin) {
	uint8_t hum_int=0;
	uint8_t hum_dec=0;
	uint8_t temp_int=0;
	uint8_t temp_dec=0;
	uint8_t check_sum=0;
	uint8_t start=AM2301A_Start (GPIOx,GPIO_Pin);

	if(start==1) {
		hum_int=GetByte(GPIOx,GPIO_Pin);
		hum_dec=GetByte(GPIOx,GPIO_Pin);
		am2301->humidity=(float)(((hum_int<<8)|hum_dec)/10.0);
		temp_int=GetByte(GPIOx,GPIO_Pin);
		temp_dec=GetByte(GPIOx,GPIO_Pin);
		am2301->temperature=(float)(((temp_int<<8)|temp_dec)/10.0);
		check_sum=GetByte(GPIOx,GPIO_Pin);
		uint8_t real_sum=hum_int+hum_dec+temp_int+temp_dec;
		if (check_sum==real_sum){
			return true;
		} else {
			printf("Incorrect checksum %d + %d + %d + %d != %d \n",hum_int,hum_dec,temp_int,temp_dec,check_sum);
			return false;
		}
	} else {
		printf("Incorrect start \n");
		return false;
	}
}
