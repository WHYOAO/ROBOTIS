#include "SV.h"
void SV_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOC_CLK_ENABLE();					//����GPIOBʱ��
	  
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|
										 GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|
	                   GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12;			//PB0��1
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  		//�������
    GPIO_Initure.Pull=GPIO_NOPULL;         			//����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_LOW;  	//����
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);     		//��ʼ��GPIOB.0��GPIOB.1
	
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);	//PB0��0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);	//PB1��1 
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);	//PB0��0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);	//PB1��1 
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);	//PB0��0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);	//PB1��1 
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);	//PB0��0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);	//PB1��1 
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);	//PB0��0
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);	//PB1��1 
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET);
		__HAL_RCC_GPIOB_CLK_ENABLE();
		GPIO_Initure.Pin=GPIO_PIN_8;
		GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  		//�������
    GPIO_Initure.Pull=GPIO_NOPULL;         			//����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_LOW;  	//����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure); 
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
}
