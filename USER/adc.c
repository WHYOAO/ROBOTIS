//#include "adc.h"	  

//ADC_HandleTypeDef ADC1_Handler;//ADC句柄 

//uint16_t adc1;
//uint16_t adc2;
//uint16_t adc3;
//uint16_t adc4;

//uint16_t P1;
//uint16_t P2;
//uint16_t P3;
//uint16_t P4;

//uint8_t adc_1_H;
//uint8_t adc_1_L;
//uint8_t adc_2_H;
//uint8_t adc_2_L;
//uint8_t adc_3_H;
//uint8_t adc_3_L;
//uint8_t adc_4_H;
//uint8_t adc_4_L;

//void MX_ADC1_Init(void)
//{
//  ADC_MultiModeTypeDef multimode = {0};
//  ADC_ChannelConfTypeDef sConfig = {0};

//  ADC1_Handler.Instance = ADC1;
//  ADC1_Handler.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;//fen ping
//  ADC1_Handler.Init.Resolution = ADC_RESOLUTION_16B;//wei shu
//  ADC1_Handler.Init.ScanConvMode = ADC_SCAN_DISABLE;//非扫描模式
//  ADC1_Handler.Init.EOCSelection = ADC_EOC_SINGLE_CONV;//Eoc中断
//  ADC1_Handler.Init.LowPowerAutoWait = DISABLE;
//  ADC1_Handler.Init.ContinuousConvMode = DISABLE;//连续转换模式关闭
//  ADC1_Handler.Init.NbrOfConversion = 1;//不连续采样通道数1
//  ADC1_Handler.Init.DiscontinuousConvMode = DISABLE;
//  
//  ADC1_Handler.Init.ExternalTrigConv = ADC_SOFTWARE_START;//软件触发
//  ADC1_Handler.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;//触发极性
//  ADC1_Handler.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
//  ADC1_Handler.Init.Overrun = ADC_OVR_DATA_PRESERVED;
//  ADC1_Handler.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
//  ADC1_Handler.Init.OversamplingMode = DISABLE;

//	HAL_ADC_Init(&ADC1_Handler);  
//	
//  sConfig.Channel = ADC_CHANNEL_14;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
//  sConfig.SingleDiff = ADC_SINGLE_ENDED;
//  sConfig.OffsetNumber = ADC_OFFSET_NONE;
//  sConfig.Offset = 0;

//}



//uint16_t Get_Adc(ADC_HandleTypeDef *hadc,uint32_t ch)   
//{
//	ADC_ChannelConfTypeDef ADC_ChanConf;
//    
//	ADC_ChanConf.Channel=ch;                                   //通道
//	ADC_ChanConf.Rank=ADC_REGULAR_RANK_1;                  	//1个序列
//	ADC_ChanConf.SamplingTime=ADC_SAMPLETIME_64CYCLES_5;      	//采样时间       
//	ADC_ChanConf.SingleDiff=ADC_SINGLE_ENDED;  				//单边采集          		
//	ADC_ChanConf.OffsetNumber=ADC_OFFSET_NONE;             	
//	ADC_ChanConf.Offset=0;   
//	HAL_ADC_ConfigChannel(hadc,&ADC_ChanConf);        //通道配置

//	HAL_ADC_Start(hadc);                               //开启ADC
//	
//	HAL_ADC_PollForConversion(hadc,10);                //轮询转换
//	return (uint16_t)HAL_ADC_GetValue(hadc);	            //返回最近一次ADC规则组的转换结果
//}

//uint16_t Get_Adc_Average(ADC_HandleTypeDef *hadc,uint32_t ch,uint8_t times)
//{
//	uint32_t temp_val=0;
//	uint8_t t;
//	for(t=0;t<times;t++)
//	{
//		temp_val+=Get_Adc(hadc,ch);
//		HAL_Delay(5);
//	}
//	return temp_val/times;
//} 

//void Read_All_Ad()
//{
//		adc1 = Get_Adc_Average(&ADC1_Handler,ADC_CHANNEL_16,AD_AVERAGE);
//		adc2 = Get_Adc_Average(&ADC1_Handler,ADC_CHANNEL_17,AD_AVERAGE);
//		adc3 = Get_Adc_Average(&ADC1_Handler,ADC_CHANNEL_14,AD_AVERAGE);
//		adc4 = Get_Adc_Average(&ADC1_Handler,ADC_CHANNEL_15,AD_AVERAGE);

//		adc_1_H = adc1>>8;
//		adc_1_L = adc1&0xff;
//		adc_2_H = adc2>>8;
//		adc_2_L = adc2&0xff;
//		adc_3_H = adc3>>8;
//		adc_3_L = adc3&0xff;
//		adc_4_H = adc4>>8;
//		adc_4_L = adc4&0xff;
//	}

//void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  if(hadc->Instance==ADC1)
//  {
//    __HAL_RCC_ADC12_CLK_ENABLE();
//    __HAL_RCC_GPIOA_CLK_ENABLE();
//    /**ADC1 GPIO Configuration    
//		PA0			------> ADC1_INP16
//    PA1     ------> ADC1_INP17
//    PA2     ------> ADC1_INP14
//    PA3     ------> ADC1_INP15

//    */
//    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
//    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//  }
//}
//void Switch()
//{
//	
//	P1 = 50*(adc1-2.5);
//	P2 = 50*(adc2-2.5);
//	P3 = 50*(adc3-2.5);
//	P4 = 50*(adc4-2.5);
//}