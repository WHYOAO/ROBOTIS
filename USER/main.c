#include "sys.h"
#include <stdlib.h>
#include "string.h"
#include "delay.h"
#include "uart.h" 
#include "pwm.h"
#include "adc.h"
#include "can.h"
#include "gait.h"
#include "SV.h"
//#include "matrix.h"
#include "timer.h"
#include "VOFA.h"

#include "Pcpg_controller.h"
#include "Basic_cpg_controller.h"
#include "Delay_line.h"

#define PCPG_THRES 0.848 // control swingcd, phase pcpg signal 16 period
#define PCPG_LIFT_THRES 0.87 // control swing phase pcpg signal 16 period
#define PCPG_PP_THRES 0.87 // control pneumatic positive pcpg signal 8 period
#define PCPG_PN_THRES 0.865 // control pneumatic negetive pcpg signal 12 period
#define PCPG_1_SLOPE 20. // control the downward slope of pcpg signal joint 1
#define PCPG_2_SLOPE 2.  // joint 23
#define PCPG_PP_SLOPE 80.
#define PCPG_PN_SLOPE 80.

#define DELAYSIZE 96
#define DELAYRF 0
#define DELAYRH 24
#define DELAYLF 48
#define DELAYLH 72
#define DELAYPN 0
#define DELAYPP 14
#define DELAYSS 2

int Buf[128];

int main()
{
	Cache_Enable();                		//打开L1-Cache
	HAL_Init();				        	//初始化HAL库
	Stm32_Clock_Init(160,5,2,4);  	    //设置时钟,400Mhz 
	delay_init(400);					//延时初始化
	SV_Init();
	HAL_Delay(1000);
	TIM5_Init(10-1,200-1);//1us *10
	USART1_init(115200);
	MY_UART7_Init();			//VOFA
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_RESET);

	
	struct Basic_cpg_controller cpg;
	CLASS_BCC_setParameter(&cpg, 0.885, -0.59268, 0, 1.4, 0.18, 0.025); // 20 Hz 50 ms for 1 period

	struct Pcpg_controller pcpg_1;
	CLASS_PC_Pcpg_controller_Ini(&pcpg_1);
	CLASS_PC_setParameter(&pcpg_1, PCPG_1_SLOPE,PCPG_LIFT_THRES);  // lift motor        CPG_1_SLOPE,PCPG_LIFT_THRES
	struct Pcpg_controller pcpg_2;
	CLASS_PC_Pcpg_controller_Ini(&pcpg_2);
	CLASS_PC_setParameter(&pcpg_2, PCPG_2_SLOPE,PCPG_THRES);  // swing phase & stance phase     PCPG_2_SLOPE,PCPG_THRES
	struct Pcpg_controller pcpg_pneumatic_pos;
	CLASS_PC_Pcpg_controller_Ini(&pcpg_pneumatic_pos);
	CLASS_PC_setParameter(&pcpg_pneumatic_pos, PCPG_PP_SLOPE,PCPG_PP_THRES);  // PCPG_PP_SLOPE,PCPG_PP_THRES
	struct Pcpg_controller pcpg_pneumatic_neg;
	CLASS_PC_Pcpg_controller_Ini(&pcpg_pneumatic_neg);
	CLASS_PC_setParameter(&pcpg_pneumatic_neg, PCPG_PP_SLOPE,PCPG_PN_THRES);  // PCPG_PP_SLOPE,PCPG_PN_THRES

	struct Delay_line pneumatic_neg_delay;
	CLASS_DL_setParameter(&pneumatic_neg_delay, DELAYSIZE);
	struct Delay_line pneumatic_pos_delay;
	CLASS_DL_setParameter(&pneumatic_pos_delay, DELAYSIZE);
	struct Delay_line joint23_delay;
	CLASS_DL_setParameter(&joint23_delay, DELAYSIZE);
	struct Delay_line joint1_delay;
	CLASS_DL_setParameter(&joint1_delay, DELAYSIZE);


	// output data
//	FILE *fp = NULL , *fop = NULL;
//	fp = fopen("D:/why/code/stm32h7_geckoCPG/pcpg_signal.txt","w") ;
//	fop = fopen("D:/why/code/stm32h7_geckoCPG/op.txt","w") ;
	for(int i=0; i<300; i++)
	{
			CLASS_BCC_run(&cpg);
			float op1, op2;
			op1 = CLASS_BCC_getSignal(&cpg, 1);
			op2 = CLASS_BCC_getSignal(&cpg, 2); 
			
			//fprintf(fop,"%f\t%f\n", op1, op2);

			CLASS_PC_run(&pcpg_1, op1, op2);
			CLASS_PC_run(&pcpg_2, op1, op2);
			CLASS_PC_run(&pcpg_pneumatic_pos, op1, op2);
			CLASS_PC_run(&pcpg_pneumatic_neg, op1, op2);

			CLASS_DL_writeIn(&pneumatic_neg_delay, CLASS_PC_getSignal(&pcpg_pneumatic_neg, 2));
			CLASS_DL_writeIn(&pneumatic_pos_delay, CLASS_PC_getSignal(&pcpg_pneumatic_pos, 2));
			CLASS_DL_writeIn(&joint1_delay, CLASS_PC_getSignal(&pcpg_1, 2));
			CLASS_DL_writeIn(&joint23_delay, CLASS_PC_getSignal(&pcpg_2, 2));

			float pcpg_signal[16];
			pcpg_signal[0] = CLASS_DL_readFr(&pneumatic_neg_delay, DELAYRF + DELAYPN);
			pcpg_signal[1] = CLASS_DL_readFr(&joint1_delay, DELAYRF + DELAYSS);
			pcpg_signal[2] = CLASS_DL_readFr(&joint23_delay, DELAYRF + DELAYSS);
			pcpg_signal[3] = CLASS_DL_readFr(&pneumatic_pos_delay, DELAYRF + DELAYPP);
			pcpg_signal[4] = CLASS_DL_readFr(&pneumatic_neg_delay, DELAYRH + DELAYPN);
			pcpg_signal[5] = CLASS_DL_readFr(&joint1_delay, DELAYRH + DELAYSS);
			pcpg_signal[6] = CLASS_DL_readFr(&joint23_delay, DELAYRH + DELAYSS);
			pcpg_signal[7] = CLASS_DL_readFr(&pneumatic_pos_delay, DELAYRH + DELAYPP);
			pcpg_signal[8] = CLASS_DL_readFr(&pneumatic_neg_delay, DELAYLF + DELAYPN);
			pcpg_signal[9] = CLASS_DL_readFr(&joint1_delay, DELAYLF + DELAYSS);
			pcpg_signal[10] = CLASS_DL_readFr(&joint23_delay, DELAYLF + DELAYSS);
			pcpg_signal[11] = CLASS_DL_readFr(&pneumatic_pos_delay, DELAYLF + DELAYPP);
			pcpg_signal[12] = CLASS_DL_readFr(&pneumatic_neg_delay, DELAYLH + DELAYPN);
			pcpg_signal[13] = CLASS_DL_readFr(&joint1_delay, DELAYLH + DELAYSS);
			pcpg_signal[14] = CLASS_DL_readFr(&joint23_delay, DELAYLH + DELAYSS);
			pcpg_signal[15] = CLASS_DL_readFr(&pneumatic_pos_delay, DELAYLH + DELAYPP);

			CLASS_DL_step_one(&pneumatic_neg_delay);
			CLASS_DL_step_one(&pneumatic_pos_delay);
			CLASS_DL_step_one(&joint1_delay);
			CLASS_DL_step_one(&joint23_delay);
	}


	for(int i=0; i<300; i++)
	{
			CLASS_BCC_run(&cpg);
			float op1, op2;
			op1 = CLASS_BCC_getSignal(&cpg, 1);
			op2 = CLASS_BCC_getSignal(&cpg, 2);

//			fprintf(fop,"%f\t%f\n", op1, op2);

			CLASS_PC_run(&pcpg_1, op1, op2);
			CLASS_PC_run(&pcpg_2, op1, op2);
			CLASS_PC_run(&pcpg_pneumatic_pos, op1, op2);
			CLASS_PC_run(&pcpg_pneumatic_neg, op1, op2);

			CLASS_DL_writeIn(&pneumatic_neg_delay, CLASS_PC_getSignal(&pcpg_pneumatic_neg, 2));
			CLASS_DL_writeIn(&pneumatic_pos_delay, CLASS_PC_getSignal(&pcpg_pneumatic_pos, 2));
			CLASS_DL_writeIn(&joint1_delay, CLASS_PC_getSignal(&pcpg_1, 2));
			CLASS_DL_writeIn(&joint23_delay, CLASS_PC_getSignal(&pcpg_2, 2));

			float pcpg_signal[16];
			pcpg_signal[0] = CLASS_DL_readFr(&pneumatic_neg_delay, DELAYRF + DELAYPN);
			pcpg_signal[1] = CLASS_DL_readFr(&joint1_delay, DELAYRF + DELAYSS);         //pcpg_1
			pcpg_signal[2] = CLASS_DL_readFr(&joint23_delay, DELAYRF + DELAYSS);    //pcpg_2
			pcpg_signal[3] = CLASS_DL_readFr(&pneumatic_pos_delay, DELAYRF + DELAYPP);
			pcpg_signal[4] = CLASS_DL_readFr(&pneumatic_neg_delay, DELAYRH + DELAYPN);
			pcpg_signal[5] = CLASS_DL_readFr(&joint1_delay, DELAYRH + DELAYSS);
			pcpg_signal[6] = CLASS_DL_readFr(&joint23_delay, DELAYRH + DELAYSS);
			pcpg_signal[7] = CLASS_DL_readFr(&pneumatic_pos_delay, DELAYRH + DELAYPP);
			pcpg_signal[8] = CLASS_DL_readFr(&pneumatic_neg_delay, DELAYLF + DELAYPN);
			pcpg_signal[9] = CLASS_DL_readFr(&joint1_delay, DELAYLF + DELAYSS);
			pcpg_signal[10] = CLASS_DL_readFr(&joint23_delay, DELAYLF + DELAYSS);
			pcpg_signal[11] = CLASS_DL_readFr(&pneumatic_pos_delay, DELAYLF + DELAYPP);
			pcpg_signal[12] = CLASS_DL_readFr(&pneumatic_neg_delay, DELAYLH + DELAYPN);
			pcpg_signal[13] = CLASS_DL_readFr(&joint1_delay, DELAYLH + DELAYSS);
			pcpg_signal[14] = CLASS_DL_readFr(&joint23_delay, DELAYLH + DELAYSS);
			pcpg_signal[15] = CLASS_DL_readFr(&pneumatic_pos_delay, DELAYLH + DELAYPP);
			float pneumatic_flag[4];
			//会把4条数据归一到1-0
//			for(int j=0; j<4; j++)
//			{
//				 if(pcpg_signal[j*4]>-1) pcpg_signal[j*4] = 1; else pcpg_signal[j*4] = 0;
//				 if(pcpg_signal[j*4+3]>-1) pcpg_signal[j*4+3] = 1; else pcpg_signal[j*4+3] = 0;
//				 pneumatic_flag[j] = pcpg_signal[j*4] - pcpg_signal[j*4+3];
//			}
				for(int j=0;j<16;j++)
			{
				my_data_buffer[j] = pcpg_signal[j];				
			}
			Send_channels=16;
			vofa_send_lines();
//			for(int j=0; j<16; j++)
//			{
//				 if(j!=15)
//						 printf("%f, ",pcpg_signal[j]);
//				 else
//						 printf("%f \n",pcpg_signal[j]);
//			}

//			for(int j=0; j<16; j++)
//			{
//					if(j!=15)
//							fprintf(fp,"%f\t",pcpg_signal[j]);
//					else
//							fprintf(fp,"%f\n",pcpg_signal[j]);
//			}
			

			CLASS_DL_step_one(&pneumatic_neg_delay);
			CLASS_DL_step_one(&pneumatic_pos_delay);
			CLASS_DL_step_one(&joint1_delay);
			CLASS_DL_step_one(&joint23_delay);
	}
	CLASS_DL_free(&pneumatic_neg_delay);
	CLASS_DL_free(&pneumatic_pos_delay);
	CLASS_DL_free(&joint1_delay);
	CLASS_DL_free(&joint23_delay);


//	fclose(fp);
//	fclose(fop);


	while(1)
	{
		//usleep(1/loopRate4*1e6 - (timeUse) - 10); 

		//vofa_send_lines();
		//HAL_UART_Transmit(&huart7,&c,sizeof(c),0xffff);
		delay_ms(100);

	}




}





