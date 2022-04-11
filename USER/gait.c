#include "gait.h"
#include <math.h>
#include "pwm.h"	
#include "capture.h"

const short L1=38;
const short L2=47;
const short L3=22;
float cx[4]={0};
float cy[4]={0};
float cz[4]={0};

u8 stepm =30;//迈腿的步数
u8 stepl =30;//落腿的步数
u8 steph =30;//回拉步数

int t =0;
int i =0;

KinematicsArm KMGecko;

void setcurrentposition(int ft,float x,float y,float z)
{
	if (ft==LF)
	{cx[0]=x;
	 cy[0]=y;
	 cz[0]=z;
	}
	
	if (ft==RF)
	{cx[1]=x;
	 cy[1]=y;
	 cz[1]=z;
	}
	
	if (ft==LH)
	{cx[2]=x;
	 cy[2]=y;
	 cz[2]=z;
	}

	if (ft==RH)
	{cx[3]=x;
	 cy[3]=y;
	 cz[3]=z;
	}

}


void reverse4(float width,float length,float height,u8 step)
{
	float x[4]={0};
	float y[4]={0};
	float z[4]={0};
	
	float a[4][3] = {0};
	float theta[4][2] = {0};
	
	for(t=0;t<step + 1;t++)		
		{		
			for(i=0;i<4;i++)
				{	
					x[i]=cx[i] + width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] - (height / (step * step) * t * t) + (2 * height / step * t);
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
					
					theta[0][0]=(a[0][0]+a[0][1]);
			    theta[0][1]=(a[0][1]-a[0][0]);
			
					theta[0][0]=(int) (theta[0][0]/3.1415926*180*100);//rf alpha 角，最内侧
					theta[0][1]=(int) (theta[0][1]/3.1415926*180*100);
					a[0][2]=(int) (a[0][2]/3.1415926*180*100);

					theta[0][0]= (float) (theta[0][0]/100);
					theta[0][1]= (float) (theta[0][1]/100);
					a[0][2]= (float) (a[0][2]/100);
					
					theta[1][0]=(a[1][1]+a[1][0]);
			    theta[1][1]=(a[1][1]-a[1][0]);
			
			    theta[1][0]=(int) (theta[1][0]/3.1415926*180*100);//rf alpha 角，最内侧
			    theta[1][1]=(int) (theta[1][1]/3.1415926*180*100);
			    a[1][2]=(int) (a[1][2]/3.1415926*180*100);

			    theta[1][0]= (float) (-theta[1][0]/100);
			    theta[1][1]= (float) (-theta[1][1]/100);
			    a[1][2]= (float) (-a[1][2]/100);
					
					theta[2][0]=(a[2][1]-a[2][0]);
			    theta[2][1]=(a[2][1]+a[2][0]);
			
			    theta[2][0]=(int) (theta[2][0]/3.1415926*180*100);//lf alpha 角，最内侧
					theta[2][1]=(int) (theta[2][1]/3.1415926*180*100);
					a[2][2]=(int) (a[2][2]/3.1415926*180*100);
			
	
					theta[2][0]= (float) (theta[2][0]/100);
					theta[2][1]= (float) (theta[2][1]/100);
					a[2][2]= (float) (a[2][2]/100);
					
					theta[3][0]=(int) (theta[3][0]/3.1415926*180*100);//rf alpha 角，最内侧
		    	theta[3][1]=(int) (theta[3][1]/3.1415926*180*100);
					a[3][2]=(int) (a[3][2]/3.1415926*180*100);

					theta[3][0]= (float) (-theta[3][0]/100);
					theta[3][1]= (float) (-theta[3][1]/100);
					a[3][2]= (float) (-a[3][2]/100);
			
					
				}
					
				
				
					Angle(theta[0][0]+KMGecko.StartAngle[RF_J3],RF_J3);
					Angle(theta[0][1]+KMGecko.StartAngle[RF_J2],RF_J2);
					Angle(a[0][2]+KMGecko.StartAngle[RF_J1],RF_J1);
						
					Angle(theta[1][0]+KMGecko.StartAngle[RH_J3],RH_J3);
					Angle(theta[1][1]+KMGecko.StartAngle[RH_J2],RH_J2);
					Angle(a[1][2]+KMGecko.StartAngle[RH_J1],RH_J1);
						
					Angle(theta[2][0]+KMGecko.StartAngle[LF_J3],LF_J3);
					Angle(theta[2][1]+KMGecko.StartAngle[LF_J2],LF_J2);
					Angle(a[2][2]+KMGecko.StartAngle[LF_J1],LF_J1);
						
					Angle(theta[3][0]+KMGecko.StartAngle[LH_J3],LH_J3);
					Angle(theta[3][1]+KMGecko.StartAngle[LH_J2],LH_J2);
					Angle(a[3][2]+KMGecko.StartAngle[LH_J1],LH_J1);		
					
					//Read_All_Ad();
					//HAL_UART_Transmit(&huart4,(uint8_t*)256,2,1000);
					//HAL_UART_Transmit(&huart4,(uint8_t*)512,2,1000);
					//HAL_UART_Transmit(&huart4,(uint8_t*)1024,2,1000);
					HAL_Delay(20);
	}
					setcurrentposition(RF,x[0],y[0],z[0]);
					setcurrentposition(RH,x[1],y[1],z[1]);
					setcurrentposition(LF,x[2],y[2],z[2]);
					setcurrentposition(LH,x[3],y[3],z[3]);

	
}

void reverse(int fn,float width,float length,float height,u8 step)
	//length x 左右
  //width y 前进
{
	float x;
	float y;
	float z;
	
	float a[4][3] = {0};     //   a1 = a[0]   a2=a[LF]
	float theta[4][2] = {0};
	if (fn==LF)
{
		for(t=0;t<step + 1;t++)		
		{
			x=cx[0] + width / step * t;
			y=cy[0] + length / step * t;
			z=cz[0] - (height / (step * step) * t * t) + (2 * height / step * t);
			
			a[0][0] = asin(-L3 / sqrt(x * x + z * z) ) - atan2(z,x);
			a[0][1] = asin((x * x + y * y + z * z + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x * x +  y * y + z * z - L3 * L3)) )
							- atan2(sqrt(x * x + z * z - L3 * L3) , y);
			a[0][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x * x - y * y - z * z) / (2 * L1 * L2));
			
			
			theta[0][0]=(a[0][0]+a[0][1]);
			theta[0][1]=(a[0][1]-a[0][0]);
			
			theta[0][0]=(int) (theta[0][0]/3.1415926*180*100);//rf alpha 角，最内侧
			theta[0][1]=(int) (theta[0][1]/3.1415926*180*100);
			a[0][2]=(int) (a[0][2]/3.1415926*180*100);

			theta[0][0]= (float) (theta[0][0]/100);
			theta[0][1]= (float) (theta[0][1]/100);
			a[0][2]= (float) (a[0][2]/100);
			
//			Angle(theta[0][0]+KMGecko.StartAngle[LF_J3],LF_J3);//jianxiao
//			Angle(theta[0][1]+KMGecko.StartAngle[LF_J2],LF_J2);//jianda
//			Angle(a[0][2]+KMGecko.StartAngle[LF_J1],LF_J1);//zhou
			
			HAL_Delay(20);
		}
			setcurrentposition(LF,x,y,z);
}
	
	if (fn==RF)
	{
		//length=-length;
		for(t=0;t<step + 1;t++)		
		{
			x=cx[1] + width / step * t;
			y=cy[1] + length / step * t;
			z=cz[1] - (height / (step * step) * t * t) + (2 * height / step * t);
			
			a[1][0] = asin(-L3 / sqrt(x * x + z * z) ) - atan2(z,x);
			a[1][1] = asin((x * x + y * y + z * z + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x * x +  y * y + z * z - L3 * L3)) )
					 - atan2(sqrt(x * x + z * z - L3 * L3) , y);
			a[1][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x * x - y * y - z * z) / (2 * L1 * L2));
			
			theta[1][0]=(a[1][1]+a[1][0]);
			theta[1][1]=(a[1][1]-a[1][0]);
			
			theta[1][0]=(int) (theta[1][0]/3.1415926*180*100);//rf alpha 角，最内侧
			theta[1][1]=(int) (theta[1][1]/3.1415926*180*100);
			a[1][2]=(int) (a[1][2]/3.1415926*180*100);

			theta[1][0]= (float) (-theta[1][0]/100);
			theta[1][1]= (float) (-theta[1][1]/100);
			a[1][2]= (float) (-a[1][2]/100);
			
//			Angle(theta[1][0]+KMGecko.StartAngle[RF_J3],RF_J3);
//			Angle(theta[1][1]+KMGecko.StartAngle[RF_J2],RF_J2);
//			Angle(a[1][2]+KMGecko.StartAngle[RF_J1],RF_J1);
			
			
			HAL_Delay(20);
		}
			setcurrentposition(RF,x,y,z);
	}
	if (fn==LH)
	{
		for(t=0;t<step + 1;t++)		
		{
			x=cx[2] + width / step * t;
			y=cy[2] + length / step * t;
			z=cz[2] - (height / (step * step) * t * t) + (2 * height / step * t);
			
			a[2][0] = asin(-L3 / sqrt(x * x + z * z) ) - atan2(z,x);
			a[2][1] = asin((x * x + y * y + z * z + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x * x +  y * y + z * z - L3 * L3)) )
					    - atan2(sqrt(x * x + z * z - L3 * L3) , y);
	  //a[0][1] = asin((x * x + y * y + z * z + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x * x +  y * y + z * z - L3 * L3)) )
		//				- atan2(sqrt(x * x + z * z - L3 * L3) , y);
			a[2][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x * x - y * y - z * z) / (2 * L1 * L2));
			
			theta[2][0]=(a[2][1]-a[2][0]);
			theta[2][1]=(a[2][1]+a[2][0]);
			
			theta[2][0]=(int) (theta[2][0]/3.1415926*180*100);//lf alpha 角，最内侧
			theta[2][1]=(int) (theta[2][1]/3.1415926*180*100);
			a[2][2]=(int) (a[2][2]/3.1415926*180*100);
			

			theta[2][0]= (float) (theta[2][0]/100);
			theta[2][1]= (float) (theta[2][1]/100);
			a[2][2]= (float) (a[2][2]/100);
			
//			Angle(theta[2][0]+KMGecko.StartAngle[LH_J3],LH_J3);
//			Angle(theta[2][1]+KMGecko.StartAngle[LH_J2],LH_J2);
//			Angle(a[2][2]+KMGecko.StartAngle[LH_J1],LH_J1);
			
		
			
			HAL_Delay(20);
		}
			setcurrentposition(LH,x,y,z);
	}
	
	if (fn==RH)
	{
		//length=-length;
		for(t=0;t<step + 1;t++)		
		{
			x=cx[3] + width / step * t;
			y=cy[3] + length / step * t;
			z=cz[3] - (height / (step * step) * t * t) + (2 * height / step * t);
			
			a[3][0] = asin(-L3 / sqrt(x * x + z * z) ) - atan2(z,x);
			a[3][1] = asin((x * x + y * y + z * z + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x * x +  y * y + z * z - L3 * L3)) )
					 - atan2(sqrt(x * x + z * z - L3 * L3) , y);
			a[3][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x * x - y * y - z * z) / (2 * L1 * L2));
			
			theta[3][0]=(a[3][1]-a[3][0]);
			theta[3][1]=(a[3][0]+a[3][1]);
			
			theta[3][0]=(int) (theta[3][0]/3.1415926*180*100);//rf alpha 角，最内侧
			theta[3][1]=(int) (theta[3][1]/3.1415926*180*100);
			a[3][2]=(int) (a[3][2]/3.1415926*180*100);

			theta[3][0]= (float) (-theta[3][0]/100);
			theta[3][1]= (float) (-theta[3][1]/100);
			a[3][2]= (float) (-a[3][2]/100);
			
//			Angle(theta[3][0]+KMGecko.StartAngle[RH_J3],RH_J3);
//			Angle(theta[3][1]+KMGecko.StartAngle[RH_J2],RH_J2);
//			Angle(a[3][2]+KMGecko.StartAngle[RH_J1],RH_J1);
			
			
			HAL_Delay(20);
		}
			setcurrentposition(RH,x,y,z);
	}
	
}

void reverse4R(float width,float length,float height,u8 step)
{
	float x[4]={0};
	float y[4]={0};
	float z[4]={0};
	
	float a[4][3] = {0};
	float theta[4][2] = {0};
	for(t=0;t<step + 1;t++)		
		{		
			for(i=0;i<2;i++)
				{	
					x[i]=cx[i] + width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] + height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
					
					theta[i][0]=(a[i][0]+a[i][1])/2;
			    theta[i][1]=(a[i][0]-a[i][1])/2;
					
					theta[i][0]=(int) (theta[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					theta[i][1]=(int) (theta[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					theta[i][0]= (float) (theta[i][0]/100);
					theta[i][1]= (float) (theta[i][1]/100);
					a[i][2]= (float) (a[i][2]/100);
				}
				
			for(i=2;i<4;i++)
				{	
					x[i]=cx[i] + width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] + height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
					
					theta[i][0]=(a[i][0]+a[i][1])/2;
					theta[i][1]=(a[i][0]-a[i][1])/2;
					
					theta[i][0]=(int) (theta[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					theta[i][1]=(int) (theta[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					theta[i][0]= (float) (theta[i][0]/100);
					theta[i][1]= (float) (theta[i][1]/100);
					a[i][2]= (float) (a[i][2]/100);
				}	
				
					theta[0][0]=-theta[0][0];
					theta[0][1]=-theta[0][1];
				
					theta[1][1]=-theta[1][1];
				
					a[2][2]=-a[2][2];
				
					theta[3][0]=-theta[3][0];
					a[3][2]=-a[3][2];
				
					Angle(theta[0][0]+KMGecko.StartAngle[RF_J3],RF_J3);
					Angle(theta[0][1]+KMGecko.StartAngle[RF_J2],RF_J2);
					Angle(a[0][2]+KMGecko.StartAngle[RF_J1],RF_J1);
						
					Angle(theta[1][0]+KMGecko.StartAngle[RH_J3],RH_J3);
					Angle(theta[1][1]+KMGecko.StartAngle[RH_J2],RH_J2);
					Angle(a[1][2]+KMGecko.StartAngle[RH_J1],RH_J1);
						
					Angle(theta[2][0]+KMGecko.StartAngle[LF_J3],LF_J3);
					Angle(theta[2][1]+KMGecko.StartAngle[LF_J2],LF_J2);
					Angle(a[2][2]+KMGecko.StartAngle[LF_J1],LF_J1);
						
					Angle(theta[3][0]+KMGecko.StartAngle[LH_J3],LH_J3);
					Angle(theta[3][1]+KMGecko.StartAngle[LH_J2],LH_J2);
					Angle(a[3][2]+KMGecko.StartAngle[LH_J1],LH_J1);		
								
					HAL_Delay(20);
	}
					setcurrentposition(RF,x[0],y[0],z[0]);
					setcurrentposition(RH,x[1],y[1],z[1]);
					setcurrentposition(LF,x[2],y[2],z[2]);
					setcurrentposition(LH,x[3],y[3],z[3]);	
}

void reverse4L(float width,float length,float height,u8 step)
{
	float x[4]={0};
	float y[4]={0};
	float z[4]={0};
	
	float a[4][3] = {0};
	float theta[4][2] = {0};

	for(t=0;t<step + 1;t++)		
		{		
			for(i=0;i<2;i++)
				{	
					x[i]=cx[i] + width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] - height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
					
          theta[i][0]=(a[i][0]+a[i][1])/2;
			    theta[i][1]=(a[i][0]-a[i][1])/2;
					
					theta[i][0]=(int) (a[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					theta[i][1]=(int) (a[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					theta[i][0]= (float) (theta[i][0]/100);
					theta[i][1]= (float) (theta[i][1]/100);
					a[i][2]= (float) (a[i][2]/100);
				}
				
			for(i=2;i<4;i++)
				{	
					x[i]=cx[i] + width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] + height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
					
          theta[i][0]=(a[i][0]+a[i][1])/2;
			    theta[i][1]=(a[i][0]-a[i][1])/2;
					
					theta[i][0]=(int) (theta[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					theta[i][1]=(int) (theta[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					theta[i][0]= (float) (theta[i][0]/100);
					theta[i][1]= (float) (theta[i][1]/100);
					a[i][2]= (float) (a[i][2]/100);
				}
					
					theta[0][0]=-theta[0][0];//RF
					theta[0][1]=-theta[0][1];
				
					theta[1][1]=-theta[1][1];//RR
				
					a[2][2]=-a[2][2];//LF
				
					theta[3][0]=-theta[3][0];//LR
					a[3][2]=-a[3][2];
				
					Angle(theta[0][0]+KMGecko.StartAngle[RF_J3],RF_J3);
					Angle(theta[0][1]+KMGecko.StartAngle[RF_J2],RF_J2);
					Angle(a[0][2]+KMGecko.StartAngle[RF_J1],RF_J1);
						
					Angle(theta[1][0]+KMGecko.StartAngle[RH_J3],RH_J3);
					Angle(theta[1][1]+KMGecko.StartAngle[RH_J2],RH_J2);
					Angle(a[1][2]+KMGecko.StartAngle[RH_J1],RH_J1);
						
					Angle(theta[2][0]+KMGecko.StartAngle[LF_J3],LF_J3);
					Angle(theta[2][1]+KMGecko.StartAngle[LF_J2],LF_J2);
					Angle(a[2][2]+KMGecko.StartAngle[LF_J1],LF_J1);
						
					Angle(theta[3][0]+KMGecko.StartAngle[LH_J3],LH_J3);
					Angle(theta[3][1]+KMGecko.StartAngle[LH_J2],LH_J2);
					Angle(a[3][2]+KMGecko.StartAngle[LH_J1],LH_J1);		
								
					HAL_Delay(20);
	}
					setcurrentposition(RF,x[0],y[0],z[0]);
					setcurrentposition(RH,x[1],y[1],z[1]);
					setcurrentposition(LF,x[2],y[2],z[2]);
					setcurrentposition(LH,x[3],y[3],z[3]);

	
}




void Angle(float angle,int8_t footnumber)
{
	int16_t ArrValue = 0;
	//ArrValue = 250 + (int32_t) (angle *0.18);
	//ArrValue = angle * 5 + 750;
	ArrValue = (int32_t) (angle * 5.56 + 750)	;
	if(footnumber == LF_J1)
	{
	//	User_PWM_SetValue(&htim4, TIM_CHANNEL_2,ArrValue);
		TIM_SetCompare1(&TIM4_Handler,ArrValue);
	}	
	else if(footnumber == LF_J2)
	{
	//	User_PWM_SetValue(&htim4, TIM_CHANNEL_1,ArrValue);
		TIM_SetCompare2(&TIM4_Handler,ArrValue);
	}	
	else if(footnumber == LF_J3)
	{
	//	User_PWM_SetValue(&htim4, TIM_CHANNEL_1,ArrValue);
		TIM_SetCompare3(&TIM4_Handler,ArrValue);
	}	
	else if(footnumber == LH_J1)
	{
	//	User_PWM_SetValue(&htim4, TIM_CHANNEL_1,ArrValue);
		TIM_SetCompare1(&TIM12_Handler,ArrValue);
	}	
	else if(footnumber == LH_J2)
	{
	//	User_PWM_SetValue(&htim4, TIM_CHANNEL_1,ArrValue);
		TIM_SetCompare2(&TIM12_Handler,ArrValue);
	}	
	else if(footnumber == LH_J3)
	{
	//	User_PWM_SetValue(&htim4, TIM_CHANNEL_1,ArrValue);
		TIM_SetCompare1(&TIM15_Handler,ArrValue);
	}	
	else if(footnumber == RF_J1)
	{
	//	User_PWM_SetValue(&htim3, TIM_CHANNEL_3,ArrValue);
		TIM_SetCompare1(&TIM1_Handler,ArrValue);
	}
	else if(footnumber == RF_J2)
	{
	//	User_PWM_SetValue(&htim3, TIM_CHANNEL_2,ArrValue);
		TIM_SetCompare2(&TIM1_Handler,ArrValue);
	}
	else if(footnumber == RF_J3)
	{
	//	User_PWM_SetValue(&htim1, TIM_CHANNEL_2,ArrValue);
		TIM_SetCompare3(&TIM1_Handler,ArrValue);
	}
	else if(footnumber == RH_J1)
	{
//		User_PWM_SetValue(&htim1, TIM_CHANNEL_3,ArrValue);
		TIM_SetCompare1(&TIM3_Handler,ArrValue);
	}
	else if(footnumber == RH_J2)
	{
//		User_PWM_SetValue(&htim1, TIM_CHANNEL_3,ArrValue);
		TIM_SetCompare2(&TIM3_Handler,ArrValue);
	}
	else if(footnumber == RH_J3)
	{
//		User_PWM_SetValue(&htim1, TIM_CHANNEL_3,ArrValue);
		TIM_SetCompare3(&TIM3_Handler,ArrValue);
	}
}
void Robot_Run_Line(void)
{
	int x=0;//左右width
	int y=15;//前进length
	int z=30;//高度height

	
//	reverse(RR,0,y,z,stepm);
//	reverse(RR,0,0,-z,stepl);
	
	reverse4(0,5,10,steph);//50
//	  reverse(LF,0,0,10,stepm);
//	  reverse(LF,0,10,0,stepm);
//	  reverse(LF,0,0,-12,stepm);
//	
//    reverse(LH,0,0,10,stepm);
//	  reverse(LH,0,10,0,stepm);
//	  reverse(LH,0,0,-12,stepm);
//	  reverse(RF,0,0,10,stepm);
//	  reverse(RF,0,10,0,stepm);
//	  reverse(RF,0,0,-12,stepm);
//	  reverse(RH,0,0,10,stepm);
//	  reverse(RH,0,10,0,stepm);
//	  reverse(RH,0,0,-12,stepm);
//	
//	reverse(LR,0,y,z,stepm);
//	reverse(LR,0,0,-z,stepl);
	
//	reverse4(0,-y/2,-z,steph);//50	
}
void Robot_Run_back(void)
{
	int x=0;//左右length
	int y=-20;//前进width
	int z=40;//高度height
	
	reverse(RF,0,y,z,stepm);//50
	reverse(RF,0,0,-z,stepl);//50
	
	reverse(RH,0,y,z,stepm);
	reverse(RH,0,0,-z,stepl);
	
	reverse4(0,-y/2,0,steph);//50
	
	reverse(LF,0,y,z,stepm);
	reverse(LF,0,0,-z,stepl);
	
	reverse(LH,0,y,z,stepm);
	reverse(LH,0,0,-z,stepl);
	
	reverse4(0,-y/2,0,steph);//50	
}
void Robot_Run_Left(void)
{
	int x=16;//左右length
	int y=10;//前进width
	int z=50;//高度height
		
	reverse(LF,x,y,z,stepm);
	reverse(LF,0,0,-z,stepl);
	
	reverse(LH,x,y,z,stepm);
	reverse(LH,0,0,-z,stepl);	
		
	reverse4L(-x/2,-y/2,0,steph);
			
	reverse(RF,-x,y,z,stepm);
	reverse(RF,0,0,-z,stepl);
	
	reverse(RH,-x,y,z,stepm);
	reverse(RH,0,0,-z,stepl);

	reverse4L(-x/2,-y/2,0,steph);

}

void Robot_Run_Right(void)
{
	int x=16;//左右length
	int y=10;//前进width
	int z=20;//高度height
		
	reverse(RF,x,y,z,stepm);
	reverse(RF,0,0,-z,stepl);
	
	reverse(RH,x,y,z,stepm);
	reverse(RH,0,0,-z,stepl);	
		
	reverse4R(-x/2,-y/2,0,steph);
			
	reverse(LF,-x,y,z,stepm);
	reverse(LF,0,0,-z,stepl);
	
	reverse(LH,-x,y,z,stepm);
	reverse(LH,0,0,-z,stepl);

	reverse4R(-x/2,-y/2,0,steph);
}


void Robot_test1(void)
{
		reverse(LF,0,0,10,stepm);
		reverse(LF,0,10,0,stepm);
	  reverse(LF,0,0,-12,stepm);
	  reverse(LH,0,0,10,stepm);
	  reverse(LH,0,10,0,stepm);
	  reverse(LH,0,0,-12,stepm);
	  reverse(RF,0,0,10,stepm);
	  reverse(RF,0,10,0,stepm);
	  reverse(RF,0,0,-12,stepm);
	  reverse(RH,0,0,10,stepm);
	  reverse(RH,0,10,0,stepm);
	  reverse(RH,0,0,-12,stepm);
}
void diagonalgait1(float width,float length,float height,u8 step)
{
	float x[4]={0};
	float y[4]={0};
	float z[4]={0};
	
	float a[4][3] = {0};
	float theta[4][2] = {0};
	for(t=0;t<step + 1;t++)		
		{		
			for(i=0;i<1;i++)
				{	
					x[i]=cx[i] + width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] + height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
					
					theta[i][0]=(a[i][0]+a[i][1]);
			    theta[i][1]=(a[i][1]-a[i][0]);
					
					theta[i][0]=(int) (theta[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					theta[i][1]=(int) (theta[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					theta[i][0]= (float) (theta[i][0]/100);
					theta[i][1]= (float) (theta[i][1]/100);
					a[i][2]= (float) (a[i][2]/100);
				}
				
			for(i=1;i<2;i++)
				{	
					x[i]=cx[i] + width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] + height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
					
					theta[i][0]=(a[i][0]+a[i][1]);
					theta[i][1]=(a[i][1]-a[i][0]);
					
					theta[i][0]=(int) (theta[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					theta[i][1]=(int) (theta[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					theta[i][0]= (float) (-theta[i][0]/100);
					theta[i][1]= (float) (-theta[i][1]/100);
					a[i][2]= (float) (-a[i][2]/100);
				}	
			for(i=2;i<3;i++)
				{	
					x[i]=cx[i] + width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] + height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
					
					theta[i][0]=(a[i][1]-a[i][0]);
					theta[i][1]=(a[i][1]+a[i][0]);
					
					theta[i][0]=(int) (theta[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					theta[i][1]=(int) (theta[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					theta[i][0]= (float) (theta[i][0]/100);
					theta[i][1]= (float) (theta[i][1]/100);
					a[i][2]= (float) (a[i][2]/100);
				}	
				for(i=3;i<4;i++)
				{	
					x[i]=cx[i] + width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] + height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
					
					theta[i][0]=(a[i][1]-a[i][0]);
					theta[i][1]=(a[i][1]+a[i][0]);
					
					theta[i][0]=(int) (theta[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					theta[i][1]=(int) (theta[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					theta[i][0]= (float) (-theta[i][0]/100);
					theta[i][1]= (float) (-theta[i][1]/100);
					a[i][2]= (float) (-a[i][2]/100);
				}	
      		Angle(theta[0][0]+KMGecko.StartAngle[LF_J3],LF_J3);
					Angle(theta[0][1]+KMGecko.StartAngle[LF_J2],LF_J2);
					Angle(a[0][2]+KMGecko.StartAngle[LF_J1],LF_J1);
						
		
						
					Angle(theta[3][0]+KMGecko.StartAngle[RH_J3],RH_J3);
					Angle(theta[3][1]+KMGecko.StartAngle[RH_J2],RH_J2);
					Angle(a[3][2]+KMGecko.StartAngle[RH_J1],RH_J1);		
								
					HAL_Delay(20);
			}
					setcurrentposition(LF,x[0],y[0],z[0]);
					setcurrentposition(RF,x[1],y[1],z[1]);
					setcurrentposition(LH,x[2],y[2],z[2]);
					setcurrentposition(RH,x[3],y[3],z[3]);

}
void diagonalgait2(float width,float length,float height,u8 step)
{
	float x[4]={0};
	float y[4]={0};
	float z[4]={0};
	
	float a[4][3] = {0};
	float theta[4][2] = {0};
	for(t=0;t<step + 1;t++)		
		{		
			for(i=0;i<1;i++)
				{	
					x[i]=cx[i] + width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] + height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
					
					theta[i][0]=(a[i][0]+a[i][1]);
			    theta[i][1]=(a[i][1]-a[i][0]);
					
					theta[i][0]=(int) (theta[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					theta[i][1]=(int) (theta[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					theta[i][0]= (float) (theta[i][0]/100);
					theta[i][1]= (float) (theta[i][1]/100);
					a[i][2]= (float) (a[i][2]/100);
				}
				
			for(i=1;i<2;i++)
				{	
					x[i]=cx[i] + width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] + height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
					
					theta[i][0]=(a[i][0]+a[i][1]);
					theta[i][1]=(a[i][1]-a[i][0]);
					
					theta[i][0]=(int) (theta[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					theta[i][1]=(int) (theta[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					theta[i][0]= (float) (-theta[i][0]/100);
					theta[i][1]= (float) (-theta[i][1]/100);
					a[i][2]= (float) (-a[i][2]/100);
				}	
			for(i=2;i<3;i++)
				{	
					x[i]=cx[i] + width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] + height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
					
					theta[i][0]=(a[i][1]-a[i][0]);
					theta[i][1]=(a[i][1]+a[i][0]);
					
					theta[i][0]=(int) (theta[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					theta[i][1]=(int) (theta[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					theta[i][0]= (float) (theta[i][0]/100);
					theta[i][1]= (float) (theta[i][1]/100);
					a[i][2]= (float) (a[i][2]/100);
				}	
				for(i=3;i<4;i++)
				{	
					x[i]=cx[i] + width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] + height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
					
					theta[i][0]=(a[i][1]-a[i][0]);
					theta[i][1]=(a[i][1]+a[i][0]);
					
					theta[i][0]=(int) (theta[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					theta[i][1]=(int) (theta[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					theta[i][0]= (float) (-theta[i][0]/100);
					theta[i][1]= (float) (-theta[i][1]/100);
					a[i][2]= (float) (-a[i][2]/100);
				}	
			
         
						
					Angle(theta[1][0]+KMGecko.StartAngle[RF_J3],RF_J3);
					Angle(theta[1][1]+KMGecko.StartAngle[RF_J2],RF_J2);
					Angle(a[1][2]+KMGecko.StartAngle[RF_J1],RF_J1);
						
					Angle(theta[2][0]+KMGecko.StartAngle[LH_J3],LH_J3);
					Angle(theta[2][1]+KMGecko.StartAngle[LH_J2],LH_J2);
					Angle(a[2][2]+KMGecko.StartAngle[LH_J1],LH_J1);
						
					
								
					HAL_Delay(20);
			}
					setcurrentposition(LF,x[0],y[0],z[0]);
					setcurrentposition(RF,x[1],y[1],z[1]);
					setcurrentposition(LH,x[2],y[2],z[2]);
					setcurrentposition(RH,x[3],y[3],z[3]);

}
void reverse1(float width,float length,float height,u8 step)
{
  float x[4]={0};
	float y[4]={0};
	float z[4]={0};
	
	float a[4][3] = {0};
	float theta[4][2] = {0};
	for(t=0;t<step + 1;t++)		
		{		
			for(i=0;i<1;i++)
				{	
					x[i]=cx[i] + width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] + height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
					
					theta[i][0]=(a[i][0]+a[i][1]);
			    theta[i][1]=(a[i][1]-a[i][0]);
					
					theta[i][0]=(int) (theta[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					theta[i][1]=(int) (theta[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					theta[i][0]= (float) (theta[i][0]/100);
					theta[i][1]= (float) (theta[i][1]/100);
					a[i][2]= (float) (a[i][2]/100);
				}
				
			for(i=1;i<2;i++)
				{	
					x[i]=cx[i] + width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] + height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
					
					theta[i][0]=(a[i][0]+a[i][1]);
					theta[i][1]=(a[i][1]-a[i][0]);
					
					theta[i][0]=(int) (theta[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					theta[i][1]=(int) (theta[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					theta[i][0]= (float) (-theta[i][0]/100);
					theta[i][1]= (float) (-theta[i][1]/100);
					a[i][2]= (float) (-a[i][2]/100);
				}	
			for(i=2;i<3;i++)
				{	
					x[i]=cx[i] + width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] + height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
					
					theta[i][0]=(a[i][1]-a[i][0]);
					theta[i][1]=(a[i][1]+a[i][0]);
					
					theta[i][0]=(int) (theta[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					theta[i][1]=(int) (theta[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					theta[i][0]= (float) (theta[i][0]/100);
					theta[i][1]= (float) (theta[i][1]/100);
					a[i][2]= (float) (a[i][2]/100);
				}	
				for(i=3;i<4;i++)
				{	
					x[i]=cx[i] + width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] + height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
					
					theta[i][0]=(a[i][1]-a[i][0]);
					theta[i][1]=(a[i][1]+a[i][0]);
					
					theta[i][0]=(int) (theta[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					theta[i][1]=(int) (theta[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					theta[i][0]= (float) (-theta[i][0]/100);
					theta[i][1]= (float) (-theta[i][1]/100);
					a[i][2]= (float) (-a[i][2]/100);
				}		
				
					Angle(theta[0][0]+KMGecko.StartAngle[LF_J3],LF_J3);
					Angle(theta[0][1]+KMGecko.StartAngle[LF_J2],LF_J2);
					Angle(a[0][2]+KMGecko.StartAngle[LF_J1],LF_J1);
						
					Angle(theta[1][0]+KMGecko.StartAngle[RF_J3],RF_J3);
					Angle(theta[1][1]+KMGecko.StartAngle[RF_J2],RF_J2);
					Angle(a[1][2]+KMGecko.StartAngle[RF_J1],RF_J1);
						
					Angle(theta[2][0]+KMGecko.StartAngle[LH_J3],LH_J3);
					Angle(theta[2][1]+KMGecko.StartAngle[LH_J2],LH_J2);
					Angle(a[2][2]+KMGecko.StartAngle[LH_J1],LH_J1);
						
					Angle(theta[3][0]+KMGecko.StartAngle[RH_J3],RH_J3);
					Angle(theta[3][1]+KMGecko.StartAngle[RH_J2],RH_J2);
					Angle(a[3][2]+KMGecko.StartAngle[RH_J1],RH_J1);		
								
					HAL_Delay(20);
}
					setcurrentposition(RF,x[0],y[0],z[0]);
					setcurrentposition(RH,x[1],y[1],z[1]);
					setcurrentposition(LF,x[2],y[2],z[2]);
					setcurrentposition(LH,x[3],y[3],z[3]);	

}
void Robot_test2(void)
{
  diagonalgait1(0,0,10,20);
  diagonalgait1(0,10,0,20);
  diagonalgait1(0,0,-12,20);
	diagonalgait2(0,0,10,20);
  diagonalgait2(0,10,0,20);
	diagonalgait2(0,0,-12,20);
	reverse1(0,-10,0,30);
}
void StartAngleInit(void)
{
			/**直角初始位置**/
	//关节1、2、3分别为踝关节、膝关节、髋关节
	//number 3
	KMGecko.StartAngle[RF_J3] = -5; //52
	KMGecko.StartAngle[RF_J2] = 5; //26//large figure turning to the inside
	KMGecko.StartAngle[RF_J1] = 0; //// large figure turning to the inside
   //number 2
	KMGecko.StartAngle[RH_J3] = 5;//-10//-25;	
	KMGecko.StartAngle[RH_J2] = -5;//50;	small figure clockwise
	KMGecko.StartAngle[RH_J1] = 0 ;//30;small figure turning to the clw
  //number 0
	KMGecko.StartAngle[LF_J3] = 5;//-10
	KMGecko.StartAngle[LF_J2] = -5;//-13 small figure turning to the inside
	KMGecko.StartAngle[LF_J1] = 0;//-58 small figure turning to the inside
	//number 1
	KMGecko.StartAngle[LH_J3] = -5;//10///-30;//原值为-30
	KMGecko.StartAngle[LH_J2] = 5;//0 large figure turning to the inside
	KMGecko.StartAngle[LH_J1] = 0;//;small figure turning to the inside
}

void InitRobotPosion(void)
{
//	if(((Time8Channel3HighTime<=1600)&&(Time8Channel3HighTime>=1400))||(Time8Channel3HighTime<=500))
//	{	
	Angle(KMGecko.StartAngle[LF_J3],LF_J3);
	Angle(KMGecko.StartAngle[LF_J1],LF_J1);
	Angle(KMGecko.StartAngle[LF_J2],LF_J2);
	
	Angle(KMGecko.StartAngle[RF_J1],RF_J1);
	Angle(KMGecko.StartAngle[RF_J2],RF_J2);		
	Angle(KMGecko.StartAngle[RF_J3],RF_J3);
	
	Angle(KMGecko.StartAngle[RH_J1],RH_J1);
	Angle(KMGecko.StartAngle[RH_J2],RH_J2);
	Angle(KMGecko.StartAngle[RH_J3],RH_J3);

	Angle(KMGecko.StartAngle[LH_J1],LH_J1);
	Angle(KMGecko.StartAngle[LH_J2],LH_J2);
	Angle(KMGecko.StartAngle[LH_J3],LH_J3);
	
	setcurrentposition(RF,L1,L2,-L3);
	setcurrentposition(RH,L1,L2,-L3);
	setcurrentposition(LF,L1,L2,-L3);
	setcurrentposition(LH,L1,L2,-L3);
	HAL_Delay(30);	
}
	