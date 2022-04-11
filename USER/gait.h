#ifndef __MOTION_H__
#define __MOTION_H__

#include <sys.h>


#define JonintNum 12
#define RF_J1 0
#define LF_J1 1
#define RH_J1 2
#define LH_J1 3

#define RF_J2 4
#define LF_J2 5
#define RH_J2 6
#define LH_J2 7

#define RF_J3 8
#define LF_J3 9
#define RH_J3 10
#define LH_J3 11

#define RF 100
#define RH 200
#define LF 300
#define LH 400	

extern const short L1;
extern const short L2;
extern const short L3;
extern float cx[4];
extern float cy[4];
extern float cz[4];
void StartAngleInit(void);                              
void InitRobotPosion(void);                                
void Angle(float,int8_t);
void setcurrentposition (int,float,float,float);

void Robot_Run_Line(void);
void Robot_Run_back(void);
void Robot_Run_Right(void);
void Robot_Run_Left(void);
void Robot_test1(void);
void Robot_test2(void);

void reverse(int,float,float,float,u8);
void reverse4(float,float,float,u8);
void reverse4R(float,float,float,u8);
void reverse4L(float,float,float,u8);
void reverse1(float,float,float,u8);
void diagonalgait1(float,float,float,u8);
void diagonalgait2(float,float,float,u8);


typedef struct Kinematics
{	
	float StartAngle[JonintNum];
	uint32_t StepNum;//Êý¾Ý²½¾à
}KinematicsArm;
	
extern KinematicsArm KMGecko;

#endif
