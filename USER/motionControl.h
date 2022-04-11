#ifndef _MC
#define _MC

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
//#include <sys/mman.h>
//#include <sys/time.h>
//#include <unistd.h>
#include <math.h>
#include "matrix.h"

#ifndef _ENUM_BOOL
#define _ENUM_BOOL
typedef enum bool {true = 1, false = 0}bool;
#endif 


typedef struct MotionControl struct_MC;

struct MotionControl
{
        float timeForGaitPeriod;  // The time of the whole period
        float timePeriod;  // The time of one period
        float timePresent;  
        float timeOneSwingPeriod;  // The swing time for diag legs
        Mat timeForStancePhase;
        Mat targetCoMVelocity;
        Mat presentCoMVelocity;
        Mat targetCoMPosition;
        bool stanceFlag[4];     // True, False: LF, RF, LH, RH
        Mat timePresentForSwing;

        float yawVelocity;   // yaw velocity from imu
        float L1, L2, L3;  // The length of L
        float width, length;
        Mat shoulderPos;                // X-Y: LF, RF, LH, RH
        Mat stancePhaseStartPos;
        Mat stancePhaseEndPos;
        Mat legPresentPos;  // present X-Y-Z: LF, RF, LH, RH in Shoulder cordinate
        Mat legCmdPos;  // command X-Y-Z: LF, RF, LH, RH in Shoulder cordinate
        Mat leg2CoMPrePos;  // present X-Y-Z: LF, RF, LH, RH in CoM cordinate
        Mat leg2CoMCmdPos;  // command X-Y-Z: LF, RF, LH, RH in CoM cordinate
        Mat joint_pre_pos;  // present joint angle 0-11
        Mat joint_cmd_pos;  // command joint angle 0-11
        Mat jointPresentPos;  // present motor 0-11
        Mat jointPresentVel;  // present motor 0-11
        Mat jacobian; 
        Mat A;     //VMC
        Mat B;        //VMC
        Mat a;     //VMC
        Mat b;        //VMC
        float z_pre_vel;         // z present velocity from imu
        float jointCmdPos[12];  // command motor 0-11
        float jointCmdPosLast[12];
        float jointCmdVel[12];
        float motorTorque[1];
        float motorInitPos[12];   // init motor angle of quadruped state
        float pid_motortorque[12];
        float jacobian_motortorque[12]; //motor torque in VMC
        float motorCmdTorque[12];
        bool initFlag;

        // the parameters of creeping gait
        float motIniPoCreep[12];    // init motor angle of creeping gait
        float H_onestep;  // The height of one step
        float Yaw_rad;      // The rad of yaw angle
        float k1,k2,k3;
        float L_diag;  // half of body diagonal size
        float beta_diag, alpha_diag;// structural angle of the body
        float v_body_x, v_body_y;    // the velocity of CoM
        float v_leg[4][2];  // the velocity of 4 legs
        float endPosition[4][2];  // the final feet position after one gait cycle
        Mat initPosS2L;  // init position from Shoulder to Leg
        Mat initPosC2L;  // init position from CoM to Leg
        Mat initPosC2S;  // init position from CoM to Shoulder
        float p_w2c[4][3];  // The relative position from world to CoM
        float p_c2s[4][3];  // The relative position from CoM to Shoulder
        float p_w2f[4][3];  // The relative position from world to leg
        float yawCreep;  // The command yaw of creeping gait
        float L1_creep, L2_creep, L3_creep;  // The length of creeping legs
        Mat legCmdVia;  // The intermediate variables for legCmdPos calculation of creeping gait

};

//void CLASSMC_setInitPos(struct_MC *p, Mat initPosition);
//void CLASSMC_setCoMVel(struct_MC *p, Mat tCV);
void CLASSMC_setInitPos(struct_MC *p, float* val);
void CLASSMC_setCoMVel(struct_MC *p, float* val);
void CLASSMC_nextStep(struct_MC *p);
void CLASSMC_setInitial(struct_MC *p);
void CLASSMC_freematrix(struct_MC *p);
void MotionControl_Ini(struct_MC *p,float tP, float tFGP, Mat tFSP) ;

extern float tt,arry_view[12][6];

#endif 
