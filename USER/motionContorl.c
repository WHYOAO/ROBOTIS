#include <motionControl.h>
#define PI 3.1415926
#define BAUD 9600 // baudrate of imu


//void CLASSMC_setInitPos(struct_MC *p, Mat initPosition)    //Matrix<float, 4, 3>initPosition
//{
//		
//	MatCopy(&initPosition, &p->stancePhaseStartPos);
//	MatCopy(&initPosition, &p->stancePhaseEndPos);
//	MatCopy(&initPosition, &p->legPresentPos);
//	MatCopy(&initPosition, &p->legCmdPos);
//  MatZeros(&p->targetCoMPosition);
//}

////tCV_3x1
//void CLASSMC_setCoMVel(struct_MC *p, Mat tCV)      
//{
//	MatCopy(&tCV, &p->targetCoMVelocity);
//}
void CLASSMC_setInitPos(struct_MC *p, float* val)    //Matrix<float, 4, 3>initPosition
{
	MatSetVal(&p->stancePhaseStartPos,val);	
	MatSetVal(&p->stancePhaseEndPos,val);	
	MatSetVal(&p->legPresentPos,val);	
	MatSetVal(&p->legCmdPos,val);	
  MatZeros(&p->targetCoMPosition);
}

//tCV_3x1
void CLASSMC_setCoMVel(struct_MC *p, float* val)      
{
	MatSetVal(&p->targetCoMVelocity,val);	
}


void CLASSMC_nextStep(struct_MC *p)
{
    for(int legNum=0; legNum<4; legNum++)  // run all 4 legs
    {
			
        if(p->timePresent > p->timeForStancePhase.element[legNum][0] - p->timePeriod/2 && p->timePresent < p->timeForStancePhase.element[legNum][1]+p->timePeriod/2 )
        {     // check timePresent is in stance phase or swing phase, -timePeriod/2 is make sure the equation is suitable
            if(fabsf(p->timePresent - p->timeForStancePhase.element[legNum][0]) < 1e-4)  // if on the start pos 
            {
                MatCopy_row(&p->legCmdPos, legNum, &p->stancePhaseStartPos, legNum);
                for( int pos=0; pos<3; pos++)
                    p->targetCoMPosition.element[legNum][pos] = 0.0;
            }
            Mat trans;
						
						MatCreate(&trans, 3, 3);
            trans.element[0][0] = cos(p->targetCoMPosition.element[legNum][2]);
            trans.element[0][1] = -sin(p->targetCoMPosition.element[legNum][2]);
            trans.element[0][2] = p->targetCoMPosition.element[legNum][0];

            trans.element[1][0] = sin(p->targetCoMPosition.element[legNum][2]);
            trans.element[1][1] = cos(p->targetCoMPosition.element[legNum][2]);
            trans.element[1][2] = p->targetCoMPosition.element[legNum][1];

            trans.element[2][0] = 0;
            trans.element[2][1] = 0;
            trans.element[2][2] = 1;

            Mat oneShoulderPos_3x1, temp_3x1;
						
            MatCreate(&oneShoulderPos_3x1, 3, 1);
            MatCreate(&temp_3x1, 3, 1);				
            oneShoulderPos_3x1.element[0][0] = p->shoulderPos.element[legNum][0];
            oneShoulderPos_3x1.element[1][0] = p->shoulderPos.element[legNum][1];
            oneShoulderPos_3x1.element[2][0] = 1;

            MatMul( &trans, &oneShoulderPos_3x1, &temp_3x1);
						MatCopy(&temp_3x1, &oneShoulderPos_3x1);

            if(fabsf(p->timePresent - p->timeForStancePhase.element[legNum][0]) < 1e-4)  // if on the start pos 
            {
                MatCopy_row(&p->legCmdPos, legNum, &p->stancePhaseStartPos, legNum);
                p->shoulderPos.element[legNum][0] = oneShoulderPos_3x1.element[0][0];
                p->shoulderPos.element[legNum][1] = oneShoulderPos_3x1.element[1][0];
            }
            if(fabsf(p->timePresent - p->timeForStancePhase.element[legNum][1]) < 1e-4)  // if on the end pos
                MatCopy_row(&p->legCmdPos, legNum, &p->stancePhaseEndPos, legNum);
            
            p->legCmdPos.element[legNum][0] = p->stancePhaseStartPos.element[legNum][0]  + (p->shoulderPos.element[legNum][0]  - oneShoulderPos_3x1.element[0][0]);
            p->legCmdPos.element[legNum][1] = p->stancePhaseStartPos.element[legNum][1] + (p->shoulderPos.element[legNum][1] - oneShoulderPos_3x1.element[1][0]);
            p->stanceFlag[legNum] = true;

            MatDelete(&oneShoulderPos_3x1);
            MatDelete(&trans);
            MatDelete(&temp_3x1);
        }
        else
        {
            Mat swingPhaseVelocity, temp_matrix_1x3A, temp_matrix_1x3B;
            MatCreate(&swingPhaseVelocity, 1, 3);
            MatCreate(&temp_matrix_1x3A, 1, 3);
            MatCreate(&temp_matrix_1x3B, 1, 3);

            MatCopy_row(&p->stancePhaseEndPos, legNum, &temp_matrix_1x3A, 0);
            MatCopy_row(&p->stancePhaseEndPos, legNum, &temp_matrix_1x3B, 0);
            MatSub(&temp_matrix_1x3A, &temp_matrix_1x3B, &temp_matrix_1x3A);

            MatMul_k( 1/(p->timeForGaitPeriod - (p->timeForStancePhase.element[legNum][1] - p->timeForStancePhase.element[legNum][0]) - p->timePeriod),
                                     &temp_matrix_1x3A,  &swingPhaseVelocity);
            
            for( int pos=0; pos<3; pos++)
                p->legCmdPos.element[legNum][pos] = p->legCmdPos.element[legNum][pos] - swingPhaseVelocity.element[0][pos] * p->timePeriod;
                        
						if( ( p->timePresentForSwing.element[legNum][0] - (p->timeForGaitPeriod - (p->timeForStancePhase.element[legNum][1] - p->timeForStancePhase.element[legNum][0] ) ) /2 ) > 1e-4)
                p->legCmdPos.element[legNum][2] -= 3.0;
            if( ( p->timePresentForSwing.element[legNum][0] - (p->timeForGaitPeriod - (p->timeForStancePhase.element[legNum][1] - p->timeForStancePhase.element[legNum][0] ) ) /2 )  < -1e-4 && p->timePresentForSwing.element[legNum][0] > 1e-4)
                p->legCmdPos.element[legNum][2] += 3.0;
            p->stanceFlag[legNum] = false;
        }
    }

    p->timePresent += p->timePeriod;
    for( int leg=0; leg<4; leg++)
    {
        for( int pos=0; pos<3; pos++)
        {
					
            p->targetCoMPosition.element[leg][pos] += p->targetCoMVelocity.element[pos][0] * p->timePeriod;
 			
//			for(int row = 0 ; row < p->targetCoMPosition.row ; row++)
//				for(int col = 0 ; col < p->targetCoMPosition.col ; col++)
//					arry_view[row][col]=p->targetCoMPosition.element[row][col];   		
				}
        if(p->stanceFlag[leg] == 0) 
            p->timePresentForSwing.element[leg][0] += p->timePeriod;
        else 
            p->timePresentForSwing.element[leg][0] = 0;
    }
    if (fabsf(p->timePresent - p->timeForGaitPeriod - p->timePeriod) < 1e-4)  // check if present time has reach the gait period                                                               
    {                                                            // if so, set it to 0.0
        p->timePresent = 0.0;
    }

}

void CLASSMC_setInitial(struct_MC *p)
{
		
    MatCreate(&p->timeForStancePhase, 4, 2);// startTime, endTime: LF, RF, LH, RH
    MatCreate(&p-> targetCoMVelocity,3,1);  // X, Y , alpha c in world cordinate
    MatCreate(&p-> presentCoMVelocity,3,1);  // X, Y , alpha c in world cordinate
    MatCreate(&p-> targetCoMPosition,4,3);  // X, Y , alpha in world cordinate
    
    MatCreate(&p-> timePresentForSwing,4,1);
    MatCreate(&p-> shoulderPos,4,2);  // X-Y: LF, RF, LH, RH
    MatCreate(&p-> stancePhaseStartPos,4,3);
    MatCreate(&p-> stancePhaseEndPos,4,3);
    MatCreate(&p-> legPresentPos,4,3);  // present X-Y-Z: LF, RF, LH, RH in Shoulder cordinate
    MatCreate(&p-> legCmdPos,4,3);  // command X-Y-Z: LF, RF, LH, RH in Shoulder cordinate
    MatCreate(&p-> leg2CoMPrePos,4,3);  // present X-Y-Z: LF, RF, LH, RH in CoM cordinate
    MatCreate(&p-> leg2CoMCmdPos,4,3);  // command X-Y-Z: LF, RF, LH, RH in CoM cordinate
    MatCreate(&p-> joint_pre_pos,12,1);  // present joint angle 0-11
    MatCreate(&p-> joint_cmd_pos,12,1) ;  // command joint angle 0-11
    MatCreate(&p-> jointPresentPos,12,1) ;  // present motor 0-11
    MatCreate(&p-> jointPresentVel,12,1) ;  // present motor 0-11
    MatCreate(&p-> jacobian,4,9); 
    MatCreate(&p-> A,6,6);     //VMC
    MatCreate(&p-> B,6,1);        //VMC
    MatCreate(&p-> a,4,6);       //VMC
    MatCreate(&p-> b,4,1);       //VMC
    MatCreate(&p-> initPosS2L,4,3);  // init position from Shoulder to Leg
    MatCreate(&p-> initPosC2L,4,2);  // init position from CoM to Leg
    MatCreate(&p-> initPosC2S,4,2);  // init position from CoM to Shoulder
    MatCreate(&p-> legCmdVia,4,3);  // The intermediate variables for legCmdPos calculation of creeping gait

}

void CLASSMC_freematrix(struct_MC *p)
{
    MatDelete(&p->timeForStancePhase);
    MatDelete(&p->targetCoMVelocity);
    MatDelete(&p->presentCoMVelocity);
    MatDelete(&p->targetCoMPosition);
    MatDelete(&p->timePresentForSwing);
    MatDelete(&p->shoulderPos);
    MatDelete(&p->stancePhaseStartPos);
    MatDelete(&p->stancePhaseEndPos);
    MatDelete(&p->legPresentPos);
    MatDelete(&p->legCmdPos);
    MatDelete(&p->leg2CoMPrePos);
    MatDelete(&p->leg2CoMCmdPos);
    MatDelete(&p->joint_pre_pos);
    MatDelete(&p->joint_cmd_pos);
    MatDelete(&p->jointPresentPos);
    MatDelete(&p->jointPresentVel);
    MatDelete(&p->jacobian);
    MatDelete(&p->A);
    MatDelete(&p->B);
    MatDelete(&p->a);
    MatDelete(&p->b);
    MatDelete(&p->initPosS2L);
    MatDelete(&p->initPosC2L);
    MatDelete(&p->initPosC2S);
    MatDelete(&p->legCmdVia);

}


void MotionControl_Ini(struct_MC *p,float tP, float tFGP, Mat tFSP)    //Matrix<float, 4, 2> tFSP
{
		int row,col;

    CLASSMC_setInitial(p);

// The parameters for quadruped gait
    p->initFlag = false;
    p->timePeriod = tP;
    p->timeForGaitPeriod = tFGP;
		MatCopy(&tFSP,&p->timeForStancePhase);
    p->timePresent = 0.0;
    MatZeros(&p->timePresentForSwing);
    MatZeros(&p->targetCoMVelocity);
    p->L1 = 132.0;
    p->L2 = 138.0;
    p->L3 = 0.0;
    p->width = 132.0;
    p->length = 172.0;  
	
	  float t1[] =  {p->width/2.0f, p->length/2.0f, p->width/2.0f, -p->length/2.0f, -p->width/2.0f, p->length/2.0f, -p->width/2.0f, -p->length/2.0f};  // X-Y: LF, RF, LH, RH
    float t2[] =  {112.0, -132.0, -46.0, 112.0, 132.0, -46.0, -112.0, -132.0, -46.0, -112.0, 132.0, -46.0};
    float t3[] =  {198.0, -198.0, 198.0, 198.0, -198.0, -198.0, -198.0, 198.0};
    float t4[] =  {86.0, -66.0, 86.0, 66.0, -86.0, -66.0, -86.0, 66.0};
    MatSetVal(&p->shoulderPos, t1 );
		

    // The parameters for creeping gait
    p->timeOneSwingPeriod = 0.5 * p->timeForGaitPeriod;
    p->L1_creep = 132.0;
    p->L2_creep = 112.0;
    p->L3_creep = 46.0; 
    p->H_onestep = 15.0;
    p->k1 = 0.25;
    p->k2 = 0.50;
    p->k3 = 0.25;
    MatSetVal(&p->initPosS2L, t2 );
    MatSetVal(&p->initPosC2L, t3 );
    MatSetVal(&p->initPosC2S, t4 );
		//t= p->initPosC2L.element[0][0] * p->initPosC2L.element[0][0]  + p->initPosC2L.element[0][1] * p->initPosC2L.element[0][1];
    p->L_diag = sqrtf( p->initPosC2L.element[0][0] * p->initPosC2L.element[0][0]  + p->initPosC2L.element[0][1] * p->initPosC2L.element[0][1] );
    p->beta_diag = atan(p->initPosC2L.element[0][0] /p->initPosC2L.element[0][1] );
    p->alpha_diag = PI / 2 - p->beta_diag;
}











