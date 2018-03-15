#include "math.h"
#include "FCT_LCDAS.h"
#include "mex.h"

/*可以采用雷达的笛卡尔坐标，坐标中心位于两雷达连线的中心，沿车头向前为X轴正前方，沿车身向左为Y轴正方向
 *        ^      ^
 *        |______|  前
 *       X|      |X
 *        | CAR  |
 *        |______|
 *        |______|
 *  Y<____|______|_____ Y
 *    左  |      |  后右   
 *            ^
 *            |
 *           X|      
 *            |  
 *            |
 *            |
 *      Y<____|____原点在两雷达中间处，朝上为X轴正前方，朝左为Y轴正前方。
 *        左  |      
 */

static LCDAS_t_State        lcdasStateNow =  LCDASActive;//激活与不激活
static LCDAS_t_ActiveState  lcdasActiveStateNow = WarningLevel2;
static WarningResult_t		lcwWarningResult =  ShallWarning;
static WarningResult_t		bswWarningResult =  ShallWarning;
static WarningResult_t		cvwWarningResult =  ShallWarning;

const  LCDAS_t_EvaluationCriteria lcdas_EvaluationCriteria  =  TurnSignalEvaluation;
const  LCDAS_t_ActiveMethod       lcdas_ActiveMethod        =  AnyOneSatisfaction;

static float32 NCDIS =  1.7f; //M
static float32 NDDIS =  2.2f; //需要整车厂提供支持
static float32 EJDIS =  -2.2f;
static float32 JEDIS =  2.2f;
static float32 RADARORIGIN  = 2.2;
//#define EJDIS    -3.0
//#define JEDIS    3.0

uint8  u8_FlashingFlag = 0;//给外部接口
static uint8 u8_flag_overtaking = 0;

#define LED_ON  1
#define LED_OFF 0

static int32 AlarmLampOn(void)
{
	u8_FlashingFlag = 1;
	LED_ON;              //外部提供的灯亮接口
	return 1;
}

static int32 AlarmLampOff(void)
{
	u8_FlashingFlag = 0;
	LED_OFF;
	return 1;
}

/*static int32 AlarmLampFlashing(void)
{
	u8_FlashingFlag = 1;
	LED_ON;             //外部提供的灯灭接口
	return 1;
}*/

static int32 VibrationWarningOn(void)
{
	return 1;
}

static int32 VibrationWarningOff(void)
{
	return 1;
}

static int32 NoAction(void)
{
	AlarmLampOff();
	VibrationWarningOff();
	return 1;
}

/*
 *功能层程序的MEX方式进行验证
 **/

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    int32  s32_ret = 0;
    PerfDegr_t em_PerfDegr_data;
    Road_t em_Road;
    MovingObjectTraces_t em_MovingObjectTraces;
    EM_t_GenObjectList em_GenObjectList;
    EM_t_ARSObjectList em_ARSObjectList;
    VehPar_t vdy_VehPar;
    VehDyn_t vdy_VehDyn;
    
    int selRowsNum = 1;
    int selColsNum = 1; 

    double* pResizedArr0;

    double* em_PerfDegr_data_MEX =(double*)mxGetPr(prhs[0]);
	double* em_Road_MEX =(double*)mxGetPr(prhs[1]);
	double* em_MovingObjectTraces_MEX =(double*)mxGetPr(prhs[2]);
	double* em_GenObjectList_MEX =(double*)mxGetPr(prhs[3]);
	double* em_ARSObjectList_MEX =(double*)mxGetPr(prhs[4]);
	double* vdy_VehPar_MEX =(double*)mxGetPr(prhs[5]);
	double* vdy_VehDyn_MEX =(double*)mxGetPr(prhs[6]);

    plhs[0] = mxCreateDoubleMatrix(selRowsNum, selColsNum, mxREAL);
    pResizedArr0 =(double*)mxGetPr(plhs[0]);
    #define RARR0(row,col) pResizedArr0[(col)*selRowsNum+row]
   
	//参数初始化
	(vdy_VehPar.VehParMain).Turn_LampLeft = (uint8)(*(vdy_VehPar_MEX));
    (vdy_VehPar.VehParAdd).VehicleLength = (float)(*(vdy_VehPar_MEX + 1));
	(vdy_VehPar.VehParAdd).VehicleWidth  = (float)(*(vdy_VehPar_MEX + 2));
	(vdy_VehPar.VehParMain).Turn_LampRight = (uint8)(*(vdy_VehPar_MEX + 3));
	//mexPrintf("%d\n",(vdy_VehPar.VehParMain).Turn_LampLeft);
	//mexPrintf("%f\n",(vdy_VehPar.VehParAdd).VehicleLength);
	//mexPrintf("%f\n",(vdy_VehPar.VehParAdd).VehicleWidth);
	//mexPrintf("%d\n",(vdy_VehPar.VehParMain).Turn_LampRight);

	((vdy_VehDyn.Longitudinal).MotVar).Velocity = (float)(*(vdy_VehDyn_MEX + 0));//Vx本车向前为正，向后为负
	((vdy_VehDyn.Lateral).MotVar).Velocity = (float)(*(vdy_VehDyn_MEX + 1));     //VY本车向左为正，向右为负
	//mexPrintf("%f\n",((vdy_VehDyn.Longitudinal).MotVar).Velocity);
	//mexPrintf("%f\n",((vdy_VehDyn.Lateral).MotVar).Velocity);
	
	((em_GenObjectList.HeaderObjList).iNumOfUsedObjects) = (uint32)(*(em_GenObjectList_MEX));
	((em_GenObjectList.aObject[0]).Kinematic).fDistX = (float)(*(em_GenObjectList_MEX+1));
	((em_GenObjectList.aObject[0]).Kinematic).fDistY = (-1.0f)*(float)(*(em_GenObjectList_MEX+2));

	((em_GenObjectList.aObject[0]).Kinematic).fVrelX = (float)(*(em_GenObjectList_MEX+3));
	((em_GenObjectList.aObject[0]).Kinematic).fVrelY = (float)(*(em_GenObjectList_MEX+4));

	((em_GenObjectList.aObject[0]).aShapePointCoordinates[0]).fPosX = (float)(*(em_GenObjectList_MEX+5));
	((em_GenObjectList.aObject[0]).aShapePointCoordinates[0]).fPosY = (-1.0f)*((float)(*(em_GenObjectList_MEX+6)));
	((em_GenObjectList.aObject[0]).aShapePointCoordinates[1]).fPosX = -1000.f;
	((em_GenObjectList.aObject[0]).aShapePointCoordinates[1]).fPosY = -1000.f;
	((em_GenObjectList.aObject[0]).General).uiLifeCycles = (AlgoCycleCounter_t)(*(em_GenObjectList_MEX+7));
	/*mexPrintf("%d\n",((em_GenObjectList.HeaderObjList).iNumOfUsedObjects));
	mexPrintf("%f\n",((em_GenObjectList.aObject[0]).Kinematic).fDistX);
	mexPrintf("%f\n",((em_GenObjectList.aObject[0]).Kinematic).fDistY);
	mexPrintf("%f\n",((em_GenObjectList.aObject[0]).Kinematic).fVrelX);
	mexPrintf("%f\n",((em_GenObjectList.aObject[0]).Kinematic).fVrelY);
	mexPrintf("%f\n",((em_GenObjectList.aObject[0]).aShapePointCoordinates[0]).fPosX);
	mexPrintf("%f\n",((em_GenObjectList.aObject[0]).aShapePointCoordinates[0]).fPosY);
	mexPrintf("%d\n",((em_GenObjectList.aObject[0]).General).uiLifeCycles);*/


	((em_ARSObjectList.aObject[0]).Geometry).fWidth =  (float)(*(em_ARSObjectList_MEX+0));
	((em_ARSObjectList.aObject[0]).Geometry).fLength = (float)(*(em_ARSObjectList_MEX+1));
	//mexPrintf("%f\n",((em_ARSObjectList.aObject[0]).Geometry).fWidth);
	//mexPrintf("%f\n",((em_ARSObjectList.aObject[0]).Geometry).fLength);
	//

    s32_ret = LCDAS_Main(&em_PerfDegr_data, &em_Road, &em_MovingObjectTraces, &em_GenObjectList, &em_ARSObjectList, &vdy_VehPar, &vdy_VehDyn);
    RARR0(0,0) = s32_ret;
}
//

//提供给外部整体算法的接口函数，
int32 LCDAS_Main(PerfDegr_t *em_PerfDegr_data, Road_t *em_Road, MovingObjectTraces_t *em_MovingObjectTraces, EM_t_GenObjectList* em_GenObjectList, EM_t_ARSObjectList* em_ARSObjectList, VehPar_t* vdy_VehPar, VehDyn_t* vdy_VehDyn)
{
        int32  s32_ret = 0;

		NCDIS =  0.8f*(vdy_VehPar->VehParAdd).VehicleLength;
	    NDDIS =  (vdy_VehPar->VehParAdd).VehicleLength; 
	    JEDIS =  (vdy_VehPar->VehParAdd).VehicleWidth; 
		EJDIS =  -1.0f*(vdy_VehPar->VehParAdd).VehicleWidth; 
		RADARORIGIN = (vdy_VehPar->VehParAdd).VehicleWidth/2;

		//mexPrintf("%f\n",NCDIS);
		//mexPrintf("%f\n",EJDIS);
		//当前环境估算
		//是否产生性能下降
		//LCDAS_Params_Checked();//当前环境如何以及地方处于何处
		lcdasStateNow = LCDAS_StateJudge(vdy_VehPar,vdy_VehDyn);
		//mexPrintf("%d\n",lcdasStateNow);
		if(LCDASInactive == lcdasStateNow)
		{
			lcdasActiveStateNow = NonWarning;
		}
		else if(LCDASActive == lcdasStateNow)
		{
			lcdasActiveStateNow = LCDAS_ActiveStateJudge(vdy_VehPar, vdy_VehDyn, em_GenObjectList);
			if((BSW == SYSTEMTYPE) )
			{
				BlindSpotWarning_LCDAS_FCT(em_MovingObjectTraces,em_GenObjectList, em_ARSObjectList);
				mexPrintf("BSW = %d\n",bswWarningResult);
				if(bswWarningResult == ShallNotWarning)
				{
					lcdasActiveStateNow = NonWarning;
				}
				else 
				{
					if(bswWarningResult == MightWarning)
					{
						if(lcdasActiveStateNow == WarningLevel2)
						{
							lcdasActiveStateNow = WarningLevel1;
						}
						else
						{
							lcdasActiveStateNow = NonWarning;
						}
					}else if(bswWarningResult == ShallWarning)
					{
						;
					}
				}
			}
			else
			{
				if((CVW == SYSTEMTYPE))
				{
					ClosingVehicleWarning_LCDAS_FCT(em_MovingObjectTraces,em_GenObjectList, em_ARSObjectList,vdy_VehDyn);
					mexPrintf("CVW = %d\n",cvwWarningResult);
					if(cvwWarningResult == ShallNotWarning)
					{
						lcdasActiveStateNow = NonWarning;
					}
					else 
					{
						if(cvwWarningResult == MightWarning)
						{
							if(lcdasActiveStateNow == WarningLevel2)
							{
								lcdasActiveStateNow = WarningLevel1;
							}
							else
							{
								lcdasActiveStateNow = NonWarning;
							}
						}else if(cvwWarningResult == ShallWarning)
						{
							;
						}
					}
				}
				else
				{
					if((LCW == SYSTEMTYPE))
					{
						LaneChangeWarining_LCDAS_FCT(em_MovingObjectTraces,em_GenObjectList, em_ARSObjectList,vdy_VehDyn);
						//mexPrintf("LCW = %d\n",lcwWarningResult);
						if(lcwWarningResult == ShallNotWarning)
						{
							lcdasActiveStateNow = NonWarning;
						}
						else 
						{
							if(lcwWarningResult == MightWarning)
							{
								if(lcdasActiveStateNow == WarningLevel2)
								{
									lcdasActiveStateNow = WarningLevel1;
								}
								else
								{
									lcdasActiveStateNow = NonWarning;
								}
							}else if(lcwWarningResult == ShallWarning)
							{
								;
							}
						}
					}
					else
					{
						lcdasActiveStateNow = NonWarning;
					}
				}
			}
		}
		else
		{
			lcdasActiveStateNow = NonWarning;
		}
		if((LCDASActive == lcdasStateNow))
		{
			if(WarningLevel1 == lcdasActiveStateNow)
			{
				if((BSW == SYSTEMTYPE) && (u8_flag_overtaking))//OPTIONAL  BSW suppression
				{
					;//BSW 功能1.5秒后在恢复/记住时间到了，就要恢复功能且u8_flag_overtaking = 0；
					u8_flag_overtaking = 0;
                    s32_ret = 0;
				}
				else
                {
					AlarmLampOn();
                    s32_ret = 1;
                }
			}
			else if(WarningLevel2 == lcdasActiveStateNow)
			{
				if((BSW == SYSTEMTYPE)&& (u8_flag_overtaking))//OPTIONAL  BSW suppression
				{
					;//BSW 功能1.5秒后在恢复//记住时间到了，就要恢复功能且u8_flag_overtaking = 0；
					u8_flag_overtaking = 0;
                    s32_ret = 0;
				}
				else
				{
					AlarmLampOn();
					VibrationWarningOn();
                    s32_ret = 2;
				}
			}
			else
			{
				NoAction();
                s32_ret = 0;
			}
		}
		else
		{
			NoAction();
            s32_ret = 0;
		}
		//u8_flag_overtaking = 0;
		return s32_ret;		
}

LCDAS_t_State LCDAS_StateJudge(VehPar_t* vdy_VehPar, VehDyn_t* vdy_VehDyn)
{
	LCDAS_t_State lcdas_StateRet = LCDASInactive;

	if(AnyOneSatisfaction == lcdas_ActiveMethod || ContinuousActivation == lcdas_ActiveMethod)
	{
		//判断IGN_ONOrIGN_off
		//IF IGN_ON
		//    lcdas_StateRet = LCDASActive;
		//ELSE
		//	  lcdas_StateRet = LCDASInactive;
		//END
		//lcdas_StateRet = LCDASActive;
		;
	}

	if((AnyOneSatisfaction == lcdas_ActiveMethod || TurnSignalActivation == lcdas_ActiveMethod) && (lcdas_StateRet == LCDASInactive))
	{
		if(LeftRadar == RadarPosition)
		{
			if(1 == (vdy_VehPar->VehParMain).Turn_LampLeft)
			{
				lcdas_StateRet = LCDASActive;
			}
			else
			{
				lcdas_StateRet = LCDASInactive;
			}
		}
		else
		{
			if(RightRadar == RadarPosition)
			{
				if(1 == (vdy_VehPar->VehParMain).Turn_LampRight)
				{
					lcdas_StateRet = LCDASActive;
				}
				else
				{
					lcdas_StateRet = LCDASInactive;
				}
			}
			else
			{
				lcdas_StateRet = LCDASInactive;
			}
		}
	}
		
	if((AnyOneSatisfaction == lcdas_ActiveMethod || SubjectVehicleSpeedActivation == lcdas_ActiveMethod) && (lcdas_StateRet == LCDASInactive))
	{
		if(((vdy_VehDyn->Longitudinal).MotVar).Velocity >= THRESHOLDSPEED_X)
			lcdas_StateRet = LCDASActive;
		else
			lcdas_StateRet = LCDASInactive;
		if((LCDASInactive == lcdas_StateRet)&&(((vdy_VehDyn->Lateral).MotVar).Velocity >= THRESHOLDSPEED_Y) && LeftRadar == RadarPosition)
			lcdas_StateRet = LCDASActive;
		else
			lcdas_StateRet = LCDASInactive;
		if((LCDASInactive == lcdas_StateRet)&&(((vdy_VehDyn->Lateral).MotVar).Velocity <= (-1*THRESHOLDSPEED_Y))&& RightRadar == RadarPosition)
			lcdas_StateRet = LCDASActive;
		else
			lcdas_StateRet = LCDASInactive;
	}
			
	return lcdas_StateRet;
}

LCDAS_t_ActiveState LCDAS_ActiveStateJudge(VehPar_t* vdy_VehPar, VehDyn_t* vdy_VehDyn,EM_t_GenObjectList* em_GenObjectList)
{
	LCDAS_t_ActiveState lcdas_ActiveStateRet = WarningLevel1;
	uint8 u8_flag = 0;

	if((lcdas_EvaluationCriteria ==  TurnSignalEvaluation) \
	   || (lcdas_EvaluationCriteria == TurnSignal_SubjectVehicleSteeringInput) \
	   || (lcdas_EvaluationCriteria == TurnSignal_LateralClearance) \
	   || (lcdas_EvaluationCriteria == TurnSignal_SubjectVehicleSteeringInput_LateralClearance))
	{
		if(LeftRadar == RadarPosition)
		{
			if(1 == (vdy_VehPar->VehParMain).Turn_LampLeft)
			{
				u8_flag = (1<<0);
			}
			else
			{
				u8_flag = 0;
			}
		}
		else
		{
			if(RightRadar == RadarPosition)
			{
				if(1 == (vdy_VehPar->VehParMain).Turn_LampRight)
				{
					u8_flag = (1<<0);
				}
				else
				{
					u8_flag = 0;
				}
			}
			else
			{
				u8_flag = 0;
			}
		}
	}

	if((lcdas_EvaluationCriteria ==  SubjectVehicleSteeringInputEvaluation) \
		|| (lcdas_EvaluationCriteria == TurnSignal_SubjectVehicleSteeringInput) \
		|| (lcdas_EvaluationCriteria == SubjectVehicleSteeringInput_LateralClearance) \
		|| (lcdas_EvaluationCriteria == TurnSignal_SubjectVehicleSteeringInput_LateralClearance))
	{
		;
	}

	if((lcdas_EvaluationCriteria ==  LateralClearanceEvalution) \
		|| (lcdas_EvaluationCriteria == TurnSignal_LateralClearance) \
		|| (lcdas_EvaluationCriteria == SubjectVehicleSteeringInput_LateralClearance) \
		|| (lcdas_EvaluationCriteria == TurnSignal_SubjectVehicleSteeringInput_LateralClearance) )
	{
		;
	}
	switch(lcdas_EvaluationCriteria)
	{
		case TurnSignalEvaluation: if(u8_flag == TurnSignalEvaluation)
									   lcdas_ActiveStateRet = WarningLevel2;
									break;
		case SubjectVehicleSteeringInputEvaluation:
									if(u8_flag == SubjectVehicleSteeringInputEvaluation)
											lcdas_ActiveStateRet = WarningLevel2;
									break;
		case TurnSignal_SubjectVehicleSteeringInput:
									if(u8_flag == TurnSignal_SubjectVehicleSteeringInput)
											lcdas_ActiveStateRet = WarningLevel2;
									break;
		case LateralClearanceEvalution:
									if(u8_flag == LateralClearanceEvalution)
											lcdas_ActiveStateRet = WarningLevel2;
									break;
		case TurnSignal_LateralClearance:
									if(u8_flag == TurnSignal_LateralClearance)
											lcdas_ActiveStateRet = WarningLevel2;
									break;
		case SubjectVehicleSteeringInput_LateralClearance:
									if(u8_flag == SubjectVehicleSteeringInput_LateralClearance)
											lcdas_ActiveStateRet = WarningLevel2;
									break;
		case TurnSignal_SubjectVehicleSteeringInput_LateralClearance:
									if(u8_flag == TurnSignal_SubjectVehicleSteeringInput_LateralClearance)
											lcdas_ActiveStateRet = WarningLevel2;
									break;
		default:break;

	}

	return lcdas_ActiveStateRet;
}


static int32 BlindSpotWarning_LCDAS_FCT(MovingObjectTraces_t *em_MovingObjectTraces, EM_t_GenObjectList* em_GenObjectList, EM_t_ARSObjectList* em_ARSObjectList)
{
	uint8 u8_i,u8_j,u8_Count1 = 0,u8_Count2 = 0,u8_Count3 = 0,u8_Count4 = 0,u8_CountNone = 0;
	uint8 u8_Count = (uint8)((em_GenObjectList->HeaderObjList).iNumOfUsedObjects);
	float32 f_PosX,f_PosY,f_Width,f_Length,f_PosXTarget,f_PosYTarget;
	uint8 u8_shallNotWarningCount = 0,u8_shallWarningCount = 0,u8_mightWarningCount = 0;
	//uint16 shallWarningSpotCount = 0,shallWarningSpotCountC = 0, shallWarningSpotCountF = 0;
	//uint16 shallNotWarningSpotCount = 0;
	//mexPrintf(" %d\n",u8_Count);
	if(LCDASActive == lcdasStateNow && (BSW == SYSTEMTYPE || LCW == SYSTEMTYPE) )
	{
        #if(LeftRadar == RadarPosition)
		{
			for(u8_i = 0; u8_i < u8_Count; u8_i++)
			{
				 f_PosX = ((em_GenObjectList->aObject[u8_i]).Kinematic).fDistX;
				 f_PosY = ((em_GenObjectList->aObject[u8_i]).Kinematic).fDistY;

				 f_Width = ((em_ARSObjectList->aObject[u8_i]).Geometry).fWidth;//最左边，最右边，最上边，以及最下边
				 f_Length = ((em_ARSObjectList->aObject[u8_i]).Geometry).fLength;

				 for(u8_j = 0; u8_j < EM_GEN_OBJ_N_SHAPE_POINTS;u8_j++)
				 {
					 f_PosXTarget = ((em_GenObjectList->aObject[u8_i]).aShapePointCoordinates[u8_j]).fPosX;
					 f_PosYTarget = ((em_GenObjectList->aObject[u8_i]).aShapePointCoordinates[u8_j]).fPosY;
					 if(-1000.0f == f_PosXTarget || -1000.0f == f_PosYTarget)
						 break;
					 f_PosYTarget = ((em_GenObjectList->aObject[u8_i]).aShapePointCoordinates[u8_j]).fPosY - RADARORIGIN;
					 mexPrintf(" %f\n",((em_GenObjectList->aObject[u8_i]).aShapePointCoordinates[u8_j]).fPosY);
					 if(f_PosXTarget >= NBDIS)
					 {
						 u8_Count1++;
					 }
					 else
					 {
						 ;
					 }

					 if(f_PosYTarget <= EGDIS)
					 {
						 u8_Count2++;
					 }
					 else
					 {
						 ;
					 }

					 
					 if(f_PosXTarget <= NCDIS)
					 {
						 u8_Count3++;
					 }
					 else
					 {
						 ;
					 }
					 
					 if(f_PosYTarget >= EFDIS)
					 {
						 u8_Count4++;
					 }
					 else
					 {
						 ;
					 }

					 if((f_PosYTarget >= EHDIS) || (f_PosXTarget <= NADIS) || (f_PosXTarget >= NDDIS))
					 {
						u8_CountNone++;
					 }
					 else
					 {
						 ;
					 }
				 }
				 if(u8_Count1 >= 1 && u8_Count2 >= 1 && u8_Count3 == u8_j && u8_Count4 == u8_j && ((em_GenObjectList->aObject[u8_i]).General).uiLifeCycles > 4)
				 {
					 u8_shallWarningCount++;
					 /*#IF  u8_shallWarningCount == 1 && 超车成功
						 * u8_flag_overtaking = 1;
						 *#ELSE
						 *		u8_flag_overtaking = 0;
						 *#END
						 */
						// u8_flag_overtaking = 0;
				 }
				 else
				 {
					 if(u8_CountNone == u8_j)
						 u8_shallNotWarningCount++;
					 else
						 u8_mightWarningCount++;
				 }
				// mexPrintf(" r%d\n",u8_Count1);
				 // mexPrintf("r %d\n",u8_Count2);
				 //  mexPrintf(" r%d\n",u8_Count3);
				 //   mexPrintf("r %d\n",u8_Count4);
					 //mexPrintf(" %d\n",u8_Count1);
				 u8_Count1 = 0;
				 u8_Count2 = 0;
				 u8_Count3 = 0;
				 u8_Count4 = 0;
				 u8_CountNone = 0;

			}

			if(u8_shallWarningCount >= 1)
				bswWarningResult = ShallWarning;
			else
			{
				if(u8_shallNotWarningCount >= u8_Count-1 && u8_mightWarningCount <= 1)
					bswWarningResult = ShallNotWarning;
				else
					bswWarningResult = MightWarning;
			}

		}
		#else
		{
			#if(RightRadar == RadarPosition)
			{
				for(u8_i = 0; u8_i < u8_Count; u8_i++)
				{
					 f_PosX = ((em_GenObjectList->aObject[u8_i]).Kinematic).fDistX;
					 f_PosY = ((em_GenObjectList->aObject[u8_i]).Kinematic).fDistY;

					 f_Width = ((em_ARSObjectList->aObject[u8_i]).Geometry).fWidth;//最左边，最右边，最上边，以及最下边
					 f_Length = ((em_ARSObjectList->aObject[u8_i]).Geometry).fLength;

					 for(u8_j = 0; u8_j < EM_GEN_OBJ_N_SHAPE_POINTS;u8_j++)
					 {
						 f_PosXTarget = ((em_GenObjectList->aObject[u8_i]).aShapePointCoordinates[u8_j]).fPosX;
						 f_PosYTarget = ((em_GenObjectList->aObject[u8_i]).aShapePointCoordinates[u8_j]).fPosY;
						 if(-1000 == f_PosXTarget || -1000 == f_PosYTarget)
							 break;
						 f_PosYTarget = ((em_GenObjectList->aObject[u8_i]).aShapePointCoordinates[u8_j]).fPosY + RADARORIGIN;
						 if(f_PosXTarget >= NBDIS)
						 {
							 u8_Count1++;
						 }
						 else
						 {
							 ;
						 }

						 if(f_PosYTarget >= JLDIS)
						 {
							 u8_Count2++;
						 }
						 else
						 {
							 ;
						 }

						 if(f_PosXTarget <= NCDIS)
						 {
							 u8_Count3++;
						 }
						 else
						 {
							 ;
						 }
					 
						 if(f_PosYTarget <= JKDIS)
						 {
							 u8_Count4++;
						 }
						 else
						 {
							 ;
						 }

						 if((f_PosYTarget <= JMDIS) || (f_PosXTarget <= NADIS) || (f_PosXTarget >= NDDIS))
						 {
							u8_CountNone++;
						 }
						 else
						 {
							 ;
						 }
					 }
					 if(u8_Count1 >= 1 && u8_Count2 >= 1 && u8_Count3 == u8_j && u8_Count4 == u8_j && ((em_GenObjectList->aObject[u8_i]).General).uiLifeCycles > 4)
					 {
						 u8_shallWarningCount++;
						 /*#IF  u8_shallWarningCount == 1 && 超车成功
						 * u8_flag_overtaking = 1;
						 *#ELSE
						 *		u8_flag_overtaking = 0;
						 *#END
						 */
						// u8_flag_overtaking = 0;
					 }
					 else
					 {
						 if(u8_CountNone == u8_j)
							 u8_shallNotWarningCount++;
						 else
							 u8_mightWarningCount++;
					 }
					 u8_Count1 = 0;
					 u8_Count2 = 0;
					 u8_Count3 = 0;
					 u8_Count4 = 0;
					 u8_CountNone = 0;

				}

				if(u8_shallWarningCount >= 1)
					bswWarningResult = ShallWarning;
				else
				{
					if(u8_shallNotWarningCount >= u8_Count-1 && u8_mightWarningCount <= 1)
						bswWarningResult = ShallNotWarning;
					else
						bswWarningResult = MightWarning;
				}
			}
			#else
			{
					ErrorOutput("BlindSpotWarningPARAMSERROR");//error
			}
			#endif
		}
		#endif
	}
	else
	{
		;//ErrorOutput("BlindSpotWarningFunc Inactive or NotSurport");
	}
	return 1;
}

static int32 ClosingVehicleWarning_LCDAS_FCT(MovingObjectTraces_t *em_MovingObjectTraces, EM_t_GenObjectList* em_GenObjectList, EM_t_ARSObjectList* em_ARSObjectList,VehDyn_t* vdy_VehDyn) // TYPE 2system
{
	uint8 u8_i,u8_j,u8_Count1 = 0,u8_Count2 = 0,u8_Count3 = 0,u8_Count4 = 0,u8_CountNone1 = 0,u8_CountNone2 = 0;
	uint8 u8_Count = (uint8)((em_GenObjectList->HeaderObjList).iNumOfUsedObjects);
	float32 f_PosX,f_PosY,f_Width,f_Length,f_PosXTarget,f_PosYTarget;
	uint8 u8_shallNotWarningCount = 0,u8_shallWarningCount = 0;
	float32 f32_CTTC,f32_EgoSpeed,f32_STTC;
	if(LCDASActive == lcdasStateNow && (CVW == SYSTEMTYPE || LCW == SYSTEMTYPE) )
	{
		f32_EgoSpeed = ((vdy_VehDyn->Longitudinal).MotVar).Velocity;
		if(f32_EgoSpeed <= 10)
			f32_STTC = 2.5;
		else
		{
			if(f32_EgoSpeed <= 15)
				f32_STTC = 3.0;
			else
			{
				if(f32_EgoSpeed <= 20)
					f32_STTC = 3.5;
				else
					f32_STTC = 7.5;
			}
		}
		
		#if(LeftRadar == RadarPosition)
		{
			
			for(u8_i = 0; u8_i < u8_Count; u8_i++)
			{
				 f_PosX = ((em_GenObjectList->aObject[u8_i]).Kinematic).fDistX;
				 f_PosY = ((em_GenObjectList->aObject[u8_i]).Kinematic).fDistY;

				 f_Width = ((em_ARSObjectList->aObject[u8_i]).Geometry).fWidth;//最左边，最右边，最上边，以及最下边
				 f_Length = ((em_ARSObjectList->aObject[u8_i]).Geometry).fLength;

				 for(u8_j = 0; u8_j < EM_GEN_OBJ_N_SHAPE_POINTS;u8_j++)
				 {
					 f_PosXTarget = ((em_GenObjectList->aObject[u8_i]).aShapePointCoordinates[u8_j]).fPosX;
					 f_PosYTarget = ((em_GenObjectList->aObject[u8_i]).aShapePointCoordinates[u8_j]).fPosY;
					 if(-1000 == f_PosXTarget || -1000 == f_PosYTarget)
						 break;
					 f_PosYTarget = ((em_GenObjectList->aObject[u8_i]).aShapePointCoordinates[u8_j]).fPosY - RADARORIGIN;
					 if(f_PosXTarget <= NBDIS)
					 {
						 u8_Count1++;
					 }
					 else
					 {
						 ;
					 }

					 if(f_PosYTarget >= EFDIS)
					 {
						 u8_Count2++;
					 }
					 else
					 {
						 ;
					 }
					 
					 if(f_PosYTarget <= EGDIS)
					 {
						 u8_Count3++;
					 }
					 else
					 {
						 ;
					 }

					 if((f_PosXTarget < NADIS))
					 {
						u8_Count4++;
					 }
					 else
					 {
						 if(f_PosXTarget >= 0)
							 u8_CountNone1++;
						 else
						 {
							 ;
						 }

						 if(f_PosYTarget >= EHDIS || f_PosYTarget <= 0)
							 u8_CountNone2++;
						 else
						 {
							 ;
						 }

					 }
				 }
				 if(((em_GenObjectList->aObject[u8_i]).Kinematic).fVrelX == 0.0f)
					 ((em_GenObjectList->aObject[u8_i]).Kinematic).fVrelX = 0.00001f;
				 f32_CTTC =  f_PosX/((em_GenObjectList->aObject[u8_i]).Kinematic).fVrelX;//正负代表的靠近和远离，+：靠近，-：远离

				 if( f_PosX < 0 && (((em_GenObjectList->aObject[u8_i]).Kinematic).fVrelX >= 0))
					 f32_CTTC = (-1.0f)*f32_CTTC;
				 else
					 f32_CTTC = 10000;
				// mexPrintf("%f\n",f_PosXTarget);
				// mexPrintf("%f\n",((em_GenObjectList->aObject[u8_i]).Kinematic).fVrelX);
				// mexPrintf("%d\n",u8_Count1);
				// mexPrintf("%d\n",u8_Count3);

				 if((u8_Count1 == u8_j) && (u8_Count2 == u8_j) && (u8_Count3 >= 1) && f32_CTTC < f32_STTC && ((em_GenObjectList->aObject[u8_i]).General).uiLifeCycles > 4)
				 {
					 u8_shallWarningCount++;
					 //if()
					 //((em_GenObjectList->aObject[u8_i]).General).uiLifeCycles > 4
				 }

				 if(u8_CountNone2 == u8_j || u8_CountNone1 == u8_j)
				 {
				 	 u8_shallNotWarningCount++;
				 }
					 
				 if(u8_Count4 == u8_j && f32_CTTC >= 7.5)
				 {
					  u8_shallNotWarningCount++;
				 }
				 u8_Count1 = 0;
				 u8_Count2 = 0;
				 u8_Count3 = 0;
				 u8_Count4 = 0;
				 u8_CountNone1 = 0;
				 u8_CountNone2 = 0;

			}
			if((u8_shallWarningCount >= 1))
				cvwWarningResult = ShallWarning;
			else
			{
				if(u8_shallNotWarningCount >= u8_Count-1)
				{
					cvwWarningResult = ShallNotWarning;
				}
				else
					cvwWarningResult = MightWarning;
			}
		}
		#else
		{
			#if(RightRadar == RadarPosition)
			{
				for(u8_i = 0; u8_i < u8_Count; u8_i++)
				{
					f_PosX = ((em_GenObjectList->aObject[u8_i]).Kinematic).fDistX;
					f_PosY = ((em_GenObjectList->aObject[u8_i]).Kinematic).fDistY;

					f_Width = ((em_ARSObjectList->aObject[u8_i]).Geometry).fWidth;//最左边，最右边，最上边，以及最下边
					f_Length = ((em_ARSObjectList->aObject[u8_i]).Geometry).fLength;

					for(u8_j = 0; u8_j < EM_GEN_OBJ_N_SHAPE_POINTS;u8_j++)
					{
						 f_PosXTarget = ((em_GenObjectList->aObject[u8_i]).aShapePointCoordinates[u8_j]).fPosX;
						 f_PosYTarget = ((em_GenObjectList->aObject[u8_i]).aShapePointCoordinates[u8_j]).fPosY;
						 if(-1000 == f_PosXTarget || -1000 == f_PosYTarget)
							break;
						 f_PosYTarget = ((em_GenObjectList->aObject[u8_i]).aShapePointCoordinates[u8_j]).fPosY + RADARORIGIN;
						 if(f_PosXTarget <= NBDIS)
						 {
							 u8_Count1++;
						 }
						 else
						 {
							  ;
						 }

						 if(f_PosYTarget <= JKDIS)
						 {
							  u8_Count2++;
						 }
						 else
						 {
							 ;
						 }
					 
						 if(f_PosYTarget >= JLDIS)
						 {
							u8_Count3++;
						 }
						 else
						 {
							;
						 }

						 if((f_PosXTarget < NADIS))
						 {
							u8_Count4++;
						 }
						 else
						 {
							 if(f_PosXTarget >= 0)
								u8_CountNone1++;
							 else
							 {
								 ;
							 }

							 if(f_PosYTarget <= JMDIS || f_PosYTarget >= 0)
								u8_CountNone2++;
							 else
							 {
								;
							 }

						}
					}
					//
					 if(((em_GenObjectList->aObject[u8_i]).Kinematic).fVrelX == 0.0f)
						 ((em_GenObjectList->aObject[u8_i]).Kinematic).fVrelX = 0.00001f;
					 f32_CTTC =  f_PosX/((em_GenObjectList->aObject[u8_i]).Kinematic).fVrelX;//正负代表的靠近和远离，+：靠近，-：远离

					 if( f_PosX < 0 && (((em_GenObjectList->aObject[u8_i]).Kinematic).fVrelX >= 0))
						 f32_CTTC = (-1.0f)*f32_CTTC;
					 else
						 f32_CTTC = 10000;
					//

					 if((u8_Count1 == u8_j) && (u8_Count2 == u8_j) && (u8_Count3 >= 1) && f32_CTTC < f32_STTC && ((em_GenObjectList->aObject[u8_i]).General).uiLifeCycles > 4)
					 {
						u8_shallWarningCount++;
					 //if()
					 //((em_GenObjectList->aObject[u8_i]).General).uiLifeCycles > 4
					 }

					 if(u8_CountNone2 == u8_j || u8_CountNone1 == u8_j)
					 {
				 		u8_shallNotWarningCount++;
					 }
					 
					 if(u8_Count4 == u8_j && f32_CTTC >= 7.5)
					 {
						u8_shallNotWarningCount++;
					 }
					 u8_Count1 = 0;
					 u8_Count2 = 0;
					 u8_Count3 = 0;
					 u8_Count4 = 0;
					 u8_CountNone1 = 0;
					 u8_CountNone2 = 0;
				}
				if((u8_shallWarningCount >= 1))
					cvwWarningResult = ShallWarning;
				else
				{
					if(u8_shallNotWarningCount >= u8_Count-1)
					{
						cvwWarningResult = ShallNotWarning;
					}
					else
						cvwWarningResult = MightWarning;
				}
			}
			#else
			{
					;//ErrorOutput("BlindSpotWarningPARAMSERROR");//error
			}
			#endif
		}
		#endif
		// OPTIONAL DUAL SIDE CLOSING VEHICLE WARNING
		if(cvwWarningResult == ShallNotWarning)
		{
				u8_Count1 = 0;
				u8_Count2 = 0;
				u8_shallWarningCount = 0;
				u8_shallNotWarningCount = 0;
				for(u8_i = 0; u8_i < u8_Count; u8_i++)
				{
					f_PosX = ((em_GenObjectList->aObject[u8_i]).Kinematic).fDistX;
					f_PosY = ((em_GenObjectList->aObject[u8_i]).Kinematic).fDistY;

					f_Width = ((em_ARSObjectList->aObject[u8_i]).Geometry).fWidth;//最左边，最右边，最上边，以及最下边
					f_Length = ((em_ARSObjectList->aObject[u8_i]).Geometry).fLength;

					for(u8_j = 0; u8_j < EM_GEN_OBJ_N_SHAPE_POINTS;u8_j++)
					{
						 f_PosXTarget = ((em_GenObjectList->aObject[u8_i]).aShapePointCoordinates[u8_j]).fPosX;
						 f_PosYTarget = ((em_GenObjectList->aObject[u8_i]).aShapePointCoordinates[u8_j]).fPosY;
						 if(-1000 == f_PosXTarget || -1000 == f_PosYTarget)
							  break;
						 if(f_PosXTarget <= NODIS)
						 {
							 u8_Count1++;
						 }
						 else
						 {
							  ;
						 }

						#if(LeftRadar == RadarPosition)
						 if(f_PosYTarget <= 0 && f_PosYTarget >= EJDIS)
						 {
							u8_Count2++;
						 }
						 else
						 {
							;
						 }
						#else
						 if(f_PosYTarget >= 0 && f_PosYTarget <= JEDIS)
						 {
							u8_Count2++;
						 }
						 else
						 {
							;
						 }
						#endif

					}
					 f32_CTTC = f_PosXTarget/((em_GenObjectList->aObject[u8_i]).Kinematic).fVrelX;//正负代表的靠近和远离，+：远离，-：靠近

					 if(f32_CTTC < 0)
						f32_CTTC = -1*f32_CTTC;
					 else
						 f32_CTTC = 10000;

					 if((u8_Count1 == u8_j) && (u8_Count2 >= 1)  &&(f32_CTTC < f32_STTC)&& ((em_GenObjectList->aObject[u8_i]).General).uiLifeCycles > 4)
					 {
						u8_shallWarningCount++;
					 }
					
					 u8_Count1 = 0;
					 u8_Count2 = 0;
				}
				if((u8_shallWarningCount >= 1))
					cvwWarningResult = ShallWarning;
		}
	}
	else
	{
		;//ErrorOutput("ClosingVehicleWarningFunc Inactive or NotSurport");
	}
	return 1;
}

static int32 LaneChangeWarining_LCDAS_FCT(MovingObjectTraces_t *em_MovingObjectTraces, EM_t_GenObjectList* em_GenObjectList, EM_t_ARSObjectList* em_ARSObjectList,VehDyn_t* vdy_VehDyn)
{
	//这套系统把BSD和CVW结合在一起，但是他不提供警告，当车辆已非常快的移动速度从后方接近。快的定义
	if(LCW == SYSTEMTYPE)
	{
		BlindSpotWarning_LCDAS_FCT(em_MovingObjectTraces, em_GenObjectList, em_ARSObjectList);
		ClosingVehicleWarning_LCDAS_FCT(em_MovingObjectTraces,em_GenObjectList,em_ARSObjectList,vdy_VehDyn);
		if(ShallWarning == cvwWarningResult || ShallWarning == bswWarningResult)
		{
			lcwWarningResult = ShallWarning;
		}
		else
		{
			if(ShallNotWarning == cvwWarningResult && ShallNotWarning == bswWarningResult)
			{
				lcwWarningResult = ShallNotWarning;
			}
			else
			{
				lcwWarningResult = MightWarning;
			}
		}
	}
	else
	{
		;//ErrorOutput("LCWFuncNOTSUPPORT");
	}
	return 1;
}

//相对速度M/S
