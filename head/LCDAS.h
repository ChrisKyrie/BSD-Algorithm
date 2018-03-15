#ifndef _FCT_LCDAS_H
#define _FCT_LCDAS_H

#include "InterfaceTYPES_inf.h"

#define BSW     1 //Blind spot warning
#define CVW     2 //Closing vehicle warning
#define LCW     3 //Lane change warning

#define SYSTEMTYPE  BSW

#define LeftRadar   1
#define RightRadar  2
#define RadarPosition    LeftRadar //左右雷达选择

#define EHDIS    6.0//单位为m为矢量
#define EGDIS    3.0
#define EFDIS    0.5
#define JMDIS    -EHDIS
#define JLDIS    -EGDIS
#define JKDIS    -EFDIS
#define NADIS    -30.0
#define NODIS    -10.0
#define NBDIS    -3.0

//#define EJDIS    -3.0
//#define JEDIS    3.0


typedef enum
{
	LCDASInactive = 0,
	LCDASActive = 1
}LCDAS_t_State;

typedef enum 
{
	ContinuousActivation = 1,        //每当装有BSD雷达的车辆IGN ON的时候，系统就可以连续工作
	TurnSignalActivation = 2,        //依赖于转向灯状态
	SubjectVehicleSpeedActivation = 4, //speed < 60km/h
	AnyOneSatisfaction = 8
}LCDAS_t_ActiveMethod;

#define THRESHOLDSPEED_X    8     //(m/s)
#define THRESHOLDSPEED_Y    1     //(m/s)

typedef enum
{
	NonWarning     = 0, //系统处于激活状态，但没有满足最低级警告的要求
	WarningLevel1 = 1,  //警告请求满足但是没有满足相应的评价标准，比等级2及以上的级别要低，
	WarningLevel2 = 2    //灯亮闪烁加其他
}LCDAS_t_ActiveState;

typedef enum  //是否将危险等级提高，是依据于车是否要变更车道，要觉察到这个意图，将危险等级提高
{
	TurnSignalEvaluation = 1,//警告请求满足但是没有满足相应的评价标准，比等级2及以上的级别要低，属于次等级
	SubjectVehicleSteeringInputEvaluation = 2,
	LateralClearanceEvalution = 4,
	TurnSignal_SubjectVehicleSteeringInput = 3,
	TurnSignal_LateralClearance = 5,
	SubjectVehicleSteeringInput_LateralClearance = 6,
	TurnSignal_SubjectVehicleSteeringInput_LateralClearance = 7
}LCDAS_t_EvaluationCriteria;

typedef enum
{
	ShallWarning = 1,//警告请求满足但是没有满足相应的评价标准，比等级2及以上的级别要低，属于次等级
	MightWarning = 2,
	ShallNotWarning = 4
}WarningResult_t;

int32 LCDAS_Main(PerfDegr_t *em_PerfDegr_data, Road_t *em_Road, MovingObjectTraces_t *em_MovingObjectTraces, EM_t_GenObjectList* em_GenObjectList, EM_t_ARSObjectList* em_ARSObjectList, VehPar_t* vdy_VehPar, VehDyn_t* vdy_VehDyn);
LCDAS_t_State LCDAS_StateJudge(VehPar_t* vdy_VehPar, VehDyn_t* vdy_VehDyn);
LCDAS_t_ActiveState LCDAS_ActiveStateJudge(VehPar_t* vdy_VehPar, VehDyn_t* vdy_VehDyn,EM_t_GenObjectList* em_GenObjectList);

static int32 BlindSpotWarning_LCDAS_FCT(MovingObjectTraces_t *em_MovingObjectTraces, EM_t_GenObjectList* em_GenObjectList, EM_t_ARSObjectList* em_ARSObjectList);
static int32 ClosingVehicleWarning_LCDAS_FCT(MovingObjectTraces_t *em_MovingObjectTraces, EM_t_GenObjectList* em_GenObjectList, EM_t_ARSObjectList* em_ARSObjectList,VehDyn_t* vdy_VehDyn);
static int32 LaneChangeWarining_LCDAS_FCT(MovingObjectTraces_t *em_MovingObjectTraces, EM_t_GenObjectList* em_GenObjectList, EM_t_ARSObjectList* em_ARSObjectList,VehDyn_t* vdy_VehDyn);

#endif

