#include "planner_mode_control.h"
#include "rtthread.h"
#include "fsmCore.h"
#include "build_map.h"
#include "planner_standby.h"
#include "wash_floor_fast_clean.h"
#include "wash_floor_normal_clean.h"
#include "wash_floor_task.h"
#include "wash_wall_task.h"

fsmTypeDef                  pln_fsm;
/*  mode_run, mode_exit  */
/*  this table must align with amModeTypeDef  */
fsmFuncTypeDef pln_fsmFuncTable[PLN_FSM_MODE_COUNT] = 
{
	/*  RUN_MODE_STANDBY  */
	{plannerStandbyEnter, plannerStandbyRun, plannerStandbyExit},
    /*  RUN_MODE_WASHFLOOR  */
	{washFloorModeEnter, washFloorModeRun, washFloorModeExit},
    /*  RUN_MODE_WASHWALL  */
	{washWallModeEnter, washWallModeRun, washWallModeExit},
    /*  RUN_MODE_TEST_MAP  */
	{testMapModeEnter, testMapModeRun, testMapModeExit},
    /*  RUN_MODE_FAST_CLEAN  */
	{fastCleanModeEnter, fastCleanModeRun, fastCleanModeExit},
    /*  RUN_MODE_NORMAL_CLEAN  */
	{normalCleanModeEnter, normalCleanModeRun, normalCleanModeExit},    
};

/*  event, current mode, next mode  */
fsmTableTypeDef pln_fsmTable[PLN_FSM_TABLE_LEN] = 
{
    /*  RUN MODE STANDBY  */
    {PLN_EVENT_START_SMART_MODE,        RUN_MODE_STANDBY,     RUN_MODE_WASHFLOOR},
    {PLN_EVENT_START_WEEKLY_MODE,       RUN_MODE_STANDBY,     RUN_MODE_WASHFLOOR},
    {PLN_EVENT_START_WATERLINE_MODE,    RUN_MODE_STANDBY,     RUN_MODE_WASHWALL},
    {PLN_EVENT_START_ALL_MODE,          RUN_MODE_STANDBY,     RUN_MODE_WASHFLOOR},
    
    /*  WASH FLOOR  */
    {PLN_EVENT_WASHFLOOR_TEST_MAP,        RUN_MODE_WASHFLOOR,     RUN_MODE_TEST_MAP},
    {PLN_EVENT_WASHFLOOR_FAST_CLEAN,      RUN_MODE_WASHFLOOR,     RUN_MODE_FAST_CLEAN},
    
    {PLN_EVENT_WASHFLOOR_NORMAL_CLEAN,    RUN_MODE_WASHFLOOR,     RUN_MODE_NORMAL_CLEAN},
    
    
    /* WASH WALL */
    
    
};

/*  eventManage  */
int8_t plnFsmEventManage(void)
{

        
    
	return 0;
}


