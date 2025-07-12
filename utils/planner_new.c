#include "planner_standby.h"
#include "planner_mode_control.h"
#include "system_info.h"
#include "wash_floor_task.h"
#include "wash_wall_task.h"
#include "sensors_info.h"
#include "fsmCore.h"

#include <rtdbg.h>

int slope_label = -1;

int8_t plannerStandbyEnter(void)
{
    LOG_I("pln fsm %s\n", __FUNCTION__); 
    return 0;
}


int8_t plannerStandbyRun(void)
{
    system_info_t info = system_info_get();
    static rt_uint16_t pl_vol;
    
    LOG_I("info.work_mode = %d", info->work_mode);
    LOG_I("info.state = %d", info->state);
    
    switch (info->work_mode)
    {
        case SMART_MODE:
            LOG_I("in SMART_MODE");
            fsmSetEvent(&pln_fsm, PLN_EVENT_START_SMART_MODE);
            break;
        
        case WEEKLY_MODE:
            LOG_I("in WEEKLY_MODE");
            fsmSetEvent(&pln_fsm, PLN_EVENT_START_WEEKLY_MODE);
            break;
        
        case WATERLINE_MODE:
            LOG_I("in WATERLINE_MODE");
            fsmSetEvent(&pln_fsm, PLN_EVENT_START_WATERLINE_MODE);
            break;

        case ALL_MODE:  // floor and wall
            LOG_I("in ALL_MODE");
            fsmSetEvent(&pln_fsm, PLN_EVENT_START_ALL_MODE);
            break;

        case E_INVALID_MODE:
            LOG_I("in E_INVALID_MODE");
            break;

        default:
            break;
    }
    
	return 0;
}


int8_t plannerStandbyExit(void)
{

	LOG_I("pln fsm %s\n", __FUNCTION__);
        
	return 0;
}

