#include "build_map.h"
#include <rtdbg.h>
#include "sensors_info.h"
#include "actions_on_floor.h"
#include "wash_floor_info.h"
#include "move_basic.h"

extern int slope_label;
extern wash_floor_rows_info wash_floor_task_info;
int stright_to_deep_area(float start_yaw, float delta_slope);

static int pentagonal_path_find_slope_new(void)
{
    float yaw;
    struct slope_type slope;
    LOG_D("slope_label==%d", slope_label);
    
    int find_slope_time = 0;
#if (VERSION_TYPE!=E_AUSTRALIA)

        LOG_I("start pentagonal path find slope");
        actions_on_floor_search_slope(RT_NULL, &find_slope_time);  

         
        // 重置start_yaw
        yaw = get_current_yaw();
#else   // (VERSION_TYPE!=E_AUSTRALIA)
        // 走到起始点,并初始化yaw
        austra_goto_start_point_act(&yaw);
        LOG_D("aus start_yaw=%f", yaw);
#endif  // (VERSION_TYPE!=E_AUSTRALIA)

        slope_label = find_slope_time;
        LOG_I("slope_label=%d", find_slope_time);

        
        wash_floor_info_start_yaw_set(&wash_floor_task_info, yaw);
        LOG_I("reset start yaw=%f", yaw);
        if (slope_label == 1) // 找到坡
        {
            // 走到深水区
            LOG_D("go to deep area");
            stright_to_deep_area(yaw, DELTA_SLOPE);
            LOG_D("go to deep area end");
        }
#if (VERSION_TYPE==E_NORMAL)
        else 
        {
            LOG_D("no slope");
            yaw = reset_imu(15*1000, 1000);
            wash_floor_info_start_yaw_set(&wash_floor_task_info, yaw);
        }
#endif
    
    return 0;
}

//static int reset_area_and_slope_info_new(int wash_times)
//{
//    // 重置找深水区状态
//    deep_area_state = STATE_INITIAL;
//    state_count = 0;
//    LOG_I("init deep_area_state");

//    // 重置标志
//    LOG_D("wash_times=%d", wash_times);
//    if (wash_times%2==0)
//    {
//        LOG_I("init slope state");
//        if_find_deep_area = 0;
//    }

//    if(wash_times == 0)
//        slope_label = -1;

//    LOG_D("slope_label=%d, if_find_deep_area=%d", slope_label, if_find_deep_area);

//    return 0;
//}

int8_t testMapModeEnter(void)
{
    LOG_I("pln fsm %s\n", __FUNCTION__); 
    return 0;
}


int8_t testMapModeRun(void)
{
    laser_switch_on();
    LOG_D("turn on laser.");
    LOG_D("set pump speed=%d", get_wf_params_info()->forward_wp_speed);
    move_wp_speed_on_floor_set(get_wf_params_info()->forward_wp_speed);
    rt_thread_mdelay(1*1000);
    
    pentagonal_path_find_slope_new();
	return 0;
}


int8_t testMapModeExit(void)
{

	LOG_I("pln fsm %s\n", __FUNCTION__);
        
	return 0;
}
