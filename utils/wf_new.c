#include "wash_floor_normal_clean.h"
#include <rtdbg.h>
#include "util.h"

int8_t normalCleanModeEnter(void)
{
    LOG_I("pln fsm %s\n", __FUNCTION__); 
    return 0;
}


int8_t normalCleanModeRun(void)
{
    struct point_info tank_start_point = {0.0f, 0.0f, 0.0f}; // 机器初始位置设为坐标原点
    
//    LOG_D("start wash_floor_clean_once");
//    wash_floor_clean_once(&tank_start_point, wash_floor_task_info.wash_times); // 清洗，直到碰到第三边
//    LOG_D("end wash_floor_clean_once");

//    // 更新机器位置和坐标
//    LOG_D("start wash_floor_to_next_clean_time");
//    wash_floor_to_next_clean_time(&tank_start_point);
//    LOG_D("end wash_floor_to_next_clean_time");
    
	return 0;
}


int8_t normalCleanModeExit(void)
{
	LOG_I("pln fsm %s\n", __FUNCTION__);
        
	return 0;
}

