// 根据清洁模式和电压获取转速设置的函数
#include "planner_info.h"
#include "app_config.h"
#include "eeprom.h"
#include "system_info.h"


#define DBG_TAG "p_info"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

int get_speed_settings(enum work_mode_type work_mode, rt_uint16_t voltage, speed_info_t info) 
{
    info->motor_speed = WASH_FLOOR_MOTOR_SPEED;
    switch (work_mode) {
        case ALL_MODE:
            if (voltage > AM_WASH_FLOOR_VOL_THRES) {
                info->pump_speed = PUMP_SPEED_HIGH;
            } else {
                // 电压小于24V时
                info->pump_speed = PUMP_SPEED_LOW;
            }
            break;
//        case FLOOR_MODE:
//            if (voltage > 26000) { 
//                info->pump_speed = PUMP_SPEED_HIGH;
//            } else if (voltage > 23000) {
//                info->pump_speed = PUMP_SPEED_LOW;
//            } else {
//                info->pump_speed = PUMP_SPEED_HIGH;
//            }
//            break;
        case SMART_MODE:
            // SMART_MODE下，水泵转速固定为3300，驱动电机转速为3000
            info->pump_speed = PUMP_SPEED_HIGH;
            break;
        case WEEKLY_MODE:
            info->pump_speed = PUMP_SPEED_HIGH;
            break;
        default:
            break;
    }

    return 0;
}

