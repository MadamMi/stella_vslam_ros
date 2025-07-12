#ifndef __PLANNER_INFO_H__
#define __PLANNER_INFO_H__

#include "util.h"
#include "system_info.h"
#include "rtdef.h"

int get_speed_settings(enum work_mode_type work_mode, rt_uint16_t voltage, speed_info_t info);

#endif // __PLANNER_INFO_H__
