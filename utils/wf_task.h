#ifndef __WASH_FLOOR_TASK_H__
#define __WASH_FLOOR_TASK_H__
#include "system_info.h"
#include "util.h"

// int get_next_row_point(point_t point);
int wash_floor_thread(enum work_mode_type work_mode);
int get_next_row_point(point_info_t point);
int get_next_row_point_no_stop(point_info_t point, float yaw);
int8_t washFloorModeEnter(void);
int8_t washFloorModeRun(void);
int8_t washFloorModeExit(void);

#endif // __WASH_FLOOR_TASK_H__
