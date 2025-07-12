#ifndef __WASH_WALL_TASK_H__
#define __WASH_WALL_TASK_H__

#include <stdint.h>

int wash_wall_thread(void);
int wash_wall_routine_v(void * p_arg);
int all_mode_wash_wall_routine_v(int flag);
int8_t washWallModeEnter(void);
int8_t washWallModeRun(void);
int8_t washWallModeExit(void);

#endif // __WASH_WALL_TASK_H__
