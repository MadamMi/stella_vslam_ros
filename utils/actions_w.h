#ifndef __ACTIONS_ON_WALL_H__
#define __ACTIONS_ON_WALL_H__

#include "actions.h"
#include "move_basic.h"
#include "util.h"

int actions_on_wall_find_next_entry(void *p_arg_in, void *p_arg_out);

int actions_on_wall_climb_up(void *p_arg_in, void *p_arg_out);
int actions_on_wall_climb_down(void *p_arg_in, void *p_arg_out);

int actions_on_wall_move_forward(void *p_arg_in, void *p_arg_out);
int actions_on_wall_move_backward(void *p_arg_in, void *p_arg_out);

int actions_on_wall_clean_water_line_diff(void *p_arg_in, void *p_arg_out);
int actions_on_wall_clean_water_line(void *p_arg_in, void *p_arg_out);

int actions_on_wall_get_water_depth(void *p_arg_in, void *p_arg_out);

int actions_on_wall_vent_gas(void *p_arg_in, void *p_arg_out);
int actions_on_wall_clean_wall(void *p_arg_in, void *p_arg_out);

extern int actions_on_wall_climb_stairs(void *p_arg_in, void *p_arg_out);

int actions_on_wall_stop_on_waterline(void *p_arg_in, void *p_arg_out);

#endif
