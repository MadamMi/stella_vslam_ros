#ifndef __ACTIONS_ON_FLOOR_H__
#define __ACTIONS_ON_FLOOR_H__

#include "util.h"


// 测距相关
int actions_on_floor_rotate_measure(void *p_arg_in, void *p_arg_out); // 旋转测距
int actions_on_floor_rotate_measure_no_stop(void *p_arg_in, void *p_arg_out);
int get_current_dis(int sensor_flag, int *distance);                  // 定点测距

// 地磁相关
int get_current_gero(mag_calibrate_info_t info);                            // 定点测地磁
int actions_on_floor_magnetic_calibration(void *p_arg_in, void *p_arg_out); // 地磁标定
int get_next_area_gero(void *p_arg_in, void *p_arg_out);                    // 获取下一行地磁数据

// 机器动作相关
int actions_on_floor_move_forward_to_obstacle(void *p_arg_in, void *p_arg_out);        // 前进到距障碍物小于阈值退出
int actions_on_floor_move_to_edge(void *p_arg_in, void *p_arg_out);                    // 前进到墙边
int actions_on_floor_move_to_next_line(void *p_arg_in, void *p_arg_out);               // 换行
int actions_on_floor_move_forward_with_pid_and_time(void *p_arg_in, void *p_arg_out);  // 前进with pid and time
int actions_on_floor_move_backward_with_pid_and_time(void *p_arg_in, void *p_arg_out); // 后退with pid and time

int actions_on_floor_search_wall(void *p_arg_in, void *p_arg_out);  // 在地面上找墙
int actions_on_floor_backward_update_yaw(void *p_arg_in, void *p_arg_out);
int actions_on_judge_c_edge(void *p_arg_in, void *p_arg_out); // 判断是否在C型边缘
int actions_on_floor_search_slope(void *p_arg_in, void *p_arg_out); // 在地面上找坡
int actions_on_floor_backward_search_wall(void *p_arg_in, void *p_arg_out); // 在地面上后退找墙
int actions_on_floor_goto_shallow_turn_off(void *p_arg_in, void *p_arg_out); // 走向浅水区关机
void set_robot_weeklymode_sleep_on_time(rt_uint32_t time_s);
int rotate_and_get_max_pitch(slope_info_t info);
int _clean_edge(float start_yaw, wf_params_t params);
void stop_at_waterline(void);
void goto_waterline_and_stop(stop_on_waterline_info_t stop_info);

extern int actions_on_slope_wash_around(void *p_arg_in, void *p_arg_out);

#endif
