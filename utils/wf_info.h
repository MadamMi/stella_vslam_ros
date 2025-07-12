#ifndef __WASH_FLOOR_INFO_H__
#define __WASH_FLOOR_INFO_H__
#include "util.h"
#include "system_info.h"
#include "app_server.h"
#include "ef_def.h"


// 洗地info
typedef struct {
	// int total_cleaned_row;	// 已清洗的行数
	int total_x_move_time;	// 累计换行x方向行走时长 ms
	int total_moved_row;    // 已行走的行数
	float start_yaw;
	int wash_times;         // 已清洗的次数
} wash_floor_rows_info;
// typedef struct wash_floor_rows_info *wash_floor_rows_info_t;

// 墙边状态
enum edge_info
{
	INIT_VALUE = 0x00u,
	GET_WALL, // 直角墙
	GET_STEPS, // 台阶
	GET_SLOPE, // 坡
};
void wash_floor_info_start_yaw_set_2(float yaw);

void wash_floor_info_start_yaw_set(wash_floor_rows_info *info, float yaw);

float wash_floor_info_start_yaw_get(void);

void wash_floor_info_total_cleaned_row_set(void);

int wash_floor_info_total_cleaned_row_get(void);

int wash_floor_info_total_row_get(void);
void wash_floor_info_reset(wash_floor_rows_info *info, float yaw);
void wash_floor_info_reset_once(wash_floor_rows_info *info, float yaw);
void wash_floor_info_reset_after_get_slope(wash_floor_rows_info *info, float yaw);

// int wash_floor_info_get_magnetic_angle(mag_calibrate_info_t sg_buff);

int get_yaw_by_geom_data(const mag_calibrate_info_t p_calib_list,
                                 const int list_len,
                                 const mag_calibrate_info_t p_geom_args, float *yaw);

float cal_gero_euc_dis(mag_calibrate_info_t info1, mag_calibrate_info_t info2);

float get_dirt_score(vision_info_t info, int num);

// enum edge_info move_to_edge(move_forward_action_return_info_t info, float start_yaw, int* slope_forward_time);
//排水口脱困
int robot_escape_drain(void);

// 将获取到的距离数据转成以机器中心为原点的点
int calculate_points_by_distance(distance_sensor_out_arg_t dis_info, point_info_t pt_info_buff, int *dis_pt_info_num, int *rotate_point_count);

int Least_Squarel_Linear_Fit(float *a0, float *a1, float *a2, int rotate_point_count, point_info_t pt_info_buff);
int fit_line_by_dis_inf0(float base_yaw, const dis_sensor_buffer_info_t dis_info, float *a0, float *a1, float *a2);

int mag_calibrate_info2mag_adjest_yaw_info(mag_calibrate_info_t calibrate_info, mag_adjest_yaw_info_t adjest_info);

// 计算两个向量之间的夹角余弦
float calculate_cosine(float v1[], float v2[], int n);
// 计算向量模长
float vectorMagnitude(float v[], int n);

// 控制补光灯
int wf_ctr_light(int mode);

int move_to_deep_area(int forward_time);

// 根据pitch,计算换行前进时间
int move_time_to_next_line(void);

int update_short_line_info(int cur_line_forawrd_time, int* short_distance_lines);

// 地漏处理
int dislodged_from_drain(int if_odd_line, float target_yaw);

// 关机
int power_off_model(enum work_mode_type work_mode, int delay_time, goto_shallow_info_t info);

// 关闭本次洗地
int wash_floor_off_cur_time(void);

// 获取机器型号 P/S/V
int get_model_info(model_info_t cur_model_info, rt_uint8_t *weekly_max_clean_time);

int restrict_value_range(p_app_clean_paras_t conf_info);
// 获取洗地参数信息
int get_wf_paras(enum work_mode_type work_mode, p_app_clean_paras_t conf_info, wf_clean_info_t info);

int verify_same_sign_and_increasing(float a, float b);
// 打印洗地参数
void print_params(const wf_params_t params);

// 获取并更新参数信息
void update_params_by_mode_and_circle(enum work_mode_type work_mode, int circle);
// 获取更新的参数信息
wf_params_t get_wf_params_info(void);

// 根据最大清洗距离40m，设置最大清洗行数。
// return：最大清洗行数
int normal_version_get_max_clean_rows(float adjacent_line_interval);

int update_clean_info_data(wf_params_t test_param, wf_clean_info_t info);

int austra_goto_start_point_act(float* wash_start_yaw);

float reset_imu(int delay_ms, int wp_speed);

int init_wash_record_info(wash_record_info_t record_info, wash_floor_rows_info* wash_info, int slope_label);
// 计算缓坡面积
float calculate_trapezoid_area(float a, float c, float h, float h1);
// 用3*梯形面积计算池底面积
float calculate_pool_area(float a, float c, float h);
// 换行时绕地漏
int move_to_next_line_avoid_drain_(int if_odd_line, int speed);

#endif // __WASH_FLOOR_INFO_H__
