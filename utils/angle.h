#ifndef __ANGLE_H__
#define __ANGLE_H__

#include "rtdef.h"

typedef struct
{
	float linear_x;     /* 沿x轴的线速度，单位 m/s */
	float linear_y;     /* 沿y轴的线速度，单位 m/s */ 
	float angular_z;    /* 饶z轴的角速度，单位 rad/s */
}vel_t;

typedef struct
{
	float x;
	float y;
	float theta;
}pos_t;

float compare_yaws(float current, float target);
float calculate_yaw(float current, float delta);
vel_t get_velocities(int lf_speed, int rt_speed);
void cal_pentagonal_points_by_path(rt_uint16_t* buffer, rt_uint8_t len, const rt_uint16_t speed, pos_t *pos);
float cal_area_by_pos(pos_t *pos, rt_uint8_t len);

#endif
