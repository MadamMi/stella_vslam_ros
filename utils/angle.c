#include "angle.h"
#include "math.h"

const float PI = 3.141592653f;

/**
 * @brief 计算从current到target的角度差
 *
 * @param current 当前角度
 * @param target  目标角度
 * @return float > 0 在目标的右侧，需要从current左到target的角度
 * 				 < 0 在目标的左侧，需要从current右到target的角度
 * 				 -180<=值<=180
 */
float compare_yaws(float current, float target)
{
	float delta = target - current;
	if (delta > 180.0f)
	{
		delta = delta - 360.0f;
	}
	else if (delta < -180.0f)
	{
		delta = delta + 360.0f;
	}
	return delta;
}

/**
 * 计算新的偏航角, 按照IMU的坐标系返回(不是虚拟坐标系)
 *
 * 输入:
 * start_yaw - 输入的角度
 * delta     - 角度偏差
 *             正-往左计算, 即逆时针
 *             负-往右计算, 即顺时针
 * */
float calculate_yaw(float current, float delta)
{
	float target_yaw;

	target_yaw = current + delta;

	if (target_yaw >= 180.0f)
	{
		target_yaw = target_yaw - 360.0f; // 修正为负数
	}
	else if (target_yaw <= -180.0f)
	{
		target_yaw = 360.0f + target_yaw; // 修正为正数
	}

	return target_yaw;
}

/**
 * 移动机器人差速运动学
 *
 * 输入:
 * 	lf_speed 	- 左电机速度，正：前进，负：后退，单位：rpm
 * 	rt_speed     - 右电机速度，正：前进，负：后退，单位：rpm
 * 输出：
 *     见结构体定义   
 *             
 **/

vel_t get_velocities(int lf_speed, int rt_speed)
{
	const float GEAR_RATIO = 80 * 2.45f;			// 电机齿轮箱减速比*底盘齿轮减速比
	const float WHEEL_DIAMETER = 0.128f;			// 履带底盘轮子直径，单位：米
	const float BASE_WIDTH = 0.1707f;				// 履带底盘半径，单位：米

	vel_t vel;
	float average_rpm_x,average_rps_x;
	float average_rpm_a,average_rps_a;
	
	average_rpm_x = (lf_speed + rt_speed)/2.0;
	//convert revolutions per minute to revolutions per second
	average_rps_x = average_rpm_x / 60; // RPS
	vel.linear_x = average_rps_x / GEAR_RATIO  * (WHEEL_DIAMETER * PI); // m/s
	//
	vel.linear_y = 0.0;
	
	average_rpm_a = (rt_speed - lf_speed) / 2.0f;
	//convert revolutions per minute to revolutions per second
	average_rps_a = average_rpm_a / 60;
	vel.angular_z = (average_rps_a / GEAR_RATIO  * (WHEEL_DIAMETER * PI)) / BASE_WIDTH;
	
	return vel;
}

/**
 * 二维平面坐标变换
 *
 * 输入:
 * 	 p1		- 初始位姿
 * 	 theta	- 沿自身z轴旋转的角度
 * 	 length - 沿自身x轴平移的长度
 * 输出：
 *   		- 经过变换矩阵后的位姿
 *             
 **/
static pos_t transform_2d(pos_t p1, float theta, float length)
{
	pos_t p;

	p.theta = p1.theta + theta;
	if (p.theta > 2*PI)
		p.theta = p.theta - 2*PI;
	else if(p.theta < -2*PI)
		p.theta = p.theta + 2*PI;

	p.x = length * cosf(p.theta) + p1.x;
	p.y = length * sinf(p.theta) + p1.y;

	return p;
}

/**
 * 根据五角星的边长，计算各顶点坐标。
 *
 * 输入:
 * 		buffer   - 输入边长数组，单位 ms
 * 		len		 - 边长数组的长度
 * 		speed	 - 行进电机转速, 单位 rpm
 * 		size     - 数组的大小 
 * 输出：
 * 		point    - 顶点坐标
 * 
 * note：起始点为坐标原点，起始点行进方向为y轴正方向。相邻两条边的旋转角度为144度
 * */

void cal_pentagonal_points_by_path(rt_uint16_t* buffer, rt_uint8_t len, const rt_uint16_t speed, pos_t *pos)
{
	const float delta = -144/180.0f*PI;
	vel_t vel;
	float length;
	pos_t p0 = {0, 0, 0};

	/* 线速度 */
	vel = get_velocities(speed, speed);

	for (int i = 0; i < len; i++)
	{
		/* 每条边的长度*/
		length = vel.linear_x * buffer[i] / 1000.0f;

		if (i == 0)
		{
			pos[i] = transform_2d(p0, PI/2, length);
		}
		else
		{
			pos[i] = transform_2d(pos[i-1], delta, length);
		}
	}
}

/**
 * 坐标点求矩形面积
 *
 * 输入:
 * 		pos   - 坐标点数组
 * 		len   - 坐标点个数
 * 输出：
 * 		area  - 外接矩形的面积
 * */

float cal_area_by_pos(pos_t *pos, rt_uint8_t len)
{
	float x_min, x_max, y_min, y_max;
	float area;

	x_min = x_max = pos[0].x;
	y_min = y_max = pos[0].y;

	for (int i = 1; i < len; i++)
	{
		if (x_min > pos[i].x)
			x_min = pos[i].x;
		if (x_max < pos[i].x)
			x_max = pos[i].x;
		if (y_min > pos[i].y)
			y_min = pos[i].y;
		if (y_max < pos[i].y)
			y_max = pos[i].y;
	}

	area = (x_max - x_min) * (y_max - y_min);
	
	return area;
}
