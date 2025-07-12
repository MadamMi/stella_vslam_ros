#include <math.h>
#include "rtthread.h"
#include "wash_floor_info.h"
#include "task_comm.h"
#include "app_config.h"
#include "actions_on_floor.h"
#include "angle.h"
#include "system_info.h"
#include "sensors_info.h"
#include "move_basic.h"
#include "mtr_ctrl.h"
#include "rtdef.h"
#include "actions_on_wall.h"
#include "led_show.h"
#include "monitor.h"
#include "util.h"


#define PI 3.1415926
#define MACHINE_LENGTH 215  // 机身长度的一半
#define FILTER_ULT_THRES  1500  // mm，拟合时只用激光数据
#define YAW_DIS_THRES     10.0f    // pidyaw角纠偏记录阈值
#define PITCH_MIN         5.0f
#define PITCH_MAX         15.0f
#define PITCH_DOWN_MAX    15.0f

#define DBG_TAG "wash_floor_info"
// #define DBG_LVL DBG_INFO
#define DBG_LVL DBG_LOG
#include <rtdbg.h>


static wash_floor_rows_info wash_floor_clean_info = {0, 0, 0.0f};
static wf_params_t wf_param_info = RT_NULL;
// static wash_floor_rows_info wash_floor_info;

void wash_floor_info_start_yaw_set_2(float yaw)
{
	wash_floor_clean_info.start_yaw = yaw;
}

void wash_floor_info_start_yaw_set(wash_floor_rows_info *info, float yaw)
{
	info->start_yaw = yaw;
}

float wash_floor_info_start_yaw_get(void)
{
	return wash_floor_clean_info.start_yaw;
}

void wash_floor_info_total_cleaned_row_set(void)
{
	wash_floor_clean_info.total_moved_row++;
}

int wash_floor_info_total_cleaned_row_get(void)
{
	return wash_floor_clean_info.total_moved_row;
}

void wash_floor_info_total_moved_row_set(void)
{
	wash_floor_clean_info.total_moved_row++;
}

int wash_floor_info_total_row_get(void)
{
	return wash_floor_clean_info.total_moved_row;
}

void wash_floor_info_reset(wash_floor_rows_info *info, float yaw)
{
	info->total_moved_row = 0;
	info->total_x_move_time = 0;
	info->start_yaw = yaw;
    info->wash_times = 0;
}

void wash_floor_info_reset_once(wash_floor_rows_info *info, float yaw)
{
    LOG_D("update wash time");
	info->total_moved_row = 0;
	info->total_x_move_time = 0;
    info->start_yaw = yaw;
    info->wash_times += 1;
    LOG_D("wash time=%d", info->wash_times);
}

void wash_floor_info_reset_after_get_slope(wash_floor_rows_info *info, float yaw)
{
	info->start_yaw = yaw;
    LOG_I("reset start_yaw=%f", yaw);
    if (info->wash_times%2) // 奇数次清洗
    {
        info->wash_times -= 1;
        LOG_D("odd wash time, reset wash time=%d", info->wash_times);
    }
    info->total_moved_row = 0;
	info->total_x_move_time = 0;
}
// // 360度地磁标定
// int wash_floor_info_get_magnetic_angle(mag_calibrate_info_t sg_buff)
// {
// 	struct mag_cali_arg_in arg_in = {
//        .step_angle = 5.0f,
//        .buff = sg_buff,
//        .max_count = 72,
//     };
//     struct mag_cali_arg_out arg_out;

//     if (actions_on_floor_magnetic_calibration(&arg_in, &arg_out)!= 0)
//     {
//         LOG_E("actions_on_floor_magnetic_calibration failed");
//         return -1;
//     }
//     LOG_D("arg_out.out_count = %d", arg_out.out_count);
// 	return 0;
// }



/**
 * @brief 通过地磁数据找对应的区间(用于标定yaw角)
 *
 * @param p_calib_list 地磁-yaw角标定信息列表
 * @param list_len     列表长度
 * @param p_geom_args  当前地磁信息
 * @return int         当前地磁信息对应的区间
 *                     -1：未找到有效区间
 *                     其他：表示正常数组下标
 *                     eg.返回0，表示区间0，即对应校准信息数组下标[0, 1]
 */
static int _get_idx_by_geom_data(const mag_calibrate_info_t p_calib_list,
                                 const int list_len,
                                 const mag_calibrate_info_t p_geom_args)
{
    int pos = 0x00, min_pos = -1;
    for (; pos < list_len - 1; pos++)
    {
        if ( (p_geom_args->raw_mag_x >= p_calib_list[pos].raw_mag_x &&
              p_geom_args->raw_mag_x <= p_calib_list[pos + 1].raw_mag_x) ||
             (p_geom_args->raw_mag_x >= p_calib_list[pos + 1].raw_mag_x &&
              p_geom_args->raw_mag_x <= p_calib_list[pos].raw_mag_x) )
        {
            // printf("x find, pos: %d\n", pos);

            if ( (p_geom_args->raw_mag_y >= p_calib_list[pos].raw_mag_y &&
                  p_geom_args->raw_mag_y <= p_calib_list[pos + 1].raw_mag_y) ||
                 (p_geom_args->raw_mag_y >= p_calib_list[pos + 1].raw_mag_y &&
                  p_geom_args->raw_mag_y <= p_calib_list[pos].raw_mag_y) )
            {
                // printf("y find, pos: %d\n", pos);
                // printf("yaw: %f\n", p_calib_list[pos].yaw);
                return pos;
            }
            // else
            // {
            //     static float last_bias = 999.0f;
            //     float cur_bias = abs(p_geom_args->raw_mag_y - p_calib_list[pos].raw_mag_y);
            //     if (cur_bias < last_bias)
            //     {
            //         last_bias = cur_bias;
            //         // printf("last_bias: %f\r\n", last_bias);
            //         min_pos = pos;
            //     }
            // }
        }
    }

    return min_pos;
}


// 线性插值
static float BilinearInterpolation(const mag_calibrate_info_t p1, 
											const mag_calibrate_info_t p2,
											const mag_calibrate_info_t p) {
    float yaw_y, yaw_x;

	if (p2->raw_mag_y == p1->raw_mag_y)
	{
		yaw_y = p1->yaw;
	} else
	{
		yaw_y = p1->yaw + (p->raw_mag_y - p1->raw_mag_y)*(p2->yaw - p1->yaw)/(p2->raw_mag_y - p1->raw_mag_y);
	}
	
	if (p2->raw_mag_x == p1->raw_mag_x)
	{
		yaw_x = p1->yaw;
	} else
	{
		yaw_x = p1->yaw + (p->raw_mag_x - p1->raw_mag_x)*(p2->yaw - p1->yaw)/(p2->raw_mag_x - p1->raw_mag_x);
	}

    return (yaw_x + yaw_y)/2.0f;
}


int get_yaw_by_geom_data(const mag_calibrate_info_t p_calib_list,
                                 const int list_len,
                                 const mag_calibrate_info_t p_geom_args, float *yaw)
{
	int idx = _get_idx_by_geom_data(p_calib_list, list_len, p_geom_args);
	if (idx == -1)
	{
		LOG_E("Error, return -1");
		return -1;
	}

	*yaw = BilinearInterpolation(&p_calib_list[idx], &p_calib_list[idx+1], p_geom_args);
	LOG_D("get_yaw_by_geom_data yaw = %f", *yaw);
	return 0;
}


float cal_gero_euc_dis(mag_calibrate_info_t info1, mag_calibrate_info_t info2)
{
    float dx = info1->raw_mag_x - info2->raw_mag_x;
    float dy = info1->raw_mag_y - info2->raw_mag_y;

    float distance = sqrt(dx*dx + dy*dy);
    return distance;
}


int is_class_id_1_exist(vision_info_t vision)
{
    for (int i = 0; i < vision->box_num; i++) {
        if (vision->box_buff[i].class_id == 1) {
            return 1;
        }
    }
    return 0;
}

// 通过视觉info，对一整行脏污程度打分
float get_dirt_score(vision_info_t info, int num)
{
    float score = 0.0f;
    int dirty_num = 0;
    for (int i = 0; i < num; i++)
    {
        int dirty_bool = is_class_id_1_exist(info+i);
        if ((info[i].class_id == 1) || (info[i].class_id == 2) || dirty_bool)
        {
            dirty_num += 1;
        }
        
    }

    score = (float)dirty_num / num;
    return score;
}


rt_ssize_t coordinate_calc(float base_yaw, const dis_sensor_buffer_info_t dis_info, point_info_t dest_buf)
{
    // LOG_D("base_yaw=%f", base_yaw);
    float angle = 0;
    rt_uint16_t i = 0, j = 0;

    if (dis_info->data_count == 0)
    {
        return -1;
    }

    for(i = 0; i < dis_info->data_count; i++)
    {
        angle = (float)compare_yaws(base_yaw, dis_info->dis_buff[i].yaw);
        // LOG_D("angle=%f", angle);

        // LOG_D("dis_info->dis_buff[%d].distance=%d, dis_info->dis_buff[%d].yaw=%f", i, dis_info->dis_buff[i].distance, i, dis_info->dis_buff[i].yaw);
        angle = angle * (PI / 180.0);  //角度转换为弧度
        // LOG_D("angle=%f", angle);
        if (dis_info->dis_buff[i].distance < FILTER_ULT_THRES)
        {
            dest_buf[j].x = (dis_info->dis_buff[i].distance + MACHINE_LENGTH) * sin(-angle);
            dest_buf[j].y = (dis_info->dis_buff[i].distance + MACHINE_LENGTH) * cos(-angle);
            // LOG_D("dest_buf[%d].x=%f, dest_buf[%d].y = %f", j, dest_buf[j].x, j, dest_buf[j].y);
            j += 1;
        } 
    }
    return j;
}


// 将获取到的距离数据转成以机器中心为原点的点
int calculate_points_by_distance(distance_sensor_out_arg_t dis_info, point_info_t pt_info_buff, int *dis_pt_info_num, int *rotate_point_count)
{
    rt_memset(pt_info_buff, 0, sizeof(pt_info_buff));
	*dis_pt_info_num = 0;
	dis_sensor_info_t points_dis;

#if DIS_SENSOR_MODE  // 1:超声优先 0:激光优先
	points_dis = dis_info->ultrasonic_buff;
	*rotate_point_count = dis_info->ultrasonic_count;	
#else
    points_dis = dis_info->laser_buff;
	*rotate_point_count = dis_info->laser_count;
#endif

    for (size_t i = 0; i < *rotate_point_count; i++)
	{
        struct point_info cur_point_t;
        // 转到以机器中心为原点的坐标系下
        cur_point_t.x = (float)(points_dis[i].distance + 200) * sin(-points_dis[i].yaw * 3.1415926f/180.0f);
        cur_point_t.y = (float)(points_dis[i].distance + 200) * cos(-points_dis[i].yaw * 3.1415926f/180.0f);
        
        // 放到buffer里
        if (i >= 100)
        {
            LOG_W("buff full...");
            return -1;
        }
        
        pt_info_buff[i] = cur_point_t;
		*dis_pt_info_num = i;
	}
    return 0;
}

// 拟合二次曲线。输入x数组，y数组，point的长度n。输出系数，y=a2*x**2+a1*x+a0
static int Least_Squarel_Linear_Fit_y_2a0_4a1x_4a2xx(float *x, float *y, int n, float *a0, float *a1, float *a2)
{
    LOG_D("n=%d", n);
    for (int i = 0; i < n; i++)
    {
        // LOG_D("x[%d]=%f, y[%d]=%f", i, x[i], i, y[i]);
    }
    
    float temp=0;
    float x0=n, x1=0, x2=0, x3=0, x4=0, x0y1=0, x1y1=0, x2y1=0;//表示各项的求和
    float x0x2,x0x3,x0x4,x1x1,x1x2,x1x3,x1x4,x2x2,x2x3,x2x4,x3x3;
    float a_banshui[3][3],a_ni[3][3];//这个是伴随矩阵和逆矩阵
    float a_abs;//矩阵a的行列式
    for(int i=0;i<n;i++)
    {
        x0y1+=*(y+i);
        temp=*(x+i);
        x1+=temp;
        x1y1+=temp*(*(y+i));
        temp*=*(x+i);
        x2+=temp;
        x2y1+=temp*(*(y+i));
        temp*=*(x+i);
        x3+=temp;
        temp*=*(x+i);
        x4+=temp;
    }
    //后面要用到的数据，先算出来
    x0x2=x0*x2;
    x0x3=x0*x3;
    x0x4=x0*x4;
    x1x1=x1*x1;
    x1x2=x1*x2;
    x1x3=x1*x3;
    x1x4=x1*x4;
    x2x2=x2*x2;
    x2x3=x2*x3;
    x2x4=x2*x4;
    x3x3=x3*x3;
    //计算伴随矩阵,行列式，求逆矩阵，其实可以利用对称性再减少运算
    a_banshui[0][0]= (x2x4-x3x3);  a_banshui[0][1]=-(x1x4-x2x3);  a_banshui[0][2]= (x1x3-x2x2);
    a_banshui[1][0]=-(x1x4-x2x3);  a_banshui[1][1]= (x0x4-x2x2);  a_banshui[1][2]=-(x0x3-x1x2);
    a_banshui[2][0]= (x1x3-x2x2);  a_banshui[2][1]=-(x0x3-x1x2);  a_banshui[2][2]= (x0x2-x1x1);
    //计算矩阵对应行列式的值
    a_abs=(x0*a_banshui[0][0]+x1*a_banshui[0][1]+x2*a_banshui[0][2]);  
    //计算逆矩阵
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            a_ni[i][j]=a_banshui[i][j]/a_abs;
        }
    }
    *a0=a_ni[0][0]*x0y1+a_ni[0][1]*x1y1+a_ni[0][2]*x2y1;
    *a1=a_ni[1][0]*x0y1+a_ni[1][1]*x1y1+a_ni[1][2]*x2y1;
    *a2=a_ni[2][0]*x0y1+a_ni[2][1]*x1y1+a_ni[2][2]*x2y1;
    LOG_D("a0=%f, a1=%f, a2=%f", *a0, *a1, *a2);
    return 0;
}

// 根据测到的距离值换算的points，得到曲线方程。
int Least_Squarel_Linear_Fit(float *a0, float *a1, float *a2, const int rotate_point_count, point_info_t pt_info_buff)
{
    int n = rotate_point_count;
    float pt_x[rotate_point_count];
    float pt_y[rotate_point_count];
    for (size_t i = 0; i < n; i++)
    {
        pt_x[i] = pt_info_buff[i].x;
        pt_y[i] = pt_info_buff[i].y;
    }
    
    Least_Squarel_Linear_Fit_y_2a0_4a1x_4a2xx(pt_x, pt_y, n, a0, a1, a2);
    return 0;
}


// 从buffer拟合出曲线方程
int fit_line_by_dis_inf0(float base_yaw, const dis_sensor_buffer_info_t dis_info, float *a0, float *a1, float *a2)
{
    int n = dis_info->data_count;
    LOG_D("get %d point", n);

    if (n < 3)
    {
        LOG_D("get less than 3 point, fit fail");
        return -1;
    } else
    {
        struct point_info point_buffer[dis_info->data_count];
        rt_ssize_t p_count = coordinate_calc(base_yaw, dis_info, point_buffer);
        if (p_count < 0)
        {
            LOG_D("fit fail");
            return -1;
        }
        if (p_count < 3)
        {
            LOG_D("too little point, fit fail");
            return -1;
        }
        
        LOG_D("point_buffer count %d", p_count);
        for (size_t i = 0; i < p_count; i++)
        {
            // LOG_D("%d %.2f %.2f\n", i, point_buffer[i].x, point_buffer[i].y);
        }
        
        float pt_x[p_count];
        float pt_y[p_count];
        for (size_t i = 0; i < p_count; i++)
        {
            pt_x[i] = point_buffer[i].x;
            pt_y[i] = point_buffer[i].y;
        }
        
        Least_Squarel_Linear_Fit_y_2a0_4a1x_4a2xx(pt_x, pt_y, p_count, a0, a1, a2);
    }
    
    return 0;
}


static int is_same_sign(float a, float b) {
    return (a * b >0) ? 1 :0;
}
/*
a:current yaw dis
b:pre yaw dis
*/
static int is_bigger_than_thres(float a, float b) {
    if ((fabs(a) > fabs(b)) && (fabs(b) > YAW_DIS_THRES))
    {
        return 1;
    } else
    {
        return 0;
    }
}

/*
a:current yaw dis
b:pre yaw dis
*/
int verify_same_sign_and_increasing(float a, float b) { 
    int if_same_sign = is_same_sign(a, b);
    int if_bigger_than_thres = is_bigger_than_thres(a, b);
    if (if_same_sign==1 && if_bigger_than_thres==1)
    {
        return 1;
    } else
    {
        return 0;
    }
}

int mag_calibrate_info2mag_adjest_yaw_info(mag_calibrate_info_t calibrate_info, mag_adjest_yaw_info_t adjest_info)
{
    adjest_info->raw_mag_x = calibrate_info->raw_mag_x;
    adjest_info->raw_mag_y = calibrate_info->raw_mag_y;
    adjest_info->raw_mag_z = calibrate_info->raw_mag_z;
    adjest_info->yaw = calibrate_info->yaw;
	return 0;
}


// 计算向量v1和v2的数量积
static float dotProduct(float v1[], float v2[], int n) {
    float sum = 0.0;
    for (int i = 0; i < n; i++) {
        sum += v1[i] * v2[i];
    }
    return sum;
}

// 计算向量v的模长
float vectorMagnitude(float v[], int n) {
    float sum = 0.0;
    for (int i = 0; i < n; i++) {
        sum += v[i] * v[i];
    }
    return sqrt(sum);
}

// 计算两个向量v1和v2之间的夹角（以弧度为单位）
float calculate_cosine(float v1[], float v2[], int n) {
    float dot = dotProduct(v1, v2, n);
    float magnitudeV1 = vectorMagnitude(v1, n);
    float magnitudeV2 = vectorMagnitude(v2, n);
    float cosineOfAngle = dot / (magnitudeV1 * magnitudeV2);
    
    // 确保余弦值在-1到1之间，以避免数学错误
    if (cosineOfAngle > 1.0f) cosineOfAngle = 1.0f;
    if (cosineOfAngle < -1.0f) cosineOfAngle = -1.0f;
    
    // 使用反余弦函数计算夹角
    return cosineOfAngle;
}

int wf_ctr_light(int mode)
{
    float soc_det_light_value = 0.0f;
    if (mode == 0) // 关闭补光灯
    {
        LOG_I("turn off light");
        if(set_soc_light(0)!= RT_EOK)
        {
            LOG_D("set soc light fail");
            rt_thread_mdelay(500);
            if(set_soc_light(0) != RT_EOK)
            LOG_E("turn off soc light fail");
        }
    } else
    {
        get_soc_light_info(&soc_det_light_value);
        LOG_I("soc_det_light_value=%f", soc_det_light_value);
        if (soc_det_light_value < TURN_LIGHT_THRES)
        {
            LOG_I("turn on light");
            if(set_soc_light(LIGHT_VALUE)!= RT_EOK)
            {
                LOG_D("set soc light fail");
                rt_thread_mdelay(500);
                if(set_soc_light(LIGHT_VALUE) != RT_EOK)
                LOG_E("turn on soc light fail");
            }
        }
        
    }
    
    return 0;
}



// 排水口脱困
int robot_escape_drain(void)
{
    int ret;
    suct_esc_t escape_vals={MTR_FWD, 6000, MTR_FWD, 3700, MTR_FWD, 3700, 20*1000};
    LOG_D("escape, wp_speed=%d", escape_vals.e0_speed);
    ret = suctin_escape(escape_vals);
    return ret;
}

int move_to_deep_area(int forward_time)
{
    int cur_time, start_time;
    float cur_slope;
    int flat_count = 0;

    // turn right 90°
    move_rotate_on_floor(-90.0f, MOVE_CONTROL_RIGHT_SAFEZONE);
    rt_thread_mdelay(1000); 
    float start_yaw = get_current_yaw(), cur_yaw;
    LOG_D("yaw=%f", start_yaw);
    
    // 前进到深水区
    start_time = rt_tick_get_millisecond();
    cur_time = start_time;
    while (cur_time - start_time < forward_time)
    {
        cur_slope = get_current_slope();
        cur_yaw = get_current_yaw();
        LOG_D("get deep area, cur_slope=%f", cur_slope);
        if (fabs(cur_slope) < SLOPE_THRES_MIN)
        {
            LOG_D("flat");
            flat_count += 1;
        } else
        {
            flat_count = 0;
        }
        if (flat_count > 4)
        {
            LOG_D("arrive deep area");
            break;
        }
        move_forward_with_pid(WASH_FLOOR_MOTOR_SPEED, cur_yaw, start_yaw);
        rt_thread_mdelay(100);
        LOG_D("cur_yaw=%f", cur_yaw);
        cur_time = rt_tick_get_millisecond();
    }
    move_stop();
    rt_thread_mdelay(1000);

    // 左转90度
    move_rotate_on_floor(90.0f, MOVE_CONTROL_RIGHT_SAFEZONE);
    rt_thread_mdelay(1000); 

    // 前进2s 
    forward_time = 2*1000;
    start_time = rt_tick_get_millisecond();
    cur_time = start_time;
    while (cur_time - start_time < forward_time)
    {
        cur_yaw = get_current_yaw();
        move_forward_with_pid(WASH_FLOOR_MOTOR_SPEED, cur_yaw, start_yaw);
        rt_thread_mdelay(100);
        LOG_D("cur_yaw=%f", cur_yaw);
        cur_time = rt_tick_get_millisecond();
    }
    move_stop();
    rt_thread_mdelay(500);
    LOG_D("before clean slope, yaw=%f", get_current_yaw());
    return 0;
}


// 上坡情况
static int calculate_forward_time_up(float pitch) {
    float m = -150; // 斜率
    float b = 2250; // 截距
    return (int)(m * pitch + b);
}

// 下坡情况
static int calculate_forward_time_down(float pitch) {
    float m = -150; // 斜率
    float b = 3750; // 截距
    return (int)(m * pitch + b);
}


int move_time_to_next_line(void)
{
    int forward_time = 3*1000;
    // slope和时间线性曲线
    float slope = get_current_slope();
    LOG_D("slope=%f", slope);
    int min_time = 1500, max_time = 4500, flat_time = 3000;
    if (fabs(slope) < PITCH_MIN)
    {
        forward_time = flat_time;
    } else if (slope >= PITCH_DOWN_MAX)
    {
        forward_time = min_time;
    } else if (slope <= -PITCH_MAX)
    {
        forward_time = max_time;
    } else if ((slope <= -PITCH_MIN) && (slope > -PITCH_MAX))
    {
        forward_time = calculate_forward_time_up(slope);
    } else if ((slope >= PITCH_MIN) && (slope < PITCH_DOWN_MAX))
    {
        forward_time = calculate_forward_time_down(slope);
    }
    LOG_D("forward_time=%d", forward_time);

    return forward_time;
}

int update_short_line_info(int cur_line_forawrd_time, int* short_distance_lines)
{
    if (cur_line_forawrd_time < SHORT_LINE_TIME_THRES)
    {
        *short_distance_lines += 1;
        LOG_D("add short line");
    } else
    {
        *short_distance_lines = 0;
        LOG_D("reset short line");
    }
    LOG_D("short_distance_lines=%d", *short_distance_lines);
    return 0;
}

int dislodged_from_drain(int if_odd_line, float target_yaw)
{
    int ret = 0;
    // 是否陡坡
    float slope = get_current_slope();
    LOG_D("slope=%f", slope);
    rt_uint8_t label;
    rt_uint16_t dis;
    rt_tick_t sensor_tick, cur_tick;
    float yaw_diff, cur_yaw, after_yaw;
    int odd_bool;          // 偶数行-1， 奇数行1
    odd_bool = (if_odd_line==1) ? 1 : -1;
    LOG_D("odd_bool=%d", odd_bool);
    int if_same_direction;    // 偏移方向和清洗方向一致为正，相反为负

    if (fabs(slope) > 2*SLOPE_THRES_MIN)
    {
        // 有坡，不做处理
        LOG_D("yaw change from slope");
        return 0;
    }
    // 是否有距离
    cur_tick = rt_tick_get_millisecond();
    get_distance_fusion(&sensor_tick, &label, &dis);
    LOG_D("src dis=%d, cur_time=%d, sensor_time=%d", dis, cur_tick, sensor_tick);
    if (label == 1 && (abs(cur_tick - sensor_tick) < 4000)) // 数据可信
    {
        if (dis < ONE_LINE_LENGTH)
        {
            // 到墙边，不做处理
            LOG_D("yaw change from wall");
            return 0;
        } 
    }

    // 其他情况，认为有地漏
    cur_yaw = get_current_yaw();
    yaw_diff = compare_yaws(target_yaw, cur_yaw);     // 机器左偏为正，右偏为负
    if_same_direction = odd_bool*yaw_diff;
    LOG_D("cur_yaw=%f, yaw_diff=%f, if_same_direction=%d", cur_yaw, yaw_diff, if_same_direction);
    if (if_same_direction > 0)     // 偏移方向和清洗方向一致
    {
        ret = 1;
        LOG_D("same direction");
    } else if (if_same_direction < 0) // 偏移方向和清洗方向相反
    {
        ret = 2;
        LOG_D("diff direction");
    }
    
    // 纠正机器姿态
    move_rotate_on_floor_using_target(target_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
    rt_thread_mdelay(500);
    
    // 脱困
    after_yaw = get_current_yaw();
    LOG_D("after rotate, yaw=%f", after_yaw);
    yaw_diff = compare_yaws(after_yaw, cur_yaw);
    LOG_D("rotate %f", yaw_diff);

    if (fabs(yaw_diff) < 3.0f) // 没转动
    {
        // 认为卡住地漏, 脱困
        LOG_D("escape drain");
        robot_escape_drain();
        LOG_D("escape drain end");
        rt_thread_mdelay(500);
        
        // 开水泵
        move_wp_speed_on_floor_set(PUMP_SPEED_HIGH);
        // rt_thread_mdelay(3*1000);
        rt_thread_mdelay(1*1000);

        // 纠正角度
        LOG_D("target_yaw=%f", target_yaw);
        move_rotate_on_floor_using_target(target_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
        rt_thread_mdelay(500);
    }

    // 纠正姿态  
    if (fabs(compare_yaws(after_yaw, target_yaw)) > 3.0f)
    {
        move_rotate_on_floor_using_target(target_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
        rt_thread_mdelay(500);
    }

    LOG_I("judge drain, %d", ret);
    return ret;
}



int get_model_info(model_info_t cur_model_info, rt_uint8_t *weekly_max_clean_time)
{
    cur_model_info->model_mode = eeprom_model_get();
    *weekly_max_clean_time = get_weekly_mode_run_num();

    LOG_D("model mode=%d, week time=%d", cur_model_info->model_mode, *weekly_max_clean_time);
    return 0;
}

int wash_floor_off_cur_time()
{
    laser_switch_off();
    LOG_D("off laser");
    rt_thread_mdelay(500);

    move_stop();
    LOG_D("motor stop");
    rt_thread_mdelay(500);

    return 0;
}

#if 0
extern int vision_clean_time;
int restrict_value_range(p_app_clean_paras_t conf_info)
{
    LOG_D("before restrict, mtr %d, pump %d, num %d, inter %f", conf_info->fl_wa_mtr_speed, 
            conf_info->fl_wa_pump_speed, conf_info->fl_wa_num, conf_info->fl_wa_interval);
    vision_clean_time = 0;  // 关闭根据鸡爪数量增加水泵转速
    conf_info->fl_wa_pump_speed = init_pump_speed + (vision_clean_time/10)*50;
    LOG_D("vision_clean_time=%d, init_pump_speed=%d, fl_wa_pump_speed=%d", 
        vision_clean_time, init_pump_speed, conf_info->fl_wa_pump_speed);

    if (conf_info->fl_wa_interval < MIN_LINE_INTERL)
    {
        conf_info->fl_wa_interval = MIN_LINE_INTERL;
    } else if (conf_info->fl_wa_interval > MAX_LINE_INTERL)
    {
        conf_info->fl_wa_interval = MAX_LINE_INTERL;
    }
    
    if (conf_info->fl_wa_pump_speed < PUMP_SPEED_LOW)
    {
        conf_info->fl_wa_pump_speed = PUMP_SPEED_LOW;
    } else if (conf_info->fl_wa_pump_speed > PUMP_SPEED_HIGH)
    {
        conf_info->fl_wa_pump_speed = PUMP_SPEED_HIGH;
    }

    if (conf_info->fl_wa_mtr_speed < WASH_FLOOR_MOTOR_MIN_SPEED)
    {
        conf_info->fl_wa_mtr_speed = WASH_FLOOR_MOTOR_MIN_SPEED;
    } else if (conf_info->fl_wa_mtr_speed > WASH_FLOOR_MOTOR_MAX_SPEED)
    {
        conf_info->fl_wa_mtr_speed = WASH_FLOOR_MOTOR_MAX_SPEED;
    }

    LOG_I("after restrict, mtr %d, pump %d, num %d, inter %f", conf_info->fl_wa_mtr_speed, 
            conf_info->fl_wa_pump_speed, conf_info->fl_wa_num, conf_info->fl_wa_interval);
    return 0;
}
#endif


#if (VERSION_TYPE==E_AUSTRALIA)
// 计算最大清洗行数
static int get_max_clean_rows(float adjacent_line_interval)
{
    const int MAX_LINE = 250;  // 按默认最大100米清洗区域
    int clean_rows = MAX_LINE;
    const int line_intervel = 40*adjacent_line_interval;   // 行间距40cm
    pool_area_params_t area_info_t = get_pool_area_params();
    if (area_info_t!= NULL)
    {
        LOG_I("get area info. x %d, y %d, x1 %d, y1 %d, x2 %d, y2 %d", 
            area_info_t->x, area_info_t->y, area_info_t->x1, area_info_t->y1,
            area_info_t->x2, area_info_t->y2);
        clean_rows = (area_info_t->x2 - area_info_t->x1)/line_intervel + 3;

        if (area_info_t->x2 < area_info_t->x1)
        {
            LOG_W("params error.x2 must bigger than x1.");
        }
        clean_rows = (clean_rows > 0) ? clean_rows : MAX_LINE;
    }else
    {
        LOG_W("get no clean area info.");
    }
    LOG_D("clean_row=%d", clean_rows);

    return clean_rows;
}
#endif

#if (VERSION_TYPE==E_NORMAL)
// 普通版本计算最大清洗行数
int normal_version_get_max_clean_rows(float adjacent_line_interval)
{
    const int MAX_CLEAN_LIEGTH = 4000;  // 最大清洗长度设为40m
    float line_intervel = adjacent_line_interval*40;  // 算上设置的行间隔，两行之间的距离cm
    int clean_rows = MAX_CLEAN_LIEGTH/line_intervel;
    LOG_D("normal version, clean_rows=%d", clean_rows);
    return clean_rows;
}
#endif

int get_wf_paras(enum work_mode_type work_mode, p_app_clean_paras_t conf_info, wf_clean_info_t info)
{
    // extern p_app_clean_paras_t get_app_clean_eep_paras(void);

    // p_app_clean_paras_t p_src_conf_info;
    // p_src_conf_info = get_app_clean_eep_paras();
    
    // 获取最大清洗次数、最小清洗次数
    struct f_e_clean_count times_info;
    get_f_e_clean_count(&times_info);
    LOG_D("max clean time=%d, min clean time=%d", times_info.max, times_info.def);

    if (work_mode == WEEKLY_MODE)
    {
        conf_info->fl_wa_num = 1;
    } else if (work_mode == ALL_MODE)
	{
		extern p_app_clean_paras_t get_all_clean_eep_paras(void);
		conf_info = get_all_clean_eep_paras();
	} else
    {
        conf_info->fl_wa_num = times_info.max;
    }
	

    // restrict_value_range(conf_info);

    // 获取清洗参数
#if (VERSION_TYPE==E_EXHIBITION)
    info->max_clean_rows = 5;
    info->wash_times = 9999;  // 循坏清洗
    info->once_clean_max_clean_time = ONCE_CLEAN_TIME_MAX; 
#elif (VERSION_TYPE==E_EXHIBITION_P10)
    info->max_clean_rows = 1;
    info->wash_times = 9999;  // 循坏清洗
    info->once_clean_max_clean_time = ONCE_CLEAN_TIME_MAX; 
#elif (VERSION_TYPE==E_AUSTRALIA)
    info->max_clean_rows = get_max_clean_rows(info->line_interval);
    info->wash_times = 1;
    info->once_clean_max_clean_time = 600*60*1000;  // 澳洲商用版本放开最大清洗时间,6h
#elif (VERSION_TYPE==E_NORMAL)
    // info->max_clean_rows = normal_version_get_max_clean_rows(info->line_interval);   // 根据最大清洗距离40m，设置最大清洗行数
    info->max_clean_rows = 9999;  // normal版本不设置行数限制
    info->wash_times = CLEAN_FLOOR_TIMES*conf_info->fl_wa_num;
    info->min_wash_times = CLEAN_FLOOR_TIMES*times_info.def;	//因为根据脏污进行次数设置时最小次数就是2次，所以不用再对all mode限制
    info->once_clean_max_clean_time = ONCE_CLEAN_TIME_MAX; 
#endif

    rt_uint8_t waterline_label = get_climb_up_to_waiting_waterline_sta();

#if (VERSION_TYPE==E_AUSTRALIA)
    waterline_label = 1;
#endif
    LOG_I("VERSION_TYPE=%d, waterline_label=%d", VERSION_TYPE, waterline_label);
    
    if ((work_mode == ALL_MODE || work_mode == SMART_MODE) && (waterline_label == 1))
    {
        info->off_value = STOP_WATERLINE_THRES;
    } else
    {
        info->off_value = OFF_VOL_THRES;
    }
    LOG_D("total wash time=%d, min_wash_time=%d, max_rows=%d, off_value=%d, max_clean_time=%d", 
            info->wash_times, info->min_wash_times, info->max_clean_rows, info->off_value, info->once_clean_max_clean_time);
    return 0;
}


#if (VERSION_TYPE==E_NORMAL)
int update_clean_info_data(wf_params_t test_param, wf_clean_info_t info)
{
    info->line_interval = test_param->adjacent_line_interval;
    // info->max_clean_rows = normal_version_get_max_clean_rows(info->line_interval);
    info->max_clean_rows = 9999;  // 关闭normal模式的行数限制
    LOG_D("update cleaninfo, line_interval=%f, max_clean_rows=%d", info->line_interval, info->max_clean_rows);
    return 0;
}
#endif

static int _goto_shallow_action(goto_shallow_info_t info)
{
    LOG_I("goto shallow");
    if (info->turn_mode == 0)
    {
        LOG_D("at shallow area.");
        return 0;
    } else {
        const int forward_speed = 5000, wp_speed = 2800; //rpm
        move_wp_speed_on_floor_set(wp_speed);
        rt_thread_mdelay(500);

        move_forward_with_pid_and_time(forward_speed, get_current_yaw(), info->half_line_time);
        rt_thread_mdelay(500);
        if (info->turn_mode == 1)
        {
            // 左转
            move_rotate_on_floor(90.0f, MOVE_CONTROL_RIGHT_SAFEZONE);
            LOG_D("cur_yaw=%f", get_current_yaw());
        } else if (info->turn_mode == 2)
        {
            // 右转
            move_rotate_on_floor(-90.0f, MOVE_CONTROL_RIGHT_SAFEZONE);
            LOG_D("cur_yaw=%f", get_current_yaw());
        }
        rt_thread_mdelay(500);

        // go ahead until get shallow edge
        int count = 2;
        const int forward_timeout = 180*1000;
        rt_tick_t start_time = rt_tick_get_millisecond();
        float cur_slope;
        int if_rotated = 0;
        while (count > 0)
        {
            cur_slope = get_current_slope();
            if (cur_slope < -SLOPE_THRES_MAX)
            {
                LOG_D("get area. stop");
                move_stop_time(1000);
                break;
            }

            move_forward_with_speed(forward_speed);    
            rt_thread_mdelay(1000);

            if (rt_tick_get_millisecond() - start_time >  forward_timeout)
            {
                LOG_D("time out. count=%d", count);
                count -= 1;
                move_stop_time(500);
                if(if_rotated == 0)
                {
                    move_rotate_on_floor(-90.0f, MOVE_CONTROL_RIGHT_SAFEZONE);
                    if_rotated = 1;
                    rt_thread_mdelay(500);
                }
                start_time = rt_tick_get_millisecond();
            }
        }
        return 0;
    }

}

int power_off_model(enum work_mode_type work_mode, int delay_time, goto_shallow_info_t info)
{
    rt_uint8_t if_goto_waterline = 0;
    _goto_shallow_action(info);
    rt_thread_mdelay(500);
    LOG_D("goto shallow end.");

    if ((work_mode == SMART_MODE) || (work_mode == ALL_MODE))
    {
        if_goto_waterline = get_climb_up_to_waiting_waterline_sta();
    }
    if (if_goto_waterline == 1)
    {
        LOG_D("yaw=%f, start goto waterline", get_current_yaw());
        stop_at_waterline();
    } else
    {
        power_off_ac(delay_time);
    }
    
    return 0;
}


int init_wash_record_info(wash_record_info_t record_info, wash_floor_rows_info* wash_info, int slope_label)
{
    record_info->start_time = rt_tick_get_millisecond();
    record_info->wash_time = wash_info->wash_times;
    record_info->wash_num = eeprom_total_clean_counts_get();

    if (slope_label==1)
    {
        if (wash_info->wash_times==0 || wash_info->wash_times%2==1)
        {
            record_info->region_flag = REGION_FLOOR_DEEP;
        }
    } else
    {
        record_info->region_flag = REGION_FLOOR_SHALLOW;
    }
    get_battery_soc(&record_info->start_soc);
    LOG_D("init record info at %d, flag=%d", record_info->start_time, REGION_FLOOR_SHALLOW);

    return 0;
}

int austra_goto_start_point_act(float* wash_start_yaw)
{
    pool_area_params_t area_info_t = NULL;
    area_info_t = get_pool_area_params();
    LOG_I("goto start point. x1 %d, y1 %d, x2 %d, y2 %d", 
            area_info_t->x1, area_info_t->y1,
            area_info_t->x2, area_info_t->y2);
    
    // 转180度尾冲墙, 后退校正yaw角
    int slope_ = 0;
    actions_on_floor_backward_search_wall(RT_NULL, &slope_);
    float yaw = get_current_yaw();
    LOG_D("yaw=%f", yaw);

    // 前进到指定位置
    int forward_time = 1000*(area_info_t->x1 * 10)/MOTOR_SPEED_MOVE_SPEED;
    LOG_D("goto starting point.forward_time=%d ms", forward_time);
    move_forward_with_pid_and_time(WASH_FLOOR_MOTOR_SPEED, yaw, forward_time);
    rt_thread_mdelay(1*1000);

    // 左转90度
    move_rotate_on_floor(90.0f, MOVE_CONTROL_RIGHT_SAFEZONE);
    rt_thread_mdelay(1*1000);

    // 返回当前时刻的yaw作为start_yaw
    yaw = get_current_yaw();
    LOG_D("wash start yaw=%f", yaw);

    *wash_start_yaw = yaw;
    return 0;
}
void print_params(const wf_params_t params)
{
    if (params != NULL) {
        LOG_D("forward_wp_speed: %d", params->forward_wp_speed);
        LOG_D("forward_mtr_speed: %d", params->forward_mtr_speed);
        LOG_D("clean_leaves_wp_speed: %d", params->clean_leaves_wp_speed);
        LOG_D("clean_leaves_mtr_speed: %d", params->clean_leaves_mtr_speed);
        LOG_D("forward_clean_sediment_wp_speed: %d", params->forward_clean_sediment_wp_speed);
        LOG_D("forward_clean_sediment_mtr_speed: %d", params->forward_clean_sediment_mtr_speed);
        LOG_D("edge_clean_sediment_wp_speed: %d", params->edge_clean_sediment_wp_speed);
        LOG_D("edge_clean_sediment_mtr_speed: %d", params->edge_clean_sediment_mtr_speed);

        LOG_D("clean_steep_incline_wp_speed: %d", params->clean_steep_incline_wp_speed);
        LOG_D("clean_steep_incline_mtr_speed: %d", params->clean_steep_incline_mtr_speed);
        LOG_D("clean_steep_incline_edge_wp_speed: %d", params->clean_steep_incline_edge_wp_speed);
        LOG_D("clean_steep_incline_edge_mtr_speed: %d", params->clean_steep_incline_edge_mtr_speed);
        LOG_D("pentagonal_path_find_slope_wp_speed: %d", params->pentagonal_path_find_slope_wp_speed);
        LOG_D("pentagonal_path_find_slope_mtr_speed: %d", params->pentagonal_path_find_slope_mtr_speed);
        LOG_D("if_clean_steep_incline: %d", params->if_clean_steep_incline);
        LOG_D("adjacent_line_interval: %f", params->adjacent_line_interval);
    } else {
        LOG_W("No parameters found.");
    }
    return;
}

static int cp_and_rebuild_params(const wf_params_t info, wf_params_t rebuild_info)
{
    if (info!=RT_NULL)
    {
        LOG_D("rebuild info.");
        rebuild_info->forward_wp_speed = (info->forward_wp_speed < PUMP_SPEED_LOW) ? PUMP_SPEED_LOW : ((info->forward_wp_speed > PUMP_SPEED_HIGH) ? PUMP_SPEED_HIGH : info->forward_wp_speed);
        rebuild_info->forward_mtr_speed = (info->forward_mtr_speed < WASH_FLOOR_MOTOR_MIN_SPEED) ? WASH_FLOOR_MOTOR_MIN_SPEED : ((info->forward_mtr_speed > WASH_FLOOR_MOTOR_MAX_SPEED) ? WASH_FLOOR_MOTOR_MAX_SPEED : info->forward_mtr_speed);
        rebuild_info->clean_leaves_wp_speed = (info->clean_leaves_wp_speed < PUMP_SPEED_LOW) ? PUMP_SPEED_LOW : ((info->clean_leaves_wp_speed > PUMP_SPEED_HIGH) ? PUMP_SPEED_HIGH : info->clean_leaves_wp_speed);
        rebuild_info->clean_leaves_mtr_speed = (info->clean_leaves_mtr_speed < WASH_FLOOR_MOTOR_MIN_SPEED) ? WASH_FLOOR_MOTOR_MIN_SPEED : ((info->clean_leaves_mtr_speed > WASH_FLOOR_MOTOR_MAX_SPEED) ? WASH_FLOOR_MOTOR_MAX_SPEED : info->clean_leaves_mtr_speed);
        rebuild_info->forward_clean_sediment_wp_speed = (info->forward_clean_sediment_wp_speed < PUMP_SPEED_LOW) ? PUMP_SPEED_LOW : ((info->forward_clean_sediment_wp_speed > PUMP_SPEED_HIGH) ? PUMP_SPEED_HIGH : info->forward_clean_sediment_wp_speed);
        rebuild_info->forward_clean_sediment_mtr_speed = (info->forward_clean_sediment_mtr_speed < WASH_FLOOR_MOTOR_MIN_SPEED) ? WASH_FLOOR_MOTOR_MIN_SPEED : ((info->forward_clean_sediment_mtr_speed > WASH_FLOOR_MOTOR_MAX_SPEED) ? WASH_FLOOR_MOTOR_MAX_SPEED : info->forward_clean_sediment_mtr_speed);
        rebuild_info->edge_clean_sediment_wp_speed = (info->edge_clean_sediment_wp_speed < PUMP_SPEED_LOW) ? PUMP_SPEED_LOW : ((info->edge_clean_sediment_wp_speed > PUMP_SPEED_HIGH) ? PUMP_SPEED_HIGH : info->edge_clean_sediment_wp_speed);
        rebuild_info->edge_clean_sediment_mtr_speed = (info->edge_clean_sediment_mtr_speed < WASH_FLOOR_MOTOR_MIN_SPEED) ? WASH_FLOOR_MOTOR_MIN_SPEED : ((info->edge_clean_sediment_mtr_speed > WASH_FLOOR_MOTOR_MAX_SPEED) ? WASH_FLOOR_MOTOR_MAX_SPEED : info->edge_clean_sediment_mtr_speed);
        rebuild_info->clean_steep_incline_wp_speed = info->clean_steep_incline_wp_speed;
        rebuild_info->clean_steep_incline_mtr_speed = info->clean_steep_incline_mtr_speed;
        rebuild_info->clean_steep_incline_edge_wp_speed = info->clean_steep_incline_edge_wp_speed;
        rebuild_info->clean_steep_incline_edge_mtr_speed = info->clean_steep_incline_edge_mtr_speed;
        rebuild_info->pentagonal_path_find_slope_wp_speed = info->pentagonal_path_find_slope_wp_speed;
        rebuild_info->pentagonal_path_find_slope_mtr_speed = info->pentagonal_path_find_slope_mtr_speed;
        rebuild_info->if_clean_steep_incline = info->if_clean_steep_incline;
        rebuild_info->adjacent_line_interval = (info->adjacent_line_interval < MIN_LINE_INTERL) ? MIN_LINE_INTERL : ((info->adjacent_line_interval > MAX_LINE_INTERL) ? MAX_LINE_INTERL : info->adjacent_line_interval);
    } else
    {
        LOG_W("no param data.");
        return -1;
    }
    
    return 0;
}

// 函数用于根据模式和圈数获取参数
void update_params_by_mode_and_circle(enum work_mode_type work_mode, int circle) 
{
    LOG_D("work mode=%d, circle=%d", work_mode, circle);
    uint8_t circle_in_com = (uint8_t)circle;

    if (work_mode == WEEKLY_MODE)
    {
        circle_in_com = 0;
    } else if (work_mode == ALL_MODE)
    {
        circle_in_com = (circle_in_com > 2) ? 2 : circle_in_com;
    } else if (work_mode == SMART_MODE)
    {
        circle_in_com = (circle_in_com > 5) ? 5 : circle_in_com;
    }
    
    LOG_D("circle_in_com=%d", circle_in_com);
    const wf_params_t wf_param_struct = get_cleaning_params(work_mode, circle_in_com);
    static struct wf_params rebuild_info;
    cp_and_rebuild_params(wf_param_struct, &rebuild_info);
    wf_param_info = &rebuild_info;
    
    LOG_D("update params ok.");
    // print_params(wf_param_info);
    return;
}

wf_params_t get_wf_params_info(void)
{
    return wf_param_info;
}


float reset_imu(int delay_ms, int wp_speed)
{
    LOG_D("reboot submcu");
    int pre_wp_pump = move_wp_speed_on_floor_get();
    move_wp_speed_on_floor_set(wp_speed);
    rt_thread_mdelay(5*1000);

    reset_submcu(1*1000);
    led_show_robot_action(ROBOT_ACTION_CAL_POOL_AREA);
    submcu_led_show_cal_pool_area(delay_ms);
    LOG_D("reboot submcu ok");

	//为了恢复灯效时同时恢复，都在此统一操作
    system_info_t info = system_info_get();
	rt_uint16_t bat_soc;
	get_battery_soc(&bat_soc);
    submcu_led_show_work_mode_by_soc(info, bat_soc);
    led_show_robot_action(ROBOT_ACTION_STOP);

    float yaw = get_current_yaw();
    LOG_D("after reset imu, yaw=%f", yaw);
    int count = 0;

    // 重启的yaw应该在0附近，如果yaw和0差太远，则再读
    while (fabs(yaw) > 10.0f)
    {
        LOG_W("cur count read yaw fail");
        count += 1;
        if (count > 10)
        {
            LOG_E("read yaw fail");
            break;
        }
        rt_thread_mdelay(100);
        yaw = get_current_yaw();
        LOG_D("reboot, yaw=%f", yaw);
    }
    
    move_wp_speed_on_floor_set(pre_wp_pump);
    rt_thread_mdelay(1000);
    return yaw;
}


float calculate_trapezoid_area(float a, float c, float h, float h1)
{
    // 计算长边 b 的长度
    float area = a * h + 0.5f * (h*c - a*h)*h/h1;
    LOG_D("trapezoid_area=%f", area);
    return area;
}

float calculate_pool_area(float a, float c, float h)
{
    float area = 0.5f * (a + c) * h * 3.0f;
    return area;
}



static int _avoid_drain_act(int if_odd_line, leaves_info_t info, int speed)
{
    float delta = 90.0f;
    int forward_time;
    if (if_odd_line == 0) // 偶数行
    {
        delta *= -1.0f;
        if (info->left_have_leaves == 1)
        {
            forward_time = 1500;
        } else if (info->right_have_leaves == 1)
        {
            forward_time = 1500*2;
        }
    } else if (if_odd_line == 1) // 奇数行
    {
        if (info->left_have_leaves == 1)
        {
            forward_time = 1500*2;
        } else if (info->right_have_leaves == 1)
        {
            forward_time = 1500;
        }
    }
    LOG_D("get drain left=%d, right=%d", info->left_have_leaves, info->right_have_leaves);
    LOG_D("avoid drain forward time %d", forward_time);
    move_rotate_on_floor(delta, MOVE_CONTROL_RIGHT_SAFEZONE);
    move_stop_time(500);

    move_forward_with_speed_and_time(speed, forward_time);
    move_stop_time(500);

    delta *= -1.0f;
    move_rotate_on_floor(delta, MOVE_CONTROL_RIGHT_SAFEZONE);
    move_stop_time(500);
    
    return 0;
}


int move_to_next_line_avoid_drain_(int if_odd_line, int speed)
{
    float start_yaw = get_current_yaw();
    // detect drain
    struct leaves_info info = {.object = UNKNOWN, .left_have_leaves = 0, .middle_have_leaves = 0, .right_have_leaves = 0};
    rt_err_t ret = left_middle_right_leaves_info(&info);
    if (ret != RT_EOK)
    {
        LOG_W("object detect error.");
        return -2;
    }

    // 根据奇偶行，判断如何绕地漏
    if(info.object == DRAIN)
    {
        if (info.left_have_leaves != 1 && info.right_have_leaves != 1)
        {
            LOG_D("drain at safe area.");
            return 0;
        }
        ret = _avoid_drain_act(if_odd_line, &info, speed);

        // 纠一下yaw角
        float cur_yaw = get_current_yaw();
        if (fabs(cur_yaw - start_yaw) > 3.0f)
        {
            move_rotate_on_floor_using_target(start_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
            move_stop_time(500);
        }
    }
    return 0;
}
