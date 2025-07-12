/******************************************************************************************
 *
 * Robot Control Thread的动作执行代码 - 机器人在墙壁上的动作
 *
 ******************************************************************************************/
#include <math.h>
#include <stdlib.h>
#include "rtthread.h"
#include "app_config.h"
#include "led_show.h"
#include "move_basic.h"
#include "sensors_info.h"
#include "actions.h"
#include "angle.h"
#include "move_basic.h"
#include "actions_on_floor.h"
#include "system_info.h"
#include "actions_on_wall.h"
#include "mtr_ctrl.h"
#include "distance_drive.h"
#include "util.h"
#include "angle.h"

#define DBG_TAG "wall"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define STAIRS_FLAG (100) // 台阶标志

struct wall_params
{
    float wall_angle_thres;
    float over_pitch_thres;
    float lean_angle_thres;
    float touch_down_pitch_thres;
    int rotate_timeout;
    int wp_speed;
};
typedef struct wall_params *wall_params_t;
/******************************************************************************************
 * 外部变量引用区域
 */
extern struct wash_record_info g_wash_wall_record_info;

/******************************************************************************************
 * 全局变量定义区域
 */
int g_water_depth_time = 0;   // 当前水深时间，默认是 0，单位ms
int g_climbup_stair_flag = 0; // 0：上爬时遇到台阶；1：
short g_acc_x;

/******************************************************************************************
 * 本地常量与变量定义区域
 */

static const float SG_WALL_ANGLE = -75.0f;
static const float SG_OVER_PITCH = -100.0f;
static const float SG_LEAN_ANGLE = 50.0f;
static const float SG_TOUCH_DOWN_PITCH = -80.0f;
static const float SG_CLIMBDOWN_PITCH_THRES = -20.0f;

static const int SG_ROTATE_TIMEOUT = 20 * 1000;
#ifdef POOL_MATERIAL_STEEL                  // 不锈钢材质泳池
static const int SG_CLIMB_UP_WP_SPEED_1 = 4500;
static const int SG_ON_WALL_WP_SPEED = 4000;
#else
static const int SG_CLIMB_UP_WP_SPEED_1 = 3300;
static const int SG_ON_WALL_WP_SPEED = 3000;
#endif
static const int SG_CLIMB_UP_WP_SPEED_2 = 800;
static const int SG_MOVE_DOWN_WP_SPEED = 4500;
// static const int SG_MOVE_DOWN_WP_SPEED_SMALL_BATTERY = 3500;
static const int SG_ON_FLOOR_WP_SPEED = 3300;

static const int SG_MOVE_UP_SPEED = 4500;
static const int SG_MOVE_DOWN_SPEED_1 = 2500;
static const int SG_MOVE_DOWN_SPEED_2 = 3800;

static const int SG_UP_DOWN_DIS = 700; // 上下一次时,横移动距离 mm
static const int SG_UP_DOWN_MAX = 5;   // 上下爬时，每次爬的最大次数

// static const struct wall_params SG_WALL_PARAMS_DEFAULT = {
//     .wall_angle_thres = SG_WALL_ANGLE,
//     .over_pitch_thres = SG_OVER_PITCH,
//     .lean_angle_thres = SG_LEAN_ANGLE,
//     .touch_down_pitch_thres = SG_TOUCH_DOWN_PITCH,
//     .rotate_timeout = SG_ROTATE_TIMEOUT,
//     .wp_speed = SG_WP_SPEED,
// };

static int sg_forward_time;  // 记录当前前进时间
static float sg_climbup_yaw; // 上爬时，记录当前yaw
static int sg_up_down_count; // 洗墙时的次数
static int sg_row_index;     // 记录行数
static int sg_water_depth;     // 记水深度 mm
static rt_uint32_t sg_out_water_detect_count = 0;
/******************************************************************************************
 * 本地函数定义区域
 */

// static wall_params_t _get_wall_params(void)
//{
//     return (wall_params_t)&SG_WALL_PARAMS_DEFAULT;
// }

/*
 * 用于计算不同水深时斜线上行和后退时的斜角（对应该函数返回值的一半）:
 *    速度 = 墙上1秒倒退的距离（平均速度 15.6 cm 每秒）
 *           墙上1秒前进的距离（平均速度 13 cm 每秒）
 *           转弯速度：0.022度每ms，23度每秒
 *    水深 = 速度 * 水深时间（单位ms）
 *    斜角 = 当前水深为高，两倍刷轮宽度（29 x 2 = 58cm）为底，构成的直角三角形的顶角。
 *
 *    斜角理论值：
 *    水深200cm，顶角16度，斜边 208cm
 *    水深150cm，顶角21度，斜边 160cm
 *    水深100cm，顶角30度，斜边 115cm
 *    水深50cm， 顶角49度，斜边  76cm -- 斜角过大
 */
// static float get_backoff_on_wall_angle(int water_depth_time)
//{
//     float water_depth;
//     float angle = 16.0f;

//    // 默认采用100cm水深的斜角
//    if (water_depth_time <= 0)
//        return 30.0f;

//    // 根据水深，计算斜角
//    water_depth = 15.6 * water_depth_time; // 单位是 cm * ms
//    if (water_depth >= 200.0f * 1000)
//        angle = 16.0f;
//    else if (water_depth >= 150.0f * 1000)
//        angle = 22.0f;
//    else if (water_depth >= 125.0f * 1000)
//        angle = 26.0f;
//    else if (water_depth >= 100.0f * 1000)
//        angle = 30.0f;
//    else if (water_depth >= 70.0f * 1000)
//        angle = 35.0f;
//    else
//        angle = 36.0f; // 超过36度则过大

//    return angle;
//}

/******************************************************************************************
 * 设置和获取当前水深时间
 */

int get_water_depth_time(void)
{
    return g_water_depth_time;
}

void set_water_depth_time(int water_depth_time)
{
    g_water_depth_time = water_depth_time;
}

void reset_water_depth_time(void)
{
    g_water_depth_time = 0;
}

// static int _is_pitch_stable(rt_uint32_t during_time)
//{
//     int ret = 0;
//     float pitch, current_pitch = 0.0F;

//    for (int i = 0; i < during_time; i++)
//    {
//        move_stop_time(1000);
//        pitch = get_current_pitch();
//        if (fabs(pitch - current_pitch) <= POSE_STABLE_THRESHOLD)
//        {
//            ret = 1;
//            break;
//        }
//        current_pitch = pitch;
//        LOG_D("%.2f", current_pitch);
//    }

//    return ret;
//}

static int _is_value_stable(int value, int *count, int max_count, const int thres)
{
    if (value < thres)
    {
        *count = *count + 1;
    }
    else
    {
        *count = 0; // 重置count
    }
    LOG_D("[%d %d]: %d %d", *count, max_count, value, thres);
    return (*count >= max_count ? 1 : 0);
}

static int _is_out_of_water(void)
{
    const rt_uint32_t DIVIDER = 0xFFFF;
    int ret = 0;
	static enum medium_type pre_type = IN_WATER;
	static mtr_in_water_sta_t pre_mtr_water_state = IN_WATER_STA;
	
    struct medium_msg msg;
    is_in_water(&msg);
    if (msg.type == IN_AIR)
    {
		if(IN_AIR != pre_type)
		{
			LOG_I("in air by ultrasonic");
		}
        ret = 1;
    }
    else if (get_out_water_by_mtr() == OUT_WATER_STA) // 检测到水外
    {    
		if(OUT_WATER_STA != pre_mtr_water_state)
		{
			LOG_I("in air by pump");
		}
        ret = 1;
    }
	
	pre_type = msg.type;
	pre_mtr_water_state = get_out_water_by_mtr();	//假设状态不会变那么快

    return ret;
}

/**
 * @brief 前进找墙
 *
 * @param timeout
 * @return int 0:成功，1:失败
 */
static int _move_forward_to_wall(int timeout, int dis_thres)
{
    int move_speed = 8000;
    int ret = 1;

    // 按时间进行前进的找墙
    move_forward_with_speed(move_speed);
    rt_tick_t start_time = rt_tick_get();
    while (1)
    {
        // 获取距离
        int dis_tof = get_current_distance_from_laser();
        // LOG_D("tof %d", dis_tof);
        int tof_flag = (dis_tof > 50) && (dis_tof < dis_thres);
        int dis_ult = get_current_distance_from_ultrasonic();
        RT_UNUSED(tof_flag);
        RT_UNUSED(dis_ult);
        // LOG_D("ult %d", dis_ult);
        int ult_flag = (dis_ult > 0) && (dis_ult < dis_thres);
        RT_UNUSED(ult_flag);
        // if (tof_flag || ult_flag)
        // {
        //     ret = 0;
        //     break;
        // }

        float pitch = get_current_pitch();
        if (pitch < -10.0f) // 向上抬头
        {
            ret = 0;
            LOG_I("over pitch %.2f", pitch);
            break;
        }

        if (rt_tick_get() - start_time > timeout)
        {
            ret = 1;
            LOG_I("timeout");
            break;
        }
    }

    move_stop();
    return ret;
}

static int _is_on_wall(void)
{
    int ret = 1;
    float pitch = get_current_pitch();
    if (pitch > SG_WALL_ANGLE)
    {
        ret = 0;
        LOG_I(" off wall %.2f", pitch);
    }

    return ret;
}

static int _is_touch_down(void)
{
    int ret = 0;
    float pitch = get_current_pitch();
    if (pitch >= SG_TOUCH_DOWN_PITCH)
    {
        ret = 1;
        LOG_I("touch down %.2f", pitch);
    }

    return ret;
}

static int _is_lean_too_much(void)
{
    int ret = 0;
    float yaw = get_current_yaw_on_wall();
    if (fabs(yaw) > SG_LEAN_ANGLE)
    {
        ret = 1;
        LOG_I("lean too much %.2f", yaw);
    }

    return ret;
}

static int _is_over_pitch(void)
{
    int ret = 0;
    float pitch = get_current_pitch();
    if (pitch < SG_OVER_PITCH)
    {
        ret = 1;
        LOG_I("over pitch %.2f", pitch);
    }

    return ret;
}

#if 0
/**
 * @brief 直行到出水，或超时，或从墙上落下来
 * 
 * @param move_speed 上行速度
 * @param timeout    超时时间
 * @return int  0 - 出水
 *             -1 - 超时
 *             -2 - 从墙上落下来
 */
static int _move_forward_until_out_of_water(int move_speed, int timeout)
{
    int ret = 0;

    move_forward_with_speed(move_speed);
    rt_tick_t start_time = rt_tick_get();
    while (1)
    {
        if (_is_out_of_water() == 1)
        {
            ret = 0;
            break;
        }

        if (rt_tick_get() - start_time > timeout)
        {
            ret = -1;
            break;
        }

        if (_is_on_wall() != 1)
        {
            ret = -2;
            break;
        }
    }

    move_stop();
    return ret;
}
#endif

static int _make_robot_vertical_up(void)
{
    int ret = 0;
    float yaw;

    move_stop_time(500);
    if (_is_on_wall() == 0)
    {
//        LOG_I("off the wall");
        ret = -1;
        goto OUT;
    }

    yaw = get_current_yaw_on_wall();
    if (fabs(yaw) > 10.0f)
    {
        move_rotate_on_wall_using_target(0.0f, SG_ROTATE_TIMEOUT);
        move_stop_time(500);
    }
    LOG_I("make vertical done,vyaw %f", get_current_yaw_on_wall());
OUT:
    return ret;
}

static int _move_backward_in_water(void)
{
    const int TIMEOUT = 4 * 1000;
    rt_tick_t start_time = rt_tick_get();

    _make_robot_vertical_up();

    move_backward_with_speed(SG_MOVE_DOWN_SPEED_1);
    sg_out_water_detect_count = 0;
    while (1)
    {
        if (_is_out_of_water() == 0)
        {
            break;
        }

        if (rt_tick_get() - start_time > TIMEOUT)
        {
            LOG_I("timeout");
            break;
        }
    }
    move_stop();
    return 0;
}

static int _wash_water_line_2(void)
{
    const int forward_wp_speed = 2500;
    const int backward_wp_speed = 3500;
    const int move_speed = 3000;
    const float delta = 20.0f;
    int count = 8;

    for (size_t i = 0; i < count; i++)
    {
        // rotate vertical
        _make_robot_vertical_up();
        // back down
        move_wp_speed_on_wall_set(backward_wp_speed);
        move_backward_with_speed_and_time(move_speed, 2500);
        rt_thread_mdelay(500);

        // check pitch
        if (get_current_pitch() > -50.0f)
        {
            LOG_I("fall off the wall");
            break;
        }

        // rotate to right
        move_rotate_on_wall_using_target(-delta, 5 * 1000);

        // up
        move_wp_speed_on_wall_set(forward_wp_speed);
        move_forward_with_speed(move_speed);
        rt_tick_t start_time = rt_tick_get();
        const int timeout = 30 * 1000;
        sg_out_water_detect_count = 0;
        while (1)
        {
            if (rt_tick_get() - start_time > timeout)
            {
                LOG_I("timeout");
                goto _end;
            }

            if (_is_out_of_water() == 1)
            {
                break;
            }

            if (_is_lean_too_much() == 1)
            {
                break;
            }

            if (_is_over_pitch() == 1)
            {
                break;
            }
        }
        rt_thread_mdelay(1000);
    }
_end:
    move_stop();

    return 0;
}

#if 0
/**
 * @brief V型清理墙策略
 *
 * @return int
 */
static int _wash_wall_routine_v(int count, float angle)
{
    int move_speed = 5000;
    int ret = 0;
    const int time_bias = 10 * 1000;

    for (size_t i = 0; i < count; i++)
    {
        int wp_speed = move_wp_speed_on_wall_get();
        move_wp_speed_on_wall_set(SG_MOVE_DOWN_WP_SPEED);
        // reposition and back into water
        _move_backward_in_water();

        // rotate left
        ret = move_rotate_on_wall(angle, SG_ROTATE_TIMEOUT);
        if (ret == 3)
        {
            goto _end;
        }

        // backward until touch down or timeout
        rt_tick_t start_time = rt_tick_get(), cur_time;
        move_speed = 2000;
        move_backward_with_speed(move_speed);
        while (1)
        {
            if (_is_touch_down() == 1)
            {
                ret = 1;
                LOG_D("touch down");
                break;
            }

            cur_time = rt_tick_get();
            if (cur_time - start_time > sg_forward_time + time_bias)
            {
                ret = 2; // out of time
                LOG_D("out of time");
                break;
            }
        }
        move_stop_time(500);
        move_speed = 5000;
        if (ret == 1)
        {
            // move back to the wall
            move_forward_with_speed_and_time(move_speed, 2 * 1000);
        }
        _make_robot_vertical_up();

        // rotate right
        move_rotate_on_wall(-angle, SG_ROTATE_TIMEOUT);
        move_stop_time(500);

        // forward until out of water or timeout
        move_wp_speed_on_wall_set(wp_speed);
        move_forward_with_speed(move_speed);
        start_time = rt_tick_get();
        while (1)
        {
            // out of water
            if (_is_out_of_water() == 1)
            {
                ret = 3;
                LOG_D("out of water");
                break;
            }

            // lean too much
            if (_is_lean_too_much() == 1)
            {
                ret = 4;
                LOG_D("lean too much");
                break;
            }

            // over pitch
            if (_is_over_pitch() == 1)
            {
                ret = 5;
                LOG_D("over pitch");
                break;
            }

            // time out
            cur_time = rt_tick_get();
            if (cur_time - start_time > sg_forward_time + 10 * 1000)
            {
                ret = 6;
                LOG_D("time out");
                break;
            }
        }

        LOG_D("clean wall count = %d", i);
        move_stop_time(500);
    }
_end:
    return ret;
}
#endif
static int _wash_wall_routine_jagged(int count, float angle)
{
    const int time_bias = 2 * 1000; // 时间修正值

    int move_speed = 5000;
    int ret = 0, cnt = 0;

    for (size_t i = 0; i < sg_up_down_count; i++)
    {
        int wp_speed = move_wp_speed_on_wall_get();
        // int tmp_wp_speed = (get_bat_cap_type() == BAT_10000MAH) ? SG_MOVE_DOWN_WP_SPEED : SG_MOVE_DOWN_WP_SPEED_SMALL_BATTERY;
        int tmp_wp_speed = SG_MOVE_DOWN_WP_SPEED;
        move_wp_speed_on_wall_set(tmp_wp_speed);
        // reposition and back into water
        LOG_I("reposition and back into water");
        _move_backward_in_water();
        move_wp_speed_on_wall_set(wp_speed);

        // backward until touch down or timeout
        LOG_I("backward until touch down or timeout");
        rt_tick_t start_time = rt_tick_get(), cur_time, end_time = 0;
        move_speed = SG_MOVE_DOWN_SPEED_2;
        move_backward_with_speed(move_speed);
        while (1)
        {
            if (_is_touch_down() == 1)
            {
//				LOG_I("touch down");
                ret = 1;
                break;
            }

            cur_time = rt_tick_get();
            end_time = cur_time - start_time;
            if (end_time > sg_forward_time + time_bias)
            {
                // ret = 2; // out of time
                LOG_I("out of time %d", end_time);
                break;
            }
        }
        move_stop_time(500);

        float cur_pitch = get_current_pitch();
        if (cur_pitch > -30.0f)
        {
            LOG_I("pitch angle %.2f", cur_pitch);
            ret = -1;
            goto _end;
        }

        if (ret == 1) // move back to the wall
        {
            LOG_I("move forward back to the wall");
            move_forward_with_speed_and_time(5000, 2 * 1000);
            rt_thread_mdelay(500);
        }
        LOG_D("make robot vertical up");
        ret = _make_robot_vertical_up();
        if (ret == -1)
        {
            goto _end;
        }

        // rotate right
        LOG_I("rotate right:%.2f", -angle);
        move_rotate_on_wall(-angle, SG_ROTATE_TIMEOUT);
        move_stop_time(500);

        // forward until out of water or timeout
        LOG_D("move forward until out of water or timeout");
        move_speed = SG_MOVE_UP_SPEED;
        move_forward_with_speed(move_speed);
        start_time = rt_tick_get();
        sg_out_water_detect_count = 0;
        while (1)
        {
            // out of water
            if (_is_out_of_water() == 1)
            {
                ret = 3;
                break;
            }

            // lean too much
            if (_is_lean_too_much() == 1)
            {
                ret = 4;
                break;
            }

            // over pitch
            if (_is_over_pitch() == 1)
            {
                ret = 5;
                break;
            }

            // time out
            cur_time = rt_tick_get();
            if (cur_time - start_time > sg_forward_time + time_bias)
            {
                ret = 6;
                LOG_I("time out");
                break;
            }
        }

        if (ret == 3)
        {
            for (size_t i = 0; i < 10; i++)//从原来的30调整到10
            {
                rt_thread_mdelay(100);
                if (_is_lean_too_much() == 1)
                {
                    ret = 4;
                    break;
                }
            }
        }
        _make_robot_vertical_up();

        LOG_I("clean wall cnt = %d", i);
        cnt = i;
        move_stop_time(100);
    }
_end:
    for (size_t i = sg_row_index; i < sg_row_index + cnt; i++)
    {
        g_wash_wall_record_info.info_lists[i].row_id = i;
        g_wash_wall_record_info.info_lists[i].length = sg_water_depth;
        g_wash_wall_record_info.info_lists[i].width = SG_UP_DOWN_DIS;
    }
    sg_row_index += cnt;
    
    g_wash_wall_record_info.total_rows_num = sg_row_index;
    if (sg_row_index % EF_RCD_LIST_LEN == EF_RCD_LIST_LEN - 1)
    {
        ef_rcd_write(&g_wash_wall_record_info);
    }
    return ret;
}

int actions_on_wall_find_entry(void *p_arg_in, void *p_arg_out)
{
    RT_ASSERT(p_arg_in != RT_NULL);
    RT_ASSERT(p_arg_out != RT_NULL);
    int mode = *((int *)p_arg_in);
    int *p_result = (int *)p_arg_out;

    const float rotate_angle = 90.0f;
    int dis_thres = 200; // mm
    int ret = 0;

    // step1:前进3m，进行爬墙。成功就结束，失败就调头。
    int forward_time = 10 * 1000; // 1s
    ret = _move_forward_to_wall(forward_time, dis_thres);
    if (0 == ret)
    {
        goto OUT;
    }
    LOG_I("1: move forward to wall 10s end");
    rt_thread_mdelay(1000);
    move_wp_speed_on_floor_get();
    move_rotate_on_floor(rotate_angle, SG_ROTATE_TIMEOUT);
    rt_thread_mdelay(1000);
    LOG_I("rotate:%.2f end cur yaw: %.2f", rotate_angle, get_current_yaw());
    move_rotate_on_floor(rotate_angle, SG_ROTATE_TIMEOUT);
    rt_thread_mdelay(1000);
    LOG_D("rotate:%.2f end cur yaw: %.2f", rotate_angle, get_current_yaw());

    // step2: 前进6m，进行爬墙。成功就结束，失败就调头。
    forward_time = 20 * 1000;
    ret = _move_forward_to_wall(forward_time, dis_thres);
    if (0 == ret)
    {
        goto OUT;
    }
    LOG_I("2: move forward to wall 20s end");
    rt_thread_mdelay(1000);
    move_rotate_on_floor(rotate_angle, SG_ROTATE_TIMEOUT);
    rt_thread_mdelay(1000);
    LOG_I("rotate:%.2f end cur yaw: %.2f", rotate_angle, get_current_yaw());
    move_rotate_on_floor(rotate_angle, SG_ROTATE_TIMEOUT);
    rt_thread_mdelay(1000);
    LOG_I("rotate:%.2f end cur yaw: %.2f", rotate_angle, get_current_yaw());

    // step3: 前进3m，进行爬墙。成功就结束，失败就左转90度。
    forward_time = 10 * 1000;
    ret = _move_forward_to_wall(forward_time, dis_thres);
    if (0 == ret)
    {
        goto OUT;
    }
    LOG_I("3: move forward to wall 10s end");
    rt_thread_mdelay(1000);
    move_rotate_on_floor(rotate_angle, SG_ROTATE_TIMEOUT);
    rt_thread_mdelay(1000);
    LOG_I("rotate:%.2f end cur yaw: %.2f", rotate_angle, get_current_yaw());

    // step4: 前进3m，进行爬墙。成功就结束，失败就调头。
    forward_time = 10 * 1000;
    ret = _move_forward_to_wall(forward_time, dis_thres);
    if (0 == ret)
    {
        goto OUT;
    }
    LOG_I("4: move forward to wall 10s end");
    rt_thread_mdelay(1000);
    move_rotate_on_floor(rotate_angle, SG_ROTATE_TIMEOUT);
    rt_thread_mdelay(1000);
    LOG_I("rotate:%.2f end cur yaw: %.2f", rotate_angle, get_current_yaw());
    move_rotate_on_floor(rotate_angle, SG_ROTATE_TIMEOUT);
    rt_thread_mdelay(1000);
    LOG_I("rotate:%.2f end cur yaw: %.2f", rotate_angle, get_current_yaw());

    // step5: 前面老是失败，不管啦，不断前进，遇到墙就退出 或是 超时 120秒退出
    // 每障爬3次
    LOG_I("5: move forward to wall 60s end");
    forward_time = 60 * 1000;
    ret = _move_forward_to_wall(forward_time, dis_thres);

OUT:               // finally, it's on the wall or end!~
    if (mode == 0) // 为洗地找墙
    {
        move_backward_with_speed_and_time(5000, 2000);
        move_rotate_on_floor(rotate_angle, SG_ROTATE_TIMEOUT);
        rt_thread_mdelay(1000);
        move_rotate_on_floor(rotate_angle, SG_ROTATE_TIMEOUT);
		LOG_I("rotate:%.2f end cur yaw: %.2f", 2*rotate_angle, get_current_yaw());
    }

    *p_result = ret;
    return ret;
}

// #define U_TURN_LEFT
int actions_on_wall_find_next_entry(void *p_arg_in, void *p_arg_out)
{
    RT_ASSERT(p_arg_in != RT_NULL);
    RT_ASSERT(p_arg_out != RT_NULL);

    const int find_wall_timeout = 10 * 1000; // 20s的找墙时间
    const int back_time_max = 60 * 1000;
    const int sundesk_back_time = 18 * 1000;
    const int offwall_back_time = 6 * 1000;

    int back_time = ((struct find_next_edge_info *)p_arg_in)->back_time;
    int forward_time = ((struct find_next_edge_info *)p_arg_in)->forward_time;
    int dis_thres = ((struct find_next_edge_info *)p_arg_in)->thres;
    int detect_time = ((struct find_next_edge_info *)p_arg_in)->detect_time;
    int pre_event = ((struct find_next_edge_info *)p_arg_in)->pre_event;

    int *p_result = (int *)p_arg_out;

    int ret = 0; // U routine find wall
    int move_speed = 5000;

    rt_tick_t cur_tick, sensor_tick, start_time;
    float start_yaw, initial_yaw, cur_slope;
    int end_time;
    int near_wall_detected = 0;

    RT_UNUSED(dis_thres);

    rt_thread_mdelay(1000);
    const float SLOPE_THRES_LEFT_DETECT = 10.0f;
    const float SLOPE_THRES = 20.0f;
    cur_slope = get_current_slope();
    LOG_I("cur slope %f", cur_slope);
    if (fabs(cur_slope) > SLOPE_THRES)
    {
        move_rotate_on_wall_using_target_without_pitch_check(0.0f, SG_ROTATE_TIMEOUT);
        rt_thread_mdelay(1000);
        for (size_t i = 0; i < 50; i++)
        {
            cur_slope = get_current_slope();
            if (fabs(cur_slope) < SLOPE_THRES)
            {
                LOG_I("now slope: %f, ready to u turn", cur_slope);
                break;
            }

            move_backward_with_speed(move_speed);
            rt_thread_mdelay(100);
        }
        move_stop_time(1000);
    }

    sg_up_down_count = SG_UP_DOWN_MAX;

    mtr_reset_error();
    move_wp_speed_on_wall_set(SG_ON_FLOOR_WP_SPEED);
    rt_thread_mdelay(1000);

    if (sg_climbup_yaw != 0.0f)
    {
        float yaw = get_current_yaw();
        float delta_yaw = compare_yaws(yaw, sg_climbup_yaw);
        LOG_I("prv yaw %.2f now yaw %.2f delta yaw: %.2f", sg_climbup_yaw, yaw, delta_yaw);
        if (fabs(delta_yaw) >= 3.0f)
        {
            move_rotate_on_floor_using_target(sg_climbup_yaw, 20 * 1000);
            rt_thread_mdelay(1000);
        }
    }

    LOG_I("prv_event: %d", pre_event);
    if (pre_event != 3)
    {
        ret = _move_forward_to_wall(find_wall_timeout, dis_thres);
        if (ret == 0)
        {
            LOG_I("find wall success");
        }
    }

    // 后退
    rt_thread_mdelay(1000);
    start_yaw = get_current_yaw();
    initial_yaw = start_yaw;
    LOG_I("start yaw: %.2f", start_yaw);
    if (pre_event == 2 || pre_event == 4) // 大平台或从墙上off
    {
        int run_time = 0;
        if (pre_event == 2)
        {
            run_time = sundesk_back_time;
        }
        else if (pre_event == 4)
        {
            run_time = offwall_back_time;
        }
        else
        {
        }

        LOG_I("->backward %ds", run_time / 1000);
        start_time = rt_tick_get();
        while (1)
        {
            rt_tick_t end_time = rt_tick_get() - start_time;
            if (end_time > run_time)
            {
                LOG_D("backward %dms end", run_time);
                break;
            }

            cur_slope = get_current_slope();
            if (fabs(cur_slope) > 10.0f)
            {
                LOG_I("back to wall, cur_slope: %.2f", cur_slope);
                break;
            }

            float cur_yaw = get_current_yaw();
            float delta = compare_yaws(cur_yaw, start_yaw);
            if (delta >= 30.0f)
            {
                LOG_I("off too much, delta: %.2f", delta);
                // corect angle
                move_stop_time(500);
                move_rotate_on_floor_using_target(start_yaw, SG_ROTATE_TIMEOUT);
                move_stop_time(500);
            }
            move_backward_with_pid(move_speed, cur_yaw, start_yaw);
        }
        move_stop_time(500);
        back_time = 3 * 1000;
    }
    else if (pre_event == 3) // 首次找墙不做后退
    {
        back_time = 0;
    }
    else // 其它情况下的后退时间
    {
        back_time = 3 * 1000;
    }

    LOG_I("backward detect time: %d", back_time);
    move_backward_with_speed(move_speed);
	rt_thread_mdelay(1000);	//延时1s，以防找墙找到类似深水区缓坡，找墙时用的10°阈值，回退到平面用的20°阈值，以防后退太少
#if 0   
    rt_thread_mdelay(back_time);
    start_time = rt_tick_get();
    for (size_t i = 0; i < (back_time_max / 100); i++)
    {
        rt_thread_mdelay(100);
        float slope_degree = get_current_slope();
        if (fabs(slope_degree) <= 10.0f)
        {
            LOG_D("find flat floor");
            break;
        }
        else
        {
            LOG_D("slope degree: %.2f", slope_degree);
        }
    }
#endif
    int count = 0;
    for (size_t i = 0; i < (back_time_max / 1000); i++)
    {
        rt_thread_mdelay(1000);
        float cur_pitch = get_current_pitch();
        LOG_D("cur pitch: %.2f", cur_pitch);
		int ret = _is_value_stable((int)(fabs(cur_pitch)), &count, back_time / 1000, SLOPE_THRES);//将10°阈值增大，以防下墙后机器处于斜坡(比如肾形池缓坡)
        if (ret == 1)
        {
            LOG_I("pitch:%.2f stable,find flat floor", cur_pitch);
            break;
        }

        cur_slope = get_current_slope();
        if (fabs(cur_slope) > SLOPE_THRES)	//根据上面10°阈值调大，这里也调大到20°
        {
            LOG_I("back to wall, cur_slope: %.2f", cur_slope);
            break;
        }
    }

    move_stop_time(500);

    // 校正角度
    move_rotate_on_floor_using_target(initial_yaw, SG_ROTATE_TIMEOUT);
    move_stop_time(500);
    LOG_I("cur yaw before hit: %.2f", get_current_yaw());
    ret = _move_forward_to_wall(find_wall_timeout, dis_thres);
    if (ret == 0)
    {
        LOG_I("find wall success");
    }
    move_stop_time(500);

    float cur_yaw = get_current_yaw();
    LOG_I("cur yaw after hit: %.2f", cur_yaw);
#ifdef E_HYWORLD
    int back_time_for_hyworld = 5 * 1000;
    if (pre_event == 3)
        back_time_for_hyworld = 0;
    move_backward_with_speed_and_time(move_speed, back_time_for_hyworld);
#else
    cur_slope = get_current_slope();
    LOG_I("check machine slope %0.2f", cur_slope);
    if (fabs(cur_slope) > 10.0f)
    {
        move_backward_with_speed(move_speed);
		rt_thread_mdelay(1000);	//延时1s，以防找墙找到类似深水区缓坡，找墙时用的10°阈值，回退到平面用的20°阈值，以防后退太少
        for (size_t i = 0; i < (back_time_max / 1000); i++)
        {
            rt_thread_mdelay(1000);
            float cur_pitch = get_current_pitch();
            LOG_D("cur pitch: %.2f", cur_pitch);
            int ret = _is_value_stable((int)(fabs(cur_pitch)), &count, back_time / 1000, SLOPE_THRES);//将10°阈值增大，以防下墙后机器处于斜坡(比如肾形池缓坡)
            if (ret == 1)
            {
                LOG_I("pitch:%.2f stable,find flat floor", cur_pitch);
                break;
            }
        }
    }
#endif
    move_stop_time(1000);
    float target_yaw = calculate_yaw(cur_yaw, -90.0f);
    move_rotate_on_floor_using_target(target_yaw, SG_ROTATE_TIMEOUT);

    // #ifndef U_TURN_LEFT
    //     move_rotate_on_floor(-90.0f, SG_ROTATE_TIMEOUT); // 右转
    // #else
    //     move_rotate_on_floor(90.0f, SG_ROTATE_TIMEOUT); // 左转
    // #endif
    move_stop_time(1000);

    int measure_count = 0;
    // 若是洗墙时检测到墙上面有问题，则直接走U型
    if (pre_event == 5)
    {
        LOG_I("wall top reason, go to check wall");
        goto _ensure_wall;
    }

    move_backward_with_speed_and_time(5000, 1000);
_re_measure:
    // detect wall by sensors
    cur_tick = rt_tick_get();
    rt_uint8_t label;
    rt_uint16_t dis;
    get_distance_fusion(&sensor_tick, &label, &dis);    
    if (label == 1 && (abs(cur_tick - sensor_tick) < 4000)) // 数据可信且新
    {
		LOG_I("[%d]src dis=%d, cur_time=%d, sensor_time=%d", measure_count, dis, cur_tick, sensor_tick);
        int count = dis / SG_UP_DOWN_DIS;
        if (count > SG_UP_DOWN_MAX)
        {
            count = SG_UP_DOWN_MAX;
        }
        LOG_I("calc wash wall count=%d", count);

        if (pre_event == -2 || pre_event == -1 || pre_event == 1 || pre_event == 2 || pre_event == 4) // 在遇到step sundesk 的时候，U型不用走太多
        {
            detect_time = 2 * 1000;
            goto _ensure_wall;
        }

        if (count <= 1)
        {
            LOG_I("near the adjacent wall, go to check it");
            near_wall_detected = 1;
            goto _ensure_wall;
        }
        else
        {
            sg_up_down_count = count;
            move_forward_with_speed_and_time(5000, 1000);
            goto _end;
        }
    }
    else
    {
        measure_count++;
        if (measure_count > 5)
        {
			LOG_I("label[%d], src dis=%d, cur_time=%d, sensor_time=%d", label, measure_count, dis, cur_tick, sensor_tick);
            LOG_I("get sensors data timeout[%d], just ensure the wall", measure_count);
            move_forward_with_speed_and_time(5000, 1000);
            detect_time = 4 * 1000;
            goto _ensure_wall;
        }
        goto _re_measure;
    }

_ensure_wall:
    LOG_I("prv event=%d detect time=%d", pre_event, detect_time);
    // 前进
    start_time = rt_tick_get();
    start_yaw = get_current_yaw();
    if (near_wall_detected == 1)
    {
        const int bias_time = 4 * 1000;
        detect_time += bias_time;
    }

    while (1)
    {
        float pitch = get_current_pitch();
        if (pitch < -18.0f)
        {
			//再次测量距离并判断以防机器在缓坡上
			cur_tick = rt_tick_get();
			get_distance_fusion(&sensor_tick, &label, &dis);
			LOG_I("label=%d, src dis=%d, cur_time=%d, sensor_time=%d", label, dis, cur_tick, sensor_tick);
			if (label == 1 && (abs(cur_tick - sensor_tick) < 4000)) // 数据可信且新
			{
				int count_2 = dis / SG_UP_DOWN_DIS;
				if(count_2 > 1)
				{
					LOG_I("on gentle slope, is not wall,dis:%d", dis);
					end_time = rt_tick_get() - start_time;
					ret = 1; // 离墙远
					break;
				}
			}else
			{
				LOG_I("get dis fail, label:%d", label);
				end_time = rt_tick_get() - start_time;
				ret = 1; // 测距没测成功
				break;
			}
			
            ret = 0; // 确认是墙
            LOG_I("is wall indeed %0.2f", pitch);
#ifdef E_HYWORLD
            move_stop_time(500);
            move_rotate_on_floor(85.0f, SG_ROTATE_TIMEOUT); // 左转
            move_stop_time(500);
            move_forward_with_speed_and_time(move_speed, back_time_for_hyworld);
            move_stop_time(500);
            move_rotate_on_floor(-85.0f, SG_ROTATE_TIMEOUT); // 右转
            goto _end2;
#else
            move_stop_time(1000);
            goto _end2;
#endif
        }

        end_time = rt_tick_get() - start_time;
        if (end_time > detect_time + forward_time)
        {
			move_stop_time(1000);	//先停止
			
			float pitch = get_current_pitch();
			if (pitch < -10.0f)
			{
				ret = 1; // 探索墙超时，不是墙
				LOG_I("timeout on gentle slope %0.2f", pitch);
				break;
			}
			
			//以防是怼到墙上抬不了头，所以后退，测量距离
			move_backward_with_pid_and_time(move_speed, start_yaw, 2000);	//后退2s
			move_stop_time(500);
			cur_tick = rt_tick_get();
			get_distance_fusion(&sensor_tick, &label, &dis);	//测量距离
			LOG_I("label=%d, src dis=%d, cur_time=%d, sensor_time=%d", label, dis, cur_tick, sensor_tick);
			if (label == 1 && (abs(cur_tick - sensor_tick) < 4000)) // 数据可信且新
			{
				int count_3 = dis / 340;	// 以5000转后退2s，最多走340mm
				if(count_3 < 1)
				{
					LOG_I("ensure_wall,near wall,dis:%d", dis);
					ret = 0; // 离墙近，是墙
					goto _end2;
				}
			}
			move_forward_with_pid_and_time(move_speed, start_yaw, 2000);	//前进2s
			move_stop_time(500);
			
            ret = 1; // 探索墙超时，不是墙
            LOG_I("[%d]time out, is not wall", end_time);
            break;
        }

        float cur_yaw = get_current_yaw();
        move_forward_with_pid(move_speed, cur_yaw, start_yaw);
    }
    move_stop_time(1000);

    if (ret == 1 && pre_event == 0) // no find wall then move back
    {
        LOG_I("backward due to no wall");
        move_backward_with_pid_and_time(move_speed, start_yaw, end_time);
        move_stop_time(1000);
        move_rotate_on_floor_using_target(start_yaw, SG_ROTATE_TIMEOUT);
        move_stop_time(1000);
    }

_end:
#ifndef U_TURN_LEFT
    move_rotate_on_floor(85.0f, SG_ROTATE_TIMEOUT); // 左转
#else
    move_rotate_on_floor(-90.0f, SG_ROTATE_TIMEOUT); // 右转
#endif
    move_stop_time(500);

    // 前进
    move_forward_with_speed_and_time(move_speed, back_time);
    LOG_I("ready to wash wall %d times, yaw %.2f", sg_up_down_count, get_current_yaw());

    cur_slope = get_current_slope();
    LOG_D("cur slope %.2f", cur_slope);
    if (fabs(cur_slope) > SLOPE_THRES_LEFT_DETECT)
    {
        LOG_I("floor is too steep, %.2f", cur_slope);
        goto _end2;
    }

#if 0   //L型墙检测机制，由于不是常见场景，怕影响正常清洗策略，暂时关闭
    // check front wall
    cur_tick = rt_tick_get();
    get_distance_fusion(&sensor_tick, &label, &dis);
    LOG_D("[%d]src dis=%d, cur_time=%d, sensor_time=%d", measure_count, dis, cur_tick, sensor_tick);
    if (label == 1 && (abs(cur_tick - sensor_tick) < 4000)) // 数据可信且新
    {
        if (dis > 700)
        {
            LOG_D("front wall is too far, %d", dis);
            move_rotate_on_floor(85.0f, SG_ROTATE_TIMEOUT); // 左转
            move_stop_time(500);

            // check front wall
            cur_tick = rt_tick_get();
            get_distance_fusion(&sensor_tick, &label, &dis);
            LOG_D("[%d]src dis=%d, cur_time=%d, sensor_time=%d", measure_count, dis, cur_tick, sensor_tick);
            if (label == 1 && (abs(cur_tick - sensor_tick) < 4000)) // 数据可信且新
            {
                if (dis < 700 && dis > 50)
                {
                    LOG_D("left wall is near, %d", dis);
                    goto _end2;
                }
            }
            move_rotate_on_floor(-85.0f, SG_ROTATE_TIMEOUT); // 右转
            move_stop_time(500);
        }
    }
    else
    {
    }
#endif
_end2:
    *p_result = ret;
    return ret;
}

int actions_on_wall_climb_up(void *p_arg_in, void *p_arg_out)
{
    RT_ASSERT(p_arg_out != RT_NULL);
    int *p_result = (int *)p_arg_out;

    const float STAIRS_OFFSET = 20.0f;
    int start_wp = SG_CLIMB_UP_WP_SPEED_1, wall_wp = SG_ON_WALL_WP_SPEED;
    int move_speed = SG_MOVE_UP_SPEED;
    int try_count = 0, stairs_flag = 0;
    int head_up_flag = 0;
    int timeout = 45 * 1000; // climb up
    float cur_pitch, cur_yaw, cur_roll, dummy_yaw = -666.0f, pitch_min;
    struct slope_type info;
	rt_uint8_t pitch_range = 0;

_start:
    pitch_min = 0.0f;
    LOG_I("climbup[%d] %d %d %d\n", try_count, start_wp, wall_wp, move_speed);
    int ret = 0;
    mtr_reset_error();

    move_wp_speed_on_floor_set(start_wp);
    rt_thread_mdelay(1000);
    move_forward_with_speed(move_speed);
    rt_tick_t start_time = rt_tick_get(), end_time = 0;
    sg_out_water_detect_count = 0;
    while (1)
    {
        // get_eluer_data(&cur_roll, &cur_pitch, &cur_yaw);
        get_current_slope_info(&info);
        cur_pitch = info.euler_pitch;
        cur_roll = info.euler_roll;
        cur_yaw = info.euler_yaw;
        
        if (cur_pitch < SG_WALL_ANGLE || info.angle > fabs(SG_WALL_ANGLE)) // -75 -- -180
        {
			LOG_I("pitch %.2f yaw %.2f roll %.2f", cur_pitch, cur_yaw, cur_roll);
            LOG_I("climb on wall....");
			pitch_range = 1;
            sg_climbup_yaw = dummy_yaw;
            LOG_D("sg_climbup_yaw %.2f", dummy_yaw);
            laser_switch_off();
            LOG_D("laser off and move on some time");
            rt_thread_mdelay(1000);
            if (_is_out_of_water() == 1)
            {
                goto _out;
            }
            LOG_D("make robot vertical up");
            _make_robot_vertical_up();
            LOG_I("climb up done");			
            ret = 0;
            break;
        }
        else if (cur_pitch < -30.0f) // -30 -- -75
        {
			if(pitch_range != 2)
			{
				pitch_range = 2;
				LOG_I("pitch %.2f yaw %.2f roll %.2f", cur_pitch, cur_yaw, cur_roll);
				dummy_yaw = cur_yaw;
				move_wp_speed_on_wall_set(wall_wp);
				head_up_flag = 1;
			}
        }
        else // -20 -- -1
        {
			if(pitch_range != 3)
			{
				pitch_range = 3;
				LOG_I("pitch %.2f yaw %.2f roll %.2f", cur_pitch, cur_yaw, cur_roll);
				move_wp_speed_on_floor_set(start_wp);
			}
            
        }
		
        pitch_min = cur_pitch < pitch_min ? cur_pitch : pitch_min;
        if ((cur_pitch - pitch_min > STAIRS_OFFSET) && head_up_flag)
        {
            stairs_flag = 1;
            LOG_D("stairs detected....");
        }

        // if (fabs(cur_roll) > 40.0f)
        // {
        //     LOG_D("lean too much....");
        //     _make_robot_vertical_up();
        // }

        end_time = rt_tick_get() - start_time;
        if (_is_out_of_water() == 1 && end_time > 1 * 1000)
        {
_out:
            LOG_D("out of water....");
            ret = -2;
            break;
        }

        end_time = rt_tick_get() - start_time;
        if (end_time > timeout)
        {
            ret = -1;
            LOG_I("timeout....");
            break;
        }
    }
    LOG_I("climbup during time %d", end_time);

    if (ret == -1) // 爬墙超时
    {
        try_count++;
        if (try_count == 1)
        {
            LOG_I("climbup timeout, try again....");
            move_backward_with_speed_and_time(move_speed, 2 * 1000);
            rt_thread_mdelay(1000);
            start_wp = SG_CLIMB_UP_WP_SPEED_2;
            goto _start;
        }
    }
	if(stairs_flag == 1)
	{
		LOG_I("stairs detected....");
	}

    if (ret != 0)
    {
        *p_result = stairs_flag ? stairs_flag : ret;
    }
    else
    {
        *p_result = ret;
    }
    LOG_I("action end....%d", *p_result);
    return ret;
}

int actions_on_wall_climb_down(void *p_arg_in, void *p_arg_out)
{
    RT_ASSERT(p_arg_in != RT_NULL);
    RT_ASSERT(p_arg_out != RT_NULL);
    int timeout = *(int *)p_arg_in;
    int *p_result = (int *)p_arg_out;

    int ret = 0;
    int move_speed = 5000;
    float pitch, yaw;
    rt_tick_t start_time, run_time;

    LOG_I("climb down start....%d", timeout);
    float slope_degree = get_current_slope();
    if (fabs(slope_degree) < 45.0f)
    {
        LOG_I("slope degree %.2f, not need to climb down", slope_degree);
        goto _end;
    }

    yaw = get_current_yaw_on_wall();
    if (fabs(yaw) > 10.0f)
    {
        move_rotate_on_wall_using_target_without_pitch_check(0.0f, SG_ROTATE_TIMEOUT);
        move_stop_time(500);
    }
    LOG_I("after rotate vyaw %f", get_current_yaw_on_wall());

    // move_wp_speed_on_wall_set(2000);
    move_backward_with_speed(move_speed);
    start_time = rt_tick_get();
    while (1)
    {
        pitch = get_current_pitch();
        if (pitch > SG_CLIMBDOWN_PITCH_THRES)
        {
            ret = 1;
            LOG_I("climbup down on floor....\n");
            laser_switch_on();
            break;
        }

        run_time = rt_tick_get() - start_time;
        if (run_time > (timeout * 2 / 3))
        {
            move_wp_speed_on_wall_set(0);
            LOG_D("close water pump... %d %d", run_time, timeout * 2 / 3);
        }

        if (run_time > timeout)
        {
            ret = 0;
            LOG_I("timeout.... %d %d", run_time, timeout);
            break;
        }
    }
_end:
    move_stop();
    *p_result = ret;
    LOG_I("climb down end....%d", *p_result);
    return ret;
}

int actions_on_wall_move_forward(void *p_arg_in, void *p_arg_out)
{
    RT_ASSERT(p_arg_in != RT_NULL);
    RT_ASSERT(p_arg_out != RT_NULL);

    int ret = 0;
    int timeout = *(int *)p_arg_in;
    int move_speed = SG_MOVE_UP_SPEED;
    float cur_slope = 0;

    float start_yaw = get_current_yaw_on_wall();
    LOG_I("vyaw start on wall %.2f timeout %d", start_yaw, timeout);
    rt_tick_t start_time = rt_tick_get(), move_time = 0;
    struct slope_type info = {0};
    int useless_time = 0;
    sg_out_water_detect_count = 0;
    while (1)
    {
        if (_is_out_of_water() == 1)
        {
            ret = 1;
            LOG_D("water out");
            break;
        }

        get_current_slope_info(&info);
        cur_slope = info.angle;
        if (cur_slope <= 60.0f)
        {
            LOG_I("off the wall ,slope %.2f", cur_slope);
            ret = 2;
            break;
        }

#if 0
        if (_is_on_wall() == 0)
        {
            ret = 2;
            LOG_D("off wall");
            break;
        }
#endif
        move_time = rt_tick_get() - start_time;
        if (move_time - useless_time > timeout)
        {
            ret = 3;
            LOG_I("timeout");
            break;
        }

        float cur_yaw = -info.euler_roll;
        // LOG_D("move forward yaw %.2f", cur_yaw);
        if (fabs(cur_yaw) > 30.0f)	//从45°修改为30°
        {
            LOG_I("yaw out of range %.2f", cur_yaw);
            rt_tick_t initial_time = rt_tick_get();
            move_stop_time(500);
            move_rotate_on_wall_using_target_without_pitch_check(0.0f, SG_ROTATE_TIMEOUT);
            move_stop_time(500);
            start_yaw = get_current_yaw_on_wall();
            useless_time += (rt_tick_get() - initial_time);
        }
//		move_forward_with_pid(move_speed, cur_yaw, start_yaw);
		move_forward_with_pid(move_speed, 0, 0);
    }

    sg_forward_time = move_time - useless_time;
    move_stop_time(500);
    LOG_I("move forward time: %d", sg_forward_time);
    if (ret == 1)
    {
        _make_robot_vertical_up();
        // move_backward_with_speed_and_time(move_speed, (2 * move_time / 3));
    }
    else if (ret == 2) // off the wall, maybe encounter sundesk
    {
        // wait pitch stable and is on floor
        LOG_I("wait pitch stable");
        // move_wp_speed_on_wall_set(2000);
        rt_thread_mdelay(3 * 1000);
        if (get_current_pitch() < -20.0f)
        {
            rt_thread_mdelay(3 * 1000);
        }

        move_forward_with_speed_and_time(move_speed, 3 * 1000);
        rt_thread_mdelay(1 * 1000);
        LOG_I("move rotate for correct angle %p:%f", &sg_climbup_yaw, sg_climbup_yaw);
        move_rotate_on_floor_using_target(sg_climbup_yaw, 20 * 1000);
        actions_on_wall_climb_stairs(RT_NULL, RT_NULL);
        ret = 4;
    }
    else
    {
        _make_robot_vertical_up();
    }

    ((forward_on_wall_info_out_t)p_arg_out)->status = ret;
    ((forward_on_wall_info_out_t)p_arg_out)->forward_time = move_time;
    LOG_I("action end....%d %d", ret, move_time);
    vel_t vector = get_velocities(move_speed, move_speed);
    float speed = vector.linear_x;
    sg_water_depth = speed * move_time;
    g_wash_wall_record_info.info_lists[sg_row_index].length = sg_water_depth;
    return ret;
}

int actions_on_wall_move_backward(void *p_arg_in, void *p_arg_out)
{
    RT_ASSERT(p_arg_in != RT_NULL);
    RT_ASSERT(p_arg_out != RT_NULL);

    int timeout = *(int *)p_arg_in;
    int *p_result = (int *)p_arg_out;

    int ret = 0;

    move_wp_speed_on_wall_set(SG_ON_WALL_WP_SPEED);
    // reposition
    _make_robot_vertical_up();

    // move backward
    int move_speed = SG_MOVE_DOWN_SPEED_2;
    move_backward_with_speed(move_speed);
    rt_tick_t start_time = rt_tick_get();
    while (1)
    {
        if (_is_on_wall() == 0)
        {
//			LOG_I("off the wall");
            ret = 1;
            break;
        }

        if (rt_tick_get() - start_time > timeout)
        {
            ret = 2;
            LOG_I("timeout");
            break;
        }
    }

    move_stop();
    *p_result = ret;
    return 0;
}

int actions_on_wall_get_water_depth(void *p_arg_in, void *p_arg_out)
{

    return 0;
}

int actions_on_wall_clean_wall(void *p_arg_in, void *p_arg_out)
{
    RT_ASSERT(p_arg_in != RT_NULL);
    RT_ASSERT(p_arg_out != RT_NULL);

    int num = ((wash_wall_info_t)p_arg_in)->num;
    float delta_angle = ((wash_wall_info_t)p_arg_in)->angle;
    int ret = _wash_wall_routine_jagged(num, delta_angle);

    *(int *)p_arg_out = ret;
    return ret;
}

// 向右洗洗水线
int actions_on_wall_clean_water_line_diff(void *p_arg_in, void *p_arg_out)
{
    _wash_water_line_2();
    return 0;
}

// 波浪洗水线
int actions_on_wall_clean_water_line(void *p_arg_in, void *p_arg_out)
{
    // int move_speed = 8000;
    // move_forward_with_different_speed_and_time(move_speed, move_speed - 1000, 5000);
    return 0;
}

// 排气
int actions_on_wall_vent_gas(void *p_arg_in, void *p_arg_out)
{
    return 0;
}

// 清理石头与爬墙，上墙，下墙
void clean_stone(void)
{
    //    float prv_pitch = 0.0f;
    led_show_robot_action(ROBOT_ACTION_STOP);
    rt_thread_mdelay(30 * 1000);

    led_show_robot_action(ROBOT_ACTION_FORWARD);

    move_wp_speed_on_floor_set(5000);
    rt_thread_mdelay(2 * 1000);

    struct move_info move_info;
    move_info.start_yaw = get_current_yaw();
    move_info.move_time = 10 * 1000;
    move_info.move_speed = 3000;
    actions_on_floor_move_forward_with_pid_and_time(&move_info, NULL);

    int result = 1234;
    actions_on_wall_climb_up(NULL, &result); // 上墙

    // 墙上上行并水线上停最大时间10秒
    int in = 6 * 1000;
    actions_on_wall_move_forward(&in, &result);
    // rt_thread_mdelay(20 * 1000);
    rt_tick_t start_time = rt_tick_get();
    const int TIMEOUT = 20 * 1000;
    while (1)
    {
        if (rt_tick_get() - start_time > TIMEOUT)
        {
            break;
        }

        if (get_current_pitch() > -65.0f) // 被用手拿起的情况
        {
            LOG_I("fall off, pitch %f", get_current_pitch());
            break;
        }
    }

    // 直接下行，下墙
    // actions_on_wall_climb_down(&in, &result);
    // rt_thread_mdelay(1 * 1000);
    // 纠偏后退，停止
    // move_rotate_on_floor_using_target(move_info.start_yaw, 10 * 1000);
    // rt_thread_mdelay(1 * 1000);
    // move_backward_with_speed_and_time(5000, 2 * 1000);

    move_wp_off_on_floor();
}

static int sg_count = 0;
int actions_on_wall_climb_stairs(void *p_arg_in, void *p_arg_out)
{
    int timeout = 18 * 1000;
    //    int backward_end_time = 0;

    // move forward with pid
    float start_yaw = get_current_yaw();
    move_forward_with_pid(5000, start_yaw, start_yaw);
    rt_tick_t start_time = rt_tick_get(), forward_end_time = 0, climb_stair_start_time = 0, head_up_time = 0;
    RT_UNUSED(climb_stair_start_time);

    sg_count = 0;
    RT_UNUSED(sg_count);
    float cur_yaw;
	float pitch;
    sg_out_water_detect_count = 0;
    while (1)
    {
        pitch = get_current_pitch();
        LOG_D("pitch %f", pitch);
        if (pitch > -60.0f)
        {
            cur_yaw = get_current_yaw();
            move_forward_with_pid(5000, cur_yaw, start_yaw);
            head_up_time = rt_tick_get() - start_time;
        }
        else
        {
            cur_yaw = get_current_yaw_on_wall();
            move_forward_with_pid(5000, cur_yaw, 0);
            LOG_D("head up time record: %d, pitch %f", head_up_time, pitch);

            if (_is_lean_too_much() == 1)
            {
                break;
            }

            if (_is_over_pitch() == 1)
            {
                break;
            }
        }
#if 0
        if (rt_tick_get() - start_time > 10 * 1000)
        {
            int ret = _is_value_stable((int)(fabs(pitch)), &sg_count, 10, 20);
            if (ret == 1)
            {
                LOG_D("pitch stable, on land");
                // rt_thread_mdelay(2 * 1000);
                forward_end_time = rt_tick_get() - climb_stair_start_time;
                LOG_D("forward end time %d", forward_end_time);
                break;
            }
        }

        if (climb_stair_start_time == 0 && pitch < -30.0f)
        {
            climb_stair_start_time = rt_tick_get();
        }
#endif
        // if have big yaw change, adjust it
        if (fabs(compare_yaws(cur_yaw, start_yaw)) > 25.0f && pitch > -40.0f)
        {
            LOG_D("yaw change %.2f pitch %.2f", compare_yaws(cur_yaw, start_yaw), pitch);
            move_stop_time(500);
            move_rotate_on_floor_using_target(start_yaw, 10 * 1000);
            rt_thread_mdelay(500);
        }

        // timeout
        if (rt_tick_get() - start_time > timeout)
        {
            LOG_I("timeout");
            break;
        }

        // reach out water
        if (_is_out_of_water() == 1)
        {
            LOG_I("reach out water");
            break;
        }
    }
    forward_end_time = rt_tick_get() - start_time;
    sg_forward_time += forward_end_time;  
    move_stop();
	LOG_I("head up time record: %d, yaw change %.2f pitch %.2f", head_up_time, compare_yaws(cur_yaw, start_yaw), pitch);
	LOG_I("wash sundesk use time: %d", sg_forward_time);

    int in = 40 * 1000;
    int result;
    LOG_I("climb down");
    actions_on_wall_climb_down(&in, &result);

    move_rotate_on_floor_using_target(start_yaw, 10 * 1000);
#if 0    
    LOG_D("back off");
    timeout = head_up_time;
    if (p_arg_in != RT_NULL)
    {
        timeout = *(int *)p_arg_in;
    }

    sg_count = 0;
    start_time = rt_tick_get();
    while (1)
    {
        float cur_yaw = get_current_yaw();
        move_backward_with_pid(5000, cur_yaw, start_yaw);

        float pitch = get_current_pitch();
        LOG_D("pitch %f", pitch);
#if 0      
        if (pitch > 0.0f && pitch < 20.0f) // fall off stairs
        {
            LOG_D("fall off stairs");
            break;
        }
#endif

#if 0       
        int pitch_time = forward_end_time > 0 ? forward_end_time : 10 * 1000;
        if (rt_tick_get() - start_time > pitch_time)
        {
            int ret = _is_value_stable((int)(fabs(pitch)), &sg_count, 10, 20);
            if (ret == 1)
            {
                LOG_D("pitch stable, on land");
                rt_thread_mdelay(2 * 1000);
                break;
            }
        }
#endif
        // if have big yaw change, adjust it
        if (fabs(compare_yaws(cur_yaw, start_yaw)) > 10.0f && pitch > -40.0f)
        {
            move_stop_time(500);
            move_rotate_on_floor_using_target(start_yaw, 10 * 1000);
            rt_thread_mdelay(500);
        }

        backward_end_time = rt_tick_get() - start_time;
        if (backward_end_time > timeout)
        {
            LOG_D("backward timeout");
            break;
        }
    }
    LOG_D("backward end use time: %d", backward_end_time);
    move_rotate_on_floor_using_target(start_yaw, 10 * 1000);
    rt_thread_mdelay(2 * 1000);
    // move_wp_off_on_wall();
#endif
    return 0;
}

int actions_on_wall_stop_on_waterline(void *p_arg_in, void *p_arg_out)
{
    int ret = 0;
    const int time_out = 30 * 1000;
    const int vol_min = 21 * 1000; // 21V
	rt_uint16_t vol;

    if (p_arg_in == RT_NULL)
    {
        LOG_E("p_arg_in is null");
        return -1;
    }

    int on_waterline_time = ((stop_on_waterline_info_t)p_arg_in)->on_waterline_time; // s 在水线上的停留时间
    int climb_wall_count = ((stop_on_waterline_info_t)p_arg_in)->repeat_count;       // 上水线的次数
    int in_water_time = ((stop_on_waterline_info_t)p_arg_in)->in_water_time;         // s 在水中停的时间
    int power_off_flag = ((stop_on_waterline_info_t)p_arg_in)->power_off_flag;
    int result = 0xFF;
	int climbup_times = 0;

    LOG_I("on_waterline_time %d, repeat_count %d, in_water_time %d", on_waterline_time, climb_wall_count, in_water_time);
    for (size_t i = 0; i < climb_wall_count; i++)
    {
        LOG_I("stop on waterline %d", i);
        // low power        
        get_battery_voltage(&vol);
		LOG_I("current voltage: %dmV", vol);
        if (vol <= vol_min)
        {
            LOG_I("low power, stop climb up");
            break;
        }

		while(result != 0)
		{
			// climb up
			actions_on_wall_climb_up(RT_NULL, &result);
			if(result != 0)
			{
				LOG_I("stop waterline climb up failed %d", climbup_times);
				rt_thread_mdelay(1000);
			}
			climbup_times++;
			if(climbup_times > 5)
			{
				break;
			}
		}
		
		if(climbup_times > 5)
		{
			continue;
		}

        // forward on wall
        actions_on_wall_move_forward((void *)&time_out, &result);

        // stop on waterline
        for (size_t i = 0; i < on_waterline_time; i++)
        {
            rt_thread_mdelay(1000);
            if (i % 5 == 0) // send ble msg per five seconds
            {
                json_update_clean_state(CLEAN_END_STA);
            }
        }

        // backward on wall and climb down
        actions_on_wall_climb_down((void *)&time_out, &result);
        rt_thread_mdelay(1000);
        move_wp_off_on_wall();
        rt_thread_mdelay(in_water_time * 1000);
    }
	

    // power off
    if (power_off_flag == 1)
    {
        LOG_D("power off");
        set_battery_power_off_time(1);
    }
    else
    {
		//关闭水泵、行走电机、眉灯
		move_wp_off_on_wall();
		move_stop();
		led_show_robot_action(ROBOT_ACTION_STOP);
		
		//获取当前电压
		get_battery_voltage(&vol);
		LOG_I("stop waterline done,vol:%d", vol);
		while(vol > IMM_OFF_VOL_THRES)
		{
			get_battery_voltage(&vol);
			rt_thread_mdelay(1000);
		}
		LOG_I("power off, vol %d", vol);
		rt_thread_mdelay(1000);
		power_off_ac(1);	//1s后关机
    }


    return ret;
}

void ww(int argc, char *argv[])
{
    int result = 1234;
    struct find_next_edge_info in_u_info = {.back_time = 2000, .forward_time = 5000, .thres = 200};

    if (argc < 2)
    {
        rt_kprintf("usage: ww <mode> <time> \n ");
        rt_kprintf("\t 1: <mode> find wall \n");
        rt_kprintf("\t 2: <wp_speed> climb up \n ");
        rt_kprintf("\t 3: <timeout> move forward \n ");
        rt_kprintf("\t 4: water line diff \n ");
        rt_kprintf("\t 5: move backward \n ");
        rt_kprintf("\t 6: climb down \n ");
        rt_kprintf("\t 7: u turn \n");
        rt_kprintf("\t 8: v routine wash wall \n");
        rt_kprintf("\t 11: climb up stairs \n");
        return;
    }

    int order = atoi(argv[1]);
    int in = atoi(argv[2]);
    int timeout, num;
    float angle;
    int *p_arg = NULL;
    rt_kprintf("order: %d, in: %d\n", order, in);

    RT_UNUSED(num);
    RT_UNUSED(angle);

    switch (order)
    {
    case 1:
        actions_on_wall_find_entry(&in, &result);
        break;
    case 2:
        p_arg = in ? &in : NULL;
        actions_on_wall_climb_up(p_arg, &result);
        break;
    case 3:
        timeout = atoi(argv[2]);
        actions_on_wall_move_forward(&timeout, &result);
        break;
    case 4:
        actions_on_wall_clean_water_line_diff(&in, NULL);
        break;
    case 5:
        actions_on_wall_move_backward(&in, &result);
        break;
    case 6:
        actions_on_wall_climb_down(&in, &result);
        break;
    case 7:
        if (in != 0)
        {
            in_u_info.forward_time = in * 1000;
            in_u_info.detect_time = atoi(argv[3]) * 1000;
            LOG_D("forward time: %d detect time %d", in_u_info.forward_time, in_u_info.detect_time);
        }

        actions_on_wall_find_next_entry(&in_u_info, &result);
        break;
    case 8:
        if (argc != 4)
        {
            rt_kprintf("usage: ww 8 <num> <angle> \n ");
            return;
        }

        num = atoi(argv[2]);
        angle = atof(argv[3]);
        //        _wash_wall_routine_v(num, angle);
        break;
    case 9:
        actions_on_wall_clean_water_line(NULL, NULL);
    case 10:
        actions_on_wall_vent_gas(NULL, NULL);
        break;
    case 11:
        actions_on_wall_climb_stairs(NULL, NULL);
        break;

    default:
        break;
    }
    // move_wp_off_on_floor();
    rt_kprintf("result: %d\n", result);
}
MSH_CMD_EXPORT(ww, wash wall actions test);
