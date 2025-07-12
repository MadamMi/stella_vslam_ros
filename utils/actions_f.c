#include <math.h>

#include "rtthread.h"
#include "sensors_info.h"
#include "move_basic.h"
#include "actions_on_floor.h"
#include "util.h"
#include "angle.h"
#include "move_basic.h"
#include "led_show.h"
#include "app_entry.h"
#include "mtr_ctrl.h"
#include "app_config.h"
#include "app_server.h"
#include "eeprom.h"


#define DBG_TAG "af"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define LOWER(x, lower, upper) ((x) < (lower) ? (lower) : ((x) > (upper) ? (upper) : (x)))
#define TIME_OFFSET (8UL)

struct floor_params // 洗地运动参数
{
    int forward_speed, backward_speed;
    int rotate_time_max;

    float on_wall_pitch_thres;

    int suck_leaves_time_max;
    int suck_leaves_wp_speed, suck_leaves_forward_speed;
    float suck_leaves_rotate_thres;

    int rotate_measure_no_stop_motor_speed; // 不停旋转测量时的电机速度
};
typedef struct floor_params *floor_params_t;

const struct floor_params SG_FLOOR_PARAMS_DEFAULT = {
    .forward_speed = WASH_FLOOR_MOTOR_SPEED,
    .backward_speed = WASH_FLOOR_MOTOR_SPEED,
    .rotate_time_max = 20 * 1000,
    .on_wall_pitch_thres = SLOPE_THRES_MAX,
    // .on_wall_pitch_thres = 50.0f,
    .suck_leaves_time_max = 8 * 1000,
#if (VERSION_TYPE==E_EXHIBITION_P10)
    .suck_leaves_time_max = 4 * 1000,
#endif
    .suck_leaves_wp_speed = 3800,
    .suck_leaves_forward_speed = 3000,
    .suck_leaves_rotate_thres = 20.0f,
    .rotate_measure_no_stop_motor_speed = 2000};

const struct floor_params SG_FLOOR_PARAMS_SMALL_BAT = {
    .forward_speed = WASH_FLOOR_MOTOR_SPEED,
    .backward_speed = WASH_FLOOR_MOTOR_SPEED,
    .rotate_time_max = 20 * 1000,
    .on_wall_pitch_thres = SLOPE_THRES_MAX,
    // .on_wall_pitch_thres = 50.0f,
    .suck_leaves_time_max = 8 * 1000,
    .suck_leaves_wp_speed = 3500,
    .suck_leaves_forward_speed = 3000,
    .suck_leaves_rotate_thres = 20.0f,
    .rotate_measure_no_stop_motor_speed = 2000};
    
const int PID_AJUST_TIME = 1;
extern const int LED_SHOW_TIME;

static struct mag_calibrate_info sg_gero_info[10];
static int if_right_have_dirt = 0;
int if_uturn_slips = 0;
static wf_params_t wf_act_params = NULL;
static rt_tick_t get_dirty_tick = 0;   // 检测到脏污的时间戳
static int sg_move_detect_dis = 0;
static struct wash_row_info sg_vision_info = {0};

static floor_params_t _get_floor_params(int mode)
{
    floor_params_t p_params = RT_NULL;
    p_params = (get_bat_cap_type() == BAT_10000MAH) ? ((floor_params_t)&SG_FLOOR_PARAMS_DEFAULT) : ((floor_params_t)&SG_FLOOR_PARAMS_SMALL_BAT);
    return p_params;
}

static int _is_ult_data_valid(int dis)
{
    return (dis > 0 && dis < 10000);
}

static int _is_laser_data_valid(int dis)
{
    return (dis > 50 && dis < 10000);
}

// 前进或后退time时长到墙边。如果上墙，需要下墙。is_front=RT_TRUE表示前进，否则为后退
void move_to_edge_prevent_climbing(rt_bool_t is_front, move_info_t info)
{
    const floor_params_t p_params = _get_floor_params(0);
    int move_time = info->move_time;
    float cur_yaw = get_current_yaw();
    if (0 == move_time)
    {
        return;
    }
    LOG_D("%s enter", __FUNCTION__);
    mtr_reset_motor_error();

    float target_yaw = info->start_yaw;
    int move_speed = info->move_speed;
    int dirty_flag = info->dirty_flag;
    LOG_I("move_time %d, move_speed %d, dirty_flag %d", move_time, move_speed, dirty_flag);

    int wp_speed_prv = 0;
    if (dirty_flag == 1)
    {
        wp_speed_prv = move_wp_speed_on_floor_get();
        extern wf_params_t get_wf_params_info(void);
        wf_params_t wf_param = get_wf_params_info();
        move_wp_speed_on_floor_set(wf_param->edge_clean_sediment_wp_speed);
        // rt_thread_mdelay(3 * 1000);
        rt_thread_mdelay(1000);
        move_speed = wf_param->edge_clean_sediment_mtr_speed;
    }

    rt_tick_t start_time = rt_tick_get_millisecond();
    float slope;

    if (is_front == RT_TRUE) // 前进
    {
        led_show_robot_action(ROBOT_ACTION_FORWARD);
        while (1)
        {
            float pitch = get_current_pitch();
            if (pitch < (-1.0f * p_params->on_wall_pitch_thres)) // 上墙了
            {
                LOG_I("on wall %.2f", pitch);
                break;
            }
            rt_tick_t cur_time = rt_tick_get_millisecond();
            if ((cur_time - start_time) > move_time)
            {
                LOG_I("m:%u,c:%u,s:%u", move_time, cur_time, start_time);
                break;
            }
            cur_yaw = get_current_yaw();
            move_forward_with_pid(move_speed, cur_yaw, target_yaw);
            rt_thread_mdelay(60);
        }
        move_stop();
    }
    else // 后退
    {
        led_show_robot_action(ROBOT_ACTION_BACKWARD);
        LOG_I("move_to_edge_prevent_climbing: move backward\n");
        while (1)
        {
            cur_yaw = get_current_yaw();
            move_backward_with_pid(move_speed, cur_yaw, target_yaw);
            slope = get_current_slope();
            LOG_D("slope %f", slope);
            // move_backward_with_speed(move_speed);
            // if (slope > p_params->on_wall_pitch_thres) // 后退上墙了
            if (slope > 15.0f) // 后退上墙了
            {
                LOG_I("over slope occur! %.2f", slope);
                break;
            }
            rt_tick_t cur_time = rt_tick_get_millisecond();
            if ((cur_time - start_time) > move_time)
            {
                LOG_I("timeout");
                break;
            }
            // rt_thread_mdelay(60);
        }
		LOG_I("slope %f", slope);
        move_stop();
    }
    rt_thread_mdelay(500);

#if 0  
    // 下墙
    if (fabs(get_current_pitch()) > 45.0f)
    {
        int counter = 0;
        // 关水泵,执行后退操作
        move_wp_off_on_floor();
        rt_thread_mdelay(1000);

        if (get_current_pitch() > 0.0f) // pitch>0,表示机器屁股在墙上
        {
            move_forward_with_speed(MOTOR_SPEED);
        }
        else // 机头在墙上
        {
            move_backward_with_speed(MOTOR_SPEED);
        }

        while (1)
        {
            // 这里忽略了坡
            if (fabs(get_current_pitch()) < IMU_COLLISION_SAFEZONE_PITCH)
            {
                move_wp_speed_on_floor_set(WP_SPEED);
                break;
            }
            rt_thread_mdelay(500);

            counter += 1;
            if (counter == 10) // delay 5s
            {
                move_wp_speed_on_floor_set(WP_SPEED);
                break;
            }
        }

        move_stop();
    }
#endif
    mtr_reset_motor_error();
    // make sure it's on floor
    int count = 0;
    while (1)
    {
        float pitch = get_current_pitch();
        if (fabs(pitch) < 10.0f)
        {
            count++;
        }
        else
        {
            cur_yaw = get_current_yaw();
            if (is_front == RT_TRUE)
            {
                if (pitch > 10.0f)
                {
                    LOG_I("backward on wall %.2f", pitch);
                    break;
                }
                move_backward_with_pid(move_speed, cur_yaw, target_yaw);
            }
            else
            {
                if (pitch < -10.0f)
                {
                    LOG_I("forward on wall %.2f", pitch);
                    break;
                }
                move_forward_with_pid(move_speed, cur_yaw, target_yaw);
            }
            count = 0;
            // move_stop();
        }
        if (count >= 1)
        {
            move_stop();
            break;
        }
    }
    move_stop();
    if (dirty_flag == 1)
    {
        move_wp_speed_on_floor_set(wp_speed_prv);
        // rt_thread_mdelay(3 * 1000);
        rt_thread_mdelay(1000);
    }
    LOG_I("move_to_edge_prevent_climbing: end\n");
}

int actions_on_floor_rotate_measure(void *p_arg_in, void *p_arg_out)
{
    RT_ASSERT(p_arg_in != RT_NULL);
    RT_ASSERT(p_arg_out != RT_NULL);

    const floor_params_t p_params = _get_floor_params(0);

    const float step_angle = ((struct dis_rotate_in_arg *)p_arg_in)->single_rotate_angle;
    const float total_angle = ((struct dis_rotate_in_arg *)p_arg_in)->total_rotate_angle;
    const int time_interval = ((struct dis_rotate_in_arg *)p_arg_in)->time_interval;

    dis_sensor_info_t ultrasonic_buff = ((struct distance_sensor_out_arg *)p_arg_out)->ultrasonic_buff;
    const int ult_buff_size = ((struct distance_sensor_out_arg *)p_arg_out)->ult_buff_sz;
    int *ult_olen = &(((struct distance_sensor_out_arg *)p_arg_out)->ultrasonic_count);

    dis_sensor_info_t laser_buff = ((struct distance_sensor_out_arg *)p_arg_out)->laser_buff;
    const int laser_buff_size = ((struct distance_sensor_out_arg *)p_arg_out)->laser_buff_sz;
    int *laser_olen = &(((struct distance_sensor_out_arg *)p_arg_out)->laser_count);

    float sum = 0.0f;
    int ult_index = 0, laser_index = 0;

    // open pump
    // if (!is_wp_running())
    // {
    //     move_wp_speed_on_floor_set(WP_SPEED);
    //     rt_thread_mdelay(2 * 1000);
    // }
    LOG_I("rotate start\r\n");
    while (1)
    {
        // 转
        float start_yaw = get_current_yaw();
        int ret = move_rotate_on_floor(step_angle, p_params->rotate_time_max);
        rt_thread_mdelay(time_interval);

        // 量
        float yaw = get_current_yaw();
        int ult_distance = get_current_distance_from_ultrasonic();
        int laser_distance = get_current_distance_from_laser();

        // 存
        if (_is_ult_data_valid(ult_distance))
        {
            ultrasonic_buff[ult_index].yaw = yaw;
            ultrasonic_buff[ult_index].distance = ult_distance;
            ult_index++;
            if (ult_index >= ult_buff_size)
                break;
        }

        if (_is_laser_data_valid(laser_distance))
        {
            laser_buff[laser_index].yaw = yaw;
            laser_buff[laser_index].distance = laser_distance;
            laser_index++;
            if (laser_index >= laser_buff_size)
                break;
        }

        sum += compare_yaws(yaw, start_yaw);
        LOG_D("------------>start_yaw=%.2f end_yaw=%.2f sum=%.2f total_angle=%.2f\r\n", start_yaw, yaw, sum, total_angle);
        // 控制delta 变量
        if (fabs(sum) >= fabs(total_angle))
            break;
    }

    *ult_olen = ult_index;
    *laser_olen = laser_index;

    return 0;
}

#define INFO_COUNT 1000
static struct dis_sensor_info sg_dis_info[INFO_COUNT];
int actions_on_floor_rotate_measure_no_stop(void *p_arg_in, void *p_arg_out)
{
    RT_ASSERT(p_arg_in != RT_NULL);
    RT_ASSERT(p_arg_out != RT_NULL);

    const floor_params_t p_params = _get_floor_params(0);
    const float delta_yaw = *(float *)p_arg_in;
    dis_sensor_buffer_info_t buff_info = (dis_sensor_buffer_info_t)p_arg_out;
    buff_info->data_count = 0;
    buff_info->dis_buff = sg_dis_info;

    int left_rotation = (delta_yaw > 0);
    enum robot_action_type action = (left_rotation ? ROBOT_ACTION_TURN_LEFT : ROBOT_ACTION_TURN_RIGHT);
    led_show_robot_action(action);
    rt_thread_mdelay(LED_SHOW_TIME);

    typedef void (*rotate_func_t)(int);
    rotate_func_t rotate_func = left_rotation ? move_turn_left : move_turn_right;

    float start_yaw = get_current_yaw();
    float target_yaw = calculate_yaw(start_yaw, delta_yaw);
    rt_err_t status = -RT_ETIMEOUT;

    LOG_I("rotate start");
    rotate_func(p_params->rotate_measure_no_stop_motor_speed);
    rt_tick_t prv_time = 0, data_tick;
    rt_uint8_t lable;
    rt_uint16_t dis;
    // compare yaw difference
    float cur_yaw = get_current_yaw();
    float yaw_diff = compare_yaws(cur_yaw, target_yaw); // 获取最新的偏角
    LOG_I("delta_yaw: %.2f yaw_diff: %.2f", delta_yaw, yaw_diff);    
    rt_tick_t start_time = rt_tick_get(), end_time = 0;
    while (1)
    {
        // compare yaw difference
        cur_yaw = get_current_yaw();
        yaw_diff = compare_yaws(cur_yaw, target_yaw); // 获取最新的偏角
        if ((delta_yaw > 0 && yaw_diff < 4.0f) || (delta_yaw < 0 && yaw_diff > -4.0f))
        {
            status = RT_EOK;
            LOG_I("rotate end");
            break;
        }

        if (buff_info->data_count >= INFO_COUNT)
        {
            status = -RT_EFULL;
            LOG_I("buff full");
            break;
        }

        end_time = rt_tick_get() - start_time;
        if (end_time > p_params->rotate_time_max)
        {
            status = -RT_ETIMEOUT;
            LOG_I("rotate timeout");
            break;
        }

        get_distance_fusion(&data_tick, &lable, &dis);
        if (lable == 1 && (data_tick != prv_time))
        {
            prv_time = data_tick;
            buff_info->dis_buff[buff_info->data_count].yaw = cur_yaw;
            buff_info->dis_buff[buff_info->data_count].distance = dis;
            buff_info->data_count++;
        }
    }
    LOG_I("delta_yaw: %.2f yaw_diff: %.2f", delta_yaw, yaw_diff);  
    
    move_stop();

    return status;
}

/**
 * @brief 测距sensor融合。当前是激光+超声
 *
 * @param sensor_id 0 ：激光优先  1：超声优先
 * @param distance
 * @return int
 */
int get_current_dis(int sensor_id, int *distance)
{
    int ult_dis = -1;
    int laser_dis = -1;

    ult_dis = get_current_distance_from_ultrasonic();
    laser_dis = get_current_distance_from_laser();

    int ult_valid = _is_ult_data_valid(ult_dis);
    int laser_valid = _is_laser_data_valid(laser_dis);

    if (sensor_id == 0)
    {
        if (laser_valid)
        {
            *distance = laser_dis;
        }
        else if (ult_valid)
        {
            *distance = ult_dis;
        }
        else
        {
            *distance = -1;
        }
    }
    else
    {
        if (ult_valid)
        {
            *distance = laser_dis;
        }
        else if (laser_valid)
        {
            *distance = ult_dis;
        }
        else
        {
            *distance = -1;
        }
    }

    return 0;
}

int get_current_gero(mag_calibrate_info_t info)
{
    // 关水泵、关驱动
    int speed = move_wp_speed_on_floor_get();
    move_wp_off_on_floor();
    move_stop_time(1000);

    struct mag_info gero_info;
    magnetic_raw_data_get(&gero_info);
    info->raw_mag_x = gero_info.raw_mag_x;
    info->raw_mag_y = gero_info.raw_mag_y;
    info->raw_mag_z = gero_info.raw_mag_z;
    info->yaw = get_current_yaw();
    rt_thread_mdelay(1000);

    move_wp_speed_on_floor_set(speed);
    return 0;
}

int actions_on_floor_magnetic_calibration(void *p_arg_in, void *p_arg_out)
{
    RT_ASSERT(p_arg_in != RT_NULL);
    RT_ASSERT(p_arg_out != RT_NULL);

    const float ANGLE_MAX = 360.0f;
    const floor_params_t p_params = _get_floor_params(0);

    float step_angle = ((struct mag_cali_arg_in *)p_arg_in)->step_angle;
    mag_calibrate_info_t buff = ((struct mag_cali_arg_in *)p_arg_in)->buff;
    int max_count = ((struct mag_cali_arg_in *)p_arg_in)->max_count;

    int *out_count = &(((struct mag_cali_arg_out *)p_arg_out)->out_count);

    float sum = 0.0f;
    int index = 0;
    while (1)
    {
        // open pump
        // move_wp_speed_on_floor_set(WP_SPEED);
        // rt_thread_mdelay(3000);

        // 转
        float start_yaw = get_current_yaw();
        int ret = move_rotate_on_floor(step_angle, p_params->rotate_time_max);
        rt_thread_mdelay(2000);

        // stop pump
        move_wp_off_on_floor();
        rt_thread_mdelay(2 * 1000);

        // 量
        struct mag_info mag_info;
        magnetic_raw_data_get(&mag_info);

        // 存
        buff[index].yaw = get_current_yaw();
        buff[index].raw_mag_x = mag_info.raw_mag_x;
        buff[index].raw_mag_y = mag_info.raw_mag_y;
        buff[index].raw_mag_z = mag_info.raw_mag_z;
        index++;
        if (index >= max_count)
            break;

        sum += compare_yaws(start_yaw, buff[index - 1].yaw);
        LOG_D("-------->start yaw: %0.2f, sum: %0.2f\n", start_yaw, sum);
        // 控制delta 变量
        if (fabs(sum) >= ANGLE_MAX)
            break;
    }

    *out_count = index;
    move_wp_off_on_floor();
    return 0;
}

int actions_on_floor_magnetic_calibrate_yaw(void *p_arg_in, void *p_arg_out)
{

    return 0;
}

static int _is_detect_wall(void)
{
    // return 0;
    float pitch = get_current_pitch();
    LOG_I("pitch: %f", pitch);
    if (pitch < -20.0f)
    {
        return 1;
    }

    int dis = get_current_distance_from_ultrasonic();
    LOG_I("ult dis: %d", dis);
    if (dis > 0 && dis < 200)
    {
        return 1;
    }

    dis = get_current_distance_from_laser();
    LOG_I("tof dis: %d", dis);
    if (dis > 50 && dis < 200)
    {
        return 1;
    }

    return 0;
}

int get_next_area_gero(void *p_arg_in, void *p_arg_out)
{
    RT_ASSERT(p_arg_in != RT_NULL);
    RT_ASSERT(p_arg_out != RT_NULL);

    const floor_params_t p_params = _get_floor_params(0);

    get_next_gero_in_t in = (get_next_gero_in_t)p_arg_in;
    int forward_time;
    int forward_2_end_time = 0;

    get_next_gero_out_t out = (get_next_gero_out_t)p_arg_out;
    out->gero_buff = sg_gero_info;

    float target_yaw, delta;
    RT_UNUSED(delta);
    rt_tick_t start_time, forward_1_end_time;
    int ret = 1;
    int move_speed = p_params->forward_speed;

    out->gero_buff_sz = 0;
    LOG_I("out %p, sz %d", out, out->gero_buff_sz);

    // 停止运动
    move_stop_time(1000);

    // 右转90度
    // LOG_D("rotate right 90");
    // delta = -90.0f;
    // move_rotate_on_floor(delta, ROTATE_TIMEOUT);
    // rt_thread_mdelay(1 * 1000);

    // 直行move_rows行
    forward_time = in->move_rows * in->one_line_move_time;
    LOG_I("1-forward %d", forward_time);
    target_yaw = get_current_yaw();
    start_time = rt_tick_get_millisecond();
    while (rt_tick_get_millisecond() - start_time <= forward_time)
    {
        if (_is_detect_wall())
        {
            LOG_I("detect wall");
            ret = -1;
            break;
        }
        move_forward_with_pid(move_speed, get_current_yaw(), target_yaw);
        rt_thread_mdelay(PID_AJUST_TIME);
    }
    forward_1_end_time = rt_tick_get_millisecond() - start_time;
    LOG_I("actually forward time %d", forward_1_end_time);
    move_stop_time(1 * 1000);
    LOG_I("get gero");
    get_current_gero(&sg_gero_info[0]);
    out->gero_buff_sz++;
    if (ret == -1)
    {
        goto end;
    }

    forward_time = in->one_line_move_time;
    LOG_I("2-forward %d to measure %d times", forward_time, in->row_num);
    for (size_t i = 0; i < in->row_num; i++)
    {
        // 直行1行
        target_yaw = get_current_yaw();
        start_time = rt_tick_get_millisecond();
        while (rt_tick_get_millisecond() - start_time <= forward_time)
        {
            if (_is_detect_wall())
            {
                LOG_I("detect wall");
                ret = -1;
                break;
            }
            move_forward_with_pid(move_speed, get_current_yaw(), target_yaw);
            rt_thread_mdelay(PID_AJUST_TIME);
        }
        forward_2_end_time += rt_tick_get_millisecond() - start_time;
        move_stop_time(1 * 1000);

        // 左转90度
        // LOG_D("rotate left 90\r\n");
        // delta = 90.0f;
        // move_rotate_on_floor(delta, ROTATE_TIMEOUT);
        // rt_thread_mdelay(1 * 1000);

        LOG_I("get gero");
        get_current_gero(&sg_gero_info[1 + i]);
        out->gero_buff_sz++;

        // if (i < in->row_num - 1)
        // {
        //     // 右转90度
        //     LOG_D("rotate right 90");
        //     delta = -90.0f;
        //     move_rotate_on_floor(delta, ROTATE_TIMEOUT);
        //     rt_thread_mdelay(1 * 1000);
        // }
    }
    LOG_I("actually forward 2 end time %d", forward_2_end_time);
end:
    // 左转90度,开始折返
    // LOG_D("rotate left 90 to return");
    // delta = 90.0f;
    // move_rotate_on_floor(delta, ROTATE_TIMEOUT);
    // rt_thread_mdelay(1 * 1000);

    // 直行row_num行，回到原始位置
    forward_time = forward_2_end_time + forward_1_end_time;
    LOG_I("3-go back %d", forward_time);
    target_yaw = get_current_yaw();
    start_time = rt_tick_get_millisecond();
    while (rt_tick_get_millisecond() - start_time <= forward_time)
    {
        move_backward_with_pid(move_speed, get_current_yaw(), target_yaw);
        // rt_thread_mdelay(PID_AJUST_TIME);
    }
    LOG_I("actually time %d", rt_tick_get_millisecond() - start_time);
    move_stop_time(1 * 1000);

    // 右转90度
    // LOG_D("rotate right 90 to reposition");
    // delta = -90.0f;
    // move_rotate_on_floor(delta, ROTATE_TIMEOUT);
    // rt_thread_mdelay(1 * 1000);

    out->ret = ret;
    LOG_I("out %p, sz %d", out, out->gero_buff_sz);
    return 0;
}

#if 0
int actions_on_floor_move_forward_to_obstacle_1(void *p_arg_in, void *p_arg_out)
{
    RT_ASSERT(p_arg_in != RT_NULL);
    RT_ASSERT(p_arg_out != RT_NULL);

    int result = ACTION_END_SUCCESS;
    const floor_params_t p_params = _get_floor_params(0);

    const int RUN_TIME_MAX = 3 * 60 * 1000;
    float start_yaw = ((move_forward_info_t)p_arg_in)->start_yaw;
    unsigned char change_speed_flag = ((move_forward_info_t)p_arg_in)->if_change_speed;
    int *end_dis = ((int *)p_arg_out);

    // int ult_dis = -1, tof_dis = -1, start_comp_speed_dis = 0;
    int tof_dis = -1, start_comp_speed_dis = 0;

    // int ult_dis_buff[3] = {0};
    // int tof_dis_buff[3] = {0};
    // unsigned int ult_time_buff[3] = {0};
    // unsigned int tof_time_buff[3] = {0};

    LOG_D("pid start start_yaw=%f", start_yaw);
    unsigned int timestamp_start = rt_tick_get(), speed_start_time;
    int prv_time = rt_tick_get_millisecond();
    float cur_yaw;

    rt_tick_t sensor_tick;
    rt_uint8_t label;
    rt_uint16_t dis;



    while (1)
    {
        // 如果pitch绝对值大于阈值，认为机器上墙了，立马break
        if (fabs(get_current_pitch()) > p_params->on_wall_pitch_thres)
        {
            LOG_D("end method pitch");
            *end_dis = tof_dis;
            result = ACTION_END_TO_WALL;
            break;
        }

        int leaves_flag = 0;
        if (change_speed_flag == 1)
        {
            leaves_flag = is_find_leaves(10);
        }

        if (leaves_flag == 1)
        {
            LOG_I("find leaves...");
            move_stop();
            led_show_robot_action(ROBOT_ACTION_CLEAN_LEAVES);
            rt_thread_mdelay(500);

            int speed = move_wp_speed_on_floor_get();
            move_wp_speed_on_floor_set(p_params->suck_leaves_wp_speed); // 水泵最大
            unsigned int start_time = rt_tick_get();
            while (1) // 降速
            {
                if (rt_tick_get() - start_time > p_params->suck_leaves_time_max)
                {
                    break;
                }

                cur_yaw = get_current_yaw();
                move_forward_with_pid(p_params->suck_leaves_forward_speed, cur_yaw, start_yaw);
            }
            move_wp_speed_on_floor_set(speed); // 恢复水泵
        }

        int cur_time = rt_tick_get_millisecond();
        if (cur_time - prv_time > 10) // 200ms 查询一次tof //HPU:200->10MS FOR TEST
        {
            prv_time = rt_tick_get_millisecond();
            // tof_dis = get_current_distance_from_laser(); //HPU: SWITCH TO NEW METHOD

            // cur_tick = rt_tick_get_millisecond();
            get_distance_fusion(&sensor_tick, &label, &dis);
            tof_dis = dis;
            // if ((abs(sensor_tick - cur_tick) < 4000) && (label == 1)) // 3s以内的数才信
            // {
                
            // }
            
            // int tof_valid = _is_laser_data_valid(tof_dis);
            if ((abs(sensor_tick - prv_time) < 4000) && (label == 1))
            {
#if 0
                LOG_D("tof_dis_buff[0]=%d", tof_dis_buff[0]);
                // 判断buff里的数据
                if (tof_dis_buff[0] > 0)
                {
                    if ((tof_dis_buff[0] - tof_dis < 15) || (tof_dis_buff[0] - tof_dis > 2000)) // 如果没有连续减小的距离值，则认为上一距离失效
                    {
                        tof_dis_buff[0] = tof_dis;
                        tof_time_buff[0] = rt_tick_get();
                    }
                    else
                    {
                        if (start_comp_speed_dis == 0)
                        {
                            start_comp_speed_dis = tof_dis_buff[0];
                            speed_start_time = tof_time_buff[0];
                        }

                        if (tof_dis < EDGE_THRES)
                        {
                            *end_dis = tof_dis;
                            LOG_D("end method tof");
                            break;
                        }

                        tof_dis_buff[0] = tof_dis;
                        tof_time_buff[0] = rt_tick_get();
                    }
                }
                else
                {
                    tof_dis_buff[0] = tof_dis;
                    tof_time_buff[0] = rt_tick_get();
                }
#else
                    if (tof_dis < EDGE_THRES)
                    {
                        *end_dis = tof_dis;
                        LOG_D("end method tof = %d", tof_dis);
                        break;
                    }
#endif
            }
            // else
            // {
            //     tof_dis_buff[0] = 0;
            //     tof_time_buff[0] = rt_tick_get();
            // }
        }

        // ult_dis = get_current_distance_from_ultrasonic();

        if (rt_tick_get() - timestamp_start > RUN_TIME_MAX)
        {
            // LOG_D("end method tiemout,tof=%d ult=%d", tof_dis, ult_dis);
            LOG_D("end method tiemout,dis=%d", dis);
            *end_dis = dis;
            result = ACTION_END_TIMEOUT;
            break;
        }

        // int ult_valid = _is_ult_data_valid(ult_dis);

        // if (ult_valid) // 测到有效数据
        // {
        //     // LOG_D("ult_dis_buff[0]=%d\n", ult_dis_buff[0]);
        //     // 判断buff里的数据
        //     if (ult_dis_buff[0] > 0)
        //     {
        //         if ((ult_dis_buff[0] - ult_dis < 15) || (ult_dis_buff[0] - ult_dis > 2000)) // 如果没有连续减小的距离值，则认为上一距离失效
        //         {
        //             ult_dis_buff[0] = ult_dis;
        //             ult_time_buff[0] = rt_tick_get();
        //         }
        //         else
        //         {
        //             if (start_comp_speed_dis == 0)
        //             {
        //                 start_comp_speed_dis = ult_dis_buff[0];
        //                 speed_start_time = ult_time_buff[0];
        //             }

        //             if (ult_dis < EDGE_THRES)
        //             {
        //                 *end_dis = ult_dis;
        //                 LOG_D("end method ult\n");
        //                 break;
        //             }

        //             ult_dis_buff[0] = ult_dis;
        //             ult_time_buff[0] = rt_tick_get();
        //         }
        //     }
        //     else
        //     {
        //         ult_dis_buff[0] = ult_dis;
        //         ult_time_buff[0] = rt_tick_get();
        //     }
        // }
        // else
        // {
        //     ult_dis_buff[0] = 0;
        //     ult_time_buff[0] = rt_tick_get();
        // }

        cur_yaw = get_current_yaw();
        LOG_D("get_current_yaw()=%f", cur_yaw);
        move_forward_with_pid(p_params->forward_speed, cur_yaw, start_yaw);
        rt_thread_mdelay(PID_AJUST_TIME);
    }

    unsigned int timestamp_end = rt_tick_get();
    int forward_time = timestamp_end - timestamp_start;
    unsigned int time = timestamp_end - speed_start_time;
    int speed = 0;

    // *end_dis = dis;
    LOG_D("end_dis=%d", *end_dis);
    LOG_D("time=%d", time);

    if ((end_dis > 0) && (time > 0))
    {
        speed = (float)(start_comp_speed_dis - *end_dis) / time;
        LOG_D("speed=%d", speed);
    }

    LOG_D("forward_time=%d", forward_time);
    LOG_D("forward_speed=%d", speed);

#if 0
    // 如果上墙，需要先下墙
    float cur_pitch = get_current_pitch();
    if (fabs(cur_pitch) > 45.0f)
    {
        // 关水泵,执行后退操作
        move_wp_off_on_floor();
        rt_thread_mdelay(1000);

        void (*pf_move)(int speed) = RT_NULL;
        pf_move = (cur_pitch > 0.0f) ? move_forward_with_speed : move_backward_with_speed;
        pf_move(p_params->forward_speed);

        const int RUN_TIME_MAX = 5 * 1000; // 5s
        rt_tick_t start_time = rt_tick_get_millisecond();
        while (1)
        {
            int flag1 = (fabs(get_current_pitch()) < IMU_COLLISION_SAFEZONE_PITCH); // 这里忽略了坡
            int flag2 = (rt_tick_get_millisecond() - start_time > RUN_TIME_MAX);    // 5s后结束
            if (flag1 || flag2)
            {
                break;
            }
        }
        move_wp_speed_on_floor_set(WP_SPEED);

#if 0
        // 再后退2秒
        float target_yaw = get_current_yaw();
        start_time = rt_tick_get_millisecond();
        while (rt_tick_get_millisecond() - start_time > 2 * 1000)
        {
            move_backward_with_pid(MOTOR_SPEED, get_current_yaw(), target_yaw);
            rt_thread_mdelay(PID_AJUST_TIME);
        }
        move_stop();
#endif
        result = ACTION_END_ON_WALL_THEN_DOWN; // 执行过程中遇到坡，结束
    }
#endif
    move_stop();
    return result;
}
#endif

#if 0
static int _is_cross_plane(float cur_pitch, float start_pitch)
{
    int ret = 0;
    if (fabs(cur_pitch - start_pitch) > 10.0f)
    {
        LOG_D("cross plane cur_pitch=%f,start_pitch=%f", cur_pitch, start_pitch);
        ret = 1;
    }
    return ret;
}
#endif

#if 0
static int _leaves_handler1(int enable, float start_yaw)
{
    const floor_params_t p_params = _get_floor_params(0);
    int leaves_flag = 0;
    float cur_yaw;

    if (enable == 0)
    {
        return -1;
    }

    leaves_flag = is_find_leaves(10);
    if (leaves_flag == 1)
    {
        LOG_I("find leaves...");
        move_stop();
        led_show_robot_action(ROBOT_ACTION_CLEAN_LEAVES);
        rt_thread_mdelay(500);

        // 左转，前进8秒；后退8秒；
        move_rotate_on_floor(20.0f, 20 * 1000);
        rt_thread_mdelay(3 * 1000);
        float dst_yaw = get_current_yaw();
        int speed = move_wp_speed_on_floor_get();
        move_wp_speed_on_floor_set(p_params->suck_leaves_wp_speed); // 水泵最大
        rt_thread_mdelay(1000);
        // 前进8秒
        unsigned int start_time = rt_tick_get();
        while (1)
        {
            if (rt_tick_get() - start_time > 8 * 1000)
            {
                break;
            }

            cur_yaw = get_current_yaw();
            move_forward_with_pid(p_params->suck_leaves_forward_speed, cur_yaw, dst_yaw);
        }
        move_stop_time(3 * 1000);
        // 后退8秒
        start_time = rt_tick_get();
        while (1)
        {
            if (rt_tick_get() - start_time > 8 * 1000)
            {
                break;
            }

            cur_yaw = get_current_yaw();
            move_backward_with_pid(p_params->suck_leaves_forward_speed, cur_yaw, dst_yaw);
        }
        move_wp_speed_on_floor_set(speed); // 恢复水泵
        move_stop_time(3 * 1000);

        // 右转，前进8秒；后退8秒；
        move_rotate_on_floor(-40.0f, 20 * 1000);
        rt_thread_mdelay(3 * 1000);
        dst_yaw = get_current_yaw();

        speed = move_wp_speed_on_floor_get();
        move_wp_speed_on_floor_set(p_params->suck_leaves_wp_speed); // 水泵最大
        rt_thread_mdelay(1000);
        // 前进8秒
        start_time = rt_tick_get();
        while (1)
        {
            if (rt_tick_get() - start_time > 8 * 1000)
            {
                break;
            }

            cur_yaw = get_current_yaw();
            move_forward_with_pid(p_params->suck_leaves_forward_speed, cur_yaw, dst_yaw);
        }
        move_stop_time(3 * 1000);
        // 后退8秒
        start_time = rt_tick_get();
        while (1)
        {
            if (rt_tick_get() - start_time > 8 * 1000)
            {
                break;
            }

            cur_yaw = get_current_yaw();
            move_backward_with_pid(p_params->suck_leaves_forward_speed, cur_yaw, dst_yaw);
        }
        move_wp_speed_on_floor_set(speed); // 恢复水泵
        move_stop_time(3 * 1000);
        // 中，前进8秒；
        move_rotate_on_floor_using_target(start_yaw, 20 * 1000);
        move_stop_time(3 * 1000);

        speed = move_wp_speed_on_floor_get();
        move_wp_speed_on_floor_set(p_params->suck_leaves_wp_speed); // 水泵最大
        rt_thread_mdelay(1000);
        start_time = rt_tick_get();
        while (1) // 降速
        {
            if (rt_tick_get() - start_time > p_params->suck_leaves_time_max)
            {
                break;
            }

            cur_yaw = get_current_yaw();
            move_forward_with_pid(p_params->suck_leaves_forward_speed, cur_yaw, start_yaw);
        }
        move_wp_speed_on_floor_set(speed); // 恢复水泵
        rt_thread_mdelay(3 * 1000);
    }

    return 0;
}
#endif

/**
 * @brief 前进/后退一段时间,遇障或超时退出
 *
 * @param dir 1 - forward, -1 - backward
 * @param speed
 * @param target_yaw
 * @param time_max
 * @return int 运动用时(前24位) + 停止原因(后8位)
 */
static int _move_with_pid_and_time(int dir, int speed, float target_yaw, int time_max)
{
    const float slope_degree_thres = 30.0f; // TODO:增加统一管理池子相关参数

    int ret = 0, status = 0;
    rt_uint8_t label;
    rt_uint16_t dis;
    float slope_degree, cur_yaw;
	struct leaves_info info;

    void (*move_func)(int, float, float) = (dir == 1) ? move_forward_with_pid : move_backward_with_pid;
    (dir == 1) ? LOG_I("forward %d", speed) : LOG_I("backward %d", speed);
	
	

    rt_tick_t start_time = rt_tick_get(), cur_tick, sensor_tick;
    while (1)
    {
        rt_tick_t cur_time = rt_tick_get();
        if (cur_time - start_time > time_max)
        {
            ret = cur_time - start_time;
            LOG_I("move_with_pid_time timeout %d", ret);
            status = 0;
            break;
        }
		
		rt_err_t ret_d = left_middle_right_leaves_info(&info);
		if (ret_d != RT_EOK)
		{
		   LOG_I("drain detect error %d", ret_d);
		}
		
		//检测到地漏
		if(info.object == DRAIN)
		{
			cur_time = rt_tick_get();
			ret = cur_time - start_time;
			LOG_I("Encounter a drain %d", ret);
			status = 0;	//若不是0，则会导致机器掉头
			break;
		}

        // 提前发现了墙
        cur_tick = rt_tick_get_millisecond();
        if (dir == 1)
        {
            get_distance_fusion(&sensor_tick, &label, &dis); // TODO:这个接口需要优化
                                                             //  这个函数只能在前进时使用，不能在后退中使用；
            if (label == 1 && (abs(cur_tick - sensor_tick) < 4000)) // 数据可信且新
            {
                if (dis < EDGE_THRES)
                {
                    cur_time = rt_tick_get();
                    ret = cur_time - start_time;
                    LOG_I("move_with_pid_time hit wall %d", ret);
                    status = 1;
                    sg_move_detect_dis = dis;
                    break;
                }
            }
        }

        // 或是上了墙或坡
        slope_degree = get_current_slope();
        if (fabs(slope_degree) > slope_degree_thres)    //TODO:有可能发现坡的时机错过，因为采样时间问题
        {
            cur_time = rt_tick_get();
            ret = cur_time - start_time;
            LOG_I("move_with_pid_time hit slope %d", ret);
            status = 2;
            break;
        }

        // TODO:CONDSIDER WALK ALONG THE WALL CONDITIONS
        cur_yaw = get_current_yaw();
        move_func(speed, cur_yaw, target_yaw);
    }
    move_stop();
    ret = (ret << TIME_OFFSET);
    ret |= (status & 0xFF);
    return ret;
}
#if 0
static int _drain_handler1(void)
{
    const int wp_speed = 2500;
    const int forward_speed = 8000;
    const int forward_time = 5000;

    LOG_D("start drain handler 1");
    // 准备
    move_stop_time(500);
    int floor_wp = move_wp_speed_on_floor_get();
    float dst_yaw = get_current_yaw();
    led_show_robot_action(ROBOT_ACTION_CLEAN_LEAVES);
    // 前进
    move_wp_speed_on_floor_set(wp_speed);
    rt_thread_mdelay(3000);
    move_forward_with_pid_and_time(forward_speed, dst_yaw, forward_time);
    move_stop_time(500);
    // 结束
    move_wp_speed_on_floor_set(floor_wp);
    rt_thread_mdelay(3000);
    LOG_D("end drain handler 1");

    return forward_time;
}
#endif

static int _drain_handler2(int left, int right)
{
    const floor_params_t p_params = _get_floor_params(0);
    const int forward_time = 1500;
    
    if(left != 1 && right != 1)
        return 0;
    // 准备
    LOG_I("start drain handler 2");
    move_stop_time(500);
    led_show_robot_action(ROBOT_ACTION_CLEAN_LEAVES);
    // 绕过地漏
    if(left == 1)// 地漏在左侧
    {
        // 右旋转90度
        LOG_I("rotate right");
        move_rotate_on_floor(-90.0f, p_params->rotate_time_max);
        rt_thread_mdelay(500);
        // 前进半个机身
        LOG_I("forward half body");
        move_forward_with_speed_and_time(p_params->forward_speed, forward_time);
        rt_thread_mdelay(500);
        //左旋转90度
        LOG_I("rotate left");
        move_rotate_on_floor(90.0f, p_params->rotate_time_max);
        rt_thread_mdelay(500);
    }
    else if(right == 1)// 地漏在右侧
    {
        // 左旋转90度
        LOG_I("rotate left");
        move_rotate_on_floor(90.0f, p_params->rotate_time_max);
        rt_thread_mdelay(500);
        // 前进半个机身
        LOG_I("forward half body");
        move_forward_with_speed_and_time(p_params->forward_speed, forward_time);
        rt_thread_mdelay(500);
        //右旋转90度
        LOG_I("rotate right");
        move_rotate_on_floor(-90.0f, p_params->rotate_time_max); 
        rt_thread_mdelay(500);
    }
    LOG_I("end drain handler 2");

    return 0;
}

static int _dy_regin_handler2(int row_parity)
{
    struct region_info info = {0};
    rt_uint8_t id_thd = 2; // 重度脏污才计入
    rt_err_t ret = get_srf_dy_regin_info(&info, id_thd);
    if (ret != RT_EOK)
    {
        LOG_I("get dirty info error!");
        return -1;
    }
    LOG_I("id_thd=%d, left=%d, middle=%d, right=%d", id_thd, info.left, info.middle, info.right);
    
    if (info.left == 0 && info.middle == 0 && info.right == 0)
    {
        LOG_I("ahead no dirty...");
        return 0;
    }

    sg_vision_info.type.bits.sand = 1;
    if (info.dirty_index == 1)
    {
        sg_vision_info.medium_dirty_times ++;
    }
    else if (info.dirty_index == 2)
    {
        sg_vision_info.heavy_dirty_times ++;
    }

    LOG_I("%d, get dirty area", row_parity);
    if (row_parity == 0) // 偶数行
    {
        if (info.right == 1)
        {
            LOG_I("right dirty");
            if_right_have_dirt = 1;
        }
        
    } else // 奇数行
    {
        // 判断左侧
        if (info.left == 1)
        {
            LOG_I("left dirty");
            if_right_have_dirt = 1;
        }
    }

    if (info.middle == 1)
    {
        LOG_I("dirty ahead");
        return 1;
    }

    LOG_I("clean ahead");
    return 0;
}

static int _leaves_handler2(int enable, float start_yaw, int row_parity,uint8_t* dirty_ignore)
{
    const floor_params_t p_params = _get_floor_params(0);
    struct leaves_info info = {.object = UNKNOWN, .left_have_leaves = 0, .middle_have_leaves = 0, .right_have_leaves = 0};
    float roate_angle = 0.0f;
    int count = 0, prv_wp_speed;
    int run_time = 0;
    int if_ahead_dy = 0;  // 前方是否脏污
    int wash_wp_speed;

    if (enable == 0)    //TODO:这个参数的外部实际意义
    {
        return -1;
    }

    rt_err_t ret = left_middle_right_leaves_info(&info);
    if (ret != RT_EOK)
    {
        return -2;
    }
    //TODO：增加距离传感器墙体识别，直接退出；
    float cur_slope = get_current_slope();
    if (cur_slope < -SLOPE_THRES_MAX)// 用坡度判断，上坡且度数大于30，停止。
    {
        LOG_I("leaves end method slope %.2f", cur_slope);
        return -4;
    }
    
    // 地漏优先处理
    if(info.object == DRAIN)
    {
        sg_vision_info.type.bits.drn = 1;
        sg_vision_info.drain_times ++;
        ret = _drain_handler2(info.left_have_leaves, info.right_have_leaves);
        // 纠一下yaw角
        move_rotate_on_floor_using_target(start_yaw, p_params->rotate_time_max);
        move_stop_time(500);
        return ret;
    }
    p_ide_branch_leaves_sta_t p_enbale_leaves = get_identify_leaves_sta();
    if (info.object == LEAVES && p_enbale_leaves->leaves == 0)
    {
        info.left_have_leaves = 0;
        info.middle_have_leaves = 0;
        info.right_have_leaves = 0;
    }
    
    LOG_I("start_yaw %f row_parity %d", start_yaw, row_parity);
    LOG_I("left_have_leaves=%d,middle_have_leaves=%d,right_have_leaves=%d", info.left_have_leaves, info.middle_have_leaves, info.right_have_leaves);

    wash_wp_speed = wf_act_params->clean_leaves_wp_speed;

    if (info.left_have_leaves == 0 && info.middle_have_leaves == 0 && info.right_have_leaves == 0)
    {
        LOG_I("no leaves...");
        // add one pid control
        move_forward_with_pid(wf_act_params->forward_wp_speed, get_current_yaw(), start_yaw);
        if_ahead_dy = _dy_regin_handler2(row_parity);
        wash_wp_speed = wf_act_params->forward_clean_sediment_wp_speed;

        if (if_ahead_dy != 1)
        {
            return -3;
        } 
        else
        {
            if_ahead_dy = _dy_regin_handler2(row_parity);
            if (if_ahead_dy != 1)
            {   
                return -3;
            }
            else
            {
                LOG_D("forward dirty.");
                get_dirty_tick = rt_tick_get();
                LOG_I("get dirty tick=%d", get_dirty_tick);
            }
        }
    }

    LOG_D("wash_wp_speed=%d, wash_sediment_mtr_speed=%d", wash_wp_speed, wf_act_params->forward_clean_sediment_mtr_speed);

    if(info.middle_objects_num == 0 && ((info.left_objects_num <=2 && info.right_objects_num == 0) || (info.left_objects_num == 0 && info.right_objects_num <=2)))
    {
        *dirty_ignore = 1;
    }
    else
        *dirty_ignore = 0;

    LOG_I("find leaves:%d,%d,%d|%d",info.left_objects_num,info.middle_objects_num,info.right_objects_num,*dirty_ignore);
        
    sg_vision_info.type.bits.lv = 1;
    sg_vision_info.leaves_times ++;
    move_stop();
    led_show_robot_action(ROBOT_ACTION_CLEAN_LEAVES);
    rt_thread_mdelay(500);

    // 水泵最大
    prv_wp_speed = move_wp_speed_on_floor_get();
    move_wp_speed_on_floor_set(wash_wp_speed);  //TODO:要改成慢速向上拉，为了将IMU的影响变得更soft
    // rt_thread_mdelay(3000);
    rt_thread_mdelay(1000);

    // 纠一下yaw角
    // move_rotate_on_floor_using_target(start_yaw, p_params->rotate_time_max);
    // move_stop_time(500);

    //TODO：增加陡坡度或墙体识别，不支持鸡爪；
    // 偶数行左转，奇数行右转
    roate_angle = (row_parity == 0) ? p_params->suck_leaves_rotate_thres : (0 - p_params->suck_leaves_rotate_thres);
    if (row_parity == 0) // 偶数行
    {
        // 先判左,再判右
        if (info.left_have_leaves == 1)
        {
            LOG_I("left clean...");
            /////////////////// 先左 ///////////////////
        _start1:
            move_rotate_on_floor(roate_angle, p_params->rotate_time_max);   
            rt_thread_mdelay(500);
            float dst_yaw = get_current_yaw();

            // 前进8s
            run_time = _move_with_pid_and_time(1, wf_act_params->clean_leaves_mtr_speed, dst_yaw, p_params->suck_leaves_time_max);
            move_stop_time(500);
            // TODO:增加坡度识别
            // TODO:增加一个统一rotate接口，无论是坡上，墙上，地上，都能用
            // 纠一下yaw角
            move_rotate_on_floor_using_target(dst_yaw, p_params->rotate_time_max);
            move_stop_time(500);
            // 后退8秒
            run_time = (run_time >> TIME_OFFSET);
            _move_with_pid_and_time(-1, wf_act_params->clean_leaves_mtr_speed, dst_yaw, run_time);
            move_stop_time(500);

            // 更新下一个动作的角度
            roate_angle *= 2.0f;
            ///////////////////// 后右 ///////////////////
            if (count == 0)
            {
                roate_angle = 0 - roate_angle;
                count++;
                LOG_I("right clean...");
                goto _start1;
            }

            ///////////////////// 再中间 ///////////////////
        }
        else if (info.right_have_leaves == 1)
        {
            ///////////////////// 先右 ///////////////////
            LOG_I("right clean...");
            if_right_have_dirt = 1;
            LOG_I("even, if_right_have_dirt=%d", if_right_have_dirt);
            roate_angle = 0 - roate_angle;

            move_rotate_on_floor(roate_angle, p_params->rotate_time_max);
            rt_thread_mdelay(500);
            float dst_yaw = get_current_yaw();

            // 前进8s
            run_time = _move_with_pid_and_time(1, wf_act_params->clean_leaves_mtr_speed, dst_yaw, p_params->suck_leaves_time_max);
            move_stop_time(500);
            // 纠一下yaw角
            move_rotate_on_floor_using_target(dst_yaw, p_params->rotate_time_max);
            move_stop_time(500);
            // 后退8秒
            run_time = (run_time >> TIME_OFFSET);
            _move_with_pid_and_time(-1, wf_act_params->clean_leaves_mtr_speed, dst_yaw, run_time);
            move_stop_time(500);

            ///////////////////// 再中间 ///////////////////
        }
    }
    else // 奇数行
    {
        // 先判右,再判左
        if (info.right_have_leaves == 1)
        {
            LOG_I("right clean...");
            /////////////////// 先右 ///////////////////
        _start2:
            move_rotate_on_floor(roate_angle, p_params->rotate_time_max);
            rt_thread_mdelay(500);
            float dst_yaw = get_current_yaw();

            // 前进8s
            run_time = _move_with_pid_and_time(1, wf_act_params->clean_leaves_mtr_speed, dst_yaw, p_params->suck_leaves_time_max);
            move_stop_time(500);
            // 纠一下yaw角
            move_rotate_on_floor_using_target(dst_yaw, p_params->rotate_time_max);
            move_stop_time(500);
            // 后退8秒
            run_time = (run_time >> TIME_OFFSET);
            _move_with_pid_and_time(-1, wf_act_params->clean_leaves_mtr_speed, dst_yaw, run_time);
            move_stop_time(500);

            // 更新下一个动作的角度
            roate_angle *= 2.0f;
            ///////////////////// 后左 ///////////////////
            if (count == 0)
            {
                roate_angle = 0 - roate_angle;
                count++;
                LOG_I("left clean...");
                goto _start2;
            }

            ///////////////////// 再中间 ///////////////////
        }
        else if (info.left_have_leaves == 1)
        {
            // 先左
            LOG_I("left clean...");
            if_right_have_dirt = 1;
            LOG_I("odd, if_right_have_dirt=%d", if_right_have_dirt);
            roate_angle = 0 - roate_angle;

            move_rotate_on_floor(roate_angle, p_params->rotate_time_max);
            rt_thread_mdelay(500);
            float dst_yaw = get_current_yaw();

            // 前进8s
            run_time = _move_with_pid_and_time(1, wf_act_params->clean_leaves_mtr_speed, dst_yaw, p_params->suck_leaves_time_max);
            move_stop_time(500);
            // 纠一下yaw角
            move_rotate_on_floor_using_target(dst_yaw, p_params->rotate_time_max);
            move_stop_time(500);
            // 后退8秒
            run_time = (run_time >> TIME_OFFSET);
            _move_with_pid_and_time(-1, wf_act_params->clean_leaves_mtr_speed, dst_yaw, run_time);
            move_stop_time(500);

            // 后中间
        }
    }

    // 中，前进8秒；
    if ((info.left_have_leaves == 1 || info.middle_have_leaves == 1 || info.right_have_leaves == 1)||(if_ahead_dy == 1))
    {
        LOG_I("middle clean...");
        move_rotate_on_floor_using_target(start_yaw, p_params->rotate_time_max);
        move_stop_time(500);
        if ((info.left_have_leaves == 1 || info.middle_have_leaves == 1 || info.right_have_leaves == 1))
        {
            // TODO:wf_act_params参数在EEPROM损坏的情况下，有没有默认值？
            run_time = _move_with_pid_and_time(1, wf_act_params->clean_leaves_mtr_speed, start_yaw, p_params->suck_leaves_time_max);
        } else
        {
            run_time = _move_with_pid_and_time(1, wf_act_params->forward_clean_sediment_mtr_speed, start_yaw, p_params->suck_leaves_time_max);
        }
        
        move_stop_time(500);
        LOG_I("middle clean...end");
    }

    move_wp_speed_on_floor_set(prv_wp_speed); // 恢复水泵
    // rt_thread_mdelay(3 * 1000);
    rt_thread_mdelay(1000);

    // 纠一下yaw角
    move_rotate_on_floor_using_target(start_yaw, p_params->rotate_time_max);
    move_stop_time(500);
    
    return run_time;
}

#if 0
static int process_baseon_dirty_classify(int row_parity)
{
    struct region_info info = {0};
    rt_uint8_t id_thd = 2; // 重度脏污才计入
    rt_err_t ret = get_srf_dy_regin_info(&info, id_thd);
    if (ret != RT_EOK)
    {
        LOG_D("get dirty info error!");
        return -1;
    }

    LOG_D("left=%d,middle=%d,right=%d", info.left, info.middle, info.right);

    if (info.left == 0 && info.middle == 0 && info.right == 0)
    {
        LOG_D("no dirty...");
        return -2;
    }
    LOG_D("%d, get dirty area", row_parity);
    if (row_parity == 0) // 偶数行
    {
        if (info.right == 1)
        {
            LOG_D("right dirty");
            if_right_have_dirt = 1;
        }
        
    } else // 奇数行
    {
        // 判断左侧
        if (info.left == 1)
        {
            LOG_D("left dirty");
            if_right_have_dirt = 1;
        }
    }
    
    LOG_D("if_right_have_dirt=%d", if_right_have_dirt);
    return 0;
} 
#endif

static int judge_diff_bigger(float cur_diff, float pre_diff) { 
    int if_same_sign = (cur_diff * pre_diff >0) ? 1 :0;
    LOG_D("if_same_sign=%d", if_same_sign);

    int if_bigger_than_thres;
    if ((fabs(cur_diff) > fabs(pre_diff)) && (fabs(pre_diff) > JUDGE_DRAIN_YAW_DIFF))
    {
        if_bigger_than_thres = 1;
    } else {
        if_bigger_than_thres = 0;
    }
    LOG_D("if_bigger_than_thres=%d", if_bigger_than_thres);

    if (if_same_sign==1 && if_bigger_than_thres==1)
    {
        return 1;
    } else
    {
        return 0;
    }
}


int actions_on_floor_move_forward_to_obstacle(void *p_arg_in, void *p_arg_out)
{
    RT_ASSERT(p_arg_in != RT_NULL);
    RT_ASSERT(p_arg_out != RT_NULL);

    int result = ACTION_END_SUCCESS;
    const floor_params_t p_params = _get_floor_params(0);
    const float edge_pitch_thres = -15.0f;
    const int DIVIDER = 10;
    RT_UNUSED(edge_pitch_thres);

    // const int RUN_TIME_MAX = 3 * 60 * 1000;
    const int RUN_TIME_MAX = ((move_forward_info_t)p_arg_in)->time_out;
    float start_yaw = ((move_forward_info_t)p_arg_in)->start_yaw;
    unsigned char change_speed_flag = ((move_forward_info_t)p_arg_in)->if_change_speed;
    int row_parity = ((move_forward_info_t)p_arg_in)->if_odd_row;
    int find_slope_time = ((move_forward_info_t)p_arg_in)->find_slope_time;
    wf_act_params = ((move_forward_info_t)p_arg_in)->params;
    if (wf_act_params!=NULL)
    {
        LOG_I("clean_leaves_wp_speed=%d", wf_act_params->clean_leaves_wp_speed);
    }

    // int *end_dis = ((int *)p_arg_out);
    move_forward_action_return_info_t action_ret_info = (move_forward_action_return_info_t)p_arg_out;
    action_ret_info->forward_time = 0;
    action_ret_info->info.down_count = 0;
    action_ret_info->info.flat_count = 0;
    action_ret_info->info.up_count = 0;
    action_ret_info->act_get_dirty_time = 0;
    // dirty info
    rt_memset(&sg_vision_info, 0, sizeof(sg_vision_info));

    int start_comp_speed_dis = 0;
    int vision_clean_time = 0;
//    int ult_dis = -1,
    rt_uint16_t dis = 0;
    if_right_have_dirt = 0;
//    int ult_dis_buff[3] = {0};
//    int tof_dis_buff[3] = {0};
//    unsigned int ult_time_buff[3] = {0};
//    unsigned int tof_time_buff[3] = {0};
    int useless_time = 0, log_counter = 0;
    uint8_t sum_clean_time_ignore =0;

    LOG_I("start_yaw=%.2f", start_yaw);
    unsigned int timestamp_start = rt_tick_get(), speed_start_time;
    int prv_time = rt_tick_get_millisecond();
    float cur_yaw, start_pitch, cur_pitch, pre_yaw = get_current_yaw();
    LOG_I("pre_yaw=%.2f", pre_yaw);
    float pre_yaw_diff, cur_yaw_diff; 
    int diff_count = DIFF_COUNT;
    start_pitch = get_current_pitch();
    float start_roll = get_current_roll(), cur_roll;
    LOG_I("start_pitch=%.2f, start_roll=%.2f", start_pitch, start_roll);
    int cur_forward_time = 0;
    int start_state_time = 0, cur_state_time = 0, state_elapsed_time = 0;
    enum cur_line_info state = STATE_INITIAL;
    float cur_slope, state_start_slope = get_current_slope(), pre_slope;
    pre_slope = state_start_slope;
    LOG_I("state_start_slope=%.2f", state_start_slope);

    pre_yaw_diff = compare_yaws(get_current_yaw(), start_yaw);
    LOG_I("pre_yaw_diff=%.2f", pre_yaw_diff);
    
    while (1)  //TODO:测试一下loop一次，用时除掉视觉用时，改用数据混合来获得数据，减少延时
    {
        log_counter %= DIVIDER;
        cur_forward_time = rt_tick_get_millisecond();
        // 如果pitch绝对值大于阈值，认为机器上墙了，立马break
        cur_pitch = get_current_pitch();
        cur_roll = get_current_roll();
        cur_slope = get_current_slope();
        if (log_counter == 0)
        {
            LOG_D("start_roll=%f, cur_roll=%f, cur_pitch=%f", start_roll, cur_roll, cur_pitch);
            LOG_D("cur_slope=%f", cur_slope);
        }
        if (row_parity == 0) // 偶数行
        {
            if (cur_roll > SLOPE_THRES_MIN) // 左轮抬起，在下坡上行走
            {
                action_ret_info->info.down_count += 1;
            }
            else if (cur_roll < -SLOPE_THRES_MIN) // 右轮抬起，在上坡上行走
            {
                action_ret_info->info.up_count += 1;
            }
            else // 平地行走
            {
                action_ret_info->info.flat_count += 1;
            }
        }
        else // 奇数行
        {
            if (cur_roll > SLOPE_THRES_MIN) // 左轮抬起，在上坡上
            {
                action_ret_info->info.up_count += 1;
            }
            else if (cur_roll < -SLOPE_THRES_MIN) // 右轮抬起，在下坡上
            {
                action_ret_info->info.down_count += 1;
            }
            else // 平地行走
            {
                action_ret_info->info.flat_count += 1;
            }
        }

        if (log_counter == 0)
        {
            LOG_D("info, up %d, down %d, flat %d", action_ret_info->info.up_count,
                  action_ret_info->info.down_count, action_ret_info->info.flat_count);
        }

        // 用坡度判断，上坡且度数大于30，停止。
        if (cur_slope < -SLOPE_THRES_MAX)
        {
            LOG_I("end method slope %.2f", cur_slope);
            // *end_dis = tof_dis;
            action_ret_info->end_dis = dis;
            action_ret_info->ret_idx = 2;
            result = ACTION_END_TO_WALL;
            break;
        }
        if (row_parity == 0) // 偶数行
        {
            /* 右轮抬起 */
            if (cur_roll < -SLOPE_THRES_MAX)
            {
                LOG_I("even line, end method roll. %.2f", cur_roll);
                // *end_dis = tof_dis;
                action_ret_info->end_dis = dis;
                action_ret_info->ret_idx = 3;
                result = ACTION_END_TO_WALL;
                break;
            }
        } else // 奇数行
        {
            /* 左轮抬起 */
            if (cur_roll > SLOPE_THRES_MAX)
            {
                LOG_I("odd line, end method roll %.2f", cur_roll);
                // *end_dis = tof_dis;
                action_ret_info->end_dis = dis;
                action_ret_info->ret_idx = 3;
                result = ACTION_END_TO_WALL;
                break;
            }
        }
        
        switch (state)
        {
        case STATE_INITIAL:
            if ((cur_slope >= SLOPE_THRES_MIN) && (cur_slope < SLOPE_THRES_MAX))
            {
                state = GROUND_DOWNWARD;
                LOG_D("state up to GROUND_DOWNWARD");
                start_state_time = rt_tick_get_millisecond();
                state_start_slope = cur_slope;
            }
            else
            {
                pre_slope = cur_slope;
            }
            break;
        case GROUND_DOWNWARD:
            if ((cur_slope >= SLOPE_THRES_MIN) && (cur_slope < SLOPE_THRES_MAX))
            {
                if (fabs(state_start_slope - cur_slope) < 5.0f)
                {
                    cur_state_time = rt_tick_get_millisecond();
                    state_elapsed_time = cur_state_time - start_state_time;
                    pre_slope = cur_slope;

                    if (state_elapsed_time > SUSTAIN_TIME_S)
                    {
                        state = GROUND_FLAT;
                        start_state_time = rt_tick_get_millisecond(); // 重置开始时间
                        state_start_slope = cur_slope;                // 重置参考slope
                        LOG_D("state up to GROUND_FLAT");
                    }
                }
                else
                {
                    state = STATE_INITIAL;
                }
            }
            else
            {
                state = STATE_INITIAL;
            }
            break;
        case GROUND_FLAT:
            if (fabs(cur_slope) < SLOPE_THRES_MIN || (fabs(cur_slope - pre_slope) > SLOPE_THRES_MIN))
            {
                cur_state_time = rt_tick_get_millisecond();
                state_elapsed_time = cur_state_time - start_state_time;

                if (state_elapsed_time > SUSTAIN_TIME_S)
                {
                    state = GROUND_UPWARD;
                    start_state_time = rt_tick_get_millisecond(); // 重置开始时间
                    pre_slope = cur_slope;
                    LOG_D("state up to GROUND_UPWARD");
                }
            }
            else if (cur_slope < -SLOPE_THRES_MIN)
            {
                state = GROUND_UPWARD;
                cur_state_time = rt_tick_get_millisecond();
                pre_slope = cur_slope;
            }
            else
            {
                start_state_time = rt_tick_get_millisecond(); // 重置开始时间
            }
            break;
        case GROUND_UPWARD:
            if ((cur_slope < -SLOPE_THRES_MIN) && (fabs(cur_slope - pre_slope) > SLOPE_THRES_MIN))
            {
                cur_state_time = rt_tick_get_millisecond();
                state_elapsed_time = cur_state_time - start_state_time;

                if (state_elapsed_time > SUSTAIN_TIME_S)
                {
                    state = GROUND_GET_SLOPE;
                    LOG_D("state up to GROUND_GET_SLOPE");
                }
            }
            else
            {
                start_state_time = rt_tick_get_millisecond(); // 重置开始时间
            }
            break;
        case GROUND_GET_SLOPE:
            break;
        }
        LOG_D("state_start_slope=%f, start_state_time=%d", state_start_slope, start_state_time);
        if ((find_slope_time == 0) && (state > GROUND_DOWNWARD))
        {
            LOG_I("state:%d,find slope, break action", state);
            break;
        }

        action_ret_info->forward_time += rt_tick_get_millisecond() - cur_forward_time;

        rt_tick_t start_time = rt_tick_get_millisecond(), end_time;
        int ret = _leaves_handler2(change_speed_flag, start_yaw, row_parity,&sum_clean_time_ignore); // TODO:调用的位置不合适, 考虑接近墙边或坡的情况
        end_time = rt_tick_get_millisecond() - start_time;
        if (ret > 0)
        {
            int time = (ret >> TIME_OFFSET);
            int flag = (ret & 0xFF);
            if_right_have_dirt = 1; //TODO:参数策略的变化要同步一下
            vision_clean_time += 1;
            useless_time += (end_time - time);
            LOG_I("vision_clean_time=%d, useless_time=%d | %d", vision_clean_time, useless_time,sum_clean_time_ignore);
            if (flag != 0)
            {
                LOG_I("end method vision encounter obstacle %d", flag);
                if (flag == 1) // near the wall
                {
                    action_ret_info->ret_idx = 0;
                    action_ret_info->end_dis = sg_move_detect_dis;
                }
                else if (flag == 2) // encountter slope
                {
                    action_ret_info->ret_idx = 2;
                }
                else
                {
                }
                break;
            }
        }
        // // 未清洗区域有重度脏污，记入标置位。控制下一行不跳行——可以合入前方脏污处理中。
        // process_baseon_dirty_classify(row_parity);
        

        //TODO:调用新的融合数据接口，replace old code segment --add1
#if 0       
        int cur_time = rt_tick_get_millisecond();
        if (cur_time - prv_time > 200) // 200ms 查询一次tof
        {
            prv_time = rt_tick_get_millisecond();
            tof_dis = get_current_distance_from_laser();
            int tof_valid = _is_laser_data_valid(tof_dis);
            if (tof_valid)
            {
                LOG_D("tof_dis_buff[0]=%d", tof_dis_buff[0]);
                // 判断buff里的数据
                if (tof_dis_buff[0] > 0)
                {
                    if ((tof_dis_buff[0] - tof_dis < 15) || (tof_dis_buff[0] - tof_dis > 2000)) // 如果没有连续减小的距离值，则认为上一距离失效
                    {
                        tof_dis_buff[0] = tof_dis;
                        tof_time_buff[0] = rt_tick_get();
                    }
                    else
                    {
                        if (start_comp_speed_dis == 0)
                        {
                            start_comp_speed_dis = tof_dis_buff[0];
                            speed_start_time = tof_time_buff[0];
                        }

                        if (tof_dis < EDGE_THRES)
                        {
                            // *end_dis = tof_dis;
                            action_ret_info->end_dis = tof_dis;
                            LOG_D("end method tof");
                            action_ret_info->forward_time += rt_tick_get_millisecond() - cur_time;
                            action_ret_info->ret_idx = 0;
                            break;
                        }

                        tof_dis_buff[0] = tof_dis;
                        tof_time_buff[0] = rt_tick_get();
                    }
                }
                else
                {
                    tof_dis_buff[0] = tof_dis;
                    tof_time_buff[0] = rt_tick_get();
                }
            }
            else
            {
                tof_dis_buff[0] = 0;
                tof_time_buff[0] = rt_tick_get();
            }
        }

        ult_dis = get_current_distance_from_ultrasonic();

        if (rt_tick_get() - timestamp_start > RUN_TIME_MAX)
        {
            LOG_D("end method tiemout,tof=%d ult=%d", tof_dis, ult_dis);
            // *end_dis = tof_dis;
            action_ret_info->end_dis = tof_dis;
            result = ACTION_END_TIMEOUT;
            action_ret_info->forward_time += rt_tick_get_millisecond() - cur_time;
            action_ret_info->ret_idx = 1;
            break;
        }

        int ult_valid = _is_ult_data_valid(ult_dis);

        if (ult_valid) // 测到有效数据
        {
            // LOG_D("ult_dis_buff[0]=%d\n", ult_dis_buff[0]);
            // 判断buff里的数据
            if (ult_dis_buff[0] > 0)
            {
                if ((ult_dis_buff[0] - ult_dis < 15) || (ult_dis_buff[0] - ult_dis > 2000)) // 如果没有连续减小的距离值，则认为上一距离失效
                {
                    ult_dis_buff[0] = ult_dis;
                    ult_time_buff[0] = rt_tick_get();
                }
                else
                {
                    if (start_comp_speed_dis == 0)
                    {
                        start_comp_speed_dis = ult_dis_buff[0];
                        speed_start_time = ult_time_buff[0];
                    }

                    if (ult_dis < EDGE_THRES)
                    {
                        // *end_dis = ult_dis;
                        action_ret_info->end_dis = ult_dis;
                        LOG_D("end method ult\n");
                        action_ret_info->forward_time += rt_tick_get_millisecond() - cur_time;
                        action_ret_info->ret_idx = 0;
                        break;
                    }

                    ult_dis_buff[0] = ult_dis;
                    ult_time_buff[0] = rt_tick_get();
                }
            }
            else
            {
                ult_dis_buff[0] = ult_dis;
                ult_time_buff[0] = rt_tick_get();
            }
        }
        else
        {
            ult_dis_buff[0] = 0;
            ult_time_buff[0] = rt_tick_get();
        }
#else
        int cur_time = rt_tick_get_millisecond();
        rt_tick_t cur_tick, sensor_tick;
        rt_uint8_t label;
        get_distance_fusion(&sensor_tick, &label, &dis);        // TODO:这个接口需要优化
        if (label == 1 && (abs(cur_tick - sensor_tick) < 4000)) // 数据可信且新
        {
            if (dis < EDGE_THRES)
            {
                action_ret_info->end_dis = dis;
                LOG_I("end method dis %d", dis);
                action_ret_info->forward_time += rt_tick_get_millisecond() - cur_time;
                action_ret_info->ret_idx = 0;
                break;
            }
        }

        if (rt_tick_get() - timestamp_start - useless_time > RUN_TIME_MAX)
        {
            LOG_I("end method tiemout,%d", dis);
            action_ret_info->end_dis = dis;
            result = ACTION_END_TIMEOUT;
            action_ret_info->forward_time += rt_tick_get_millisecond() - cur_time;
            action_ret_info->ret_idx = 1;
            break;
        }

#endif
        cur_yaw = get_current_yaw();
        LOG_D("get_current_yaw()=%f", cur_yaw);
        cur_yaw_diff = compare_yaws(cur_yaw, start_yaw);
        LOG_D("cur_yaw_diff=%f, pre_yaw_diff=%f, %d", cur_yaw_diff, pre_yaw_diff, diff_count);
        if (judge_diff_bigger(cur_yaw_diff, pre_yaw_diff))
        {
            diff_count -= 1;
            LOG_I("diff bigger, cur_yaw_diff=%f, pre_yaw_diff=%f", cur_yaw_diff, pre_yaw_diff);
        } else
        {
            diff_count = DIFF_COUNT;
            LOG_D("diff no bigger");
        }
        if (diff_count < 1) // 角度持续纠不过来，退出
        {
            LOG_I("end method yaw diff, yaw_diff=%f", cur_yaw_diff);
            // *end_dis = tof_dis;
            action_ret_info->end_dis = dis;
            action_ret_info->forward_time += rt_tick_get_millisecond() - cur_time;
            action_ret_info->ret_idx = 4;
            break;
        }
        pre_yaw_diff = cur_yaw_diff; 
        move_forward_with_pid(wf_act_params->forward_mtr_speed, cur_yaw, start_yaw);
        rt_thread_mdelay(PID_AJUST_TIME);
        action_ret_info->forward_time += rt_tick_get_millisecond() - cur_time;
        log_counter++;
    }
    LOG_I("info, up %d, down %d, flat %d", action_ret_info->info.up_count,
                  action_ret_info->info.down_count, action_ret_info->info.flat_count);
	action_ret_info->line_state = state;
    LOG_I("line state:%d, ret_state:%d", state, action_ret_info->line_state);
    action_ret_info->if_right_area_have_leaves = if_right_have_dirt;    // TODO:代码传值方式要优化；
    LOG_I("return if_right_area_have_leaves=%d", action_ret_info->if_right_area_have_leaves);
    
    unsigned int timestamp_end = rt_tick_get();
    int forward_time = timestamp_end - timestamp_start;
    unsigned int time = timestamp_end - speed_start_time;
    int speed = 0;

    // *end_dis = dis;
    LOG_I("end_dis=%d", action_ret_info->end_dis);
    action_ret_info->forward_time = forward_time - useless_time;
    LOG_I("time=%d", action_ret_info->forward_time);

    if ((action_ret_info->end_dis > 0) && (time > 0))
    {
        speed = (float)(start_comp_speed_dis - action_ret_info->end_dis) / time;
        LOG_I("speed=%d", speed);
    }
    action_ret_info->act_get_dirty_time = get_dirty_tick;
    if(sum_clean_time_ignore == 0)
        action_ret_info->vision_clean_time = vision_clean_time;

    action_ret_info->dirty_info->type = sg_vision_info.type;
    action_ret_info->dirty_info->leaves_times = sg_vision_info.leaves_times;
    action_ret_info->dirty_info->drain_times = sg_vision_info.drain_times;
    action_ret_info->dirty_info->medium_dirty_times = sg_vision_info.medium_dirty_times;
    action_ret_info->dirty_info->heavy_dirty_times = sg_vision_info.heavy_dirty_times;
    // LOG_D("forward_time=%d", forward_time);
    // LOG_D("forward_speed=%d", speed);

#if 0  // --k
    // 检查有没有跨平面
    LOG_D("check if cross palne");
    int count = 0;
    
    // 关闭后退 --k
    speed = 0;
    while (_is_cross_plane(cur_pitch, start_pitch) == 1)
    {
        move_backward_with_speed_and_time(speed, 2 * 1000);
        count++;
        if (count > 5)
        {
            LOG_D("move back timeout");
            break;
        }
    }

    // adjust yaw
    // --closed k
    move_rotate_on_floor_using_target(start_yaw, p_params->rotate_time_max);
#endif // 检查有没有跨平面

#if 0
    // 如果上墙，需要先下墙
    float cur_pitch = get_current_pitch();
    if (fabs(cur_pitch) > 45.0f)
    {
        // 关水泵,执行后退操作
        move_wp_off_on_floor();
        rt_thread_mdelay(1000);

        void (*pf_move)(int speed) = RT_NULL;
        pf_move = (cur_pitch > 0.0f) ? move_forward_with_speed : move_backward_with_speed;
        pf_move(p_params->forward_speed);

        const int RUN_TIME_MAX = 5 * 1000; // 5s
        rt_tick_t start_time = rt_tick_get_millisecond();
        while (1)
        {
            int flag1 = (fabs(get_current_pitch()) < IMU_COLLISION_SAFEZONE_PITCH); // 这里忽略了坡
            int flag2 = (rt_tick_get_millisecond() - start_time > RUN_TIME_MAX);    // 5s后结束
            if (flag1 || flag2)
            {
                break;
            }
        }
        move_wp_speed_on_floor_set(WP_SPEED);

#if 0
        // 再后退2秒
        float target_yaw = get_current_yaw();
        start_time = rt_tick_get_millisecond();
        while (rt_tick_get_millisecond() - start_time > 2 * 1000)
        {
            move_backward_with_pid(MOTOR_SPEED, get_current_yaw(), target_yaw);
            rt_thread_mdelay(PID_AJUST_TIME);
        }
        move_stop();
#endif
        result = ACTION_END_ON_WALL_THEN_DOWN; // 执行过程中遇到坡，结束
    }
#endif
    move_stop();
    return result;
}

int actions_on_floor_move_to_edge(void *p_arg_in, void *p_arg_out)
{
    const floor_params_t p_params = _get_floor_params(0);
    int result = ACTION_END_SUCCESS;

    int dis, pre_dis = 0;
    float cur_yaw;
    int cur_speed; // pwm
    float target_yaw = get_current_yaw();

    int count = 0;
    unsigned int time_start, time_out = 5000;
    time_start = rt_tick_get();
    move_forward_with_speed(4000);
    while (1)
    {
        get_current_dis(0, &dis);
        LOG_I("move_to_edge, dis=%d", dis);

        cur_yaw = get_current_yaw();
        if ((dis > 200) && (dis < 10000)) // 能获取到有效距离
        {
            LOG_I("count=%d dis=%d", count, dis);
            cur_speed = p_params->forward_speed - 500 * count;
            if (cur_speed < 3000)
            {
                cur_speed = 3000;
            }
            move_forward_with_pid(cur_speed, cur_yaw, target_yaw);
            rt_thread_mdelay(60);
            count++;
            pre_dis = dis;
        }
        else // 否则，认为到了墙边
        {
            if (pre_dis > 0)
            {
                LOG_I("move_to_edge, dis=%d", pre_dis);
                break;
            }
            else
            {
                pre_dis = dis;
                if (rt_tick_get() - time_start > time_out)
                {
                    LOG_I("move_to_edge timeout");
                    break;
                }
            }
        }

        float cur_pitch = get_current_pitch();
        if (fabs(cur_pitch) > 60.0f)
        {
            LOG_I("move_to_edge, pitch=%.2f", cur_pitch);
            break;
        }
    }

    move_stop();
#if 0
    float cur_pitch = get_current_pitch();
    if (fabs(cur_pitch) > 45.0f)
    {
        // 关水泵,执行后退操作
        move_wp_off_on_floor();
        rt_thread_mdelay(1000);

        void (*pf_move)(int speed) = RT_NULL;
        pf_move = (cur_pitch > 0.0f) ? move_forward_with_speed : move_backward_with_speed;
        pf_move(p_params->forward_speed);

        const int RUN_TIME_MAX = 5 * 1000; // 5s
        rt_tick_t start_time = rt_tick_get_millisecond();
        while (1)
        {
            int flag1 = (fabs(get_current_pitch()) < IMU_COLLISION_SAFEZONE_PITCH); // 这里忽略了坡
            int flag2 = (rt_tick_get_millisecond() - start_time > RUN_TIME_MAX);    // 5s后结束
            if (flag1 || flag2)
            {
                break;
            }
        }
        move_wp_speed_on_floor_set(WP_SPEED);
        result = ACTION_END_ON_WALL_THEN_DOWN; // 执行过程中遇到坡，结束
    }
#endif
    move_stop();
    return result;
}

static int forward_down_slope(float start_yaw)
{
    float cur_slope = get_current_slope();
    float start_slope = cur_slope;
    LOG_I("cur_slope=%f", cur_slope);
    
    
    // 如果大于20度，前进下坡
    if (start_slope > 20.0f)
    {
        LOG_I("forward down slope:%.2f", start_slope);
        // 前进到平面上
        rt_tick_t forward_time = 5*1000;
        int count = 0;

        // 如果在墙上，先下墙
        float cur_pitch = get_current_pitch();
        LOG_I("cur_pitch=%.2f", cur_pitch);
        while (cur_pitch > SLOPE_THRES_MAX)
        {
            move_forward_with_speed_and_time(WASH_FLOOR_MOTOR_SPEED, 1 * 1000);
            count++;
            if (count > 5)
            {
                LOG_I("down wall timeout");
                break;
            }
            cur_pitch = get_current_pitch();
            LOG_D("down wall, pitch=%.2f", cur_pitch);
        }		
        move_stop_time(500);
		LOG_I("down wall, pitch=%.2f", cur_pitch);

        rt_tick_t start_time = rt_tick_get_millisecond();
        rt_tick_t cur_time = start_time;
        while (cur_time - start_time < forward_time)
        {
            float cur_yaw = get_current_yaw();
            move_forward_with_pid(WASH_FLOOR_MOTOR_SPEED, cur_yaw, start_yaw);
            cur_slope = get_current_slope();
            LOG_D("forward, cur_yaw=%f, cur_slope=%f", cur_yaw, cur_slope);
            if (fabs(cur_slope) < 10.0f)
            {
                count += 1;
                // LOG_D("same slope as pre");
            } else
            {
                count = 0;
                LOG_I("update count=%d", count);
            }
            // LOG_D("count=%d", count);
            if (count > 5)
            {
                LOG_I("get plane.");
                break;
            }
            cur_time = rt_tick_get_millisecond();
            rt_thread_mdelay(100);
        } 
        move_stop_time(500);
    }
    return 0;
}

int actions_on_floor_move_to_next_line(void *p_arg_in, void *p_arg_out)
{
    if (!p_arg_in)
    {
        return -1;
    }

    int dir_left = ((move_to_next_row_info_t)(p_arg_in))->mode;
    int move_time = ((move_to_next_row_info_t)(p_arg_in))->move_time;
    int back_time = ((move_to_next_row_info_t)(p_arg_in))->back_time;
    int dirty_flag = ((move_to_next_row_info_t)(p_arg_in))->if_clean_dirt;
    if_uturn_slips = 0;  // 判断旋转后机器是否掉落，是：1；否：0

    const floor_params_t p_params = _get_floor_params(0);
    float bt_slope, at_slope;
    bt_slope = get_current_slope();
    LOG_I("before rotate, slope=%.2f", bt_slope); 
    int wp_speed = 0;
    // 先直行
    struct move_info move_info;
    move_info.start_yaw = get_current_yaw();
    move_info.move_time = move_time;
    move_info.move_speed = 5000;
    // actions_on_floor_move_forward_with_pid_and_time(&move_info, NULL);
    move_forward_with_speed_and_time(move_info.move_speed, move_info.move_time);
    LOG_I("move forward done");

    // detect slope
    float pitch = get_current_pitch();
    float roll = get_current_roll();
    if (fabs(pitch) > 7.0f || fabs(roll) > 7.0f)
    {
        LOG_I("p %f, r %f", pitch, roll);
        wp_speed = move_wp_speed_on_floor_get();
        move_wp_speed_on_floor_set(3500);
        rt_thread_mdelay(1000);
    }

    float delta = 0.0f;
    if (dir_left) // 向左转
    {
        LOG_I("rotate left");
        delta = 90.0f;
    }
    else // 向右转
    {
        LOG_I("rotate right");
        delta = -90.0f;
    }

    move_rotate_on_floor(delta, p_params->rotate_time_max);
    rt_thread_mdelay(500);
    float target_yaw = get_current_yaw();
    at_slope = get_current_slope();
    LOG_I("after rotate, slope=%f", at_slope);
    if ((bt_slope < 0) && (fabs(at_slope) < UTURN_DIFF_THRES) && (at_slope - bt_slope > UTURN_DIFF_THRES))
    {
        if_uturn_slips = 1;
    }
    LOG_I("u_turn, slips=%d", if_uturn_slips);

    // 后退到墙边
    LOG_I("back to wall");
    move_info.start_yaw = get_current_yaw();
    move_info.move_time = back_time;
    move_info.move_speed = 5000;
    move_info.dirty_flag = dirty_flag;
    actions_on_floor_move_backward_with_pid_and_time(&move_info, NULL);

    // 前进到平面上
    at_slope = get_current_slope();
    LOG_I("cur_slope=%f", at_slope);
    forward_down_slope(move_info.start_yaw);
    
    if (wp_speed != 0)
    {
        move_wp_speed_on_floor_set(wp_speed);
        rt_thread_mdelay(1000);
    }

    return 0;
}

int actions_on_floor_move_forward_with_pid_and_time(void *p_arg_in, void *p_arg_out)
{
    if (!p_arg_in)
    {
        return -1;
    }

    move_to_edge_prevent_climbing(RT_TRUE, (move_info_t)p_arg_in);
    return 0;
}

int actions_on_floor_move_backward_with_pid_and_time(void *p_arg_in, void *p_arg_out)
{
    if (!p_arg_in)
    {
        return -1;
    }

    move_to_edge_prevent_climbing(RT_FALSE, (move_info_t)p_arg_in);
    return 0;
}

int actions_on_floor_search_wall(void *p_arg_in, void *p_arg_out)
{
    extern int actions_on_wall_find_entry(void *p_arg_in, void *p_arg_out);
    return actions_on_wall_find_entry(p_arg_in, p_arg_out);
}

static int actions_on_slope_find_slope_top(void *p_arg_in, void *p_arg_out)
{
    const int slope_min = 5, slope_max = 30, wall_angle = 40;
    const int target_timeout = 20*1000;
    const int forward_speed = 8000, backward_speed = 5000;

    struct slope_type slope;
    float start_yaw;
    int ret = 0;

    /* 判断是否在缓坡上 */
    get_current_slope_info(&slope);
    if (slope.angle < slope_min || slope.angle > slope_max) 
    {
        LOG_E("is not on the slope!!!");
        return -RT_ERROR;
    }
    /* 旋转机器垂直朝上 */
    move_rotate_on_slope_using_target(180, target_timeout);
    rt_thread_mdelay(1000);

    start_yaw = get_current_yaw();
    LOG_I("start_yaw = %.2f", start_yaw);

    /* 前进找浅水区 */
    while(1)
    {
        get_current_slope_info(&slope);
        if (slope.angle < slope_min) /* 坡顶为浅水区*/
        {
            LOG_I("the top of slope is flat, slope angle %.2f", slope.angle);
            move_stop_time(1000);
            break;
        }
        else if (slope.angle > wall_angle) /* 坡顶为墙 */
        {
            LOG_I("the top of slope is wall");
            move_stop_time(1000);
            return -RT_ERROR;
        }
        move_forward_with_pid(forward_speed, slope.euler_yaw, start_yaw);
        rt_thread_mdelay(100);
    }

    rt_tick_t start_tick = rt_tick_get_millisecond();
    /* 后退到缓坡上 */
    while(1)
    {
        get_current_slope_info(&slope);
        /* 找到坡顶 */
        if (slope.angle > slope_min && slope.angle < slope_max)
        {
            LOG_I("backward find slope, slope angle %.2f", slope.angle);
            move_time_before_stop(2000);
            rt_thread_mdelay(1000);
            move_rotate_on_slope_using_target(90, target_timeout);
            rt_thread_mdelay(1000);
            break;
        }
        /* 超时 */
        if (rt_tick_get_millisecond() - start_tick > 10*1000)
        {
            move_stop_time(1000);
            ret = -RT_ERROR;
            LOG_E("backward timeout");
            break;
        }

        move_backward_with_pid(backward_speed, slope.euler_yaw, start_yaw);
        rt_thread_mdelay(100);
    }

    return ret;
}

int actions_on_floor_search_slope(void *p_arg_in, void *p_arg_out)
{
    // RT_ASSERT(p_arg_in != RT_NULL);
    RT_ASSERT(p_arg_out != RT_NULL);

    int *ret_val = (int *)p_arg_out;

    const int  backward_speed = 5000; //rpm
    uint16_t forward_speed = 8000, wp_speed = 3000;
    const int slope_min = 5, slope_max = 30, slope_change = 5, slope_dir_change = 10;// 度
    const int forward_timeout = 30000, rotate_timeout = 20000, backward_time = 3000; // s
    const float right_rotate_angle = 144; // 度
    const float start_rotate_angle = 179.5; //end_rotate_angle = 179.5;
    const int end_backward_speed = 2000, end_backward_time = 5, end_forward_speed = 3000;
    const int slope_delay = 100, slope_count = 10, wall_count = 8;
    const int target_slope_yaw = 90, target_timeout = 20000;
    const int meet_wall_delta_yaw = 20;

    unsigned int status = 0;
    unsigned int meet_wall_delta_time = 0, meet_wall_flag = 0;
    float forward_start_yaw, rotate_target_yaw, rotate_delta_yaw;
    rt_tick_t start_time, rotate_start_tick;
    struct slope_type slope, last_slope;
    int slope_times = 0, wall_times = 0;
    rt_tick_t total_run_time = 0;
    
    rt_tick_t run_time = rt_tick_get_millisecond();
    rt_uint16_t edge_length_time[wall_count];
    pos_t pos[wall_count];
    float area;

    // update motor params from ble
    extern wf_params_t get_wf_params_info(void);
    wf_params_t motor_params = get_wf_params_info();
    if(motor_params != RT_NULL)
    {
        wp_speed = motor_params->pentagonal_path_find_slope_wp_speed;
        forward_speed = motor_params->pentagonal_path_find_slope_mtr_speed;
    }

    move_wp_speed_on_floor_set(wp_speed);
    rt_thread_mdelay(1000);
    move_rotate_on_floor(start_rotate_angle, rotate_timeout);
    rt_thread_mdelay(1000);
    LOG_I("rotate %.2f on floor", start_rotate_angle);

    while(1)
    {
        // 超过最多的掉头次数
        if(wall_times >= wall_count && status == 3)
        {
            cal_pentagonal_points_by_path(edge_length_time, wall_count, forward_speed, pos);
            area = cal_area_by_pos(pos, wall_count);
            save_pool_size_type_by_area(area);
            LOG_I("no slope pool area is %.3f", area);
            LOG_I("not find slope, meet wall times %d", wall_times);
            float start_yaw;
            *ret_val = 0;
            move_rotate_on_floor(90.0f, rotate_timeout);
            rt_thread_mdelay(500);
            move_rotate_on_floor(90.0f, rotate_timeout);
            rt_thread_mdelay(500);
            move_backward_with_speed(end_backward_speed);
            for(int i = 0; i < end_backward_time; i++)
            {
                get_current_slope_info(&slope);
                if(slope.angle > 20)
                {
                    move_stop();
                    LOG_I("stop backward, current slope angle %f", slope.angle);
                    break;
                }
                rt_thread_mdelay(1000);
            }
            move_stop();
            rt_thread_mdelay(1000);
            start_yaw = get_current_yaw();
            move_forward_with_pid_and_time(end_forward_speed, start_yaw, 5000);
            rt_thread_mdelay(1000);
            LOG_I("not find slope, meet wall times %d", wall_times);
            total_run_time = rt_tick_get_millisecond() - run_time;
            LOG_I("not find slope, total run time %d", total_run_time);
            //move_wp_speed_on_floor_set(0);
            return total_run_time;
        }
        // 
        switch(status)
        {
            case 0: // 前进初始化
                LOG_I("forward init");
                move_stop();
                rt_thread_mdelay(1000);
                forward_start_yaw = get_current_yaw();
                move_wp_speed_on_floor_set(wp_speed);
                rt_thread_mdelay(1000);
                start_time = rt_tick_get_millisecond();
                meet_wall_flag = 0;
                meet_wall_delta_time = 0;
                LOG_I("wp_speed %d, start forward yaw: %f ", wp_speed, forward_start_yaw);
                status++;
                LOG_I("forward find slope");
                break;
            case 1: // 前进找墙或坡
                if(rt_tick_get_millisecond() - start_time <  forward_timeout)
                {
                    get_current_slope_info(&slope);
                    // 碰到墙产生yaw角偏移
                    if(fabs(compare_yaws(slope.euler_yaw, forward_start_yaw)) > meet_wall_delta_yaw)
                    {
                        if(meet_wall_flag == 0)
                        {
                            meet_wall_flag = 1;
                            meet_wall_delta_time = rt_tick_get_millisecond() - start_time;
                            LOG_I("meet wall because of the offset of yaw, time %d ms", meet_wall_delta_time);
                        }
                        /* 避免沿着圆弧泳池边沿前进 */
                        if(fabs(compare_yaws(slope.euler_yaw, forward_start_yaw)) > 70)
                        {
                            edge_length_time[wall_times] = meet_wall_delta_time;
                            wall_times++;
                            status = 3;
                            move_stop();
                            rt_thread_mdelay(1000);
                            LOG_I("forward find wall beacause of yaw offset, runtime %d ms", meet_wall_delta_time);
                            break;
                        }
                    }
                    // 前进到墙
                    if(slope.angle > 40)
                    {
                        rt_tick_t delta_tick = rt_tick_get_millisecond() - start_time;
                        if (delta_tick > 1000)
                            delta_tick = delta_tick - 1000; // 减去一秒上墙时间
                        
                        if (meet_wall_flag == 0 || (meet_wall_flag && delta_tick - meet_wall_delta_time < 5000)) 
                            edge_length_time[wall_times] = delta_tick;
                        else
                        {
                            edge_length_time[wall_times] = meet_wall_delta_time;
                        }
                        LOG_I("forward find wall, runtime %d ms", edge_length_time[wall_times]);

                        wall_times++;
                        status = 2;
                        move_stop();
                        rt_thread_mdelay(1000);
                        break;
                    }
                    else if(slope.angle > slope_min && slope.angle <= slope_max) // 前进到坡
                    {
                        if(fabs(slope.angle - last_slope.angle) < slope_change \
                            && fabs(slope.dir - last_slope.dir) < slope_dir_change)
                        {
                            slope_times++;
                            if(slope_times >= slope_count)
                            {
                                status = 4;
                                move_stop();
                                rt_thread_mdelay(1000);
                                
                                /* 找到缓坡后，后退1.5S，避免在浅水区和缓坡交界处 */
                                move_backward_with_speed_and_time(forward_speed, 1500);
                                rt_thread_mdelay(500);
                                
                                move_rotate_on_slope_using_target(target_slope_yaw,target_timeout);
                                rt_thread_mdelay(1000);
                                get_current_slope_info(&slope);
                                if(fabs(slope.angle - last_slope.angle) > slope_change \
                                   || fabs(slope.dir - last_slope.dir) > slope_dir_change)
                                {
                                    status = 1;
                                    slope_times = 0;
                                    start_time = rt_tick_get_millisecond();
                                    LOG_I("after rotate target slope yaw, the slope is different");
                                    LOG_I("slope angle,current:%f last:%f, slope dir, current:%f last:%f ",\
                                        slope.angle, last_slope.angle, slope.dir, last_slope.dir);
                                    break;
                                }
                                //move_wp_speed_on_floor_set(0);
                                LOG_I("forward find slope, runtime %d, slop %f", \
                                    rt_tick_get_millisecond() - start_time, slope.angle);
                                break;
                            }
                        }
                        else
                        {
                            slope_times = 0;
                            last_slope.angle = slope.angle;
                            last_slope.dir = slope.dir;
                            LOG_I("slope angle,current:%f last:%f, slope dir, current:%f last:%f ",\
                                slope.angle, last_slope.angle, slope.dir, last_slope.dir);
                        }
                    }
                    // 前进找坡
                    move_forward_with_speed(forward_speed);
                }
                else
                {
                    rt_tick_t delta_tick = rt_tick_get_millisecond() - start_time;
                    if (meet_wall_flag)
                        edge_length_time[wall_times] = meet_wall_delta_time;
                    else 
                        edge_length_time[wall_times] = delta_tick;
                    LOG_I("forward find slope timeout, runtime %d ms", edge_length_time[wall_times]);

                    move_stop();
                    rt_thread_mdelay(1000);
                    status = 3;
                    wall_times++;
                }
                rt_thread_mdelay(slope_delay);
                break;
            case 2: // 找到墙，后退3s
                LOG_I("backward");
                move_backward_with_speed_and_time(backward_speed , backward_time);
                rt_thread_mdelay(1000);
                status = 3;
                LOG_I("backward speed %d, time %d ms, current yaw %f", \
                    backward_speed, backward_time, get_current_yaw());
                break;
            case 3: // 原地旋转144度
                LOG_I("rotate");
                rotate_target_yaw = calculate_yaw(forward_start_yaw, -right_rotate_angle);
                rotate_delta_yaw = compare_yaws(get_current_yaw(), rotate_target_yaw);
                LOG_I("rotate target yaw %f, delta yaw %f,", rotate_target_yaw, rotate_delta_yaw);
            
                rotate_start_tick = rt_tick_get_millisecond();
                move_rotate_on_floor(rotate_delta_yaw, rotate_timeout);
                rt_thread_mdelay(1000);
                LOG_I("after rotate, current yaw %f, rotate run time: %d", \
                    get_current_yaw(), rt_tick_get_millisecond() - rotate_start_tick);
                status = 0;

                break;
            case 4: // 去坡顶
                if(actions_on_slope_find_slope_top(RT_NULL, RT_NULL) == RT_EOK)
                {
                    status = 5; /* 坡顶为平面，确认为缓坡 */
                }    
                else
                {
                    status = 2; /* 坡顶为墙，不是缓坡重新找缓坡 */
                }
                break;
            case 5: // 找到坡
                *ret_val = 1;
                // 重启imu
                extern float reset_imu(int delay_ms, int wp_speed);
                reset_imu(15*1000, 1000);
                LOG_I("cur yaw = %.2f, after reset imu", get_current_yaw());
                LOG_I("find slope done, meet wall times %d", wall_times);
                total_run_time = rt_tick_get_millisecond() - run_time;
                LOG_I("find slope done, total run time %d", total_run_time);
                return total_run_time;
            default:
                break;
        }
    }
}

void search_slope_test(int argc, char *argv)
{
    int arg_in, arg_out;
    actions_on_floor_search_slope((void *)&arg_in, (void *)&arg_out);
}

MSH_CMD_EXPORT(search_slope_test, search slope test)

int actions_on_floor_backward_search_wall(void *p_arg_in, void *p_arg_out)
{
    // RT_ASSERT(p_arg_in != RT_NULL);
    RT_ASSERT(p_arg_out != RT_NULL);

    int *ret_val = (int *)p_arg_out;
    const int wp_speed = 3000;
    const float end_rotate_angle = 179.5;
    const int rotate_timeout = 20000;
    const int backward_speed = 2000, backward_times = 5;
    struct slope_type slope;
    rt_tick_t total_run_time = 0;
    
    rt_tick_t run_time = rt_tick_get_millisecond();
    
    move_wp_speed_on_floor_set(wp_speed);
    move_rotate_on_floor(end_rotate_angle, rotate_timeout);
    move_backward_with_speed(backward_speed);
    for(int i = 0; i < backward_times; i++)
    {
        get_current_slope_info(&slope);
        if(slope.angle > 20)
        {
            move_stop();
            LOG_I("stop backward, current slope angle %f", slope.angle);
            break;
        }
        rt_thread_mdelay(1000);
    }
    move_stop();
    rt_thread_mdelay(1000);
    
    total_run_time = rt_tick_get_millisecond() - run_time;
    LOG_I("backward search wall, total run time %d", total_run_time);
    *ret_val = 0;

    return total_run_time;
}

int actions_on_floor_backward_update_yaw(void *p_arg_in, void *p_arg_out)
{
    LOG_I("start actions_on_floor_backward_update_yaw.");
    RT_ASSERT(p_arg_in != RT_NULL);
    RT_ASSERT(p_arg_out != RT_NULL);

    const float *end_rotate_angle = (float *)p_arg_in;
    int *ret_val = (int *)p_arg_out;
    const int rotate_timeout = 20000;
    const int backward_speed = 2000, backward_times = 5;
    struct slope_type slope;
    rt_tick_t total_run_time = 0;
    LOG_I("rotate_angle=%f", *end_rotate_angle);
    
    if (fabs(*end_rotate_angle) > 3.0f)
    {
        move_rotate_on_floor(*end_rotate_angle, rotate_timeout);
        rt_thread_mdelay(500);
    }

    rt_tick_t run_time = rt_tick_get_millisecond();
    move_backward_with_speed(backward_speed);
    for(int i = 0; i < backward_times; i++)
    {
        get_current_slope_info(&slope);
        if(slope.angle > 20)
        {
            move_stop();
            LOG_I("stop backward, current slope angle %f", slope.angle);
            break;
        }
        rt_thread_mdelay(1000);
    }
    move_stop();
    rt_thread_mdelay(1000);
    
    total_run_time = rt_tick_get_millisecond() - run_time;
    LOG_I("backward search wall, total run time %d", total_run_time);
    *ret_val = 0;
    LOG_I("end actions_on_floor_backward_update_yaw.");
    return total_run_time;
}

void stop_at_waterline(void)
{
    LOG_I("stop at waterline");
    // 关机光
    laser_switch_off();
    LOG_I("laser off");
    struct stop_on_waterline_info stop_info = {
        .power_off_flag = 0,        // power on
        .on_waterline_time = 3 * 60 , // 3min
        .in_water_time = 2 * 60,     // 2min
        .repeat_count = 4};      // repeat 4 times
    LOG_I("stop at waterline, power off flag %d, on waterline time %d, in water time %d, repeat count %d", \
        stop_info.power_off_flag, stop_info.on_waterline_time, stop_info.in_water_time, stop_info.repeat_count);
    extern int actions_on_wall_stop_on_waterline(void *p_arg_in, void *p_arg_out);
    actions_on_wall_stop_on_waterline(&stop_info, RT_NULL);
}

void goto_waterline_and_stop(stop_on_waterline_info_t stop_info)
{
    LOG_D("%s", __func__);
    // 关机光
    laser_switch_off();
    LOG_I("laser off");
    const int wp_speed = move_wp_speed_on_floor_get();
    // struct stop_on_waterline_info stop_info = {
    //     .on_waterline_time = 2 * 60 , // 2min
    //     .in_water_time = 1,       // 1s
    //     .repeat_count = 1};      // repeat 4 times
    LOG_I("stop at waterline, power off flag %d, on waterline time %d, in water time %d, repeat count %d", \
        stop_info->power_off_flag, stop_info->on_waterline_time, stop_info->in_water_time, stop_info->repeat_count);
    extern int actions_on_wall_stop_on_waterline(void *p_arg_in, void *p_arg_out);
    actions_on_wall_stop_on_waterline(stop_info, RT_NULL);

    // 开水泵
    LOG_I("Restore the pump speed");
    move_wp_speed_on_floor_set(wp_speed);
    // 开激光
    laser_switch_on();
    LOG_I("laser on");
    rt_thread_mdelay(1*1000);
}

int actions_on_floor_goto_shallow_turn_off(void *p_arg_in, void *p_arg_out)
{
    LOG_I("start go_shallow turn off");
    system_info_t model_info = system_info_get();
    RT_ASSERT(p_arg_in != RT_NULL);
    int *if_find_slope = (int *)p_arg_in;
    rt_uint8_t if_goto_waterline;
    rt_uint16_t vol;
    if (model_info->work_mode == WEEKLY_MODE)
    {
        if_goto_waterline = 0;
    }
    else
    {
        if_goto_waterline = get_climb_up_to_waiting_waterline_sta();
    }
    LOG_D("work_mode=%d, if_find_slope==%d, if_goto_waterline=%d", model_info->work_mode, *if_find_slope, if_goto_waterline);

    int find_slope = 0;
    if (*if_find_slope != 0) // 没有找坡或已经找到坡
    {
        // 五角星找坡
#if (POOL_BOTTOM_IS_SLOPE == RT_TRUE)
        LOG_I("start pentagonal path find slope");
        actions_on_floor_search_slope(RT_NULL, &find_slope);
        LOG_I("find slope end. find_slope=%d", find_slope);
        rt_thread_mdelay(500);
#endif
        // 重置start_yaw
        float yaw = get_current_yaw();
        LOG_D("cur_yaw=%f", yaw);
        if (find_slope == 1) // 找到坡
        {
            // 走回浅水区
            LOG_D("start go_shallow");
            goto_shallow(yaw, DELTA_SLOPE);
            LOG_I("go_shallow end");
        }
        else
        {
            LOG_I("no slope, rotate");
            move_rotate_on_floor(179.5f, MOVE_CONTROL_RIGHT_SAFEZONE);
        }
        rt_thread_mdelay(500);
    }
    if (if_goto_waterline == 1)
    {
        stop_at_waterline();
    }
    else
	{
		//关闭水泵、行走电机、眉灯
		move_wp_off_on_wall();
		move_stop();
		led_show_robot_action(ROBOT_ACTION_STOP);
        
		//获取当前电压
		get_battery_voltage(&vol);
		LOG_I("goto stop bottom done,vol:%d", vol);
		while(vol > IMM_OFF_VOL_THRES)
		{
			get_battery_voltage(&vol);
			rt_thread_mdelay(1000);
		}
		LOG_I("power off, vol %d", vol);
		rt_thread_mdelay(1000);
		power_off_ac(1);	//1s后关机
	}

    return 0;
}

//被weekly mode 清洁完一次时调用，让机器在固定时间内唤醒继续清洁
void set_robot_weeklymode_sleep_on_time(rt_uint32_t time_s)
{
	//保存休眠时间
	eeprom_weekly_mode_sleep_time_set(time_s);
	//更新保存唤醒次数
	eeprom_weekly_mode_wake_up_times_set(0);
	//设置让其定时开机
	LOG_I("power off by timing");
	set_battery_power_on_time(WEEKLY_MODE_WAKE_UP_INTV);	//定时60s开机
	
}


int _clean_edge(float start_yaw, wf_params_t params)
{
    int pre_wp_speed, clean_wp_speed = 3800, clean_mtr_speed = 3000;
    if (params!=NULL)
    {
        clean_wp_speed = params->edge_clean_sediment_wp_speed;
        clean_mtr_speed = params->edge_clean_sediment_mtr_speed;
    } else
    {
        LOG_W("clean edge sediment, get no params.");
    }
    
    LOG_I("clean edge sediment.wp=%d, mtr=%d", clean_wp_speed, clean_mtr_speed);

    // 清洗边缘
    float target_yaw = calculate_yaw(start_yaw, 180.0f); 
    move_rotate_on_floor_using_target(target_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
    rt_thread_mdelay(500);

    // 增大吸力
    pre_wp_speed = move_wp_speed_on_floor_get();
    move_wp_speed_on_floor_set(clean_wp_speed);
    if (abs(pre_wp_speed - clean_wp_speed) > 40)
    {
        // 延时3s
        LOG_I("get diff wp, delay 1s.pre=%d, cur=%d", pre_wp_speed, clean_wp_speed);
        // rt_thread_mdelay(3000);
        rt_thread_mdelay(1000);
    }

    // 后退上墙
    LOG_I("back to wall");
    rt_tick_t start_time, cur_time, move_time = 5*1000;
    float cur_slope = get_current_slope();
    LOG_I("cur_slope=%f", cur_slope);
    start_time = rt_tick_get_millisecond();
    cur_time = start_time;
    while (cur_time - start_time < move_time)
    {      
        move_backward_with_speed(clean_mtr_speed);
        rt_thread_mdelay(500);
        cur_slope = get_current_slope();
        LOG_D("back clean edge, cur_slope=%f", cur_slope);
        if (cur_slope > SLOPE_THRES_MAX)
        {
            LOG_I("get wall");
            break;
        } 
        cur_time = rt_tick_get_millisecond();
    }
    move_stop_time(500);

    // 前进下墙
    LOG_I("forward down wall");
	LOG_I("cur_slope=%f", cur_slope);
    move_time = 5*1000;
    start_time = rt_tick_get_millisecond();
    cur_time = start_time;
    while (cur_time - start_time < move_time)
    {
        LOG_D("clean edge, down wall");
        move_forward_with_speed(clean_mtr_speed);
        cur_slope = get_current_slope();
        if (fabs(cur_slope) < SLOPE_THRES_MAX)
        {
            LOG_I("down wall");
            break;
        }
        cur_time = rt_tick_get_millisecond();
        rt_thread_mdelay(500);
    } 
    move_stop_time(500);

    // 后退到稳定坡面
    move_time = 4*1000;
    int count = 0;
    cur_slope = get_current_slope();
    LOG_I("after down wall,cur_slope=%f", cur_slope);
    float start_slope = cur_slope;
	LOG_I("start find smooth surface,cur_slope=%f", cur_slope);
    start_time = rt_tick_get_millisecond();
    cur_time = start_time;
    while (cur_time - start_time < move_time)
    {      
        move_forward_with_speed(clean_mtr_speed);
        cur_slope = get_current_slope();
        LOG_D("forward, cur_slope=%f", cur_slope);
        if (fabs(start_slope - cur_slope) < 5.0f)
        {
            count += 1;
        } else
        {
            count = 0;
            start_slope = cur_slope;
            LOG_I("update slope, slope=%f", start_slope);
        }
        LOG_D("count=%d", count);
        if (count > 20)
        {
            LOG_I("get smooth surface");
            break;
        }
        rt_thread_mdelay(100);
        cur_time = rt_tick_get_millisecond();
    } 
    move_stop_time(500);

    // 减小吸力
    move_wp_speed_on_floor_set(pre_wp_speed);
    if (abs(pre_wp_speed - clean_wp_speed) > 40)
    {
        // 延时3s
        LOG_I("set wp, delay 1s.");
        // rt_thread_mdelay(3000);
        rt_thread_mdelay(1000);
    }

    return 0;
}


static int _is_approch_start_angle(float cur_angle, float start_angle)
{
    return (fabs(cur_angle - start_angle) < 10.0f);
}

int actions_on_judge_c_edge(void *p_arg_in, void *p_arg_out)
{
    RT_ASSERT(p_arg_in != RT_NULL);
    RT_ASSERT(p_arg_out != RT_NULL);

    int ret = 0;

    float start_yaw = ((move_info_t)p_arg_in)->start_yaw;
    int run_time = ((move_info_t)p_arg_in)->move_time;
    int move_speed = ((move_info_t)p_arg_in)->move_speed;
    float pitch_thres = ((move_info_t)p_arg_in)->judge_edge_pitch_thres;
    const int TIME_OUT = 2 * run_time;

    LOG_I("start_yaw=%.2f, run_time=%d, move_speed=%d, pitch_thres=%.2f", start_yaw, run_time, move_speed, pitch_thres);
    int *ret_val = (int *)p_arg_out;

    // find wall
    rt_tick_t start_time = rt_tick_get(), cur_time, forward_end_time = 0, delta_time;
    float start_pitch = get_current_pitch(), cur_yaw, cur_pitch;
    rt_tick_t backward_time;
    LOG_I("start_pitch=%.2f", start_pitch);
    while (1)
    {
        cur_pitch = get_current_pitch();
        if (cur_pitch < pitch_thres) // 判断是否过抬头
        {
            LOG_I("get wall, cur_pitch=%.2f", cur_pitch);
            ret = 1;
            break;
        }

        cur_time = rt_tick_get();
        delta_time = cur_time - start_time;
        if (_is_approch_start_angle(cur_pitch, start_pitch) == 1) // record on floor time
        {
            forward_end_time = delta_time;
        }

        if (delta_time > run_time) // 判断是否超时
        {
            LOG_I("timeout %d", delta_time);
            ret = 0;
            break;
        }

        cur_yaw = get_current_yaw();
        move_forward_with_pid(move_speed, cur_yaw, start_yaw);
    }
    move_stop_time(500);
    LOG_I("find wall end ret=%d , using time %d", ret, forward_end_time);
    cur_pitch = get_current_pitch();
    if (cur_pitch < -10.0f && cur_pitch > -SLOPE_THRES_MAX)
    {
        LOG_I("find %f slope, do not backward", cur_pitch);
        goto _end;
    }

    // to make sure it's on initial status
    start_time = rt_tick_get();
    while (1)
    {
        cur_pitch = get_current_pitch();
        if (_is_approch_start_angle(cur_pitch, start_pitch))
        {
            move_stop_time(500);
            LOG_I("reset pitch end");
            break;
        }

        cur_time = rt_tick_get();
        if (cur_time - start_time > TIME_OUT)
        {
            move_stop_time(500);
            LOG_I("reset pitch timeout");
            break;
        }

        cur_yaw = get_current_yaw();
        move_backward_with_speed(move_speed);
    }
    move_rotate_on_floor_using_target(start_yaw, 20 * 1000);
    LOG_I("reset yaw end");

    // move back to start point
	LOG_I("move back, time=%d", forward_end_time);
    start_time = rt_tick_get();
    while (1)
    {
        LOG_D("forward time=%d", forward_end_time);
        backward_time = (rt_tick_t)(forward_end_time * 0.8);
        // 如果低头后退，增加后退时间
        if (get_current_pitch() > 5.0f)
        {
            backward_time = (rt_tick_t)(forward_end_time * 0.9);
        }
        LOG_D("backward time=%d", backward_time);
        cur_time = rt_tick_get();
        // if (cur_time - start_time > forward_end_time)
        if (cur_time - start_time > backward_time) // 減小後退時間
        {
            move_stop_time(500);
            break;
        }

        cur_yaw = get_current_yaw();
        move_backward_with_pid(move_speed, cur_yaw, start_yaw);
    }
    LOG_I("move back to start point end,time=%d", backward_time);
_end:
    *ret_val = (ret ? forward_end_time : 0);
    return ret;
}

// 旋转360度，记录最小pitch角对应的yaw
int rotate_and_get_max_pitch(slope_info_t info)
{
    const floor_params_t p_params = _get_floor_params(0);

    // 找到低头时的最大pitch，低头pitch>0
    float max_pitch = -180.0f, cur_pitch;
    float max_pitch_yaw, cur_yaw;
    float start_yaw = get_current_yaw();
    // 先后退2s
    led_show_robot_action(ROBOT_ACTION_BACKWARD);
    LOG_I("move backward");
    rt_tick_t start_time = rt_tick_get_millisecond();
    while (1)
    {
        cur_yaw = get_current_yaw();
        move_backward_with_pid(3000, cur_yaw, start_yaw);
        rt_tick_t cur_time = rt_tick_get_millisecond();
        if ((cur_time - start_time) > 2 * 1000)
        {
            LOG_I("backward end");
            break;
        }
    }
    move_stop();

    float cur_slope = get_current_slope();
    if (fabs(cur_slope) < SLOPE_THRES_MIN)
    {
        LOG_I("no slope");
        return 1;
    }

    const int MAX_SPEED = 4000, MIN_SPEED = 4000;
    const float ANGLE_THRESHOLD = 10.0f, ANGLE_OFFSET_MIN = 0.5f, ANGLE_OFFSET_MAX = 1.0f;

    const float DELTA_THRESHOLD = 30.0f, SMALL_DELTA_THRESHOLD = 20.0f;
    const float BIG_DELTA_GCC_Z_MIN = 30.0f, BIG_DELTA_GCC_Z_MAX = 31.0f,
                BIG_DELTA_MOTOR_STEP = 500.0f;
    const float SMALL_DELTA_GCC_Z_MIN = 20.0f, SMALL_DELTA_GCC_Z_MAX = 21.0f,
                SMALL_DELTA_MOTOR_STEP = 500.0f;

    int tmp_speed, time;
    time = p_params->rotate_time_max;
    float delta = 0.0f, tmp_range = 0.0f;
    RT_UNUSED(tmp_range);

    if (time <= 0)
        return -RT_ERROR;

    tmp_speed = MIN_SPEED;
    delta = 90.0f;
    int left_rotation = (delta > 0);
    (left_rotation) ? led_show_robot_action(ROBOT_ACTION_ROTATE_FIND_MAX_PITCH) : led_show_robot_action(ROBOT_ACTION_ROTATE_FIND_MAX_PITCH);
    rt_thread_mdelay(LED_SHOW_TIME);

    tmp_range = (fabs(delta) > ANGLE_THRESHOLD) ? ANGLE_OFFSET_MAX : ANGLE_OFFSET_MIN;

    rt_err_t status = -RT_ETIMEOUT;

    float gyro_z_max, gyro_z_min;
    int step;
    for (size_t i = 0; i < 4; i++)
    {
        start_yaw = get_current_yaw();
        float target_yaw = calculate_yaw(start_yaw, delta);
        LOG_I("start_yaw=%f, target_yaw=%f", start_yaw, target_yaw);

        while (1)
        {
            cur_yaw = get_current_yaw();
            cur_pitch = get_current_pitch();
            LOG_D("cur_yaw=%f, cur_pitch=%f", cur_yaw, cur_pitch);
            if (cur_pitch > max_pitch)
            {
                max_pitch = cur_pitch;
                max_pitch_yaw = cur_yaw;
                LOG_D("max_pitch=%f, max_pitch_yaw=%f", max_pitch, max_pitch_yaw);
            }
            float tmp_delta = compare_yaws(cur_yaw, target_yaw);                        // 获取最新的偏角
            int flag1 = left_rotation && (tmp_delta <= 0.0f) && (tmp_delta != -180.0f); // 左转过头
            int flag2 = !left_rotation && (tmp_delta >= 0.0f) && (tmp_delta != 180.0f); // 右转过头

            // 调速
            if (fabs(tmp_delta) > DELTA_THRESHOLD)
            {
                gyro_z_min = BIG_DELTA_GCC_Z_MIN;
                gyro_z_max = BIG_DELTA_GCC_Z_MAX;
                step = BIG_DELTA_MOTOR_STEP;
                tmp_speed = 5000;
            }
            else if (fabs(tmp_delta) < SMALL_DELTA_THRESHOLD)
            {
                gyro_z_min = SMALL_DELTA_GCC_Z_MIN;
                gyro_z_max = SMALL_DELTA_GCC_Z_MAX;
                step = 0;
                tmp_speed = MIN_SPEED;
            }
            else
            {
                gyro_z_min = SMALL_DELTA_GCC_Z_MIN;
                gyro_z_max = SMALL_DELTA_GCC_Z_MAX;
                step = SMALL_DELTA_MOTOR_STEP;
                tmp_speed = 5000;
            }

            float gyro_z = get_current_gyro_z() / 16.4f; // 获取当前转速
            if (fabs(gyro_z) < gyro_z_min)
            {
                tmp_speed += step;
            }
            else if (fabs(gyro_z) > gyro_z_max)
            {
                tmp_speed -= step;
            }
            tmp_speed = LOWER(tmp_speed, MIN_SPEED, MAX_SPEED);

            // LOG_D("start_yaw=%.2f target_yaw=%.2f cur_yaw=%.2f diff=%.2f cur_speed=%d, gyro_z=%.2f", start_yaw, target_yaw, tmp_yaw,
            //       tmp_delta, tmp_speed, gyro_z);

            if (fabs(tmp_delta) < 2.0f || flag1 || flag2)
            {
                status = RT_EOK;
                break;
            }

            if (time <= 0)
            {
                status = -RT_ETIMEOUT;
                break;
            }

            // 旋转
            (left_rotation) ? move_turn_left(tmp_speed) : move_turn_right(tmp_speed);

            rt_thread_mdelay(20);
            time -= 20;
        }
		LOG_I("max_pitch=%f, max_pitch_yaw=%f", max_pitch, max_pitch_yaw);
    }

    led_show_robot_action(ROBOT_ACTION_STOP);
    move_stop(); // 停止旋转

    info->max_pitch = max_pitch;
    info->max_pitch_yaw = max_pitch_yaw;

    return status;
}

static void move_info_usage(void)
{
    rt_kprintf("\t wf 1 [pitch_thres] [move_time] [move_speed] \n");
}

void wf(int argc, char *argv[])
{
    int result = 1234;

    if (argc < 2)
    {
        rt_kprintf("usage: wf <mode> <options> \n ");
        rt_kprintf("\t 0: <water pump speed>\n");
        rt_kprintf("\t 1: judge C edge \n");
        rt_kprintf("\t 2: <delta> distance measure without stop \n");

        return;
    }

    int order = atoi(argv[1]);
    int in = atoi(argv[2]);
    rt_kprintf("order: %d, in: %d\n", order, in);

    struct move_info move = {0};
    float delta_yaw;
    struct dis_sensor_buffer_info dis_buff;

    //    struct move_info move = {0};
    switch (order)
    {
    case 0:
        move_wp_speed_on_floor_set(in);
        break;

    case 1:
        if (argc != 5)
        {
            move_info_usage();
            break;
        }

        move.start_yaw = get_current_yaw();
        move.judge_edge_pitch_thres = atof(argv[2]);
        move.move_time = atoi(argv[3]);
        move.move_speed = atoi(argv[4]);
        actions_on_judge_c_edge(&move, &result);
        break;

    case 2:
        delta_yaw = atof(argv[2]);
        dis_buff.data_count = 0;
        dis_buff.dis_buff = RT_NULL;
        actions_on_floor_rotate_measure_no_stop(&delta_yaw, &dis_buff);
        rt_kprintf("count %d\n", dis_buff.data_count);
        for (size_t i = 0; i < dis_buff.data_count; i++)
        {
            rt_kprintf("[%d] %dmm %.2f\n", i, dis_buff.dis_buff[i].distance, dis_buff.dis_buff[i].yaw);
        }
        break;
    default:
        break;
    }
    rt_kprintf("result: %d\n", result);
}
MSH_CMD_EXPORT(wf, wash floor actions test);
