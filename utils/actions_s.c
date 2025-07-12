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
#include "actions_on_wall.h"
#include "easyflash.h"

#define DBG_TAG "as"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static int _is_near_start_line(float start_yaw)
{
    int ret = 0;
    float cur_yaw = get_current_yaw();
    float delta = compare_yaws(cur_yaw, start_yaw);
    if (delta > 5.0f)
    {
        ret = 1;
    }
    LOG_D("near end judge, cur yaw %f, start yaw %f, delta %f", cur_yaw, start_yaw, delta);
    return ret;
}

#if 0
// 调转机头方向
static int _reverse_head_direction(void)
{
#if 0
    struct slope_info pool_info = {-360.0f, 0.0f};

    int ret = rotate_and_get_max_pitch(&pool_info);
    if ((ret == 0) && (pool_info.max_pitch > SLOPE_THRES_MIN))
    {
        LOG_D("ready to reverse direction");
        move_rotate_on_floor_using_target(pool_info.max_pitch_yaw, 20 * 1000);
        rt_thread_mdelay(3 * 1000);
        LOG_D("max pitch yaw [cal]%f [real]%f", pool_info.max_pitch_yaw, get_current_yaw());
        // 左转90度，调整机头方向到对面的坡上；
        move_rotate_on_floor(85.0f, 20 * 1000);
        rt_thread_mdelay(3 * 1000);
        LOG_D("now reverse yaw = %f", get_current_yaw());
    }
#else
    LOG_D("reverse direction %f", get_current_yaw());
    move_rotate_on_floor(90.0f, 20 * 1000);
    rt_thread_mdelay(3 * 1000);
    LOG_D("now reverse yaw1 = %f", get_current_yaw());
    move_rotate_on_floor(85.0f, 20 * 1000);
    rt_thread_mdelay(3 * 1000);
    LOG_D("now reverse yaw2 = %f", get_current_yaw());
#endif
    return 0;
}
#endif

int actions_on_slope_wash_around(void *p_arg_in, void *p_arg_out)
{  
    const int timeout = 40 * 60 * 1000;    // 40分钟
    const int find_slope_time = 60 * 1000; // 60 seconds
    
    uint16_t wp_speed = 3300, move_speed = 8000;
    uint16_t edge_wp_speed = 3800, edge_move_speed = 3000;

    const int back_move_speed = 5000, move_speed_slow = 3800, backward_time = 1000;
    const float up_slope_start_pitch = -20.5f, up_slope_end_pitch = -60.0f, up_flat_pitch = -10.0f;
    const float slope_rotate_angle_floor = 20.0f;
    const float  slope_rotate_small_angle_top = 15.0f, slope_rotate_large_angle_top = 18.0f;

    const int slope_run_time_max = 12 * 1000; // 洗坡的最大时间
    const int slope_run_time_min = 3 * 1000;  // 小洗坡的最小时间
    const int down_slope_timeout = 15 * 1000; // 下坡超时时间
    const float small_angle_thres = 3.0f;     // 小角度阈值

    int short_slope_flag = 0;             // 短坡标志
    int short_slope_time = 0;             // 遇到短坡的次数
    int first_time = 1;                   // 第一次找到坡, 0--是的，1--不是
    float start_floor_yaw, one_slope_yaw; // 第一次找到坡后，记录地面上起始yaw
    float cur_slope, yaw, floor_yaw;
    int forward_end_info = 0, forward_end_time = 0, forward_adjust_time = 0;
    rt_tick_t backward_end_time, backward_adjust_time, adjust_tick;
    int end_flag = 0;
    int floor_to_slope_back_time = 3 * 1000;
    int wash_slope_count = 0, count = 0, rows_num = 0; // 当前洗的是第几面坡，从第0面开始，到第三面结束；
    int switch_slope_flag = 0;
    float find_slope_min = 0.0f;
    float cur_yaw, delta;
    rt_tick_t down_flat_tick;
    float search_slope_time, forward_on_slope_time, backward_on_slope_time;
    float backward_length;
    float slope_rotate_angle_top = slope_rotate_small_angle_top;
    int diff_slope_flag = 0;
    
    struct slope_type cur_slope_info, start_slope_info, up_slope_info, down_slope_info;
    struct wash_record_info wash_info = {0};
    rt_uint8_t cur_list_num = 0;

    #define SIN_ANGLE(x) sinf((x)/180.0f*3.14159265f)
    #define COS_ANGLE(x) cosf((x)/180.0f*3.14159265f)

    /* 清洁记录信息开始表头 */
    wash_info.region_flag = REGION_SLOPE_STEEP;
    wash_info.wash_num = eeprom_total_clean_counts_get();
    wash_info.start_time = rt_tick_get_millisecond();
    get_battery_soc(&wash_info.start_soc);
    wash_info.wash_time = 1;
    LOG_I("wash info, start_time = %d", wash_info.start_time);

    RT_UNUSED(floor_to_slope_back_time);
    // update motor params from ble
    extern wf_params_t get_wf_params_info(void);
    wf_params_t motor_params = get_wf_params_info();
    if(motor_params != RT_NULL)
    {
        wp_speed = motor_params->clean_steep_incline_wp_speed;
        move_speed = motor_params->clean_steep_incline_mtr_speed;
        edge_wp_speed = motor_params->clean_steep_incline_edge_wp_speed;
        edge_move_speed = motor_params->clean_steep_incline_edge_mtr_speed;
    }
    LOG_I("wp_speed %d, move_speed %d, edge_wp_speed %d, edge_move_speed %d", \
        wp_speed, move_speed, edge_wp_speed, edge_move_speed);

    /* 根据当前电机速度计算前进线速度 */
    vel_t vel = get_velocities(move_speed, move_speed);
    LOG_I("velocity of linear_x is %.2f m/s ", vel.linear_x);

    float floor_end_yaw = get_current_yaw();
    LOG_I("floor_end_yaw %.2f", floor_end_yaw);

    move_wp_speed_on_floor_set(wp_speed);
    // rt_thread_mdelay(3 * 1000);
    rt_thread_mdelay(1000);
    // 先在起始的角度上左偏90度
    move_rotate_on_floor(90.0f, 20 * 1000);
    // rt_thread_mdelay(3 * 1000);
    rt_thread_mdelay(1000);
    float initial_yaw = get_current_yaw();
    LOG_I("initial yaw = %.2f", initial_yaw);

    rt_uint16_t off_value;
    rt_uint8_t waterline_label = get_climb_up_to_waiting_waterline_sta();
    if ((waterline_label == 1))
    {
        off_value = STOP_WATERLINE_THRES;
    }
    else
    {
        off_value = OFF_VOL_THRES;
    }

    system_info_t info = system_info_get();
    LOG_I("work mode %d", info->work_mode);
    if (info->work_mode == WEEKLY_MODE)
    {
        off_value = OFF_VOL_THRES;
    }

    LOG_I("off_value=%d", off_value);

    rt_tick_t start_tick = rt_tick_get();
    rt_tick_t wash_slope_start_tick = rt_tick_get();

    while (1)
    {
        if (rt_tick_get() - wash_slope_start_tick > timeout)
        {
            LOG_W("wash around timeout");
            goto _end;
        }

        rt_uint16_t vol;
        get_battery_voltage(&vol);
        LOG_D("current voltage: %dmV", vol);
        if (vol <= off_value)
        {
            /* 清洁记录信息结束表头 */
            wash_info.total_rows_num = rows_num;
            wash_info.end_time = rt_tick_get_millisecond();
            get_battery_soc(&wash_info.end_soc);
            ef_rcd_write(&wash_info);
            LOG_I("wash info, total_rows_num = %d, end_time = %d", wash_info.total_rows_num, wash_info.end_time);

            const int slope_fag = 1;
            extern int actions_on_floor_goto_shallow_turn_off(void *p_arg_in, void *p_arg_out);
            actions_on_floor_goto_shallow_turn_off((void *)&slope_fag, NULL);
        }

        // -------------------search up slope，找坡超时若干次就退出-------------------
        rows_num++;
        LOG_D("search slope");
        float start_yaw = get_current_yaw();
        LOG_I("search slope start yaw %.2f", start_yaw);
        move_forward_with_speed(move_speed);
        start_tick = rt_tick_get();
        find_slope_min = 90.0f;
        int flat_count = 0;

        while (1)
        {
            get_current_slope_info(&cur_slope_info);
            cur_yaw = cur_slope_info.euler_yaw;
            move_forward_with_pid(move_speed, cur_yaw, start_yaw);
            
            if (cur_slope_info.angle > fabs(up_slope_start_pitch) && (cur_slope_info.yaw > 90 || cur_slope_info.yaw < -90))
            {
                LOG_I("search slope end, slope angle %.2f", cur_slope_info.angle);
                break;
            }

            if (rt_tick_get() - start_tick > find_slope_time) // 找坡超时
            {
                LOG_W("find slope timeout, end search slope");
                goto _end;
            }

            // 由缓坡踩到平台上，执行倒车，切坡
            // find_slope_min = (find_slope_min > cur_slope_info.angle) ? cur_slope_info.angle : find_slope_min;
            if(cur_slope_info.angle < 7.0f)
            {
                flat_count++;
            }
            else
            {
                flat_count = 0;
            }

            if (short_slope_flag == 1 && flat_count > 20)
            {
                LOG_I("find slope top!!![%.2f, %.2f]", find_slope_min, cur_slope_info.angle);
                if (switch_slope_flag == 0)
                {
                    move_stop_time(1000);
                    move_backward_with_speed(move_speed);
                    start_tick = rt_tick_get();
                    while (1)
                    {
                        get_current_slope_info(&cur_slope_info);
                        if (cur_slope_info.angle > 10.0f)
                        {
                            LOG_I("back to slope end");
                            move_stop();
                            break;
                        }

                        if (rt_tick_get() - start_tick > find_slope_time) // 找坡超时
                        {
                            LOG_W("back to slope timeout");
                            goto _end;
                        }

                        rt_thread_mdelay(100);
                    }

                    LOG_I("reserse slope");
                    goto _reverse;
                }
            }

            // 未切坡前，找坡超时，则做切坡动作
            if (rt_tick_get() - start_tick > 10 * 1000)
            {
                if (switch_slope_flag == 0)
                {
                    LOG_W("find slope time out reverse slope");
                    move_stop();
                    goto _reverse;
                }
            }

            rt_thread_mdelay(100);
        }

        search_slope_time = (rt_tick_get_millisecond() - start_tick) / 1000.0f;
        LOG_I("search slope time %.2f s", search_slope_time);

        move_time_before_stop(1 * 1000);
        rt_thread_mdelay(1 * 1000);

        // 纠正坡上姿态
        yaw = get_current_yaw_on_wall();
        if (fabs(yaw) > 10.0f)
        {
            LOG_I("check slope vyaw on slope");
            move_rotate_on_wall_using_target_without_pitch_check(0.0f, 20 * 1000);
            move_stop_time(500);
        }
        get_current_slope_info(&cur_slope_info);
        // make sure it's on the up slope
 #if 0
        LOG_D("make sure it's on the up slope");
        if (cur_slope_info.angle < fabs(up_slope_start_pitch)) // 从坡上落下，则跳过
        {
            LOG_D("not on slope %f", cur_slope_info.angle);
            continue;
        }
        LOG_D("final vyaw %f", -cur_slope_info.euler_roll);
#endif
        floor_yaw = cur_slope_info.euler_yaw;
        up_slope_info = cur_slope_info;
        LOG_I("->floor yaw %f find slope %f", floor_yaw, cur_slope_info.angle);

        // -------------forward slope-------------
        count++;
        LOG_I("slope forward %d", count);
        short_slope_flag = 0;
        start_tick = rt_tick_get();
        forward_adjust_time = 0;

        while (1)
        {
            forward_end_time = rt_tick_get() - start_tick;
            get_current_slope_info(&cur_slope_info);
            if (cur_slope_info.angle > fabs(up_slope_end_pitch)) // detect wall
            {
                LOG_I("find wall %f", cur_slope_info.angle);
                forward_end_info = 0;
                break;
            }
            else if (cur_slope_info.angle < fabs(up_flat_pitch)) // detect flat
            {
                LOG_I("find flat %f", cur_slope_info.angle);
                forward_end_info = 1;
                break;
            }

            if (forward_end_time > slope_run_time_max) // timeout
            {
                LOG_W("find slope timeout %f", cur_slope_info.angle);
                forward_end_info = 2;
                break;
            }

            if (fabs(cur_slope_info.euler_roll) > 30.0f)
            {
                adjust_tick = rt_tick_get_millisecond();
                move_rotate_on_wall_using_target(0.0f, 20 * 1000);
                move_stop_time(500);
                floor_yaw = get_current_yaw();
                LOG_I("adjust vyaw, update floor yaw %.2f", floor_yaw);
                get_current_slope_info(&cur_slope_info);
                forward_adjust_time += rt_tick_get_millisecond() - adjust_tick;
            }

            move_forward_with_pid(move_speed, -cur_slope_info.euler_roll, 0.0f);
            rt_thread_mdelay(100);
        }

        forward_on_slope_time = (forward_end_time - forward_adjust_time) / 1000.0f;
        LOG_I("row_id %d, forward on slope time %.2f s", rows_num, forward_on_slope_time);

        /* 记录前进清洗信息 */
        cur_list_num = (rows_num - 1) % EF_RCD_LIST_LEN;
        wash_info.info_lists[cur_list_num].row_id = rows_num;
        wash_info.info_lists[cur_list_num].length = (int)(forward_on_slope_time * vel.linear_x * 1000); // mm
        if(rows_num == 1)
        {
            wash_info.info_lists[cur_list_num].width = 0; // mm
        }
        else
        {
            wash_info.info_lists[cur_list_num].width = (int)(search_slope_time * vel.linear_x * 1000 * SIN_ANGLE(slope_rotate_angle_floor)); // mm
        }
        if (rows_num % EF_RCD_LIST_LEN == 0)
        {
            ef_rcd_write(&wash_info);
            memset(wash_info.info_lists, 0, EF_RCD_LIST_LEN * sizeof(struct wash_row_info));
        }

        move_stop_time(500);

        // 停止条件分析处理
        // 找到坡，并且坡顶不是平面，则记录起始yaw
        if (forward_end_info == 0 || forward_end_info == 2) // 上墙结束或超时结束
        {
            /* 过滤坡顶是shift ledge的情况 */
            move_backward_with_speed_and_time(move_speed, 2*1000);
            move_stop_time(1000);
            // rotate on the top of slope
            yaw = get_current_yaw();
            LOG_I("adjust yaw on the top of slope, current yaw %.2f", yaw);
            /* 在第3次使用大角度进行后退 */
            if(count % 3 == 0)
                slope_rotate_angle_top = slope_rotate_large_angle_top;
            else 
                slope_rotate_angle_top = slope_rotate_small_angle_top;
            LOG_I("slope rotate angle top %.2f", slope_rotate_angle_top);
            move_rotate_on_floor(slope_rotate_angle_top, 20 * 1000);
            move_stop_time(500);
#if 0
            yaw = get_current_yaw_on_wall();
            LOG_D("current yaw on wall %.2f", yaw);
            if (fabs(yaw) < 5.0f)
            {
                LOG_D("adjust vyaw on the of slope");
                move_rotate_on_wall(slope_rotate_angle_top, 20 * 1000);
                move_stop_time(500);
            }
#endif
            if (first_time == 1) // record floor yaw
            {
                first_time = 0;
                start_floor_yaw = floor_yaw;
                start_slope_info = up_slope_info;
                LOG_I("record start floor yaw %f, start slope dir %f", start_floor_yaw, start_slope_info.dir);

                one_slope_yaw = floor_yaw; // 记录第0面坡yaw
            }
        }
        // 找到坡，但是坡顶是平面，则不记录，同时要切到对面的墙上去
        else if (forward_end_info == 1) // 踩平面结束
        {
            int time = forward_end_time / 2;
            const int time_min = 3 * 1000;
            floor_to_slope_back_time = (time > time_min) ? time : time_min;
            LOG_I("floor to slope back time %d", floor_to_slope_back_time);
            move_backward_with_speed_and_time(move_speed, time);
        }

        // 找到坡，但是此时的坡很短，则切到对面的墙上去,
        if (forward_end_time < slope_run_time_min)
        {
            LOG_I("short slope finded");
            short_slope_flag = 1;
        }
        LOG_I("forward end info %d, end time %d", forward_end_info, forward_end_time);
        // -------------backward slope
        rows_num++;
        LOG_I("slope backward %d", count);
        get_current_slope_info(&down_slope_info);
        LOG_D("backward slope dir %f", down_slope_info.dir);
        move_backward_with_speed(back_move_speed);
        down_flat_tick = rt_tick_get();     // 
        start_tick = rt_tick_get();         // 记录开始从坡上落下的时间戳
        diff_slope_flag = 0;
        backward_adjust_time = 0;

        while (1)
        {
            get_current_slope_info(&cur_slope_info);
            if(rt_tick_get() - start_tick > down_slope_timeout) // 超时退出
            {
                LOG_W("backward slope timeout");
                break;
            }

            if(cur_slope_info.angle > fabs(up_slope_start_pitch))
            {
                down_flat_tick = rt_tick_get();
            }
            else 
            {
                if (rt_tick_get()- down_flat_tick > 3000) // 从坡上落下，且持续时间超过3秒，则退出
                {
                    LOG_I("backward end slope %f", cur_slope_info.angle);
                    break;
                }
            }
             
            delta = compare_yaws(cur_slope_info.dir, down_slope_info.dir);
            if(fabs(delta) < 30) // 在同一个陡坡的调整策略
            {
                if(fabs(cur_slope_info.euler_roll) > 40.0f)
                {
                    adjust_tick = rt_tick_get_millisecond();
                    LOG_I("backward slope, slope yaw is larger than 30, delta %f", delta);
                    move_rotate_on_wall_using_target(slope_rotate_angle_top, 20 * 1000);
                    move_stop_time(500);
                    backward_adjust_time += rt_tick_get_millisecond() - adjust_tick;
                }
            }
            else if (delta < -45)// 跨越两个陡坡的调整策略
            {
                if(diff_slope_flag == 0)
                {
                    delta = compare_yaws(cur_slope_info.dir, start_slope_info.dir);
                    LOG_I("slope_dir_delta %.2f", delta);
                    if((delta < -45 && delta > -135) || fabs(delta) < 10) // 在第一面陡坡的左右两侧的陡坡交界处才进行处理
                    {
                        adjust_tick = rt_tick_get_millisecond();
                        diff_slope_flag = 1;
                        LOG_I("cur backward slope dir %f, dir_delta %f", cur_slope_info.dir, delta);
                        down_slope_info.dir = cur_slope_info.dir;
                        move_rotate_on_wall_using_target(0, 20 * 1000);
                        move_stop_time(200);
                        get_current_slope_info(&cur_slope_info);
                        floor_yaw =  cur_slope_info.euler_yaw; // 记录下次前进找坡的yaw角
                        move_rotate_on_wall_using_target(45, 20 * 1000);
                        move_stop_time(200);
                        backward_adjust_time += rt_tick_get_millisecond() - adjust_tick;
                    }
                }
            }
            
            move_backward_with_speed(back_move_speed); // 反复发送，以防有不响应的情况；
            rt_thread_mdelay(100);
            // yaw = get_current_yaw_on_wall();
            // move_backward_with_pid(move_speed, yaw, 0.0f);
        }
        move_stop_time(500);

        backward_end_time = rt_tick_get_millisecond() - start_tick;
        backward_on_slope_time = (backward_end_time - backward_adjust_time - 3000) / 1000.0f;
        backward_length =  backward_on_slope_time * vel.linear_x * 1000;
        LOG_I("row_id %d, backward on slope time %.2f s", rows_num, backward_on_slope_time);

        /* 记录后退清洗信息 */
        cur_list_num = (rows_num - 1) % EF_RCD_LIST_LEN;
        wash_info.info_lists[cur_list_num].row_id = rows_num;
        wash_info.info_lists[cur_list_num].length = (int)(backward_length * COS_ANGLE(slope_rotate_angle_top)); // mm
        wash_info.info_lists[cur_list_num].width = (int)(backward_length * SIN_ANGLE(slope_rotate_angle_top)); // mm

        if (rows_num % EF_RCD_LIST_LEN == 0)
        {
            ef_rcd_write(&wash_info);
            memset(wash_info.info_lists, 0, EF_RCD_LIST_LEN * sizeof(struct wash_row_info));
        }

        if(short_slope_flag == 0)
        {
            move_forward_with_speed_and_time(5000, 2 * 1000);
            move_stop_time(500);

            // -------------坡下行为处理
            LOG_I("slope down processing ...end_info %d first_time %d short_slope_flag %d short_slope_time %d", forward_end_info, first_time, short_slope_flag, short_slope_time);
            if (get_bat_cap_type() != BAT_5000MAH)
            {
                LOG_I("clean slope edge");
                move_wp_speed_on_floor_set(edge_wp_speed);
                //rt_thread_mdelay(3 * 1000);
                rt_thread_mdelay(1000);
                cur_yaw = get_current_yaw();
                move_forward_with_speed_and_time(edge_move_speed, 6 * 1000);
                move_stop_time(500);
                /* 与前进洗边缘方向相差太大，则调整到前进方向，再后退 */
                delta = compare_yaws(get_current_yaw(), cur_yaw);
                if(fabs(delta) > 10)
                    move_rotate_on_floor_using_target(cur_yaw, 20 * 1000);
                
                move_backward_with_speed_and_time(edge_move_speed, 5 * 1000);
                move_stop_time(500);
                move_wp_speed_on_floor_set(wp_speed);
                //rt_thread_mdelay(3 * 1000);
                rt_thread_mdelay(1000);
            }
        }

        // 纠一下坡角度
        cur_yaw = get_current_yaw();
        delta = compare_yaws(cur_yaw, floor_yaw);
        /* 过滤小角度，防止yaw角偏移 */
        if(fabs(delta) >= small_angle_thres)
        {
            move_rotate_on_floor_using_target(floor_yaw, 20 * 1000);
            move_stop_time(500);
        }

        // is near the start line on floor
        end_flag = _is_near_start_line(start_floor_yaw);
        if (end_flag == 1 && switch_slope_flag >= 1)
        {
            LOG_I("near start line, time to end");
            goto _end;
        }

        get_current_slope_info(&cur_slope_info);
        delta = compare_yaws(cur_yaw, one_slope_yaw);
        LOG_I("cur yaw %f, one slope yaw %f, delta %f", cur_yaw, one_slope_yaw, delta);
        if (fabs(delta) > 80.0f)
        {
            wash_slope_count++;
            if (wash_slope_count >= 3)
            {
                LOG_I("too much wash slope, stop wash slope %d", wash_slope_count);
                goto _end;
            }

            one_slope_yaw = cur_yaw;
            LOG_I("update wash slope %d one_slope_yaw %f", wash_slope_count, one_slope_yaw);
        }

        delta = compare_yaws(cur_yaw, start_floor_yaw);
        if (wash_slope_count >= 2 && delta > 10.0f)
        {
            if(switch_slope_flag == 0)
            {
                LOG_I("wash_slope_count %d, start reverse", wash_slope_count);
                wash_slope_count = 1;
                goto _reverse;
            }
            else
            {
                cur_slope = get_current_slope();
                if (cur_slope < -7.0f && cur_slope > up_slope_start_pitch)
                {
                    LOG_I("reach small slope time to end");
                    goto _end;
                }
            }
        }

        if ((forward_end_info == 0 || forward_end_info == 2))
        {
            if (short_slope_flag == 1) // 短坡
            {
                // if (short_slope_time == 0) // 第一次遇到短坡,调转180度，前进
                // {
                //     // LOG_D("slope down, slow backward to clean");
                //     // yaw = get_current_yaw();
                //     // move_backward_with_pid_and_time(move_speed_slow, yaw, backward_time);
                //     // move_stop_time(500);
                // }
                // 遇到两次浅水区的情况就停止；在第一次浅水区事件触发后，切换机器对边后，若是连续3次还是在浅坡区，就停止；
                if (short_slope_time >= 1 + 8) // 过滤掉N+1的坡
                {
                    LOG_W("too much short slope, stop wash slope %d", short_slope_time);
                    goto _end;
                }

                short_slope_time++;
                LOG_I("short slope times %d", short_slope_time);
                
                delta = compare_yaws(cur_slope_info.dir, start_slope_info.dir);
                LOG_I("slope dir: cur %.2f, start %.2f, delta %.2f", cur_slope_info.dir, start_slope_info.dir, delta);
                if (switch_slope_flag == 0 && (delta > 120 || delta < -120))
                {
                _reverse:
                    LOG_I("short slope, reverse head direction");
                    /* 在缓坡上旋转到坡yaw为90度，成功则更新洗地开始的yaw角 */
                    // if(move_rotate_on_slope_using_target(90, 20*1000) == RT_EOK)
                    // {
                    //     move_stop_time(500);
                    //     get_current_slope_info(&cur_slope_info);
                    // // reverse_yaw = cur_slope_info.euler_yaw;
                    // //*(float*)p_arg_out = cur_slope_info.euler_yaw;
                    //     delta = compare_yaws(cur_slope_info.euler_yaw, floor_end_yaw);
                    //     if(fabs(delta) > 20.0f)
                    //     {
                    //         LOG_D("diff floor yaw, cur yaw %f, floor end yaw %f", cur_slope_info.euler_yaw, floor_end_yaw);
                    //         extern void wash_floor_info_start_yaw_set_2(float yaw);
                    //         wash_floor_info_start_yaw_set_2(cur_slope_info.euler_yaw);
                    //     }
                    // }
                    yaw = get_current_yaw();
                    float dst_yaw = calculate_yaw(floor_end_yaw, 180.0f);
                    float delta = compare_yaws(yaw, dst_yaw);
                    LOG_I("cur_yaw %.2f, delta_yaw %.2f, dst_yaw %.2f", yaw, delta, dst_yaw);
                    if(delta > 90.0f)
                    {
                        LOG_I("left rotate 90");
                        move_rotate_on_floor(90.0f, 20*1000);
                        move_stop_time(500);
                    }
                    else if(delta < -90.0f)
                    {
                        LOG_I("right rotate 90");
                        move_rotate_on_floor(-90.0f, 20*1000);
                        move_stop_time(500);
                    }
                    move_rotate_on_floor_using_target(dst_yaw, 20 * 1000);
                    move_stop_time(500);
                    LOG_I("reverse head direction done %f", get_current_yaw());
                    // _reverse_head_direction();
                    switch_slope_flag++;
                    one_slope_yaw = get_current_yaw();
                    LOG_I("update wash slope %d one_slope_yaw %.2f switch flag %d", wash_slope_count, one_slope_yaw, switch_slope_flag);
                }
                else
                {
                    // slow backward to clean
                    LOG_I("slope down, slow backward to clean");
                    yaw = get_current_yaw();
                    move_backward_with_pid_and_time(move_speed_slow, yaw, backward_time);
                    move_stop_time(500);
                    // roate
                    move_rotate_on_floor(-slope_rotate_angle_floor, 20 * 1000);
                    move_stop_time(1000);
                    float cur_yaw = get_current_yaw();
                    delta = compare_yaws(cur_yaw, floor_yaw);
                    if (fabs(delta) > slope_rotate_angle_floor + 10.0f)
                    {
                        LOG_I("roate too much, cur yaw %.2f, prv yaw %.2f, delta %.2f", cur_yaw, floor_yaw, delta);
                        move_rotate_on_floor(fabs(delta - slope_rotate_angle_floor), 20 * 1000);
                        move_stop_time(1000);
                    }
                }
            }
            else // 不是短坡
            {
                // slow backward to clean
                LOG_I("slope down, slow backward to clean");
                yaw = get_current_yaw();
                move_backward_with_pid_and_time(move_speed_slow, yaw, backward_time);
                move_stop_time(500);
                // roate
                move_rotate_on_floor(-slope_rotate_angle_floor, 20 * 1000);
                move_stop_time(1000);
            }
        }
        else if (forward_end_info == 1) // 上行踩到平面，则调头转180度
        {
            LOG_I("on flat plane, backward to slope");
#if 0           
            yaw = get_current_yaw();
            move_backward_with_pid_and_time(move_speed_slow, yaw, backward_time);
            move_stop_time(500);
            _reverse_head_direction();
            wash_slope_count++;
            switch_slope_flag++;
            one_slope_yaw = get_current_yaw();
            LOG_D("update wash slope %d one_slope_yaw %.2f switch flag %d", wash_slope_count, one_slope_yaw, switch_slope_flag);
#else
            // slow backward to clean
            LOG_D("slope down, slow backward to clean");
            yaw = get_current_yaw();
            move_backward_with_pid_and_time(move_speed_slow, yaw, backward_time);
            move_stop_time(500);
            // roate
            move_rotate_on_floor(-slope_rotate_angle_floor, 20 * 1000);
            move_stop_time(1000);
#endif
        }
    }

_end:
    /* 清洁记录信息结束表头 */
    wash_info.total_rows_num = rows_num;
    wash_info.end_time = rt_tick_get_millisecond();
    get_battery_soc(&wash_info.end_soc);
    ef_rcd_write(&wash_info);
    LOG_I("wash info, total_rows_num = %d, end_time = %d", wash_info.total_rows_num, wash_info.end_time);

    // find deep water area
    LOG_I("find deep water area...");
    move_backward_with_speed_and_time(move_speed, 3 * 1000);
    move_stop_time(500);
    cur_yaw = get_current_yaw();
    delta = compare_yaws(cur_yaw, initial_yaw);
    if(fabs(delta) >= small_angle_thres)
    {
        move_rotate_on_floor_using_target(initial_yaw, 20 * 1000);
        rt_thread_mdelay(1 * 1000);
    }
    LOG_I("search slope");
    move_forward_with_speed(move_speed);
    start_tick = rt_tick_get();
    find_slope_min = 0.0f;
    while (1)
    {
        cur_slope = get_current_slope();
        if (cur_slope < up_slope_start_pitch)
        {
            LOG_I("find slope done %.2f", cur_slope);
            break;
        }

        if (rt_tick_get() - start_tick > find_slope_time) // 找坡超时
        {
            LOG_W("find slope timeout, end search slope");
            break;
        }
        rt_thread_mdelay(100);
    }
    move_stop_time(500);

    LOG_I("back to flat area");
    move_backward_with_speed(move_speed);
    while (1)
    {
        cur_slope = get_current_slope();
        if (cur_slope > up_slope_start_pitch) // 从坡上落下，则跳过
        {
            LOG_I("backward end slope %f", cur_slope);
            break;
        }
        move_backward_with_speed(move_speed); // 反复发送，以防有不响应的情况；
        rt_thread_mdelay(100);
        // yaw = get_current_yaw_on_wall();
        // move_backward_with_pid(move_speed, yaw, 0.0f);
    }
    move_stop_time(1000);

    LOG_I("back to floor end yaw %.2f", floor_end_yaw);
    cur_yaw = get_current_yaw();
    delta = compare_yaws(cur_yaw, floor_end_yaw);
    if(fabs(delta) >= small_angle_thres)
    {
        move_rotate_on_floor_using_target(floor_end_yaw, 20 * 1000);
        rt_thread_mdelay(1 * 1000);
    }
    LOG_I("back to floor end yaw done %.2f", get_current_yaw());
    LOG_I("wash slope action end....");

    // for debug
#if 0
    rt_thread_mdelay(30 * 1000);
    LOG_D("begin to climb up wall for fetching...");
    struct stop_on_waterline_info stop_info = {
        .on_waterline_time = 30, // 30s
        .in_water_time = 60,     // 60s
        .repeat_count = 3};      // repeat 3 times
    actions_on_wall_stop_on_waterline(&stop_info, RT_NULL);

    move_wp_off_on_floor();
    move_stop();
    while (1)
    {
        rt_thread_mdelay(1000);
    }
#endif
    return 0;
}
