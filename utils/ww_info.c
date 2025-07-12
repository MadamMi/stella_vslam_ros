#include "task_comm.h"
#include "util.h"
#include "sensors_info.h"
#include "app_config.h"
#include "wash_wall_task.h"
#include "move_basic.h"
#include "actions_on_wall.h"
#include "easyflash.h"

#define DBG_TAG "wash_wall_task"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

const int WW_WP_SPEED = 3000;
struct wash_record_info g_wash_wall_record_info = {0};

static int wash_wall_send_msg_to_action(planner_to_action_msg_t msg)
{
    return rt_mq_send(&g_planner_to_action_mq, msg, sizeof(struct planner_to_action_msg));
}

static int wash_wall_recv_msg_from_action(action_msg_t *msg, int timeout)
{
    return rt_mb_recv(&g_action_mb, (rt_ubase_t *)msg, timeout);
}

static rt_err_t _cmd_send_then_recv_msg(planner_to_action_msg_t cmd_msg, action_msg_t *ack_msg)
{
    rt_err_t ret = wash_wall_send_msg_to_action(cmd_msg);
    if (ret != RT_EOK)
    {
        LOG_E("send failed %d", ret);
        return -1;
    }

    // wait msg
    ret = wash_wall_recv_msg_from_action(ack_msg, RT_WAITING_FOREVER);
    if (ret != RT_EOK)
    {
        LOG_E("recv failed %d", ret);
        return -2;
    }

    LOG_D("recv %d", (*ack_msg)->msg);
    return 0;
}

int wash_wall_thread(void)
{
    move_wp_speed_on_floor_set(WW_WP_SPEED);
    rt_thread_mdelay(1 * 1000);

    int clean_times = 0; // 继续上下清洗的次数
    struct find_next_edge_info next_edge_info = {0};
    float start_yaw; // 记录机器姿态
    int wash_line_time, move_time;

    // 洗墙找墙
    action_msg_t action_msg_handle = RT_NULL;
    //    monitor_msg_t monitor_msg_handle = RT_NULL;
    struct planner_to_action_msg p2a_msg;
    int mode = 1, out_value; // 找墙模式，==1为洗墙找墙
    p2a_msg.cmd = CMD_ACTION_FIND_EDGE;
    LOG_D("CMD_ACTION_FIND_EDGE");
    p2a_msg.p_arg_in = &mode;
    p2a_msg.p_arg_out = &out_value;
    wash_wall_send_msg_to_action(&p2a_msg);

    // wait msg
    int ret = wash_wall_recv_msg_from_action(&action_msg_handle, RT_WAITING_FOREVER);
    LOG_D("action_msg_handle.msg=%d", action_msg_handle->msg);
    LOG_D("CMD_ACTION_FIND_EDGE p2a_msg.p_arg_out=%d", *(int *)p2a_msg.p_arg_out);
    if (ret != 0) // 没有通信成功
    {
        LOG_E("CMD_ACTION_FIND_EDGE recv msg ERR!");
        // 亮灯效
        return -1;
    }

    if (out_value == -1) // 找墙失败
    {
        LOG_E("Find edge failed!");
        // 亮灯效
        return -1;
    }
    start_yaw = get_current_yaw();
    LOG_D("start yaw = %f", start_yaw);
    int climb_times;
START:
    // 爬墙
    climb_times = 0;      // 记录爬墙次数
    if (clean_times > 30) // 共清洗30次
    {
        LOG_D("Finish wash wall.");
        goto EXIT_WASH_WALL;
    }

CLIMB:
    if (climb_times > 20) // 爬墙次数超过20次，退出洗墙
    {
        LOG_E("CMD_ACTION_WALL_CLIMB_UP failed!");
        return -1;
    }
    p2a_msg.cmd = CMD_ACTION_WALL_CLIMB_UP;
    LOG_D("CMD_ACTION_WALL_CLIMB_UP");
    p2a_msg.p_arg_out = &out_value; // 成功返回0，失败返回-1
    wash_wall_send_msg_to_action(&p2a_msg);

    // wait msg
    ret = wash_wall_recv_msg_from_action(&action_msg_handle, RT_WAITING_FOREVER);
    LOG_D("CMD_ACTION_WALL_CLIMB_UP p2a_msg.p_arg_out=%d", out_value);
    if (ret != 0) // 没有通信成功
    {
        LOG_E("CMD_ACTION_WALL_CLIMB_UP recv msg ERR!");
        // 亮灯效
        return -1;
    }
    if (out_value == -1) // 爬墙失败，u形找墙重试
    {
        climb_times += 1;
        LOG_D("Climb up wall failed! climb time = %d", climb_times);

        // u形找墙
        p2a_msg.cmd = CMD_ACTION_WALL_FIND_NEXT_ENTRY;
        LOG_D("CMD_ACTION_WALL_FIND_NEXT_ENTRY");
        next_edge_info.back_time = 1000;
        next_edge_info.forward_time = 3000;
        next_edge_info.thres = 200;
        p2a_msg.p_arg_in = &next_edge_info;
        p2a_msg.p_arg_out = &out_value;
        wash_wall_send_msg_to_action(&p2a_msg);

        // wait msg
        ret = wash_wall_recv_msg_from_action(&action_msg_handle, RT_WAITING_FOREVER);
        LOG_D("CMD_ACTION_WALL_FIND_NEXT_ENTRY p2a_msg.p_arg_out=%d", out_value);
        if (ret != 0) // 没有通信成功
        {
            LOG_E("CMD_ACTION_WALL_FIND_NEXT_ENTRY recv msg ERR!");
            // 亮灯效
            return -1;
        }
        start_yaw = get_current_yaw();
        LOG_D("start yaw = %f", start_yaw);
        goto CLIMB;
    }
    else // 爬墙成功
    {
        climb_times = 0;
        // 纠正机器姿态到初始角度，开始洗墙
        // 上墙
        p2a_msg.cmd = CMD_ACTION_WALL_MOVE_FORWARD;
        LOG_D("CMD_ACTION_WALL_MOVE_FORWARD");

        int foward_time_max = 15 * 1000; // 超时时间
        p2a_msg.p_arg_in = &foward_time_max;
        p2a_msg.p_arg_out = &out_value;
        wash_wall_send_msg_to_action(&p2a_msg);

        // wait msg
        ret = wash_wall_recv_msg_from_action(&action_msg_handle, RT_WAITING_FOREVER);
        LOG_D("CMD_ACTION_WALL_MOVE_FORWARD, p2a_msg.p_arg_out=%d", out_value);
        out_value = 1;

        if (ret != 0) // 没有通信成功
        {
            LOG_E("CMD_ACTION_WALL_MOVE_FORWARD recv msg ERR!");
            // 亮灯效
            return -1;
        }
        if (out_value == 3) // 超时，机器可能卡住了
        {
            // 下墙
            p2a_msg.cmd = CMD_ACTION_WALL_VENT_GAS;
            LOG_D("CMD_ACTION_WALL_VENT_GAS");

            wash_wall_send_msg_to_action(&p2a_msg);

            // wait msg
            ret = wash_wall_recv_msg_from_action(&action_msg_handle, RT_WAITING_FOREVER);
            if (ret != 0) // 没有通信成功
            {
                LOG_E("CMD_ACTION_WALL_VENT_GAS recv msg ERR!");
                // 亮灯效
                return -1;
            }
            // 把机器纠到初始角度，走到墙边
            move_rotate_on_floor_using_target(start_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
            p2a_msg.cmd = CMD_ACTION_FLOOR_MOVE_TO_EDGE;
            LOG_D("CMD_ACTION_FLOOR_MOVE_TO_EDGE");

            wash_wall_send_msg_to_action(&p2a_msg);
            ret = wash_wall_recv_msg_from_action(&action_msg_handle, RT_WAITING_FOREVER);
            if (ret != 0) // 没有通信成功
            {
                LOG_E("CMD_ACTION_FLOOR_MOVE_TO_EDGE recv msg ERR");
                // 亮灯效
                return -1;
            }
            // U形找墙
            goto NEXT_EDGE;
        }

        // // 锯齿状洗水线
        // struct wash_water_line_info water_line_info;
        // water_line_info.alpha = 10;
        // water_line_info.back_time = 1000;
        // water_line_info.nums = 5;
        // p2a_msg.cmd = CMD_ACTION_WALL_CLEAN_WATER_LINE;
        // p2a_msg.p_arg_in = &water_line_info;

        // 差速洗水线
        wash_line_time = 10 * 1000;
        p2a_msg.cmd = CMD_ACTION_WALL_CLEAN_WATER_LINE_DIFF;
        p2a_msg.p_arg_in = &wash_line_time;
        LOG_D("CMD_ACTION_WALL_CLEAN_WATER_LINE_DIFF");
        wash_wall_send_msg_to_action(&p2a_msg);

        // wait msg
        ret = wash_wall_recv_msg_from_action(&action_msg_handle, RT_WAITING_FOREVER);
        if (ret != 0) // 没有通信成功
        {
            LOG_E("CMD_ACTION_WALL_CLEAN_WATER_LINE_DIFF recv msg ERR!");
            // 亮灯效
            return -1;
        }

        // 下墙
        p2a_msg.cmd = CMD_ACTION_WALL_CLIMB_DOWN;
        LOG_D("CMD_ACTION_WALL_CLIMB_DOWN");
        move_time = 60 * 1000; // 超时时间
        p2a_msg.p_arg_in = &move_time;
        p2a_msg.p_arg_out = &out_value;
        wash_wall_send_msg_to_action(&p2a_msg);

        // wait msg
        ret = wash_wall_recv_msg_from_action(&action_msg_handle, RT_WAITING_FOREVER);
        LOG_D("CMD_ACTION_WALL_CLIMB_DOWN, p2a_msg.p_arg_out=%d", out_value);
        if (ret != 0) // 没有通信成功
        {
            LOG_E("CMD_ACTION_WALL_CLIMB_DOWN recv msg ERR!");
            // 亮灯效
            return -1;
        }
        if (out_value == 0) // 下墙失败
        {
            // 关水泵，机器飘下去
            p2a_msg.cmd = CMD_ACTION_WALL_VENT_GAS;
            LOG_D("CMD_ACTION_WALL_VENT_GAS");
            wash_wall_send_msg_to_action(&p2a_msg);

            // wait msg
            ret = wash_wall_recv_msg_from_action(&action_msg_handle, RT_WAITING_FOREVER);
            if (ret != 0) // 没有通信成功
            {
                LOG_E("CMD_ACTION_WALL_VENT_GAS recv msg ERR!");
                // 亮灯效
                return -1;
            }
            // 将机器纠到初始角度
            move_rotate_on_floor_using_target(start_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
            // 走到墙边
            p2a_msg.cmd = CMD_ACTION_FLOOR_MOVE_TO_EDGE;
            LOG_D("CMD_ACTION_FLOOR_MOVE_TO_EDGE");
            wash_wall_send_msg_to_action(&p2a_msg);
            ret = wash_wall_recv_msg_from_action(&action_msg_handle, RT_WAITING_FOREVER);
            if (ret != 0) // 没有通信成功
            {
                LOG_E("CMD_ACTION_FLOOR_MOVE_TO_EDGE recv msg ERR");
                // 亮灯效
                return -1;
            }
        }

        clean_times += 1;
    NEXT_EDGE:
        // u形找墙
        p2a_msg.cmd = CMD_ACTION_WALL_FIND_NEXT_ENTRY;
        LOG_D("NEXT_EDGE, CMD_ACTION_WALL_FIND_NEXT_ENTRY");
        next_edge_info.back_time = 1000;
        next_edge_info.forward_time = 3000;
        next_edge_info.thres = 200;
        p2a_msg.p_arg_in = &next_edge_info;
        p2a_msg.p_arg_out = &out_value;
        wash_wall_send_msg_to_action(&p2a_msg);

        // wait msg
        ret = wash_wall_recv_msg_from_action(&action_msg_handle, RT_WAITING_FOREVER);
        LOG_D("NEXT_EDGE, CMD_ACTION_WALL_FIND_NEXT_ENTRY, p2a_msg.p_arg_out=%d", out_value);
        if (ret != 0) // 没有通信成功
        {
            LOG_E("CMD_ACTION_WALL_FIND_NEXT_ENTRY recv msg ERR!");
            // 亮灯效
            return -1;
        }
        start_yaw = get_current_yaw();
        LOG_D("start yaw = %f", start_yaw);
        goto START;
    }

EXIT_WASH_WALL:
    // int off_time = 1; // 1s后关机
    set_battery_power_off_time(1);
    return 0;
}

int all_mode_wash_wall_routine_v(int flag)
{
    move_wp_speed_on_floor_set(WW_WP_SPEED);
    rt_thread_mdelay(1000);

    const int mode = 1; // 找墙模式，==1为洗墙找墙
    const struct wash_wall_info wall_info = {.num = 5, .angle = 20.0f};

    const int timeout = 10 * 1000;
    const int forward_time = 30 * 1000;
    const int climbup_max_times = 10; // 最大连续爬墙失败次数
                                      //    const int shallow_water_time = 12 * 1000;

    int wash_wall_max_time;
    int out_value;
    int climbup_times = 0, on_wall_counter = -1;
    struct find_next_edge_info u_info = {.back_time = 1 * 1000, .forward_time = 2 * 1000, .thres = 5000, .detect_time = 2 * 1000};
    struct forward_on_wall_info_out forward_info_out = {0};
    int shallow_no_detect_time = 0;

    RT_UNUSED(shallow_no_detect_time);

    LOG_D("slope flag = %d", flag);
    system_info_t info = system_info_get();
    LOG_D("model_mode=%d", info->work_mode);
    switch (info->work_mode)
    {
    case ALL_MODE:
        wash_wall_max_time = 90 * 60 * 1000;
        shallow_no_detect_time = 60 * 60 * 1000;
        break;
    case WATERLINE_MODE:
        wash_wall_max_time = 210 * 60 * 1000;
        shallow_no_detect_time = 120 * 60 * 1000;
        break;
    default:
        wash_wall_max_time = 4 * 60 * 60 * 1000;
        shallow_no_detect_time = 4 * 60 * 60 * 1000;
        break;
    }
    LOG_D("wash_wall_max_time=%d min", wash_wall_max_time / 60000);

    // find wall
    struct planner_to_action_msg p2a_msg;
    action_msg_t action_msg_handle = RT_NULL;

    p2a_msg.cmd = CMD_ACTION_FIND_EDGE;
    p2a_msg.p_arg_in = (void *)&mode;
    p2a_msg.p_arg_out = &out_value;
    LOG_D("find wall start");
    _cmd_send_then_recv_msg(&p2a_msg, &action_msg_handle);
    if (out_value == -1) // 找墙失败
    {
        LOG_E("Find edge failed %d", out_value);
        // 亮灯效
        return -1;
    }
    LOG_D("find wall end");
    u_info.pre_event = 3;
    float start_yaw = get_current_yaw();
    LOG_D("start yaw = %f", start_yaw);

    rt_uint16_t vol;
    get_battery_voltage(&vol);
    LOG_D("current voltage: %dmV", vol);

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
    LOG_D("off_value=%d", off_value);

    rt_tick_t start_time = rt_tick_get(), end_time = 0;
    g_wash_wall_record_info.region_flag = REGION_WALL;
    g_wash_wall_record_info.wash_num = eeprom_total_clean_counts_get();
    g_wash_wall_record_info.start_time = start_time;
    g_wash_wall_record_info.wash_time = 1;
	get_battery_soc(&g_wash_wall_record_info.start_soc);
    while (1)
    {

    _u_turn:
		get_battery_voltage(&vol);
        LOG_D("current voltage: %dmV", vol);
        if (vol <= off_value)
        {
            LOG_D("reach off_value=%d", off_value);
            break;
        }
		
        end_time = rt_tick_get() - start_time;
        if (end_time >= wash_wall_max_time)
        {
            LOG_D("wash wall timeout %d", end_time);
            break;
        }

        // if (forward_info_out.forward_time < shallow_water_time && end_time > shallow_no_detect_time)
        // {
        //     LOG_D("shallow water detect %d[%d]", forward_info_out.forward_time, end_time);
        // }

        // u turn check
        p2a_msg.cmd = CMD_ACTION_WALL_FIND_NEXT_ENTRY;
        p2a_msg.p_arg_in = (void *)&u_info;
        p2a_msg.p_arg_out = &out_value;
        LOG_D("CMD_ACTION_WALL_FIND_NEXT_ENTRY start");
        _cmd_send_then_recv_msg(&p2a_msg, &action_msg_handle);
        LOG_D("CMD_ACTION_WALL_FIND_NEXT_ENTRY end: %d", out_value);

        // climb up
        p2a_msg.cmd = CMD_ACTION_WALL_CLIMB_UP;
        p2a_msg.p_arg_in = RT_NULL;
        p2a_msg.p_arg_out = &out_value;
        LOG_D("CMD_ACTION_WALL_CLIMB_UP start");
        _cmd_send_then_recv_msg(&p2a_msg, &action_msg_handle);
        LOG_D("CMD_ACTION_WALL_CLIMB_UP end: %d", out_value);
        if (out_value != 0)
        {
            LOG_D("CMD_ACTION_WALL_CLIMB_UP fail %d", out_value);
            climbup_times++;
            if (climbup_times >= climbup_max_times)
            {
                LOG_D("CMD_ACTION_WALL_CLIMB_UP fail %d times", climbup_times);
                break;
            }
            u_info.pre_event = out_value;
            goto _u_turn;
        }
        climbup_times = 0;

        // move forward on wall
        on_wall_counter++;
        p2a_msg.cmd = CMD_ACTION_WALL_MOVE_FORWARD;
        p2a_msg.p_arg_in = (void *)&forward_time;
        p2a_msg.p_arg_out = &forward_info_out;
        LOG_D("CMD_ACTION_WALL_MOVE_FORWARD start");
        _cmd_send_then_recv_msg(&p2a_msg, &action_msg_handle);
        LOG_D("CMD_ACTION_WALL_MOVE_FORWARD end: %d", forward_info_out.status);
        if (forward_info_out.status == 3)
        {
            LOG_D("time out, maybe block on stairs or something else");
            goto _down;
        }

        if (forward_info_out.status == 4)
        {
            LOG_D("find sundesk..."); // 也可能是进气飘落，但这是机器异常情况，一出水就大量进气，逻辑上没有好的解决方案，暂不做处理
            u_info.pre_event = 2;
            goto _u_turn;
        }

        // wash wall
        p2a_msg.cmd = CMD_ACTION_WALL_CLEAN_WALL;
        p2a_msg.p_arg_in = (void *)&wall_info;
        p2a_msg.p_arg_out = &out_value;
        LOG_D("CMD_ACTION_WALL_CLEAN_WALL start");
        _cmd_send_then_recv_msg(&p2a_msg, &action_msg_handle);
        LOG_D("CMD_ACTION_WALL_CLEAN_WALL end: %d", out_value);
        if (out_value == -1)
        {
            u_info.pre_event = 4;
            goto _u_turn;
        }
        else if (out_value == 4)
        {
            u_info.pre_event = 5;
            goto _u_turn;            
        }
        else
        {
        }

    _down:
        // backward
        rt_thread_mdelay(500);
        p2a_msg.cmd = CMD_ACTION_WALL_MOVE_BACKWARD;
        p2a_msg.p_arg_in = (void *)&timeout;
        p2a_msg.p_arg_out = &out_value;
        LOG_D("CMD_ACTION_WALL_MOVE_BACKWARD start");
        _cmd_send_then_recv_msg(&p2a_msg, &action_msg_handle);
        LOG_D("CMD_ACTION_WALL_MOVE_BACKWARD end : %d", out_value);
        if (out_value != 0)
        {
            LOG_D("CMD_ACTION_WALL_MOVE_BACKWARD fail");
        }

        // climb down
        rt_thread_mdelay(500);
        p2a_msg.cmd = CMD_ACTION_WALL_CLIMB_DOWN;
        p2a_msg.p_arg_in = (void *)&timeout;
        p2a_msg.p_arg_out = &out_value;
        LOG_D("CMD_ACTION_WALL_CLIMB_DOWN start");
        _cmd_send_then_recv_msg(&p2a_msg, &action_msg_handle);
        LOG_D("CMD_ACTION_WALL_CLIMB_DOWN end: %d", out_value);
        if (out_value != 0)
        {
            LOG_D("CMD_ACTION_WALL_CLIMB_DOWN fail");
        }

        int u_flag = (on_wall_counter % 3 == 2);
        u_info.pre_event = u_flag ? -1 : 0;
        get_battery_voltage(&vol);
        LOG_D("current voltage: %dmV", vol);
    }

    rt_thread_mdelay(1000);
    LOG_D("wash wall end off water pump");
    move_wp_off_on_wall();
    g_wash_wall_record_info.end_time = rt_tick_get();
	get_battery_soc(&g_wash_wall_record_info.end_soc);
    ef_rcd_write(&g_wash_wall_record_info);
    if (flag != 0)
    {
        const int slope_fag = 1;
        extern int actions_on_floor_goto_shallow_turn_off(void *p_arg_in, void *p_arg_out);
        actions_on_floor_goto_shallow_turn_off((void *)&slope_fag, NULL);
    }
    else
    {
        if (waterline_label == 1)
        {
            extern void stop_at_waterline(void);
            stop_at_waterline();
        }
    }

    return 0;
}

extern int slope_label;

int8_t washWallModeEnter(void)
{
    LOG_I("pln fsm %s\n", __FUNCTION__); 
    return 0;
}


int8_t washWallModeRun(void)
{
    all_mode_wash_wall_routine_v(slope_label);
	return 0;
}


int8_t washWallModeExit(void)
{
	LOG_I("pln fsm %s\n", __FUNCTION__);
        
	return 0;
}


void planner_test(int argc, char *argv[])
{
    int value = 1, out_value;
    struct planner_to_action_msg p2a_msg;
    action_msg_t action_msg_handle = RT_NULL;

    p2a_msg.cmd = CMD_DEMO;
    p2a_msg.p_arg_in = &value;
    p2a_msg.p_arg_out = &out_value;

    _cmd_send_then_recv_msg(&p2a_msg, &action_msg_handle);
    LOG_D("planner_test: cmd = %d, in = %d, out = %d", p2a_msg.cmd, value, out_value);
}

MSH_CMD_EXPORT(planner_test, planner cmd to action test);
