#include "rtthread.h"
#include "task_comm.h"
#include "planner.h"
#include "app_config.h"
#include "util.h"
#include "wash_floor_task.h"
#include "wash_wall_task.h"
#include "move_basic.h"
#include "system_info.h"
#include "actions_on_floor.h"
#include "sensors_info.h"
#include "mtr_ctrl.h"
#include "planner_info.h"
#include "wash_floor_info.h"
#include "planner_mode_control.h"

#define DBG_TAG "planner"
#define DBG_LVL DBG_INFO
//#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static int INIT_WP_SPEED = 2000;
// int slope_label = -1;
/**
 * @brief extern varibles
 *
 */

/**
 * @brief extern functions
 *
 */

/**
 * @brief public variables
 *
 */

/**
 * @brief private variables
 *
 */

rt_align(RT_ALIGN_SIZE) static char planner_thread_stack[PLANNER_THREAD_STACK_SIZE];
static struct rt_thread planner_thread;

/**
 * @brief private functions
 *
 */
#if 0
static int send_msg_to_action(planner_to_action_msg_t msg)
{
    return rt_mq_send(&g_planner_to_action_mq, msg, sizeof(struct planner_to_action_msg));
}

static int send_msg_to_monitor(planner_to_monitor_msg_t msg)
{
    return rt_mq_send(&g_planner_to_monitor_mq, msg, sizeof(struct planner_to_monitor_msg));
}

static int recv_msg_from_action(action_msg_t *msg, int timeout)
{
    return rt_mb_recv(&g_action_mb, (rt_ubase_t *)msg, timeout);
}

static int recv_msg_from_monitor(monitor_msg_t *msg, int timeout)
{
    return rt_mb_recv(&g_monitor_mb, (rt_ubase_t *)msg, timeout);
}
#endif
static void planner_thread_entry(void *param)
{
    LOG_I("planner build time = [%s %s]", __DATE__, __TIME__);
#if 1
    system_info_t info = system_info_get();
    LOG_I("planner thread startup");
#ifdef FLOOR_TEST
    rt_thread_mdelay(1000 * 30);
    // ota
    if (is_ota_mode())
    {
        // LOG_I("enter suspend mode");
        while (1)
        {
            rt_thread_mdelay(1000 * 1);
        }
    }
#else

    fsmRegist(&pln_fsm, pln_fsmTable, sizeof(pln_fsmTable)/sizeof(fsmTableTypeDef), pln_fsmFuncTable);
    fsmInit(&pln_fsm);

    // planner等待入水
    const int IN_WATER_WAIT_TIME = 10; // unit: second
    int suspend_mode = 0, in_water_count = 0;
    struct speed_info set_wp_and_motor_speed;
    RT_UNUSED(set_wp_and_motor_speed);
    static rt_uint16_t pl_vol;
    while (1)
    {
        if (suspend_mode == 0 && (is_ota_mode() || is_test_mode()))
        {
            suspend_mode = 1;
            LOG_I("enter suspend mode");
        }

        if (suspend_mode == 1 && !(is_ota_mode() || is_test_mode()))
        {
            suspend_mode = 0;
            LOG_I("exit suspend mode");
        }

        if (suspend_mode != 0)
            continue;

        if (info->robot_mode == ROBOT_RUNNING_MODE)
        {
            in_water_count++;
            LOG_D("in water count %d", in_water_count);
            if (in_water_count >= IN_WATER_WAIT_TIME)
            {
                break;
            }
        }
        else
        {
            // LOG_D("wait in water, info state=%d, info woke_mode=%d", info.state, info.work_mode);
        }

        rt_thread_mdelay(1000);
    }
#endif
    
    LOG_D("reset motor error");
    mtr_reset_error();
    // info.work_mode = CLEAN_FIRST;
    // system_info_set(&info); // 确保工作模式正确
    LOG_I("info.work_mode = %d", info->work_mode);
    LOG_I("info.state = %d", info->state);
#if (VERSION_TYPE==E_AUSTRALIA)
    rt_thread_mdelay(15*1000);
    reset_imu(15*1000, 0);
#endif
    move_wp_speed_on_floor_set(INIT_WP_SPEED);
    rt_thread_mdelay(5*1000);
    // rt_thread_mdelay(1*1000);
    LOG_D("motor power on,start planner.");

    // TODO:判断落到池底
    // rt_thread_mdelay(1000*5);
    // 判断工作模式

    while(1)
    {
        fsmEventHandle(&pln_fsm);
        plnFsmEventManage();
        // rt_thread_mdelay(1000);
    }

    // switch (info->work_mode)
    // {
    // case SMART_MODE:
    //     LOG_I("in SMART_MODE");
    //     wash_floor_thread(info->work_mode);
	// 	LOG_I("smart mode end");
    //     break;
    
    // case WEEKLY_MODE:
    //     LOG_I("in WEEKLY_MODE");
    //     wash_floor_thread(info->work_mode);
	// 	LOG_I("weekly mode end");
    //     break;
    
    // case WATERLINE_MODE:
    //     LOG_I("in WATERLINE_MODE");
    //     all_mode_wash_wall_routine_v(slope_label);
    //     LOG_I("waterline mode end");
    //     break;

    // case ALL_MODE:  // floor and wall
    //     LOG_I("in ALL_MODE");
        
    //     get_battery_voltage(&pl_vol);
    //     LOG_I("current voltage: %dmV", pl_vol);
    //     if (pl_vol > OFF_VOL_THRES)
    //     {
    //         // 洗地
    //         LOG_I("start wash floor");
    //         // get_speed_settings(info->work_mode, pl_vol, &set_wp_and_motor_speed);
    //         // LOG_D("set wp=%d, motor=%d", 
    //         //     set_wp_and_motor_speed.pump_speed, set_wp_and_motor_speed.motor_speed);
    //         // move_wp_speed_on_floor_set(set_wp_and_motor_speed.pump_speed);
    //         // rt_thread_mdelay(1000);
    //         wash_floor_thread(info->work_mode);
    //         LOG_I("end wash floor");
    //     }
    //     // 洗墙
    //     if (pl_vol > OFF_VOL_THRES)
    //     {
    //         LOG_I("start wash wall");
    //         all_mode_wash_wall_routine_v(slope_label);
    //         LOG_I("end wash wall");
    //     }
    //     LOG_I("allmode end");
    //     break;

    // case E_INVALID_MODE:
    //     LOG_I("in E_INVALID_MODE");
    //     break;

    // default:
    //     break;
    // }
	
	move_wp_off_on_floor();
	move_stop();
	//获取当前电压
	get_battery_voltage(&pl_vol);
	LOG_I("end task, vol %d", pl_vol);
	while(pl_vol > IMM_OFF_VOL_THRES)
	{
		get_battery_voltage(&pl_vol);
		rt_thread_mdelay(1000);
	}
	LOG_I("power off, vol %d", pl_vol);
	rt_thread_mdelay(1000);
	power_off_ac(1);	//1s后关机
#endif // #if 1

    // struct move_to_next_row_info info = {.mode = 0, .move_time = 1000};
    // actions_on_floor_move_to_next_line(&info, RT_NULL);
    // move_stop();
    // while (1)
    // {
    //     rt_thread_mdelay(1000);
    // }

    // 开始清洗
    // struct point_info point;
    // get_next_row_point(&point);
    // LOG_D("get row point end");
    // rt_thread_mdelay(1000);
    // LOG_D("rt_thread_mdelay end");

    // action_msg_t action_msg_handle = RT_NULL;
    // monitor_msg_t monitor_msg_handle = RT_NULL;
    // struct planner_to_action_msg p2a_msg;
    // struct planner_to_monitor_msg p2m_msg;

    // float start_yaw = 0.0f;
    // int cur_dis_from_edge;
    // p2a_msg.cmd = CMD_ACTION_FLOOR_MOVE_FORWARD_AND_GET_DIS;
    // p2a_msg.p_arg_in = &start_yaw;
    // p2a_msg.p_arg_out = &cur_dis_from_edge;
    // rt_tick_t start_tick_time, move_tick_time;

    // send_msg_to_action(&p2a_msg);
    // LOG_D("CMD_ACTION_FLOOR_MOVE_FORWARD_AND_GET_DIS");
    // // wait msg
    // start_tick_time = rt_tick_get();
    // int msg_size = recv_msg_from_action(&action_msg_handle, RT_WAITING_FOREVER);
    // rt_kprintf("msg_size=%d\n", msg_size);
    // move_tick_time = rt_tick_get() - start_tick_time;  // 前进时间
    // if (msg_size <= 0) // 没有通信成功
    // {
    //     LOG_E("recv msg ERR");
    //     // 亮灯效
    //     return;
    // }
    // load and send msg to action

#if 0
    rt_thread_mdelay(10 * 1000);
    float yaw = 0.0f;
    int end_dis = 0;

    // p2a_msg.cmd = CMD_ACTION_FLOOR_MOVE_FORWARD_AND_GET_DIS;
    // p2a_msg.p_arg_in = &yaw;
    // p2a_msg.p_arg_out = &end_dis;
    // LOG_D("send to action %d", p2a_msg.cmd);
    // send_msg_to_action(&p2a_msg);

    action_msg_t action_msg_handle = RT_NULL;
    monitor_msg_t monitor_msg_handle = RT_NULL;
    struct planner_to_action_msg p2a_msg;
    struct planner_to_monitor_msg p2m_msg;

    p2a_msg.cmd = CMD_ACTION_FLOOR_GET_NEXT_MAGNETIC;
    struct get_next_gero_in in = {.move_rows = 1, .one_line_move_time = 2000, .row_num = 2};
    p2a_msg.p_arg_in = &in;
    struct get_next_gero_out out;
    p2a_msg.p_arg_out = &out;
    LOG_D("send to action %d", p2a_msg.cmd);
    send_msg_to_action(&p2a_msg);

    // wait msg
    int msg_sz = recv_msg_from_action(&action_msg_handle, RT_WAITING_FOREVER);
    rt_kprintf("msg_sz=%d\n", msg_sz);
    LOG_D("%p %d %d", out.gero_buff, out.gero_buff_sz, out.ret);
    for (size_t i = 0; i < out.gero_buff_sz; i++)
    {
        LOG_D("%f %f %f %f", out.gero_buff[i].raw_mag_x, out.gero_buff[i].raw_mag_y, out.gero_buff[i].raw_mag_z, out.gero_buff[i].yaw);
    }


    // static struct mag_cali_arg_in arg_in;
    // static struct mag_cali_arg_out arg_out;

    // static struct mag_calibrate_info sg_buff[100] = {0};
    // static struct mag_calibrate_info cur_gero_calib_info;
    // struct system_info info;

    // // do
    // // {
    // //     system_info_get(&info);
    // //     rt_thread_mdelay(1000);
    // //     LOG_D("wait in water, info state=%d, info woke_mode=%d", info.state, info.work_mode);
    // // }while(info.state != E_ROBOT_POWER_ON);
    // rt_thread_mdelay(1000*10);
    
    // system_info_get(&info);
    // LOG_D("info.work_mode = %d", info.work_mode);
    // // info.work_mode = FLOOR_ONLY;
    // rt_thread_mdelay(1000*2);
    // move_wp_speed_on_floor_set(PN_WP_SPEED);
    // rt_thread_mdelay(1000*1);

    // LOG_D("motor power on,start planner.");

    // // TODO:判断落到池底
    // // rt_thread_mdelay(1000*5);
    // // 判断工作模式
    // switch (info.work_mode)
    // {
    //     case FLOOR_ONLY:
    //         // LOG_D("FLOOR_ONLY mode");
    //         // // 做360度地磁标定
    //         // arg_in.step_angle = 5.0f;
    //         // arg_in.buff = sg_buff;
    //         // arg_in.max_count = 72;
    //         // LOG_D("start actions_on_floor_magnetic_calibration");
    //         // if (actions_on_floor_magnetic_calibration(&arg_in, &arg_out)!= 0)
    //         // {
    //         //     LOG_E("actions_on_floor_magnetic_calibration failed");
    //         //     break;
    //         // }
    //         // LOG_D("arg_out.out_count = %d", arg_out.out_count);
    //         // // wash_floor_info_get_magnetic_angle(sg_buff);
    //         // for (int i=0;i<arg_out.out_count; i++) 
    //         // {
    //         //     LOG_D("%f, %f", sg_buff[i].raw_mag_x, sg_buff[i].raw_mag_y);
    //         // }
    //         struct mag_info geo_info;
    //         int count_geo = 5000;
    //         LOG_D("start get geo");
    //         rt_thread_mdelay(1000*5);

    //         while (count_geo > 0)
    //         {
    //             magnetic_raw_data_get(&geo_info);
    //             LOG_D("%f, %f", geo_info.raw_mag_x, geo_info.raw_mag_y);
    //             count_geo -= 1;
    //         }
    //         LOG_D("get geo stop");
            
    //         break;
        
    //     case WALL_AND_WATERLINE_ONLY:
    //         LOG_D("WALL_AND_WATERLINE_ONLY mode");
    //         int count = 60;
    //         while(count >0)
    //         {
    //             get_current_gero(&cur_gero_calib_info);
    //             LOG_D("%f, %f", cur_gero_calib_info.raw_mag_x, cur_gero_calib_info.raw_mag_y);
    //             rt_thread_mdelay(1000*1);
    //         }
    //         break;
        
    //     case CLEAN_FIRST:
    //         LOG_D("CLEAN_FIRST mode");
    //         break;

    //     case E_INVALID_MODE:
    //         LOG_D("E_INVALID_MODE mode");
    //         break;

    //     default:
    //         break;
    // }

    // // while (1)
    // // {

    // //     rt_thread_mdelay(10 * 1000);
    // //     float yaw = 0.0f;
    // //     int end_dis = 0;

    // //     // p2a_msg.cmd = CMD_ACTION_FLOOR_MOVE_FORWARD_AND_GET_DIS;
    // //     // p2a_msg.p_arg_in = &yaw;
    // //     // p2a_msg.p_arg_out = &end_dis;
    // //     // LOG_D("send to action %d", p2a_msg.cmd);
    // //     // send_msg_to_action(&p2a_msg);

    // //     p2a_msg.cmd = CMD_ACTION_FLOOR_GET_NEXT_MAGNETIC;
    // //     struct get_next_gero_in in = {.move_rows = 1, .one_line_move_time = 2000, .row_num = 1};
    // //     p2a_msg.p_arg_in = &in;
    // //     struct get_next_gero_out out;
    // //     p2a_msg.p_arg_out = &out;
    // //     LOG_D("send to action %d", p2a_msg.cmd);
    // //     send_msg_to_action(&p2a_msg);

    // //     // wait msg
    // //     int msg_sz = recv_msg_from_action(&action_msg_handle, RT_WAITING_FOREVER);
    // //     rt_kprintf("msg_sz=%d\n", msg_sz);
    // //     LOG_D("%p %d %d", out.gero_buff, out.gero_buff_sz, out.ret);
    // //     for (size_t i = 0; i < out.gero_buff_sz; i++)
    // //     {
    // //         LOG_D("%f %f %f %f", out.gero_buff[i].raw_mag_x, out.gero_buff[i].raw_mag_y, out.gero_buff[i].raw_mag_z, out.gero_buff[i].yaw);
    // //     }
        
    // //     rt_thread_mdelay(60 * 60 * 1000);
    // //     if (msg_sz <= 0)
    // //     {
    // //         LOG_E("recv msg ERR");
    // //         continue;
    // //     }
    // //     LOG_D("recv from action %d", action_msg_handle->msg);
    // //     // parse action msg
    // //     move_stop();
    // // }
#endif // if 0
}

/**
 * @brief public functions
 *
 */

int planner_init(void)
{
    rt_err_t ret = rt_thread_init(&planner_thread,
                                  "planner",
                                  planner_thread_entry,
                                  RT_NULL,
                                  &planner_thread_stack[0],
                                  sizeof(planner_thread_stack),
                                  PLANNER_THREAD_PRIORITY, PLANNER_THREAD_TIMESLICE);
    if (ret == RT_EOK && RT_EOK == rt_thread_startup(&planner_thread))
    {
        LOG_I("planner_thread startup");
    }
    else
    {
        LOG_E("planner_thread error");
    }

    return ret;
}

int planner_deinit(void)
{
    rt_thread_detach(&planner_thread);
    return 0;
}
