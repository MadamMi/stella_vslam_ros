#include "rtthread.h"
#include "task_comm.h"
#include "planner.h"
#include "actions.h"
#include "actions_on_wall.h"
#include "actions_on_floor.h"
#include "app_config.h"
#include "move_basic.h"

#define DBG_TAG "action"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/**
 * @brief extern varialbes
 *
 */

/**
 * @brief extern functions
 *
 */
int _actions_demo(void *p_arg_in, void *p_arg_out);

/**
 * @brief private varibales
 *
 */
// 命令动作对应表
struct cmd_action
{
    enum planner_cmd_type cmd;
    int (*pfun)(void *p_arg_in, void *p_arg_out);
};
typedef struct cmd_action *cmd_action_t;
static const struct cmd_action CMD_ACTION_TABLE[] = {
    // 洗地动作表
    {CMD_ACTION_FLOOR_ROTATE_AND_MEATURE_DIS, actions_on_floor_rotate_measure},
    {CMD_ACTION_FLOOR_MAGNETIC_CALIBRATION, actions_on_floor_magnetic_calibration},
    {CMD_ACTION_FLOOR_GET_NEXT_MAGNETIC, get_next_area_gero},
    {CMD_ACTION_FLOOR_MOVE_FORWARD_AND_GET_DIS, actions_on_floor_move_forward_to_obstacle},
    {CMD_ACTION_FLOOR_MOVE_TO_EDGE, actions_on_floor_move_to_edge},
    {CMD_ACTION_FLOOR_MOVE_TO_NEXT_LINE, actions_on_floor_move_to_next_line},
    {CMD_ACTION_FLOOR_MOVE_FORWARD_PID, actions_on_floor_move_forward_with_pid_and_time},
    {CMD_ACTION_FLOOR_MOVE_BACKWARD_PID, actions_on_floor_move_backward_with_pid_and_time},
    {CMD_ACTION_FIND_EDGE, actions_on_floor_search_wall}, // 找墙
    {CMD_ACTION_FIND_SLOPE, actions_on_floor_search_slope}, //
    {CMD_ACTION_BACKWARD_FIND_WALL, actions_on_floor_backward_search_wall},
    {CMD_ACTION_FLOOR_MOVE_FORWARD_JUDGE_C_EDGE, actions_on_judge_c_edge},
    {CMD_ACTION_FLOOR_ROTATE_AND_MEATURE_DIS_NO_STOP, actions_on_floor_rotate_measure_no_stop},
    {CMD_ACTION_FLOOR_GOTO_SHALLOW_AND_STOP, actions_on_floor_goto_shallow_turn_off},

    // 洗墙动作表
    {CMD_ACTION_WALL_FIND_NEXT_ENTRY, actions_on_wall_find_next_entry},
    {CMD_ACTION_WALL_CLIMB_UP, actions_on_wall_climb_up},
    {CMD_ACTION_WALL_CLIMB_DOWN, actions_on_wall_climb_down},
    {CMD_ACTION_WALL_MOVE_FORWARD, actions_on_wall_move_forward},
    {CMD_ACTION_WALL_MOVE_BACKWARD, actions_on_wall_move_backward},
    {CMD_ACTION_WALL_GET_WATER_DEPTH, actions_on_wall_get_water_depth},
    {CMD_ACTION_WALL_CLEAN_WATER_LINE_DIFF, actions_on_wall_clean_water_line_diff},
    {CMD_ACTION_WALL_CLEAN_WATER_LINE, actions_on_wall_clean_water_line},
    {CMD_ACTION_WALL_VENT_GAS, actions_on_wall_vent_gas},
    {CMD_ACTION_WALL_CLEAN_WALL, actions_on_wall_clean_wall},

    {CMD_DEMO, _actions_demo},

    {CMD_BUTT, RT_NULL} // buttom end
};

rt_align(RT_ALIGN_SIZE) static char action_thread_stack[ACTIONS_THREAD_STACK_SIZE];
static struct rt_thread action_thread;

/**
 * @brief private functions
 *
 */
static int _actions_demo(void *p_arg_in, void *p_arg_out)
{
    LOG_D("in %d", *(int *)p_arg_in);
    *(int *)p_arg_out = 1;
    LOG_D("out %d", *(int *)p_arg_out);
    int ret = *(int *)p_arg_out;

    return ret;
}

// 遍历动作表
static int parse_msg(planner_to_action_msg_t msg)
{
    int ret = -RT_ERROR;
    for (int i = 0; CMD_ACTION_TABLE[i].pfun != 0; i++)
    {
        if (msg->cmd == CMD_ACTION_TABLE[i].cmd)
        {
            ret = CMD_ACTION_TABLE[i].pfun(msg->p_arg_in, msg->p_arg_out);
            break;
        }
    }

    return ret;
}

static int send_msg_to_planner(action_msg_t msg)
{
    return rt_mb_send(&g_action_mb, (rt_ubase_t)msg);
}

static int recv_msg_from_planner(planner_to_action_msg_t msg, int timeout)
{
    return rt_mq_recv(&g_planner_to_action_mq, msg, sizeof(struct planner_to_action_msg), timeout);
}

static void actions_thread_entry(void *param)
{
    struct action_msg action_msg;
    struct planner_to_action_msg planner_msg;
    LOG_D("action thread run");

    while (1)
    {
        rt_memset(&action_msg, 0, sizeof(struct action_msg));
        rt_memset(&planner_msg, 0, sizeof(struct planner_to_action_msg));
        // wait msg
        int msg_size = recv_msg_from_planner(&planner_msg, RT_WAITING_FOREVER);
        if (msg_size < 0)
        {
            LOG_E("recv msg ERR %d", msg_size);
            continue;
        }

        // parse msg and load ack msg
        int ret = parse_msg(&planner_msg);
        LOG_D("parse msg - %d %d %d", ret, planner_msg.cmd, *(int *)planner_msg.p_arg_in);

        // load msg
        action_msg.msg = ret;

        // send msg
        send_msg_to_planner(&action_msg);
    }
}

int actions_init(void)
{
    rt_err_t ret = rt_thread_init(&action_thread,
                                  "action",
                                  actions_thread_entry,
                                  RT_NULL,
                                  &action_thread_stack[0],
                                  sizeof(action_thread_stack),
                                  ACTIONS_THREAD_PRIORITY, ACTIONS_THREAD_TIMESLICE);

    if (ret == RT_EOK && RT_EOK == rt_thread_startup(&action_thread))
    {
        LOG_I("thread startup");
    }
    else
    {
        LOG_E("thread error");
    }

    return ret;
}

int actions_deinit(void)
{
    rt_thread_detach(&action_thread);
    return 0;
}
