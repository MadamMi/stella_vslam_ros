#ifndef __ACTIONS_H__
#define __ACTIONS_H__

/** 机器人行进动作执行过程中, 机器人遇到的各种状态 **/
enum robot_status_type // 机器人状态
{
    ROBOT_STATUS_HIT,
    ROBOT_STATUS_DRIFT,
    ROBOT_STATUS_REACH_WATERLINE,
    ROBOT_STATUS_REACH_FLOOR,
    ROBOT_STATUS_CLIMB,
    ROBOT_STATUS_CLIMB_STAIR,
    ROBOT_STATUS_DILOU,
    ROBOT_STATUS_BLOCKED,
    ROBOT_STATUS_OVER_HEAD, // 抬头角度过大
    ROBOT_STATUS_LEAN_OVER, // 设备侧翻在地
    ROBOT_STATUS_TIMEOUT,
    ROBOT_STATUS_NORMAL,

    ROBOT_STATUS_BUTT
};

extern int actions_init(void);
extern int actions_deinit(void);

#endif
