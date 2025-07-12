#ifndef __PLANNER_H__
#define __PLANNER_H__

#include "rtthread.h"

// 测距info
// Distance information obtained by ultrasound
struct distance_info
{
    int dis; // mm
    float yaw;
};
typedef struct distance_info *distance_info_t;

// 列举命令
enum planner_cmd_type
{
    // 洗墙命令列表          
    CMD_ACTION_WALL_FIND_NEXT_ENTRY = 0,       // u形找墙
    CMD_ACTION_WALL_CLIMB_UP,                  // 爬墙
    CMD_ACTION_WALL_CLIMB_DOWN,                // 下墙
    // CMD_ACTION_WALL_MOVE_FORWARD_TO_LINE,      // 墙上前进直到出水
    // CMD_ACTION_WALL_MOVE_BACKWARD_TO_FLOOR,    // 墙上后退直到下墙
    CMD_ACTION_WALL_MOVE_FORWARD,              // 墙上前进
    CMD_ACTION_WALL_MOVE_BACKWARD,             //墙上后退
    CMD_ACTION_WALL_CLEAN_WATER_LINE,          // 锯齿状洗水线
    CMD_ACTION_WALL_CLEAN_WATER_LINE_DIFF,     // 差速洗水线
    CMD_ACTION_WALL_CLEAN_WALL,                 // 洗墙
    CMD_ACTION_WALL_GET_WATER_DEPTH,
    CMD_ACTION_WALL_VENT_GAS,                  // 排气

    // 洗地命令列表
    CMD_ACTION_FLOOR_ROTATE_AND_MEATURE_DIS,   // 旋转测距
    CMD_ACTION_FLOOR_ROTATE_AND_MEATURE_DIS_NO_STOP, //旋转测距（机器不停止）
    CMD_ACTION_FLOOR_MAGNETIC_CALIBRATION,     // 地磁标定
    CMD_ACTION_FLOOR_GET_NEXT_MAGNETIC,        // 获取下一矫正点地磁
    CMD_ACTION_FLOOR_MOVE_FORWARD_AND_GET_DIS, // 前进到距障碍物小于阈值
    CMD_ACTION_FLOOR_MOVE_TO_EDGE,             // 前进到墙边
    CMD_ACTION_FLOOR_MOVE_TO_NEXT_LINE,        // 换行
    CMD_ACTION_FLOOR_MOVE_FORWARD_PID,         // 前进with pid and time
    CMD_ACTION_FLOOR_MOVE_BACKWARD_PID,        // 后退with pid and time
    CMD_ACTION_FLOOR_MOVE_FORWARD_CHANGE_SPEED, // 变速前进到距障碍物小于阈值
    CMD_ACTION_FLOOR_MOVE_FORWARD_JUDGE_C_EDGE,              // 用pitch确认C边
    CMD_ACTION_FLOOR_GOTO_SHALLOW_AND_STOP,      // 浅水区停机

    // 洗地&洗墙通用
    CMD_ACTION_FIND_EDGE,                       // 洗地&洗墙找墙
    CMD_ACTION_FIND_SLOPE,                      // 找坡
    CMD_ACTION_BACKWARD_FIND_WALL,              // 没有坡的池子，先掉头，再后退找墙。
    // 运动控制命令列表
    CMD_MOVE_FORWARD,
    CMD_MOVE_BACKWARD,
    CMD_MOVE_ROTATE_LEFT,
    CMD_MOVE_ROTATE_RIGHT,
    CMD_MOVE_STOP,

    // 仅测试用
    CMD_DEMO,

    CMD_BUTT
};

extern int planner_init(void);
extern int planner_deinit(void);

#endif
