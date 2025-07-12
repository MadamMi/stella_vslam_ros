#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

#include <rtthread.h>
#include "rtconfig.h"

/* APP version information */
#define APP_VERSION_MAJOR 1
#define APP_VERSION_MINOR 0
#define APP_VERSION_PATCH 18

#define VERSION_MAKE(major, minor, revise) ((major * 10000) + (minor * 100) + revise)

/* APP version */
#define APP_VERSION VERSION_MAKE(APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_PATCH)

// task config
/* Actions Thread */
#define ACTIONS_THREAD_PRIORITY             (26)
#define ACTIONS_THREAD_TIMESLICE            (10)
#define ACTIONS_THREAD_STACK_SIZE           (1024*2)
/* Monitor Thread */
#define MONITOR_THREAD_PRIORITY             (25)
#define MONITOR_THREAD_TIMESLICE            (10)
#define MONITOR_THREAD_STACK_SIZE           (1024)
/* Planner Thread */
#define PLANNER_THREAD_PRIORITY             (24)
#define PLANNER_THREAD_TIMESLICE            (10)
#define PLANNER_THREAD_STACK_SIZE           (1024 * 8)
/* System Info Thread */
#define SYSTEM_INFO_THREAD_PRIORITY         (24)
#define SYSTEM_INFO_THREAD_TIMESLICE        (10)
#define SYSTEM_INFO_THREAD_STACK_SIZE       (1024)
/* Motor Parse Task */
#define MTR_PRS_TSK_PRIO                    (20)
#define MTR_PRS_TSK_TICK                    (10)
#define MTR_PRS_TSK_STACK_SZ                (1024 * 4)
/* Motor Control Task */
#define MTR_CTRL_TSK_PRIO                   (3)
#define MTR_CTRL_TSK_TICK                   (10)
#define MTR_CTRL_TSK_STACK_SZ               (1024)
/* Ble Cmd Task */
#define BLE_CMD_PRS_TSK_PRIO                (30)
#define BLE_CMD_PRS_TSK_TICK                (10)
#define BLE_CMD_PRS_TSK_STACK_SZ            (1024)
/* Protocol Manager Task */
#define PROT_MNG_TSK_PRIO                   (15)
#define PROT_MNG_TSK_TICK                   (50)
#define PROT_MNG_TSK_STACK_SZ               (1024)
/* Ble At Task */
#define BLE_AT_TSK_PRIO                     (23)
#define BLE_AT_TSK_TICK                     (10)
#define BLE_AT_TSK_STACK_SZ                 (1024 * 2)
/* Lsr Parse Task */
#define LSR_PRS_TSK_PRIO                    (25)
#define LSR_PRS_TSK_TICK                    (10)
#define LSR_PRS_TSK_STACK_SZ                (1024)
/* Soc Parse Task */
#define SOC_PRS_TSK_PRIO                    (23)
#define SOC_PRS_TSK_TICK                    (10)
#define SOC_PRS_TSK_STACK_SZ                (1024)
/* SubMcu Parse Task */
#define SM_PRS_TSK_PRIO                     (23)
#define SM_PRS_TSK_TICK                     (10)
#define SM_PRS_TSK_STACK_SZ                 (1024 * 2)
/* Ult Parse Task */
#define ULT_PRS_TSK_PRIO                    (25)
#define ULT_PRS_TSK_TICK                    (10)
#define ULT_PRS_TSK_STACK_SZ                (1024)
/* IMU_BATTERY Task */
#define IMU_BATTERY_TSK_PRIO                    (25)
#define IMU_BATTERY_TSK_TICK                    (10)
#define IMU_BATTERY_TSK_STACK_SZ                (1024)

/* distance measurement Task */
#define DISTANCE_MEA_TSK_PRIO               (25)
#define DISTANCE_MEA_TSK_TICK               (10)
#define DISTANCE_MEA_TSK_STACK_SZ           (1024)

#define WAIT_POSE_STABLE_TIME_MAX               (5) //unit: second
#define POSE_STABLE_THRESHOLD                   (2.0F) //每�?��?�度变化量小�?2�?

#define CLIMB_DOWN_RESTART_COUNT_MAX           (2)  //下�?�逻辑重试最大�?�数


#define MOTOR_SPEED_MOVE_SPEED                  (222) // 洗地时的前进速度（mm/s�?
#define MOVE_SPEED                               (0.1317f)  // 机器的速度（m/s)
#define MOTOR_SPEED_PWM                         (7500) // 洗地时的前进速度（RPM�?
#define PITCH_DIFF_THRES                        (5)    // 判断坡的pitch变化阈�?
#define MOVE_CONTROL_RIGHT_SAFEZONE             (20*1000) // 旋转超时时间ms
#define DIS_SENSOR_MODE                         (0)    // 1:超声优先 0:激光优�?
#define COORDINATE_SYSTEM_MODE                  (1.0f)    // 左手坐标�?1�? 右手坐标�?-1
#define GAP_BETWEEN_ROWS                        (1500)  // 每�?�换行时，前进时�? ms
#define CEDGE_THRES                             (450)   // �?三边阈值，mm
#define CLEAN_FLOOR_TIMES                       (2)     // 设定清洗次数
#define NO_VISION_MODEL_WASH_TIMES				(100)	// 没有视觉的机器设定清洁次数，设置得比较大以保证洗到没电

#define ONE_LINE_LENGTH                          (500)  // 行间�?(mm)
#define AM_WASH_FLOOR_VOL_THRES                  (27000) // 洗�?�电压阈值，27V
#define OFF_VOL_THRES                            (21500) // 关机电压21.5v  停止工作
#define IMM_OFF_VOL_THRES                        (20000) // 立即关机电压20v
#define STOP_WATERLINE_THRES                     (22000) // 去水线的电压22v
#define SLOPE_THRES_MAX                          (30.0f) // 洗地坡的阈�?
#define TURN_LIGHT_THRES                         (0.4f)  // 开�?阈�?
#define LIGHT_VALUE                              (0.02f) // 补光�?�?�?
// #define SLOPE_THRES_MIN                          (10.0f) // 判定为坡的最小阈�?
#define SLOPE_THRES_MIN                          (7.0f) // 判定为坡的最小阈�?
#define SUSTAIN_TIME_S                           (4*1000) // 判定坡的状态机持续时间 ms
#define DEEP_AREA_UP_SLOPE_MAX                   (50.0f)  // 深水区右侧坡最大坡�?
#define SHORT_LINE_TIME_THRES                    (3*1000) // �?距�?��?�阈�? ms
#define SHORT_LINE_COUNT_THRES                   (5)      // 连续�?行大�?4，则认为到c�?
#define PUMP_SPEED_HIGH                          (4500)   // 洗地，水泵高�?�?
#define PUMP_SPEED_LOW                           (500)   // 洗地，水泵低�?�?
#define WASH_FLOOR_MOTOR_MIN_SPEED               (1500)   // 洗地，驱动低�?�?
#define WASH_FLOOR_MOTOR_MAX_SPEED               (4000)   // 洗地，驱动高�?�?
#define WASH_FLOOR_MOTOR_SPEED                   (3800)   // 洗地，驱动电机转�?
#define JUDGE_DRAIN_YAW_DIFF                     (10.0f)  // 机器yaw突变容�?�阈�?
#define DIFF_COUNT                               (10)     // yaw偏移容忍次数

#define MAX_LINE_INTERL                          (3.0f)  // 洗地换�?�间隔最大（倍数�?
// #define EDGE_THRES (500) // mm
#define EDGE_THRES                               (300)    // 判断到�?�边的阈值mm    
#define QUICK_WEEKLY_TIME_MAX                    (12)      // v10 weekly mode最大清洗�?�数
#define SMART_WEEKLY_TIME_MAX                    (12)      // s10 weekly mode最大清洗�?�数  
#define VISION_WEEKLY_TIME_MAX                   (12)     // p10 weekly mode最大清洗�?�数      
#define ONCE_CLEAN_TIME_MAX                      (80*60*1000) // 单次洗地最大时长ms     
#define UTURN_DIFF_THRES                         (3.0f)   // 旋转过程中坡度变化
#define MOVE_FORWARD_TIMEOUT                     (3*60*1000)  // 洗地，前进超时时间
#define DELTA_SLOPE                              (5.0f)     // 判断不同平面之间的坡度差阈值
#define MOVE_SPEED                               (0.1317f)  // 机器的速度（m/s)
// #define TEST_CLEAN                               RT_TRUE
#define WEEKLY_MODE_WAKE_UP_INTV					(55)	//weekly_mode睡眠时唤醒的间隔，单位s

enum pool_outline_type // 池子�?�?
{
    E_OUTLINE_RECTANGULAR = 0, // 矩形池子
    E_OUTLINE_OVAL,            // �?圆形
    E_OUTLINE_CIRCLE,          // 圆形
    E_OUTLINE_KIDNEY,          // 肾形
    E_OUTLINE_PEANUT,          // 花生�?

    E_POOL_BUTT
};

enum pool_bottom_type // 池子底部形状
{
    E_BOTTOM_FLAT = 0, // 平底
    E_BOTTOM_NON_FLAT, // 非平�?

    E_BOTTOM_BUTT
};

enum sell_area_type // 卖场区域
{
    E_AREA_NORMAL = 0, // 不区分地�?
    E_AREA_AU,         // �?往澳洲

    E_AREA_BUTT
};


#define E_NORMAL  0            // 通用版本
#define E_AUSTRALIA  1         // 澳洲版本
#define E_EXHIBITION  2        // 展会版本
#define E_EXHIBITION_P10 3     // 美国展会p10版本

// #define E_HYWORLD            // HYWORLD测试池子版本

#define VERSION_TYPE E_NORMAL // 先声明版本信息

#if (VERSION_TYPE==E_NORMAL)
    #define MIN_LINE_INTERL      (0.5f)
#ifdef POOL_MATERIAL_STEEL    // 不锈钢材质泳池
    #define POOL_BOTTOM_IS_SLOPE RT_FALSE 
#else
    #define POOL_BOTTOM_IS_SLOPE RT_TRUE
#endif
    #define IF_UPDATE_YAW        RT_FALSE
#elif (VERSION_TYPE==E_AUSTRALIA)
    #define MIN_LINE_INTERL      (0.5f)
    #define POOL_BOTTOM_IS_SLOPE RT_FALSE
    #define IF_UPDATE_YAW        RT_TRUE
    #define STOP_LINE_AT_B
#elif (VERSION_TYPE==E_EXHIBITION)
    #define MIN_LINE_INTERL      (0.5f)
    #define POOL_BOTTOM_IS_SLOPE RT_FALSE
    #define IF_UPDATE_YAW        RT_TRUE
    #define STOP_LINE_AT_A
    #define WF_CLAB_WALL             RT_FALSE
#elif (VERSION_TYPE==E_EXHIBITION_P10)
    #define POOL_BOTTOM_IS_SLOPE RT_FALSE
    #define IF_UPDATE_YAW        RT_TRUE
    #define RECYCLING_CLEAN
    #define MIN_LINE_INTERL      (0.0f) 
#endif

/* pool config */
// #define POOL_BOTTOM_IS_SLOPE  RT_TRUE

/* 机器电机板相�?*/
// #define MACHINE_TYPE_A
// #define MACHINE_TYPE_B
#define MACHINE_TYPE_C

// #define FLOOR_TEST              // 陆地测试

// #define DEMO_MODE_BIG_SUCKING   // 大吸力演示模�?
// #define DEMO_MODE_CLIMB_STAIRS  // �?楼�??演示模式


#define DUMP_DEBUG              // 打印16进制信息

#define NEW_FRAMEWORK_MODE      // 新�?�架模式

#define SMALL_ANGLE_THRES       (3.0f)      // 小角度阈值

#endif
