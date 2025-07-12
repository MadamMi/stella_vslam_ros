#ifndef __UTIL_H__
#define __UTIL_H__

#include "rtthread.h"
#include "easyflash.h"
#define MAX_BOXES_NUM 60


// 机器model号
typedef enum robot_model_type
{
	MODEL_P10 = 0x01u,
	MODEL_V10,
	MODEL_S10,
    MODEL_L20,
	MODEL_L10,
}robot_model_type_t;

// 当前行状态机--上下坡、平台
enum cur_line_info
{
    STATE_INITIAL = 0x00u,
	GROUND_DOWNWARD,    //  下坡
    GROUND_FLAT,        // 平
	GROUND_UPWARD,      // 上坡
    GROUND_GET_SLOPE,
};

// 地磁相关
struct mag_info
{
    int id;
    float raw_mag_x; /*unit: mGauss*/
    float raw_mag_y;
    float raw_mag_z;
};
typedef struct mag_info *mag_info_t;

struct model_info
{
    enum robot_model_type model_mode; // 机器model号（P/S/V）
    int weekly_time_inter;  // weekly time。分钟为单位
};
typedef struct model_info *model_info_t;

struct mag_xy_info
{
    float mag_x;
    float mag_y;
};

struct mag_cali_arg_in
{
    float step_angle; // 正：向左转；负：向右转
    struct mag_calibrate_info *buff;
    int max_count; // 最大数据个数
};
typedef struct mag_cali_arg_in *mag_cali_arg_in_t;

struct mag_cali_arg_out
{
    int out_count; // 实际数据个数
};
typedef struct mag_cali_arg_out *mag_cali_arg_out_t;

struct mag_calibrate_info
{
    float raw_mag_x;
    float raw_mag_y;
    float raw_mag_z;
    float yaw;
};
typedef struct mag_calibrate_info *mag_calibrate_info_t;

struct mag_adjest_yaw_info
{
    float raw_mag_x;
    float raw_mag_y;
    float raw_mag_z;
    float yaw;
    float y;
};
typedef struct mag_adjest_yaw_info *mag_adjest_yaw_info_t;

struct get_next_gero_in
{
    int move_rows;          // 前进行数
    int one_line_move_time; // 单行行走世间(ms)
    int row_num;            // 获取连续row_num行的地磁数据
};
typedef struct get_next_gero_in *get_next_gero_in_t;

struct get_next_gero_out
{
    mag_calibrate_info_t gero_buff;
    int gero_buff_sz;      // 获取gero_buff_sz组地磁数据
    int ret;
};
typedef struct get_next_gero_out *get_next_gero_out_t;

// 旋转测距
struct dis_rotate_in_arg
{
	float total_rotate_angle; //总旋转角度
	int time_interval; // 每次旋转的停顿间隔
	float single_rotate_angle; // 单次旋转角度
};
typedef struct dis_rotate_in_arg *dis_rotate_in_arg_t;


struct dis_sensor_info
{
	float yaw;
	int distance;  //mm
};
typedef struct dis_sensor_info *dis_sensor_info_t;


struct dis_sensor_buffer_info
{
    dis_sensor_info_t dis_buff;
    int data_count;        // buffer里存的数据个数
};
typedef struct dis_sensor_buffer_info *dis_sensor_buffer_info_t;


struct distance_sensor_out_arg
{
	dis_sensor_info_t ultrasonic_buff; // 获取的超声数据buff: yaw and distance
    int ult_buff_sz;
	int ultrasonic_count; // 获取的有效超声数据个数
	
	dis_sensor_info_t laser_buff; // 获取的激光数据buff: yaw and distance
    int laser_buff_sz;
	int laser_count; // 获取的有效激光数据
};
typedef struct distance_sensor_out_arg *distance_sensor_out_arg_t;


// 运动相关
struct move_info
{
    float start_yaw;
    float judge_edge_pitch_thres; // 判断上墙pitch阈值
    int move_time;                // ms
    int move_speed;               // 电机速度
    int dirty_flag;                // 00：无，1：有
};
typedef struct move_info *move_info_t;

struct move_to_next_row_info
{
    int mode;          //mode为0，右转换行，mode为1，左转换行
    int move_time;     // 换行直行时间。ms
    int back_time;     // 后退时间，ms
    int if_clean_dirt; // 是否需要清洗边缘脏污。0：不需要，1：需要
};
typedef struct move_to_next_row_info *move_to_next_row_info_t;

// 建图相关
struct point_info
{
    float x;
    float y;
    float z;
};
typedef struct point_info *point_info_t;

// action 相关
enum action_end_type
{
    ACTION_END_SUCCESS = 0x00u,           // 如期完成
    ACTION_END_TIMEOUT,                   // 超时结束
    ACTION_END_TO_WALL,                   // 遇到墙结束
    ACTION_END_ON_WALL_THEN_DOWN,         // 上墙并下来结束

    ACTION_END_BUTT
};


// 视觉相关
struct box_info
{
    float xmin, ymin, xmax, ymax; // 目标框左上角和右下角
    int class_id; // 目标所属类别
};
typedef struct box_info *box_info_t;

struct vision_info
{
    int box_num; // 目标个数
    struct box_info box_buff[MAX_BOXES_NUM]; // 目标框buffer
    int class_id; // 分类结果
};
typedef struct vision_info *vision_info_t;

enum object_info
{
    DRAIN = 0,
    LEAVES,
    UNKNOWN,
};

struct leaves_info
{
    enum object_info  object;
    unsigned int left_have_leaves;   // 左侧是否有叶子。0无1有
    unsigned int middle_have_leaves; // 中间是否有叶子。0无1有
    unsigned int right_have_leaves;  // 右侧是否有叶子。0无1有
    unsigned int left_objects_num;   // 左侧物体识别个数
    unsigned int middle_objects_num; // 中间物体识别个数
    unsigned int right_objects_num;  // 右侧物体识别个数
};
typedef struct leaves_info *leaves_info_t;

struct region_info
{
    unsigned int left;
    unsigned int middle;
    unsigned int right;
    unsigned int dirty_index;    /* 中间区域脏污类别 */
};
typedef struct region_info *region_info_t;
// 变速前进返回的结构体
struct move_forward_cs
{
    vision_info_t left_buffer;
    vision_info_t right_buffer;
    int left_count;
    int right_count;
    int dis_from_edge;
};
typedef struct move_forward_cs *move_forward_cs_t;

// 行信息
struct line_info
{
    int clean_time; // 清洗次数
    float dirt_score; // 脏污程度
    struct mag_xy_info mag;// 地磁x,y信息
    int get_mag;    // 是否已获取地磁数据
};

struct wf_clean_info
{
    int wash_times;     // 最大清洗次数
    int min_wash_times; // 最小清洗次数
    int max_clean_rows; // 最大清洗行数
    rt_uint16_t off_value;      // 停止电压
    float line_interval;     // 行间距
    rt_tick_t once_clean_max_clean_time;  // 单次清洗最大时长ms
};
typedef struct wf_clean_info *wf_clean_info_t;

// 洗墙相关
// u形找墙
struct find_next_edge_info
{
    int back_time;     // 后退时间
    int forward_time;  // 前进时间
    int thres;         // 第三边距离阈值(mm)
    int detect_time;    // 前进检测时间
    int pre_event;      // 执行本次动作的原因。-2 - 爬墙出水，-1 - 爬墙超时，0 - 正常，1 - 遇到step，2 - 遇到sundesk， 3 - 第一次找墙结束, 4 - 洗墙时掉下墙 5 - 在墙倾斜太多了
};

// 洗水线
struct wash_water_line_info
{
    int nums;          // 锯齿数
    int back_time;     // 后退时间
    float alpha;       // 锯齿的角度alpha
};

enum medium_type
{
    IN_AIR,             /* 空气 */
    IN_WATER,           /* 液体 */
    NO_SURE,            /* 不确定 */
};

// 介质检测
struct medium_msg
{
    enum medium_type type;        /* 介质类型 @medium_type */
    unsigned int  time_ms;        /* 在介质中的时间，单位：毫秒 */
};
typedef struct medium_msg* medium_msg_t;


// 洗地设置各参数
// 行间距有效范围>0
// 水泵转速有效数值范围[1, 6000]
// 行走电机转速有效数值范围[1, 10000]
#pragma pack(1)
struct wf_params
{
    uint16_t forward_wp_speed;                              // 巡航水泵转速
    uint16_t forward_mtr_speed;                             // 巡航行走电机转速
    uint16_t clean_leaves_wp_speed;                         // 识别到树叶的水泵转速
    uint16_t clean_leaves_mtr_speed;                        // 识别到树叶的行走电机转速
    uint16_t forward_clean_sediment_wp_speed;               // 巡航阶段识别到泥沙的水泵转速
    uint16_t forward_clean_sediment_mtr_speed;              // 巡航阶段识别到泥沙的行走电机转速
    uint16_t edge_clean_sediment_wp_speed;                  // 墙角识别到泥沙的水泵转速
    uint16_t edge_clean_sediment_mtr_speed;                 // 墙角识别到泥沙的行走电机转速
    uint16_t clean_steep_incline_wp_speed;                  // 长距离洗陡坡水泵转速
    uint16_t clean_steep_incline_mtr_speed;                 // 长距离洗陡坡行走电机转速
    uint16_t clean_steep_incline_edge_wp_speed;             // 尾部洗陡坡泥沙水泵转速
    uint16_t clean_steep_incline_edge_mtr_speed;            // 尾部洗坡泥沙行走电机转速
    uint16_t pentagonal_path_find_slope_wp_speed;           // 五角星找坡水泵转速
    uint16_t pentagonal_path_find_slope_mtr_speed;          // 五角星找坡行走电机转速  
    uint8_t  if_clean_steep_incline;                        // 是否清洗陡坡  
    float    adjacent_line_interval;                        // 行间距(作为倍数处理)
};
#pragma pack()
typedef struct wf_params* wf_params_t;
/*
init params
struct all_mode_wf_params = {3100, 3800, 3800, 3000, 3800, 3000, 3800, 3000, 3300, 8000, 3800, 3000, 3000, 8000, 1, 2.0f};
struct single_mode_wf_params = {3100, 3800, 3800, 3000, 3800, 3000, 3800, 3000, 3300, 8000, 3800, 3000, 3000, 8000, 1, 2.0f};
struct weekly_mode_wf_params = {2800, 3800, 3800, 3000, 3800, 3000, 3800, 3000, 3300, 8000, 3800, 3000, 3000, 8000, 1, 2.0f};
*/

// 洗地直行
struct move_forward_info
{
    unsigned char if_change_speed; // 是否变速，0：不变 1：变
    float start_yaw;               // 直行初始速度
    int time_out;           // 直行超时时间
    int if_odd_row;         // 奇偶行，0：偶 1 奇
    int find_slope_time;      // 已找到坡的次数，最大2
    // int wtr_speed;           // 驱动电机转速
    wf_params_t params;       // 各清洗参数
    int last_line_forward_time;  // 上一行的清洗时间
};
typedef struct move_forward_info* move_forward_info_t;

struct cur_line_slope_info
{
    int down_count;   // 统计到的下坡次数  统计间隔为200ms
    int up_count;     // 统计到的上坡次数
    int flat_count;   // 统计到的平面次数
};
typedef struct cur_line_slope_info* cur_line_slope_info_t;

struct move_forward_action_return_info
{
    int forward_time;  // 机器直行时间
    int end_dis;       // 停止时离墙边距离
    int ret_idx;       // 退出标置位。0距离，1超时，2坡度，3roll, 4yaw纠不回来
    enum cur_line_info line_state; // 地形状态位
    struct cur_line_slope_info info; // 统计的当前行地形信息
    int if_right_area_have_leaves;  // 清洗方向右侧是否有叶子
    rt_tick_t act_get_dirty_time;    // 最近一次检测到脏污的时间戳，如果没有检测到，为0
    int vision_clean_time;   // 鸡爪清洗的次数
    struct wash_row_info* dirty_info;  // 每行的信息
};
typedef struct move_forward_action_return_info* move_forward_action_return_info_t;

struct slope_info
{
    float max_pitch;
    float max_pitch_yaw;
};
typedef struct slope_info* slope_info_t;

// c边测距
struct c_dis_info
{
    int a_dis;    // 换行时，行走前的c边距离
    int b_dis;    // 换行时，行走后的c边距离
    int if_arrvied_c;  // 是否已到c边.0:没有 1:已到
    int n_line_to_c; // 还有几行到c边
};
typedef struct c_dis_info* c_dis_info_t;

struct change_line_info
{
    int a_dis;                // 换行时，行走前的c边距离
    int b_dis;                // 换行时，行走后的c边距离
    int cur_line_roll_get_c;  // 是否靠roll检测到了c边。0否；1是
    int cur_line_pitch_get_c; // 是否靠pitch检测到了c边
};
typedef struct change_line_info* change_line_info_t;

struct goto_shallow_info
{
    rt_tick_t half_line_time;    // 走到当前行中间的时间ms
    rt_uint8_t turn_mode;        // 机器是否需要旋转回到浅水区。0:不需要特殊处理，1：左转回到浅水区，2：右转回到浅水区
};
typedef struct goto_shallow_info* goto_shallow_info_t;


// 洗墙参数
struct wash_wall_info
{
    int num;    // 洗墙次数
    float angle; // 旋转角度
};
typedef struct wash_wall_info *wash_wall_info_t;


// 定义存储转速的结构体
struct speed_info{
    int pump_speed;    // 水泵转速
    int motor_speed;   // 驱动电机转速
};
typedef struct speed_info *speed_info_t;

struct stop_on_waterline_info
{
    int on_waterline_time; // 出水停留时间，unit second
    int in_water_time;     // 在水中停留时间, unit second
    int repeat_count;      // 此动作重复次数
    int power_off_flag;    /// 是否需要关闭电源 
};
typedef struct stop_on_waterline_info *stop_on_waterline_info_t;

struct forward_on_wall_info_out
{
    int forward_time; // 前进时间，unit ms
    int status;       // 结束后之的状态
};
typedef struct forward_on_wall_info_out *forward_on_wall_info_out_t;

//struct wash_row_info
//{
//    rt_uint8_t row_id;              // 第几行
//    rt_uint32_t length;             // 长度      单位：mm
//    rt_uint16_t width;              // 跨行宽度   单位：mm
//    rt_uint8_t type;                // 是否有脏污、地漏、泥沙 etc
//    rt_uint8_t leaves_times;        // 识别到树叶的次数
//    rt_uint8_t drain_times;         // 识别到地漏的次数
//    rt_uint8_t medium_dirty_times;  // 中度脏污次数
//    rt_uint8_t heavy_dirty_times;   // 重度脏污次数
//};
//typedef struct wash_row_info *wash_row_info_t;

///* 工作日志记录 */
//struct wash_record_info
//{
//    rt_uint8_t  region_flag;        // 区域标志，浅水区、深水区、缓坡、陡坡、墙壁 etc
//    rt_uint16_t total_rows_num;     // 总行数
//    rt_tick_t   start_time;         // 开始时间
//    rt_tick_t   end_time;           // 结束时间
//    rt_uint8_t  wash_time;          // 第几圈
//    struct wash_row_info info_lists[40];   //
//};
//typedef struct wash_record_info *wash_record_info_t;

#pragma pack(1)
typedef struct weekly_run_num_t{
    rt_uint8_t size;                    // 池子大中小尺寸
    rt_uint8_t num[3];                  // 大中小尺寸对应的运行次数
}*p_weekly_run_num_t;

typedef struct f_e_clean_count{
    rt_uint8_t def;                     // 默认次数
    rt_uint8_t max;                     // 最大次数
}*p_f_e_clean_count;

typedef struct pool_dirt_params_t{
    rt_uint8_t level;                   // 脏污程度
    struct f_e_clean_count count[3];    // 清洗次数
}*p_pool_dirt_params_t;

typedef struct ide_branch_leaves_sta_t{
    rt_uint8_t leaves;
}*p_ide_branch_leaves_sta_t;
#pragma pack()

enum app_comm_light_effect{
    LIGHT_NONE = 0,             // 不需要显示app和机器的交互灯效
    LIGHT_BLUE_FLASH,           // 尾灯蓝色闪3次（亮灭间隔500ms）
    LIGHT_MODE_COLOUR_FLASH,    // 设置后的模式指示灯闪3次（亮灭间隔500ms）
};



#define POOL_SHAPE_NAME        (20) // 泳池形状(circle/rectangle/kidney etc)
#define MAX_LINE_NUM           (50) // 行数最大值

struct pool_line_length{
    int length_list[MAX_LINE_NUM]; // 从一个c边到另一个c边,清洗的所有行行长，单位mm
    int line_num; // 本次测量的行数
};
typedef struct pool_line_length *pool_line_length_t;

struct pool_line_length_gt{
    char shape[POOL_SHAPE_NAME];     // 池子形状
    int length_list[MAX_LINE_NUM];   // 所有行的行长，单位mm。行长按0.4m间隔采样测量得到，有坡的池子行长的测量方向垂直于坡向
    int line_num;                    // 行数          
};
typedef struct pool_line_length_gt *pool_line_length_gt_t;


#endif // __UTIL_H__

