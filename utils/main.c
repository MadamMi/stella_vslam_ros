#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "rtthread.h"
#include "actions.h"
#include "app_config.h"
#include "util.h"
#include "actions_on_floor.h"
#include "wash_floor_info.h"
#include "task_comm.h"
#include "move_basic.h"
#include "angle.h"
#include "sensors_info.h"
#include "system_info.h"
#include "info.h"
#include "string.h"
#include "distance_drive.h"
#include "ef_def.h"
#include "planner_mode_control.h"


#define DBG_TAG "wash_floor_task"
// #define DBG_LVL DBG_INFO
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define POINT_BUFF_COUNT (100)
#define ADGEST_ROWCOUNT  (4)
#define ADJUST_C_COUNT (4)
#define ADGEST_YAW (3.0f)
#define GERO_DIS_THRES (20)
#define MOVE_SPEED_PWM 3700
#define DIS_INFO_BUFF_COUNT (100)
#define JUDGE_PITCH_THRES (-30.0f)
// #define JUDGE_C_TIME ((ADJUST_C_COUNT)*(GAP_BETWEEN_ROWS))
#define JUDGE_C_TIME (8*1000)
#define STATE_COUNT (1)
#define COUNT_THRES (1)
#define M_PI (3.14f)


static struct planner_to_action_msg p2a_msg;
static int msg_size;
static action_msg_t action_msg_handle = RT_NULL;

static int dis_pt_info_num = 0;
// static int rotate_point_count = 0;
int edge_point_count = 0; 


// static int move_rows_num = 0; 
// static int cleaned_rows_num = 0; 
wash_floor_rows_info wash_floor_task_info = {0};
static struct move_forward_info forward_action_info;
static struct move_forward_action_return_info forward_action_ret_info;
// static float pre_time_yaw = 0.0f;

static float cur_time_start_yaw = 0.0f, cur_line_start_yaw = 0.0f;

static float pre_y = 0.0f; 
static struct point_info edge_point; 
const int WF_WP_SPEED = 3300;
static int GERO_ADGEST_YAW = 0; 


// static struct dis_sensor_info ult_info_buff[DIS_INFO_BUFF_COUNT];
// static struct dis_sensor_info tof_info_buff[DIS_INFO_BUFF_COUNT];

static struct c_dis_info odd_info={-1, -1, 0, 40};  
static struct c_dis_info even_info={-1, -1, 0, 40}; 
static int odd_line_to_cleaned_num = -1;  
static int even_line_to_cleaned_num = -1;  
static int a_dis, b_dis;             
static int remained_lines_num = -1;      
static int from_judge_moved_lines_num = -1;  
static enum edge_info wf_cur_line_edge = INIT_VALUE;
static struct change_line_info c_info={-1, -1, 0, 0};
static rt_uint8_t c_time;              



//static struct mag_adjest_yaw_info start_gero_info, pre_gero_info, cur_gero_info;
//static struct mag_calibrate_info start_gero_calib_info;
static struct mag_calibrate_info cur_gero_calib_info, pre_gero_calib_info;
static struct mag_calibrate_info sg_buff[100] = {0};

//static struct mag_cali_arg_in arg_in;

static struct mag_cali_arg_out arg_out;

static enum work_mode_type robot_work_mode;
// static int bool_mapping;

#define DIRT_THRES 0.5f
static struct line_info *wf; 
static int line_info_size = 100;
int cur_idx, idx = 0; 
static rt_uint16_t vol;
// static struct speed_info wf_set_wp_and_motor_speed;
extern int slope_label;
// static int find_slope_time = 0; 
struct slope_info pool_info={-360.0f, 0.0f};
enum cur_line_info deep_area_state = STATE_INITIAL;
static int if_find_deep_area = 0;
static int state_count = 0;
static struct model_info cur_model_type;
static app_clean_paras_t wf_conf_para;
static struct wf_clean_info clean_info = {0, 0, 0, 0, 1.0f, ONCE_CLEAN_TIME_MAX};
rt_uint8_t if_clean_edge = 0;   
static int if_uturn_slips_pre = 0;  
static rt_tick_t move_tick_time = 0;
static int stop_at_line_time = 0;  
static wf_params_t test_param = NULL; 
static int if_clean_steep_incline = 1;  
static float line_wa_interval = 1.0f;
static int if_current_edge_dirty = 0;  
static int close_clean_edge = 1;       
static int sum_vision_time = 0;       
static int pre_topography = 0;        
static int if_cross_slope = 0;        

static float pool_area = 0.0f;        
static float area_c = 0.0f;            
static float area_a = 0.0f;            
static float area_h1 = 0.0f;           
static float area_h = 0.0f;           
// static float slope_angle = 0.0f;     
static int pre_forward_time = 270000;   


rt_uint8_t weekly_mode_sleep_flag = 0;	


static int init_malloc()
{
    wf = (struct line_info*)malloc(line_info_size * sizeof(struct line_info));
    if (wf ==NULL)
    {
        LOG_E("malloc failed");
        return -1;
    }
    memset(wf, 0, sizeof(struct line_info) * line_info_size);
    LOG_I("malloc success, size=%d", sizeof(wf)*line_info_size);
    
    return 0;
}

static int reset_malloc()
{
    if (wf == NULL)
    {
        return -1;
    }
    memset(wf, 0, sizeof(struct line_info) * line_info_size);
    LOG_I("reset wf");
    idx = 0;
    cur_idx = 0;

    return 0;
}

static int add_malloc()
{
    // if (idx > (m_size-5)) // malloc的空间不够用了
    int old_size = line_info_size;
    line_info_size += 50;
    struct line_info *new_arr = (struct line_info*)realloc(wf, line_info_size * sizeof(struct line_info));
    if (new_arr == NULL) {
        LOG_E("Memory reallocation failed!\n");
        free(wf);
        return 1;
    }
    wf = new_arr; // 更新指针
    if (wf == NULL)
    {
        // 内存开辟失败
        return -1;
    }
    memset(wf+old_size, 0, sizeof(struct line_info) * 50);
    LOG_I("add malloc success, size=%d\n", sizeof(wf)*line_info_size);
    return 0;
}

// 更新buffer中的信息
int update_clean_info(int clean_rows, struct move_forward_cs* move_forward_cs_info)
{
    float left_score, right_score = 0.0f; // 已过行和未洗行的脏度
    if (wash_floor_task_info.total_moved_row%2 == 0)  // 如果当前是偶数行，左侧为已经过的行，右侧为未洗行。
    {
        left_score = get_dirt_score(move_forward_cs_info->left_buffer, move_forward_cs_info->left_count);
        right_score = get_dirt_score(move_forward_cs_info->right_buffer, move_forward_cs_info->right_count);
    } else // 如果当前是奇数行，右侧为已经过的行，左侧为未洗行。
    {
        left_score = get_dirt_score(move_forward_cs_info->right_buffer, move_forward_cs_info->right_count);
        right_score = get_dirt_score(move_forward_cs_info->left_buffer, move_forward_cs_info->left_count);
    }
    // 每洗完一行，更新buffer中对应内容
    // 当前行清洗次数+1
    wf[cur_idx].clean_time += 1;
    // 左右行信息更新
    // 如果cur_idx < idx，说明此行是返回来清洗的。
    if (cur_idx > 0) // 更新上一行信息
    {
        wf[cur_idx - 1].dirt_score = left_score;
    }
    if (cur_idx < (line_info_size - 1)) // 更新下一行信息
    {
        wf[cur_idx + 1].dirt_score = right_score; // 根据得分更新下一行的信息
    }

    return 0;
}

// 判断去哪一行
static int clean_first_get_the_newline_cnt(int *cnt)
{
    LOG_D("new line--idx=%d, cur_idx=%d\n", idx, cur_idx);
    // 更新完信息后，根据buffer中当前行前后的信息，判断要去哪一行
    // 如果当前为第一行清洗，则只进行右侧的判断
    if (idx == 0)
    {
        if (wf[idx + 1].dirt_score > DIRT_THRES)
        {
            idx += 1;
            cur_idx += 1;
            *cnt = 1;
        } else {
            idx += 3;
            cur_idx += 3;
            *cnt = 3;
        }
    } else // 否则，需要根据前后行信息判断去哪一行
    {
        // 如果上一行需要清洗，则换向上一行
        if (wf[cur_idx - 1].clean_time == 0 && wf[cur_idx - 1].dirt_score > DIRT_THRES)
        {
            // 换向前一行，机器位置左移一行
            cur_idx -= 1;
            *cnt = -1;
        } else // 否则，向右判断
        {
            if (wf[cur_idx + 1].clean_time == 0) // 下一行如果没有清洗过
            {
                if (wf[cur_idx + 1].dirt_score > DIRT_THRES) // 如果下一行需要清洗，去下一行
                {
                    cur_idx += 1;
                    *cnt = 1;
                } else // 下一行不需要清洗，去下下下一行
                {
                    cur_idx += 3;
                    *cnt = 3;
                }
            } else // 下一行清洗过
            {
                if (wf[cur_idx + 2].dirt_score > DIRT_THRES) // 如果下下一行需要清洗
                {
                    cur_idx += 2;
                    *cnt = 2;
                } else // 基于下下行往后再跳两行
                {
                    cur_idx += 4;
                    *cnt = 4;
                }
                
            }
        }
        idx = (idx < cur_idx) ? cur_idx : idx;
    }

    return 0;
}

static int wash_floor_send_msg_to_action(planner_to_action_msg_t msg)
{
    return rt_mq_send(&g_planner_to_action_mq, msg, sizeof(struct planner_to_action_msg));
}

static int wash_floor_recv_msg_from_action(action_msg_t *msg, int timeout)
{
    return rt_mb_recv(&g_action_mb, (rt_ubase_t *)msg, timeout);
}




// 获取下一个校正点的地磁数据
static int move_and_get_next_gero(get_next_gero_out_t get_next_gero_move_out_info)
{
    p2a_msg.cmd = CMD_ACTION_FLOOR_GET_NEXT_MAGNETIC;
    
    struct get_next_gero_in get_next_gero_move_info;
    get_next_gero_move_info.move_rows = ADGEST_ROWCOUNT;
    get_next_gero_move_info.one_line_move_time = GAP_BETWEEN_ROWS;

    // clean first，获取4行地磁数据
    if (robot_work_mode == SMART_MODE)
    {
        get_next_gero_move_info.row_num = 3;
    } else if (robot_work_mode == ALL_MODE)
    {
        get_next_gero_move_info.row_num = 0;
    }
    
    p2a_msg.p_arg_in = &get_next_gero_move_info;
    p2a_msg.p_arg_out = get_next_gero_move_out_info;
    
    wash_floor_send_msg_to_action(&p2a_msg);
    
    // wait msg
    int ret = wash_floor_recv_msg_from_action(&action_msg_handle, RT_WAITING_FOREVER);
    LOG_D("CMD_ACTION_FLOOR_GET_NEXT_MAGNETIC, p2a_msg.p_arg_out.ret=%d", get_next_gero_move_out_info->ret);
    if (ret != 0) // 没有通信成功
    {
        LOG_E("CMD_ACTION_FLOOR_GET_NEXT_MAGNETIC, recv msg ERR");
        // 亮灯效
        return -1;
    }
    if (get_next_gero_move_out_info->ret == -1)
    {
        LOG_W("CMD_ACTION_FLOOR_GET_NEXT_MAGNETIC, get no mag");
        // 亮灯效
        return -1;
    }

    return 0;
}

int main()
{
    float adjust_yaw = 0.0f;
    // 首先判断是否需要校正
    float dis = cal_gero_euc_dis(&pre_gero_calib_info, &cur_gero_calib_info);
    LOG_D("pre vs cur geo dis, %f", dis);

    if (dis <= GERO_DIS_THRES)
    {
        LOG_I("adjust_yaw=%f", adjust_yaw);
        return 0.0f;
    } else {
        struct mag_calibrate_info info_a, info_b, info_c;
        info_a = pre_gero_calib_info;
        info_b = cur_gero_calib_info;
        LOG_D("info_a:%f, %f, info_b:%f, %f", info_a.raw_mag_x, info_a.raw_mag_y, info_b.raw_mag_x, info_b.raw_mag_y);

        float rotate_yaw = ADGEST_YAW*COORDINATE_SYSTEM_MODE;
        // 左转rotate_yaw度
R_T:
        LOG_D("rotate_yaw=%f", rotate_yaw);
        move_rotate_on_floor(rotate_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
        rt_thread_mdelay(500);
        get_current_gero(&info_c);
        LOG_D("infoc geo:%f, %f", info_c.raw_mag_x, info_c.raw_mag_y);

        // 计算||bc||
        float ba[] = {info_a.raw_mag_x - info_b.raw_mag_x, info_a.raw_mag_y - info_b.raw_mag_y};
        float bc[] = {info_c.raw_mag_x - info_b.raw_mag_x, info_c.raw_mag_y - info_b.raw_mag_y};
        float ca[] = {info_a.raw_mag_x - info_c.raw_mag_x, info_a.raw_mag_y - info_c.raw_mag_y};
        float cb[] = {info_b.raw_mag_x - info_c.raw_mag_x, info_b.raw_mag_y - info_c.raw_mag_y};
        float bc_l2 = vectorMagnitude(bc, 2);
        LOG_D("bc l2= %f", bc_l2);
        
        // 确定一度对应的dis
        float one_yaw_dis = bc_l2/fabs(rotate_yaw);
        LOG_D("one_yaw_dis=%f", one_yaw_dis);

        // 计算旋转方向
        float cosine = calculate_cosine(ba, bc, 2);
        LOG_D("ba, bc, cosine: %f, %f, %f, %f, %f", ba[0], ba[1], bc[0], bc[1], cosine);
        if (cosine < 0) // 转反了，需要右转校正
        {
            // 计算需要转动的角度
            float delta_yaw = vectorMagnitude(ca, 2)/one_yaw_dis;
            LOG_D("ca, delta_yaw: %f, %f, %f", ca[0], ca[1], delta_yaw);
            // 右转delta_yaw度
            move_rotate_on_floor(-delta_yaw*COORDINATE_SYSTEM_MODE, MOVE_CONTROL_RIGHT_SAFEZONE);
            rt_thread_mdelay(500);
            adjust_yaw = rotate_yaw-delta_yaw;
            LOG_D("adjust yaw=%f", adjust_yaw);
        } else if (cosine > 0) // 转的方向是对的
        {
            // 判断是否转过
            float cosine_right = calculate_cosine(ca, cb, 2);
            LOG_D("ca, cosine_right: %f, %f, %f", ca[0], ca[1], cosine_right);
            if (cosine_right > 0) // 转过了
            {
                // 需要往回转,右转
                // 计算需要转动的角度
                float delta_yaw = vectorMagnitude(ca, 2)/one_yaw_dis;
                LOG_D("delta yaw=%f", delta_yaw);
                // 右转delta_yaw度
                move_rotate_on_floor(-delta_yaw*COORDINATE_SYSTEM_MODE, MOVE_CONTROL_RIGHT_SAFEZONE);
                rt_thread_mdelay(500);
                adjust_yaw = rotate_yaw-delta_yaw;
                LOG_D("adjust_yaw=%f", adjust_yaw);
            } else if (cosine_right < 0) // 转少了
            {
                // 继续左转
                // 计算需要转动的角度
                float delta_yaw = vectorMagnitude(ca, 2)/one_yaw_dis;
                LOG_D("ca, delta_yaw: %f, %f, %f", ca[0], ca[1], delta_yaw);
                // 左转delta_yaw度
                move_rotate_on_floor(delta_yaw*COORDINATE_SYSTEM_MODE, MOVE_CONTROL_RIGHT_SAFEZONE);
                rt_thread_mdelay(500);
                adjust_yaw = rotate_yaw+delta_yaw;
                LOG_D("adjust_yaw=%f", adjust_yaw);
            } else // 转到了
            {
                adjust_yaw = rotate_yaw;
                LOG_D("adjust_yaw=%f", adjust_yaw);
            }            
        } else { // 90度无法判断方向，再次旋转一个小角度判断
            rotate_yaw += ADGEST_YAW*COORDINATE_SYSTEM_MODE;
            LOG_D("90 angle, rotate=%f", rotate_yaw);
            if (fabs(rotate_yaw) < 10.0f)
            {
                goto R_T;
            } else {
                adjust_yaw = 0.0f;
            }
        }

        LOG_I("adjust_yaw=%f", adjust_yaw);
        return adjust_yaw;
    }
    
    // int adjust_bool = 0;
    int ret = 0;
    float gero_dis = 0.0f;
    LOG_D("adjust yaw, cur_idx=%d, get_msg=%d", cur_idx, wf[cur_idx-1].get_mag);

    if (wf[cur_idx-1].get_mag == 1)
    {
        get_current_gero(&cur_gero_calib_info);
        LOG_D("cur_gero_calib_info. %f, %f", cur_gero_calib_info.raw_mag_x, cur_gero_calib_info.raw_mag_y);

        // // test geo
        // wf[cur_idx-1].mag.mag_x = 3.0f;
        // wf[cur_idx-1].mag.mag_y = 3.0f;
        // // test end

        pg.raw_mag_x = wf[cur_idx-1].mag.mag_x;
        pg.raw_mag_y = wf[cur_idx-1].mag.mag_y;
        LOG_D("pre_gero_calib_info. %f, %f", pg.raw_mag_x, pg.raw_mag_y);

        if (arg_out.out_count > 0)
        {
            float yaw1 = 0.0f, yaw2 = 0.0f;
            int yaw1_result = get_yaw_by_geom_data(sg_buff, arg_out.out_count, &cur_gero_calib_info, &yaw1);
            int yaw2_result = get_yaw_by_geom_data(sg_buff, arg_out.out_count, &pre_gero_calib_info, &yaw2);
            ret = (yaw1_result == 0) && (yaw2_result == 0);
            LOG_D("yaw1 = %f, yaw2 = %f, yaw1_result = %d, yaw2_result = %d", yaw1, yaw2, yaw1_result, yaw2_result);
            if (ret == 1)
            {
                gero_dis = yaw2 - yaw1;
                *cur_line_start_yaw += gero_dis;
            }
        }
        if (arg_out.out_count == 0 || ret == 0)
        {
            LOG_D("before adjust, cur_line_start_yaw=%f", *cur_line_start_yaw);

            float dis1 = cal_gero_euc_dis(&pre_gero_calib_info, &cur_gero_calib_info);
            LOG_D("dis1=%f", dis1);

            if (dis1 > GERO_DIS_THRES)
            {
                LOG_D("left dis_yaw=%f", ADGEST_YAW*COORDINATE_SYSTEM_MODE);
                move_rotate_on_floor(ADGEST_YAW*COORDINATE_SYSTEM_MODE, MOVE_CONTROL_RIGHT_SAFEZONE);
                rt_thread_mdelay(1*1000);

                gero_dis = ADGEST_YAW*COORDINATE_SYSTEM_MODE;

                get_current_gero(&cur_gero_calib_info);
                LOG_D("cur_gero_calib_info.gx = %f, cur_gero_calib_info.gy = %f", cur_gero_calib_info.raw_mag_x, cur_gero_calib_info.raw_mag_y);
                float dis2 = cal_gero_euc_dis(&pre_gero_calib_info, &cur_gero_calib_info);
                LOG_D("dis2=%f", dis2);

                if (dis2 > dis1)
                {
                    gero_dis += -2.0f*ADGEST_YAW*COORDINATE_SYSTEM_MODE;
                }
                
            }

            *cur_line_start_yaw += gero_dis;
            LOG_D("after adjust, cur_line_start_yaw=%f", *cur_line_start_yaw);

            wf[cur_idx-1].mag.mag_x = cur_gero_calib_info.raw_mag_x;
            wf[cur_idx-1].mag.mag_y = cur_gero_calib_info.raw_mag_y;
        }
    }

    struct get_next_gero_out get_next_gero_move_out_info;
    LOG_D("out addr %p", &get_next_gero_move_out_info);
    // if (wf[cur_idx-1].get_mag == 1 || cur_idx == 1)
    if ((cur_idx - 1)%ADGEST_ROWCOUNT== 0)
    {
        move_and_get_next_gero(&get_next_gero_move_out_info);

        // 更新line_buffer
        LOG_D("get_next_gero_move_out_info.gero_buff_sz=%d, line_info_size=%d", get_next_gero_move_out_info.gero_buff_sz, line_info_size);
        if ((cur_idx - 1 + ADGEST_ROWCOUNT + get_next_gero_move_out_info.gero_buff_sz) > (line_info_size - 1))
        {
            // 扩展malloc
            add_malloc();
        }
        for (int i = 0; i < get_next_gero_move_out_info.gero_buff_sz; i++)
        {
            wf[cur_idx-1 + ADGEST_ROWCOUNT + i].mag.mag_x = get_next_gero_move_out_info.gero_buff[i].raw_mag_x;
            wf[cur_idx-1 + ADGEST_ROWCOUNT + i].mag.mag_y = get_next_gero_move_out_info.gero_buff[i].raw_mag_y;
            wf[cur_idx-1 + ADGEST_ROWCOUNT + i].get_mag = 1;
        }
    }

    // 判断当前位置是否可以校准,当前位置有地磁信息即可校准
    // int adjust_bool = 0;
    int ret = 0;
    float gero_dis = 0.0f;
    LOG_D("adjust yaw, cur_idx=%d, get_msg=%d", cur_idx, wf[cur_idx-1].get_mag);
    LOG_D("cur_line_start_yaw=%f", *cur_line_start_yaw);

    // wf[cur_idx-1].get_mag = 1;

    if (wf[cur_idx-1].get_mag == 1)
    {
        // 获取当前地磁数据
        get_current_gero(&cur_gero_calib_info);
        LOG_D("cur_gero_calib_info. %f, %f", cur_gero_calib_info.raw_mag_x, cur_gero_calib_info.raw_mag_y);

        // // test geo
        // wf[cur_idx-1].mag.mag_x = cur_gero_calib_info.raw_mag_x + 30.0f;
        // wf[cur_idx-1].mag.mag_y = cur_gero_calib_info.raw_mag_y + 10.0f;
        // // test end

        pre_gero_calib_info.raw_mag_x = wf[cur_idx-1].mag.mag_x;
        pre_gero_calib_info.raw_mag_y = wf[cur_idx-1].mag.mag_y;
        LOG_D("pre_gero_calib_info. %f, %f", pre_gero_calib_info.raw_mag_x, pre_gero_calib_info.raw_mag_y);

        // 校准
        // 如果有地磁标定值，优先用标定值校准
        if (arg_out.out_count > 0)
        {
            // 判断地磁是否在圆上
            float yaw1 = 0.0f, yaw2 = 0.0f;
            int yaw1_result = get_yaw_by_geom_data(sg_buff, arg_out.out_count, &cur_gero_calib_info, &yaw1);
            int yaw2_result = get_yaw_by_geom_data(sg_buff, arg_out.out_count, &pre_gero_calib_info, &yaw2);
            ret = (yaw1_result == 0) && (yaw2_result == 0);
            LOG_D("yaw1 = %f, yaw2 = %f, yaw1_result = %d, yaw2_result = %d", yaw1, yaw2, yaw1_result, yaw2_result);
            if (ret == 1)
            {
                gero_dis = yaw2 - yaw1;
                *cur_line_start_yaw += gero_dis;
            }
            // 如果不能成功校准，需要用前后时刻的地磁值校
        }
        // 如果没有地磁标定值或者用地磁标定值没有成功校准
        if (arg_out.out_count == 0 || ret == 0)
        {
            LOG_D("before adjust, cur_line_start_yaw=%f", *cur_line_start_yaw);
            gero_dis = calculate_adjust_yaw();
            // 更新角度
            *cur_line_start_yaw += gero_dis;
            LOG_D("after adjust, cur_line_start_yaw=%f", *cur_line_start_yaw);
            // 更新地磁
            wf[cur_idx-1].mag.mag_x = cur_gero_calib_info.raw_mag_x;
            wf[cur_idx-1].mag.mag_y = cur_gero_calib_info.raw_mag_y;
        }
    }

    // 向后采集地磁数据
    struct get_next_gero_out get_next_gero_move_out_info;
    LOG_D("out addr %p", &get_next_gero_move_out_info);
    // if (wf[cur_idx-1].get_mag == 1 || cur_idx == 1)
    if ((cur_idx - 1)%ADGEST_ROWCOUNT== 0)
    {
        move_and_get_next_gero(&get_next_gero_move_out_info);

        // 更新line_buffer
        LOG_D("get_next_gero_move_out_info.gero_buff_sz=%d, line_info_size=%d", get_next_gero_move_out_info.gero_buff_sz, line_info_size);
        if ((cur_idx - 1 + ADGEST_ROWCOUNT + get_next_gero_move_out_info.gero_buff_sz) > (line_info_size - 1))
        {
            // 扩展malloc
            add_malloc();
        }
        for (int i = 0; i < get_next_gero_move_out_info.gero_buff_sz; i++)
        {
            wf[cur_idx-1 + ADGEST_ROWCOUNT + i].mag.mag_x = get_next_gero_move_out_info.gero_buff[i].raw_mag_x;
            wf[cur_idx-1 + ADGEST_ROWCOUNT + i].mag.mag_y = get_next_gero_move_out_info.gero_buff[i].raw_mag_y;
            wf[cur_idx-1 + ADGEST_ROWCOUNT + i].get_mag = 1;
        }
    }

    int if_c;
    struct move_info forward_judge_info;

    forward_judge_info.start_yaw = get_current_yaw();
    forward_judge_info.move_time = move_time;
    forward_judge_info.move_speed = MOVE_SPEED_PWM;
    forward_judge_info.judge_edge_pitch_thres = JUDGE_PITCH_THRES;
    p2a_msg.cmd = CMD_ACTION_FLOOR_MOVE_FORWARD_JUDGE_C_EDGE;
    p2a_msg.p_arg_in = &forward_judge_info;
    p2a_msg.p_arg_out = &if_c;

    LOG_I("CMD_ACTION_FLOOR_MOVE_FORWARD_JUDGE_C_EDGE");
    wash_floor_send_msg_to_action(&p2a_msg);
    msg_size = wash_floor_recv_msg_from_action(&action_msg_handle, RT_WAITING_FOREVER);
    if (msg_size != 0) // 没有通信成功
    {
        LOG_E("recv msg ERR");
        // 亮灯效
        return -1;
    }
    LOG_I("if_c=%d", if_c);

    float cur_pitch;
    LOG_I("back_down_wall");
    while (1)
    {
        cur_pitch = get_current_pitch();
//        LOG_D("cur_pitch=%f", cur_pitch);
        if (cur_pitch > JUDGE_PITCH_THRES)
        {
            LOG_I("stop back");
            move_stop_time(500);
            break;
        }

        // cur_yaw = get_current_yaw();
        move_backward_with_pid(1500, 0.0f, 0.0f);
    }

    float cur_pitch;
    int if_odd = wash_floor_task_info.total_moved_row%2;
    LOG_D("if odd = %d", if_odd);
    int judge_line_count, move_line_count;          

    // even_line_to_cleaned_num += 1;
    // odd_line_to_cleaned_num += 1;
    if (even_line_to_cleaned_num != -1)
    {
        even_line_to_cleaned_num += 1;
    }
    if (odd_line_to_cleaned_num != -1)
    {
        odd_line_to_cleaned_num += 1;
    }
    

    if (if_odd == 1) // 奇数行
    {
        judge_line_count = odd_info.n_line_to_c;
        move_line_count = odd_line_to_cleaned_num;
        if_last_line_get_c = even_info.if_arrvied_c;
    } else {
        judge_line_count = even_info.n_line_to_c;
        move_line_count = even_line_to_cleaned_num;
        if_last_line_get_c = odd_info.if_arrvied_c;
    }
    LOG_D("judge_line_count=%d, move_line_count=%d, if_last_line_get_c=%d",
         judge_line_count, move_line_count, if_last_line_get_c);
        
    
    cur_pitch = get_current_pitch();
    LOG_D("judege c, cur pitch=%f", cur_pitch);
    if (cur_pitch < -SLOPE_THRES_MAX)
    {
        LOG_D("pitch get c");
        cur_line_to_next_line_info.cur_line_pitch_get_c = 1;
        if (if_odd == 1) 
        {
            odd_info.if_arrvied_c = 1;
            odd_line_to_cleaned_num = -1;
            LOG_I("odd line get c from pitch");
        } else { 
            even_info.if_arrvied_c = 1;
            even_line_to_cleaned_num = -1;
            LOG_I("even line get c from pitch");
        }

        back_down_wall();
        LOG_D("get c from pitch");
        return 1;
    }
    if (cur_line_to_next_line_info.cur_line_roll_get_c == 1)
    {
        if (if_odd == 1) 
        {
            odd_info.if_arrvied_c = 1;
            odd_line_to_cleaned_num = -1;
            LOG_I("odd line get c from roll");
        } else { 
            even_info.if_arrvied_c = 1;
            even_line_to_cleaned_num = -1;
            LOG_I("even line get c from roll");
        }
        LOG_D("get c from roll");
        return 1;
    }
    if (wf_cur_line_edge == GET_STEPS)
    {
        if (if_odd == 1) 
        {
            odd_info.if_arrvied_c = 1;
            odd_line_to_cleaned_num = -1;
            LOG_I("odd line get c from step");
        } else { 
            even_info.if_arrvied_c = 1;
            even_line_to_cleaned_num = -1;
            LOG_I("even line get c from step");
        }
        LOG_D("get c from steps");
        return 1;
    }
    

    LOG_D("a_dis=%d, b_dis=%d", a_dis, b_dis);
    if (((a_dis - b_dis) > 50) && ((a_dis - b_dis) < 1000)) // 在误差范围内，两者可信
    {
        LOG_D("judge c edge from dis");
        // 更新维护的信息
        if (if_odd == 1)
        {
            odd_info.a_dis = a_dis;
            odd_info.b_dis = b_dis;
            odd_info.n_line_to_c = (int)((float)b_dis / ONE_LINE_LENGTH);
        } else 
        {
            even_info.a_dis = a_dis;
            even_info.b_dis = b_dis;
            even_info.n_line_to_c = (int)((float)b_dis / ONE_LINE_LENGTH);
        }
        // 如果B在阈值内，到边
        if ((b_dis < (CEDGE_THRES-200))&&(0 < b_dis))
        {
            LOG_D("current line arrive c edge.");
            if (if_odd == 1)
            {
                odd_info.if_arrvied_c = 1;
                odd_line_to_cleaned_num = -1;
                LOG_I("odd line get c from dis");
            } else
            {
                even_info.if_arrvied_c = 1;
                even_line_to_cleaned_num = -1;
                LOG_I("even line get c from dis");
            }
            return 1;
        }        
    } else if (((0 < b_dis)&&(b_dis < (CEDGE_THRES-100))) || ((0 < a_dis)&&(a_dis < CEDGE_THRES)) || (judge_line_count == move_line_count) || if_last_line_get_c)
    {
        // 如果B或A落入阈值内，或到计算行，或另一边到墙，用pitch前进判断
        int time = 0;
        int if_get_c_edge = 0;
        LOG_D("odd_info.if_arrvied_c=%d, odd_line_to_cleaned_num=%d", odd_info.if_arrvied_c, odd_line_to_cleaned_num);
        LOG_D("even_info.if_arrvied_c=%d, even_line_to_cleaned_num=%d", even_info.if_arrvied_c, even_line_to_cleaned_num);

        if (if_odd == 1) // 如果是奇数行
        {
            if (odd_info.if_arrvied_c!=1&&(odd_line_to_cleaned_num==-1||odd_line_to_cleaned_num>(ADJUST_C_COUNT-1)))
            {
                time = determine_c_edge_action(judge_time);
                if (time > 0)
                {
                    odd_info.if_arrvied_c = 1;
                    if_get_c_edge = 1;
                }
                odd_line_to_cleaned_num = 0;
            }
            
        } else  // 如果是偶数行
        {
            if (even_info.if_arrvied_c!=1&&(even_line_to_cleaned_num==-1||even_line_to_cleaned_num>(ADJUST_C_COUNT-1)))
            {
                time = determine_c_edge_action(judge_time);
                if (time > 0)
                {
                    even_info.if_arrvied_c = 1;
                    if_get_c_edge = 1;
                } 
                even_line_to_cleaned_num = 0;
            }
        }
        LOG_D("time=%d, if_odd=%d, even_info.if_arrvied_c=%d, odd_info.if_arrvied_c=%d", 
                time,if_odd,even_info.if_arrvied_c,odd_info.if_arrvied_c);
         
        return if_get_c_edge;
    }
    
//     static struct dis_rotate_in_arg in;
//     in.time_interval = 0;
//     if (wash_floor_task_info.total_moved_row%2) // 奇数行左转。左手坐标系下，左转为正；右手坐标系下，左转为负
//     {
//         in.single_rotate_angle = 5.0f*COORDINATE_SYSTEM_MODE;
//         in.total_rotate_angle = 90.0f*COORDINATE_SYSTEM_MODE;
//     } else
//     {
//         in.single_rotate_angle = -5.0f*COORDINATE_SYSTEM_MODE;
//         in.total_rotate_angle = -90.0f*COORDINATE_SYSTEM_MODE;
//     }

//     static struct distance_sensor_out_arg dis_buffer;
//     dis_buffer.laser_buff = tof_info_buff;
//     dis_buffer.laser_buff_sz = DIS_INFO_BUFF_COUNT;
//     dis_buffer.ultrasonic_buff = ult_info_buff;
//     dis_buffer.ult_buff_sz = DIS_INFO_BUFF_COUNT;

//     p2a_msg.cmd = CMD_ACTION_FLOOR_ROTATE_AND_MEATURE_DIS;
//     p2a_msg.p_arg_in = &in;
//     p2a_msg.p_arg_out = &dis_buffer;
//     LOG_D("CMD_ACTION_FLOOR_ROTATE_AND_MEATURE_DIS");
//     wash_floor_send_msg_to_action(&p2a_msg);

//     // wait msg
//     msg_size = wash_floor_recv_msg_from_action(&action_msg_handle, RT_WAITING_FOREVER);
//     if (msg_size != 0) // 没有通信成功
//     {
//         LOG_E("recv msg ERR");
//         // 亮灯效
//     }
//     LOG_D("rotate and meature dis %d", action_msg_handle->msg);
    
//     // 根据测到的点拟合曲线
//     calculate_points_by_distance(&dis_buffer, pt_info_buff, &dis_pt_info_num, &rotate_point_count);
//     float a0, a1, a2;
//     Least_Squarel_Linear_Fit(&a0, &a1, &a2, rotate_point_count, pt_info_buff);
//     LOG_D("a0=%f, a1=%f, a2=%f\n", a0, a1, a2);

//     // 在以当前机器为原点的坐标系下，计算点
//     float x = GAP_BETWEEN_ROWS * MOTOR_SPEED_MOVE_SPEED/1000.0f; // 工字形的换行边长x 。这里的speed单位为mm/s
// 	float y = a2*x*x + a1*x + a0;

//     point->x = x;
//     point->y = y;
//     LOG_D("point: x=%f, y=%f", point->x, point->y);
    
//     return 0;
}
























int get_next_row_point_no_stop(point_info_t point, float yaw)
{
    // 旋转测距
    float delta;
    LOG_D("wash_floor_task_info.total_moved_row=%d", wash_floor_task_info.total_moved_row);

    if (wash_floor_task_info.total_moved_row%2) // 奇数行左转。左手坐标系下，左转为正；右手坐标系下，左转为负
    {
        delta = 90.0f*COORDINATE_SYSTEM_MODE;
    } else  // 偶数行右转。左手坐标系下，右转为负；右手坐标系下，右转为正
    {
        delta = -90.0f*COORDINATE_SYSTEM_MODE;
    }

    static struct dis_sensor_buffer_info dis_buffer;
    dis_buffer.dis_buff = NULL;

    p2a_msg.cmd = CMD_ACTION_FLOOR_ROTATE_AND_MEATURE_DIS_NO_STOP;
    p2a_msg.p_arg_in = &delta;
    p2a_msg.p_arg_out = &dis_buffer;
    LOG_I("CMD_ACTION_FLOOR_ROTATE_AND_MEATURE_DIS_NO_STOP");
    wash_floor_send_msg_to_action(&p2a_msg);

    // wait msg
    msg_size = wash_floor_recv_msg_from_action(&action_msg_handle, RT_WAITING_FOREVER);
    if (msg_size != 0) // 没有通信成功
    {
        LOG_E("recv msg ERR");
        return -1;
        // 亮灯效
    }
    LOG_D("rotate and meature dis %d", action_msg_handle->msg);
    LOG_I("dis count %d", dis_buffer.data_count);
    // for (size_t i = 0; i < dis_buffer.data_count; i++)
    // {
    //     LOG_D("%d %d %.2f\n", i, dis_buffer.dis_buff[i].distance, dis_buffer.dis_buff[i].yaw);
    // }
    // 根据测到的点拟合曲
    if (dis_buffer.data_count == 0)
    {
        LOG_W("rotate and get dis, get 0 point");
        return -1;
    } else {
        float a0, a1, a2;
        // yaw = cur_line_start_yaw;
        fit_line_by_dis_inf0(yaw, &dis_buffer, &a0, &a1, &a2);
        LOG_I("a0=%f, a1=%f, a2=%f\n", a0, a1, a2);

        // 在以当前机器为原点的坐标系下，计算点
        float x, y;
        if (wash_floor_task_info.total_moved_row%2)  // 如果是奇数行,左转时拟合
        {
            x = -GAP_BETWEEN_ROWS * MOTOR_SPEED_MOVE_SPEED/1000.0f; // 工字形的换行边长x 。这里的speed单位为mm/s
        } else // 偶数行,右转时拟合
        {
            x = GAP_BETWEEN_ROWS * MOTOR_SPEED_MOVE_SPEED/1000.0f; // 工字形的换行边长x 。这里的speed单位为mm/s
        }
        y = a2*x*x + a1*x + a0;       

        point->x = x;
        point->y = y;
        // LOG_D("point: x=%f, y=%f", point->x, point->y);
    }

    return 0;
}


static int half_judge_c_edge(int* line_num)
{
    // 先测距
    rt_tick_t cur_tick, sensor_tick;
    int to_c_time;
    rt_uint8_t label;
    rt_uint16_t dis;

    cur_tick = rt_tick_get_millisecond();
    get_distance_fusion(&sensor_tick, &label, &dis);
    LOG_D("dis=%d, label=%d, sensor_tick=%d", dis, label, sensor_tick);
    if (label == 1 && (abs(cur_tick - sensor_tick) < 4000)) // 数据可信且新
    {
        if (dis < 1000)
        {
            *line_num = (int)dis/400 + 1;
            LOG_I("get half c from dis. line_num=%d", *line_num);
            return 0;
        }
    }
    {
        // 前进探边
        to_c_time = determine_c_edge_action(JUDGE_C_TIME*2);
        LOG_D("half judge c, to_c_time=%d", to_c_time);
        if (to_c_time == -1)
        {
            LOG_E("determine_c_edge_action recv msg ERR");
            *line_num = -1;
            return -1;
        }
        if (to_c_time > 0) // 碰到第三边
        {
            *line_num = (int)to_c_time/GAP_BETWEEN_ROWS + 1;
            LOG_I("get half c from time. line_num=%f", to_c_time);
        } 
        if (to_c_time == 0) // 沒有碰到第三邊
        {
            *line_num = -1;
        }
        
    }
    LOG_D("line_num=%d", *line_num);
    // 再前进探边
    return 0;
}

static int judge_next_same_forward_derection_time()
{
    return ((cur_line_to_next_line_info.cur_line_roll_get_c == 1)
            ||(cur_line_to_next_line_info.cur_line_pitch_get_c == 1)
            ||((cur_line_to_next_line_info.a_dis < 250) && (cur_line_to_next_line_info.a_dis > 0))
            ||((cur_line_to_next_line_info.b_dis < 100) && (cur_line_to_next_line_info.b_dis > 0)));
}
static int reset_cur_line_to_next_line_info()
{
    cur_line_to_next_line_info.cur_line_pitch_get_c = 0;
    cur_line_to_next_line_info.cur_line_roll_get_c = 0;
    cur_line_to_next_line_info.a_dis = -1;
    cur_line_to_next_line_info.b_dis = -1;
    if_clean_edge = 0;
    if_current_edge_dirty = 0;
    forward_action_ret_info.vision_clean_time = 0;
    return 0;
}

static int reset_once_clean_info()
{
    odd_info.if_arrvied_c = 0;
    odd_info.n_line_to_c = 40;
    even_info.if_arrvied_c = 0;
    even_info.n_line_to_c = 40;

    remained_lines_num = -1;
    from_judge_moved_lines_num = -1;
    even_line_to_cleaned_num = -1;
    odd_line_to_cleaned_num = -1;
    wf_cur_line_edge = INIT_VALUE;
    return 0;
}


static int judge_and_escape_drain(float target_yaw)
{
    float cur_yaw = get_current_yaw();
    LOG_I("judge drain, cur_yaw=%f", cur_yaw);
    float delta_yaw;
    delta_yaw = compare_yaws(cur_yaw, target_yaw);
    if (fabs(delta_yaw) > 30.0f)
    {
        // 认为卡住地漏, 脱困
        LOG_I("escape drain");
        int w_s = move_wp_speed_on_floor_get();
        LOG_D("get current wp_speed=%d", w_s);
        robot_escape_drain();
        LOG_I("escape drain end");
        rt_thread_mdelay(500);
        
        // 开水泵
        move_wp_speed_on_floor_set(w_s);
        // rt_thread_mdelay(3*1000);
        rt_thread_mdelay(1*1000);

        // 纠正角度
        LOG_D("target_yaw=%f", target_yaw);
        move_rotate_on_floor_using_target(target_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
        rt_thread_mdelay(500);
    }

    return 0;
}


// 返回是否更新了start_yaw
static int find_slope_during_washing()
{
    int ret = 0;
    int if_update_start_yaw = 0;
    float cur_pitch, pre_yaw, new_yaw;
    RT_UNUSED(cur_pitch);
    RT_UNUSED(pre_yaw);
    LOG_I("slope_label=%d, forward_action_ret_info.line_state=%d", 
            slope_label, forward_action_ret_info.line_state);

    if ((slope_label<1) && (forward_action_ret_info.line_state > GROUND_DOWNWARD))
    { 
        LOG_D("start find slope");
        if (cur_model_type.model_mode == MODEL_P10) 
        {
            LOG_D("turn off light");
            wf_ctr_light(0);
            rt_thread_mdelay(50);   
        }

        // 找到坡度和对应yaw
        if (slope_label < 1)
        {
            LOG_D("rotate max slope");
            ret = move_rotate_on_slope_using_target(90.0f, 60*1000);
            LOG_D("ret=%d", ret);
            rt_thread_mdelay(1*1000);

            if (ret==RT_EOK)
            {
                LOG_I("find slope.");
                slope_label = 1;
                // 重置start_yaw
                new_yaw = get_current_yaw();
                LOG_D("reset start yaw=%f", new_yaw);
                cur_line_start_yaw = new_yaw;
                cur_time_start_yaw = cur_line_start_yaw;
                wash_floor_info_reset_after_get_slope(&wash_floor_task_info, cur_line_start_yaw);
                LOG_I("cur_line_start_yaw = %f, cur_time_start_yaw=%f, wash_time=%d", 
                    cur_line_start_yaw, cur_time_start_yaw, wash_floor_task_info.wash_times);
                if_update_start_yaw = 1;
            }
        } 
        // else if (find_slope_time==1)
        // {
        //     cur_pitch = get_current_pitch();
        //     LOG_D("pitch=%d", cur_pitch);
        //     if (abs(cur_pitch) > 5.0f)
        //     {
        //         LOG_D("find new slopes");
        //         pre_yaw = pool_info.max_pitch_yaw;
        //         ret = rotate_and_get_max_pitch(&pool_info);
        //         if ((ret==0) && (pool_info.max_pitch > SLOPE_THRES_MIN))
        //         {
        //             find_slope_time += 1;
        //             LOG_D("find a new slope");
        //             new_yaw = pool_info.max_pitch_yaw;
        //             LOG_D("pre_yaw=%f, new_yaw=%f", pre_yaw, new_yaw);
        //             if (fabs(compare_yaws(pre_yaw, new_yaw)) < 50.0f)
        //             {
        //                 cur_line_start_yaw = (float)((pre_yaw+new_yaw)/2);
        //                 cur_time_start_yaw = cur_line_start_yaw;
        //                 wash_floor_info_reset(&wash_floor_task_info, cur_line_start_yaw);
        //                 LOG_D("cur_line_start_yaw = %f, time=%d", cur_line_start_yaw, wash_floor_task_info.wash_times);
        //                 if_update_start_yaw = 1;
        //             }
        //         }
        //     }
        // } 
    } 
    return if_update_start_yaw;
}

static int find_deep_area()
{
    if (slope_label < 1)
    {
        LOG_I("find no slope, not find deep area");
        return 0;
    }
    if (if_find_deep_area == 1)
    {
        LOG_I("slope cleared");
        return 0;
    }

    int get_up_slope = 0, get_down_slope = 0, get_flat = 0;
    float slope = get_current_slope();
    LOG_D("cur_slope=%f", slope);

    if (forward_action_ret_info.info.down_count > COUNT_THRES)
    {
        get_down_slope = 1;
    }
    if (forward_action_ret_info.info.up_count > COUNT_THRES)
    {
        get_up_slope = 1;
    }
    if (forward_action_ret_info.info.flat_count > COUNT_THRES)
    {
        get_flat = 1;
    }
    LOG_D("up=%d, down=%d, flat=%d", forward_action_ret_info.info.up_count, 
        forward_action_ret_info.info.down_count, forward_action_ret_info.info.flat_count);
    LOG_D("get_down_slope=%d, get_up_slope=%d, get_flat=%d", get_down_slope, get_up_slope, get_flat);
    
    switch (deep_area_state)
    {
        case STATE_INITIAL:
            // if (get_down_slope == 1) {
            //     deep_area_state = GROUND_DOWNWARD;
            //     state_count = 1;
            //     LOG_D("pool get GROUND_DOWNWARD");
            // }
            if (slope_label > 0)
            {
                deep_area_state = GROUND_DOWNWARD;
                state_count = 2;
                LOG_I("get slope, GROUND_DOWNWARD");
            }
            
            break;
        case GROUND_DOWNWARD:
            if (state_count > STATE_COUNT)
            {
                if (get_flat==1)
                {
                    state_count = 1;
                    deep_area_state = GROUND_FLAT;
                    LOG_I("up state to flat");
                    break;
                } 
                // state_count += 1;
            }
            if (get_down_slope == 1)
            {
                state_count += 1;
                LOG_D("down slope ,state_count ++");
            }
            // else
            // {
            //     state_count = 0;
            // }
            LOG_D("state_count=%d", state_count);
            break;
        case GROUND_FLAT:
            if(state_count > (STATE_COUNT-1))
            {
                deep_area_state = GROUND_UPWARD;
                state_count = 1;
                LOG_I("up state to upslope");
                break;
                // if (get_up_slope==1)
                // {
                //     state_count = 1;
                //     deep_area_state = GROUND_UPWARD;
                //     LOG_D("up state to upslope");
                //     break;
                // } 
            } 
            if (get_flat == 1)
            {
                state_count += 1;
                LOG_D("flat slope ,state_count ++");
            }
            // else
            // {
            //     state_count = 0;
            // }
            LOG_D("state_count=%d", state_count);
            break;
        case GROUND_UPWARD:
            LOG_D("get slope=%f, state_count=%d", slope, state_count);
            if ((slope <= -5.0f) || (get_up_slope==1))
            {
                state_count += 1;
                LOG_D("in up state, state_count ++");
            }
            // else
            // {
            //     state_count = 0;
            //     LOG_D("no upward continuous");
            // }
            if (state_count > STATE_COUNT)
            {
                LOG_I("find deep area");
                deep_area_state = GROUND_GET_SLOPE;
            }
            break;
        case GROUND_GET_SLOPE:
            break;
    }
    LOG_D("deep_area_state=%d, state_count=%d", deep_area_state, state_count);

    return 0;
}


static int wf_actions_on_clean_edge(point_info_t point)
{
    // 检测表面脏污
    if_clean_edge = 0;    // 是否做边缘处的清洗,0否1是
    struct region_info dirt_info = {0};
    rt_uint8_t id_thd = 1; // 有脏污就计入
    rt_err_t ret = get_srf_dy_regin_info(&dirt_info, id_thd);
    if (ret != RT_EOK)
    {
        LOG_D("get dirty info error!, dont clean edge %d", if_clean_edge);
    } else
    {
        LOG_I("left=%d, middle=%d, right=%d", dirt_info.left, dirt_info.middle, dirt_info.right);
        if (dirt_info.middle == 1)
        {
            if_clean_edge = 1;
            LOG_I("ahead dirty, clean edge %d", if_clean_edge);
        }
    }
    LOG_D("if_clean_edge=%d", if_clean_edge);

    // 边缘拟合
    LOG_D("get_next_row_point_no_stop(&point)");
    ret = get_next_row_point_no_stop(point, cur_line_start_yaw);
    LOG_D("get_next_row_point ret=%d", ret);
    rt_thread_mdelay(500);
    float cur_yaw = get_current_yaw();
    LOG_D("cur_yaw = %f", cur_yaw);

    if ((if_clean_edge == 1) || (if_current_edge_dirty == 1))
    {
        // 后退，大吸力清洗边缘
        LOG_D("clean egde");
        _clean_edge(cur_line_start_yaw, test_param);
        move_rotate_on_floor_using_target(cur_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
        rt_thread_mdelay(500);
    }
    
    return 0;
}


static enum edge_info move_to_edge(move_forward_action_return_info_t info, float start_yaw, int* slope_forward_time, int need_close_dis_sensor_time)
{
    LOG_I("move to edge");
    const float edge_pitch_thres = -15.0f;  
    // 获取当前pitch
    float cur_yaw, start_pitch, cur_pitch, cur_pitch_get_wall, pre_pitch;
    RT_UNUSED(edge_pitch_thres);
    start_pitch = get_current_pitch();
    pre_pitch = start_pitch;
    LOG_D("start_pitch=%f", start_pitch);

    rt_tick_t sensor_tick, cur_tick;
    rt_uint8_t label;
    rt_uint16_t dis;

    // 前进，探测坡
    // int forward_time = (dis / MOTOR_SPEED_MOVE_SPEED + 2)*1000;
    int forward_time, total_forward_time = 0;
    if (info->ret_idx == 0)
    {
        forward_time = (info->end_dis / MOTOR_SPEED_MOVE_SPEED + 1)*1000;
    } else
    {
        forward_time = 0;
    }
    
    LOG_D("from dis=%d, forward time=%d", info->end_dis, forward_time);

    // int if_get_wall = 0;
    // int if_get_steps = 0;
    enum edge_info ret_info = INIT_VALUE;
    rt_tick_t cur_time, start_time;
    start_time = rt_tick_get_millisecond();
    cur_time = start_time;
    int diff_count = DIFF_COUNT;
    float pre_yaw_diff, cur_yaw_diff; 
    float start_roll = get_current_roll();
    float cur_roll = start_roll;
    LOG_D("start_roll=%f", cur_roll);
    float start_slope, cur_slope;
    start_slope = get_current_slope();
    LOG_D("start_slope=%f", start_slope);
    int count = 0;

    // 判断是否上坡
    cur_yaw = get_current_yaw();
    LOG_D("before edge forward, start_yaw=%f", cur_yaw);
    cur_time = rt_tick_get_millisecond();
    start_time = cur_time;
    while (cur_time - start_time < forward_time)
    {
        cur_yaw = get_current_yaw();
        cur_pitch = get_current_pitch();
        cur_roll = get_current_roll();
        cur_slope = get_current_slope();
        if (cur_slope < -SLOPE_THRES_MAX)
        {
            LOG_D("pitch to wall");
            ret_info = GET_WALL;
            break;
        }       
        move_forward_with_pid(1000, cur_yaw, start_yaw);
        rt_thread_mdelay(100);
        // LOG_D("cur_yaw=%f", cur_yaw);
        cur_time = rt_tick_get_millisecond();
    }
    cur_pitch = get_current_pitch();
    pre_pitch = cur_pitch;
    LOG_D("cur_pitch=%f", cur_pitch);
    // delay 1s,如果是墙水泵把机器压下来。
    move_stop_time(1*1000);
    cur_pitch = get_current_pitch();
    cur_slope = get_current_slope();
    cur_yaw = get_current_yaw();
    LOG_D("after delay, cur_pitch=%f, cur_yaw=%f, cur_slope=%f", cur_pitch, cur_yaw, cur_slope);

    cur_roll = get_current_roll();
    LOG_D("cur_roll=%f", cur_roll);
    if (cur_slope < -SLOPE_THRES_MAX)
    {
        LOG_I("get wall, cur_slope=%f", cur_slope);
        ret_info = GET_WALL;
    } else if (start_slope - cur_slope > PITCH_DIFF_THRES)
    {
        LOG_I("get slope, cur_slope=%f", cur_slope);
        ret_info = GET_SLOPE;
    }
    

#if 1
    // 如果有抬起，继续前进直到上墙
    pre_yaw_diff = compare_yaws(cur_yaw, start_yaw);
    LOG_D("cur_yaw=%f, yaw_diff=%f", cur_yaw, pre_yaw_diff);

    start_time = rt_tick_get_millisecond();
    while (ret_info == GET_SLOPE)
    {
        LOG_I("slope, go on clean");
        cur_slope = get_current_slope();
        cur_pitch_get_wall = get_current_pitch();
        if (cur_slope < -SLOPE_THRES_MAX)
        {
            LOG_I("arrive wall, slope=%f", cur_slope);
            break;
        }
        if (rt_tick_get_millisecond() - start_time > MOVE_FORWARD_TIMEOUT)
        {
            LOG_I("clean slope time out: %d, break.", rt_tick_get_millisecond() - start_time);
            break;
        }

        cur_yaw = get_current_yaw();
        cur_yaw_diff = compare_yaws(cur_yaw, start_yaw);
        if (verify_same_sign_and_increasing(cur_yaw_diff, pre_yaw_diff))
        {
            diff_count -= 1;
            LOG_D("diff=%f, bigger", cur_yaw_diff);
        } else
        {
            diff_count = DIFF_COUNT;
        }
        if (diff_count < 1) // 角度持续纠不过来，退出前进
        {
            LOG_I("yaw always diff, end");
            break;
        }
        pre_yaw_diff = cur_yaw_diff; 
        move_forward_with_pid(test_param->forward_mtr_speed, cur_yaw, start_yaw);
        if (cur_pitch_get_wall - pre_pitch > PITCH_DIFF_THRES) // 检测到机头回平,认为有台阶
        {

            ret_info = GET_STEPS;
            LOG_I("cur_pitch=%f, get steps", cur_pitch_get_wall);
            break;
        }

        cur_tick = rt_tick_get_millisecond();
        if (cur_tick - start_time > need_close_dis_sensor_time)
        {
            get_distance_fusion(&sensor_tick, &label, &dis);
            if (label == 1 && (abs(cur_tick - sensor_tick) < 4000)) // 数据可信且新
            {
                if ((dis < EDGE_THRES) && (dis > 0))
                {
                    LOG_I("get wall from dis %d, end", dis);
                    break;
                }
            }
        }
        
        rt_thread_mdelay(100);
    }
    total_forward_time += (rt_tick_get_millisecond() - start_time);
    move_stop_time(500);
    
    LOG_D("total_forward_time=%d", total_forward_time);
    *slope_forward_time = total_forward_time;
    cur_yaw = get_current_yaw();
    cur_slope = get_current_slope();
    LOG_D("cur_yaw=%f, cur_slope=%f", cur_yaw, cur_slope);
#endif // if 1

#if (WF_CLAB_WALL==RT_TRUE)
    wf_ctr_light(0);
    rt_thread_mdelay(50); 
    // TODO:爬墙
    struct stop_on_waterline_info ww_info = {
        .power_off_flag = 0,        // no power off
        .on_waterline_time = 1 ,  // 1s
        .in_water_time = 1,       // 1s
        .repeat_count = 1};       // repeat 1 times
    goto_waterline_and_stop(&ww_info);
    move_stop_time(1000);
    LOG_D("cur yaw=%f", get_current_yaw());
    move_rotate_on_floor_using_target(start_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
    move_wp_speed_on_floor_set(test_param->forward_wp_speed);
    move_stop_time(1*1000);
    LOG_D("cur yaw=%f", get_current_yaw());
#endif  // if (WF_CLAB_WALL==RT_TRUE)

    // down steps
    if (ret_info == GET_STEPS) // 如果是台阶，先后退2s，以免落在平台上
    {
        move_backward_with_speed_and_time(test_param->forward_mtr_speed, 2 * 1000);
        move_stop_time(500);
    }

    // down wall
    cur_pitch = get_current_pitch();
    LOG_D("move to edge down wall, cur_pitch=%f", cur_pitch);
    while (cur_pitch < -SLOPE_THRES_MAX)
    {
        move_backward_with_speed_and_time(test_param->forward_mtr_speed, 1 * 1000);
        count++;
        if (count > 5)
        {
            LOG_D("down wall timeout");
            break;
        }
        cur_pitch = get_current_pitch();
        // LOG_D("down wall, pitch=%f", cur_pitch);
    }
    move_stop_time(1000);

    // back 
    forward_time = 4*1000;
    count = 0;
    cur_slope = get_current_slope();
    cur_yaw = get_current_yaw();
    cur_pitch = get_current_pitch();
    LOG_D("after down wall, cur_slope=%f, cur_yaw=%f, cur_pitch=%f", cur_slope, cur_yaw, cur_pitch);
    start_slope = cur_slope;
    LOG_D("start back to smooth surface");

    start_time = rt_tick_get_millisecond();
    cur_time = start_time;
    while (cur_time - start_time < forward_time)
    {
        cur_yaw = get_current_yaw();
        move_backward_with_pid(test_param->forward_mtr_speed, cur_yaw, start_yaw);
        cur_slope = get_current_slope();
        // LOG_D("back, cur_yaw=%f, cur_slope=%f", cur_yaw, cur_slope);
        if (fabs(start_slope - cur_slope) < 5.0f)
        {
            count += 1;
            // LOG_D("same slope as pre");
        } else
        {
            count = 0;
            start_slope = cur_slope;
            LOG_D("update slope, slope=%f", start_slope);
        }
        // LOG_D("count=%d", count);
        if (count > 7)
        {
            LOG_D("get smooth surface");
            break;
        }
        cur_time = rt_tick_get_millisecond();
        rt_thread_mdelay(100);
    } 
    move_stop_time(500);

    cur_slope = get_current_slope();
    cur_yaw = get_current_yaw();
    cur_pitch = get_current_pitch();
    LOG_D("after get surface, cur_slope=%f, cur_yaw=%f, cur_pitch=%f", cur_slope, cur_yaw, cur_pitch);

    move_forward_with_pid_and_time(test_param->forward_mtr_speed, start_yaw, 1*1000);
    move_stop_time(500);

    // adjust yaw
    if (fabs(get_current_yaw() - start_yaw) >= 3.0f)
    {
        move_rotate_on_floor_using_target(start_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
        rt_thread_mdelay(500);
    }
#if 0
    start_time = rt_tick_get_millisecond();
    cur_time = start_time;
    cur_pitch = get_current_pitch();
    
    
    while (cur_time - start_time < forward_time)
    {
        cur_yaw = get_current_yaw();
        move_backward_with_pid(WASH_FLOOR_MOTOR_SPEED, cur_yaw, start_yaw);
        if (fabs(cur_pitch - start_pitch) < 10.0f)
        {
            LOG_D("back to start pitch");
            break;
        }
        
        cur_pitch = get_current_pitch();
        cur_time = rt_tick_get_millisecond();
        rt_thread_mdelay(100);
    }
    
#endif

    return ret_info;
}

static int update_wf_interval(void)
{
    // if (forward_action_ret_info.if_right_area_have_leaves == 1)
    // {
    //     LOG_I("right area have leaves, no cross line");
    //     line_wa_interval = 1.0f;
    // } else
    // {
    //     LOG_I("right area have no leaves");
    //     line_wa_interval = test_param->adjacent_line_interval;
    // }
    line_wa_interval = test_param->adjacent_line_interval;
    LOG_D("line_wa_interval=%f", line_wa_interval);

    return 0;
}

static int _back_with_time(rt_tick_t back_time, float start_yaw)
{
    float cur_yaw;
    rt_tick_t start_time, cur_time;
    struct slope_type cur_slope_info;
    LOG_D("back time=%d, start_yaw=%f", back_time, start_yaw);
    get_current_slope_info(&cur_slope_info);
    float cur_slope = cur_slope_info.angle;
    LOG_D("cur_slope=%f", cur_slope);
    // 后退
    start_time = rt_tick_get_millisecond();
    cur_time = start_time;
    while (cur_time - start_time < back_time)
    {
        cur_yaw = get_current_yaw();
        // LOG_D("cur_yaw=%f", cur_yaw);
        move_backward_with_pid(test_param->forward_mtr_speed, cur_yaw, start_yaw);
        rt_thread_mdelay(100);
        get_current_slope_info(&cur_slope_info);
        cur_slope = cur_slope_info.angle;
        
        if (cur_slope > SLOPE_THRES_MAX)
        {
            LOG_D("wall, cur_slope=%f. break", cur_slope);
            break;
        } 
        cur_time = rt_tick_get_millisecond();
    }
    move_stop_time(1000);
    return 0;
}

int stright_to_deep_area(float start_yaw, float delta_slope)  // delta_slope =5.0f
{
    // 根据坡yaw重置start yaw
    float yaw = get_current_yaw();
    LOG_D("cur yaw=%f", yaw);
    struct slope_type cur_slope_info;
    get_current_slope_info(&cur_slope_info);
    float delta = compare_yaws(cur_slope_info.yaw, 90.0f);
    LOG_D("slope_yaw=%f, delta=%f", cur_slope_info.yaw, delta);
    start_yaw = calculate_yaw(yaw, delta);
    LOG_D("reset start_yaw=%f", start_yaw);
    wash_floor_info_start_yaw_set(&wash_floor_task_info, start_yaw);

    LOG_I("stright to deep area.");
    float start_slope, cur_slope;
    float cur_yaw;
    
    // 先后退到坡度有delta_slope变化
    start_slope = cur_slope_info.angle;
    LOG_D("start_slope=%f", start_slope);

    // slope_angle = start_slope;
    // LOG_D("slope_angle=%f", slope_angle);

    // // 水深默认2.5m，算出梯形形状的缓坡梯形高h
    // area_h = 2.5f/(sin(M_PI*slope_angle/180.0f));
    // LOG_D("area_h=%f", area_h);

    cur_yaw = get_current_yaw();
    LOG_D("cur_yaw=%f", cur_yaw);

    if (fabs(cur_yaw - start_yaw) >= 3.0f)
    {
        LOG_D("rotate");
        move_rotate_on_floor_using_target(start_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
    }
    rt_thread_mdelay(1000);

    get_current_slope_info(&cur_slope_info);
    cur_slope = cur_slope_info.angle;
    LOG_D("cur_yaw=%f, cur_slope=%f", get_current_yaw(), cur_slope);

    // 加上超时时间
    rt_tick_t start_time, cur_time, end_time;
    start_time = rt_tick_get_millisecond();
    while (cur_slope - start_slope < delta_slope)
    {
        cur_yaw = get_current_yaw();
        // LOG_D("cur_yaw=%f", cur_yaw);
        move_backward_with_pid(test_param->forward_mtr_speed, cur_yaw, start_yaw);
        rt_thread_mdelay(100);
        get_current_slope_info(&cur_slope_info);
        cur_slope = cur_slope_info.angle;
        // LOG_D("back_ward, cur_slope=%f", cur_slope);
        if (cur_slope > SLOPE_THRES_MAX)
        {
            LOG_D("wall, break");
            break;
        } 
        cur_time = rt_tick_get_millisecond();
        if (cur_time - start_time > MOVE_FORWARD_TIMEOUT)
        {
            LOG_D("back over time, break");
            break;
        } 
    }
    move_stop_time(1000);
    move_forward_with_speed_and_time(test_param->forward_mtr_speed, 2*1000);
    move_stop_time(1000);

    // 纠正角度
    cur_yaw = get_current_yaw();
    LOG_D("cur_yaw=%f", cur_yaw);
    if (fabs(cur_yaw - start_yaw) >= 3.0f)
    {
        LOG_D("rotate");
        move_rotate_on_floor_using_target(start_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
    }
    rt_thread_mdelay(1000);

    // 前进到坡度有5度变化。记录时间
    get_current_slope_info(&cur_slope_info);
    cur_slope = cur_slope_info.angle;
    start_time = rt_tick_get_millisecond();
    LOG_D("start time=%d", start_time);
    while (cur_slope - start_slope < delta_slope)
    {
        cur_yaw = get_current_yaw();
        // LOG_D("cur_yaw=%f", cur_yaw);
        move_forward_with_pid(test_param->forward_mtr_speed, cur_yaw, start_yaw);
        rt_thread_mdelay(100);
        get_current_slope_info(&cur_slope_info);
        cur_slope = cur_slope_info.angle;
        // LOG_D("cur_slope=%f", cur_slope);
        if (cur_slope > SLOPE_THRES_MAX)
        {
            LOG_D("wall, break");
            break;
        } 
        cur_time = rt_tick_get_millisecond();
        if (cur_time - start_time > MOVE_FORWARD_TIMEOUT)
        {
            LOG_D("forward over time, break");
            break;
        }
    }
    end_time = rt_tick_get_millisecond();
    LOG_D("end time=%d", end_time);
    move_stop_time(1000);

    move_backward_with_speed_and_time(test_param->forward_mtr_speed, 2*1000);
    move_stop_time(1000);
    
    // 纠正角度
    cur_yaw = get_current_yaw();
    LOG_D("cur_yaw=%f", cur_yaw);
    if (fabs(cur_yaw - start_yaw) >= 3.0f)
    {
        LOG_D("rotate");
        move_rotate_on_floor_using_target(start_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
    }
    rt_thread_mdelay(1000);

    rt_tick_t forward_time = end_time - start_time;
    // 梯形长边长度
    area_c = forward_time * MOVE_SPEED/1000;   // 转为长度（m）
    LOG_D("area_c=%fm", area_c);

    rt_tick_t back_time = forward_time/2;
    LOG_D("back_time=%d", back_time);

    // 退回到中间位置
    _back_with_time(back_time, start_yaw);

    // 右转90度，走向深水区
    float target_yaw = calculate_yaw(start_yaw, -90.0f);
    LOG_D("target_yaw=%f", target_yaw);
    move_rotate_on_floor_using_target(target_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
    move_stop_time(1000);

    LOG_D("stright");
    // // 重启imu
    // // target_yaw = reset_imu(15*1000, 1000);
    // // reset start_yaw
    // start_yaw = calculate_yaw(target_yaw, 90.0f);
    
    get_current_slope_info(&cur_slope_info);
    cur_slope = cur_slope_info.angle;
    LOG_D("slope=%f, forward 3s", cur_slope);
    // 如果机器当前在平面上，先前进3s。（为了走出浅水区平面）
    if (fabs(cur_slope) < 5.0f)
    {
        LOG_D("forward 3s");
        move_forward_with_pid_and_time(test_param->forward_mtr_speed, target_yaw, 3*1000);
    }
    get_current_slope_info(&cur_slope_info);
    cur_slope = cur_slope_info.angle;
    LOG_D("cur_slope=%f", cur_slope);

    // 走向深水区
    start_time = rt_tick_get_millisecond();
    while ((fabs(cur_slope - start_slope) < delta_slope) && (fabs(cur_slope) > 5.0f))
    {
        cur_yaw = get_current_yaw();
        get_current_slope_info(&cur_slope_info);
        cur_slope = cur_slope_info.angle;
        if (cur_slope > SLOPE_THRES_MAX)
        {
            LOG_D("cur_slope=%f, get wall", cur_slope);
            break;
        }
        move_forward_with_pid(test_param->forward_mtr_speed, cur_yaw, target_yaw);
        rt_thread_mdelay(100);
        cur_time = rt_tick_get_millisecond();
        if (cur_time - start_time > MOVE_FORWARD_TIMEOUT)
        {
            LOG_D("over time, break");
            break;
        }
    }
    area_h1 = MOVE_SPEED * (rt_tick_get_millisecond() - start_time)/1000;
    // 梯形高度， 0.4是找到坡后向深水区行走的距离
    area_h = area_h1 + 0.4f; 
    LOG_D("area_h=%fm", area_h);
    move_stop_time(1000);

    cur_time = rt_tick_get_millisecond();
    start_time = cur_time;
    while ((cur_time - start_time) < 3*1000)
    {
        move_backward_with_pid(test_param->forward_mtr_speed, cur_yaw, target_yaw);
        get_current_slope_info(&cur_slope_info);
        cur_slope = cur_slope_info.angle;
        // LOG_D("cur_slope=%f", cur_slope);
        if (fabs(cur_slope - start_slope) < 3.0f)
        {
            LOG_D("cur_slope=%f", cur_slope);
            break;
        }
        rt_thread_mdelay(100);
        cur_time = rt_tick_get_millisecond();
    }
    move_stop_time(500);

    move_rotate_on_floor_using_target(start_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
    rt_thread_mdelay(1000);
    LOG_D("cur_yaw=%f", get_current_yaw());
    return 0;
}


int update_record_clean_info(int* row_id, struct wash_row_info* one_row_info, wash_record_info_t area_info)
{
    // 更新区域清洗结束的时间
    int if_change_area = 0;
    int topography = 0;   // 0:flat;1:slope
    int cur_region_flag = area_info->region_flag;
    LOG_D("update record info, area_flag=%d", cur_region_flag);

    area_info->info_lists[*row_id%EF_RCD_LIST_LEN] = *one_row_info;
    *row_id += 1;
    LOG_D("after copy cur_row, row_num=%d", *row_id);

    if (wash_floor_task_info.wash_times%2==1 && slope_label == 1)
    {
        // 当前地势
        if (forward_action_ret_info.info.flat_count > COUNT_THRES)
        {
            topography = 0;
        }else
        {
            topography = 1;
        }
        LOG_D("topography=%d", topography);
        // 判断是第几圈，是否已经经过了斜坡，上一行的区域
        if (topography != pre_topography && if_cross_slope == 0)
        {
            if_change_area = 1;
            LOG_D("change area");
        }
    }
    
    // 如果需要更新area
    if ((*row_id % EF_RCD_LIST_LEN == 0) || (if_change_area == 1))
    {
        if (if_change_area == 1)
        {
            // 更新area结构体
            if (pre_topography == 0 && topography == 1)
            {
                cur_region_flag = REGION_FLOOR_GENTLE;
            } else if (pre_topography == 1 && topography == 0)
            {
                cur_region_flag = REGION_FLOOR_SHALLOW;
                if_cross_slope = 1;
            }
            LOG_D("change arae, write info. cur_flag=%d", cur_region_flag);
            // 上一区域已经记录完成
            area_info->total_rows_num = *row_id;
            area_info->end_time = rt_tick_get_millisecond();
            get_battery_soc(&area_info->end_soc);
            
            ef_rcd_write(area_info);
            // 清空行数组
            memset(area_info->info_lists, 0, EF_RCD_LIST_LEN * sizeof(struct wash_row_info));
            
            // 更新新的area信息
            area_info->start_time = rt_tick_get_millisecond();
            area_info->region_flag = cur_region_flag;
            area_info->wash_num = eeprom_total_clean_counts_get();
            area_info->wash_time = wash_floor_task_info.wash_times;
            area_info->total_rows_num = 0;
            area_info->end_time = 0;
            get_battery_soc(&area_info->start_soc);
            *row_id = 1;
        } else  // 40行存满了，需要再开辟一个，继续存
        {
            // area_info->total_rows_num = *row_id;
            // area_info->end_time = rt_tick_get_millisecond();
            LOG_D("full, write info");
            ef_rcd_write(area_info);
            // 清空行数组
            memset(area_info->info_lists, 0, EF_RCD_LIST_LEN * sizeof(struct wash_row_info));
        }
    }

    pre_topography = topography;

    return 0;
}

void rotate_yaw_init(float target_yaw, uint8_t max_rotate_count)
{
    while (max_rotate_count > 0)
    {
        float cur_yaw = get_current_yaw();
        LOG_D("new line rotate, cur_yaw=%f", cur_yaw);
        if (fabs(cur_yaw - target_yaw) >= 3.0f)
        {
            LOG_D("in rotate, cur yaw=%f", cur_yaw);
            move_rotate_on_floor_using_target(target_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
            rt_thread_mdelay(500);
            LOG_D("in rotate end, cur yaw=%f, rotate_count=%d", get_current_yaw(), max_rotate_count);
        } else
        {
            LOG_D("get target yaw, break");
            break;
        }
        max_rotate_count -= 1; 
    }
}

void low_power_detect(uint16_t off_vol)
{
    float cur_yaw, target_yaw;

    get_battery_voltage(&vol);
    LOG_D("current voltage: %dmV", vol);

    if (vol < off_vol)
    {
        LOG_I("low power, turn off. slope_label=%d", slope_label);
        cur_yaw = get_current_yaw();
        target_yaw = calculate_yaw(cur_yaw, 180.0f);
        LOG_D("cur_yaw=%f", cur_yaw);
        move_rotate_large_angle(target_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
        rt_thread_mdelay(1*1000);
        actions_on_floor_goto_shallow_turn_off(&slope_label, NULL);
    }
}

int action_on_floor(uint16_t cmd,void *p_arg_in,void *p_arg_out)
{
    p2a_msg.cmd = cmd;
    p2a_msg.p_arg_in = p_arg_in;
    p2a_msg.p_arg_out = p_arg_out;
    
    wash_floor_send_msg_to_action(&p2a_msg);
    msg_size = wash_floor_recv_msg_from_action(&action_msg_handle, RT_WAITING_FOREVER);
    if (msg_size != 0) // 没有通信成功
    {
        LOG_E("recv msg ERR");
        // 亮灯效
    }
    
    return action_msg_handle->msg;
}

extern int if_uturn_slips;
int main()
{
    LOG_I("start wf");
    int ret;
    int odd_ret, even_ret;                
    pre_y = start_point->y;
    float start_x = start_point->x;

    rt_bool_t if_near_c_edge = 0;
    RT_UNUSED(if_near_c_edge);
    // RT_UNUSED(get_small_dis);
    struct move_info pid_info;
    rt_tick_t start_tick_time; 
    move_tick_time = 0;


    LOG_I("cur_time_start_yaw=%f, wash_times=%d", wf.start_yaw, wf.wash_times);

    reset_malloc();                           // malloc重置为0。索引归0
    int point_count = 0; 
    
    cur_time_start_yaw = wf.start_yaw;

    cur_line_start_yaw = cur_time_start_yaw;
    
    float cur_row_move_distance, cur_row_length = 0.0f; 
    int cur_dis_from_edge; 
    struct move_forward_cs move_forward_cs_info;
    
    float delta, cur_yaw, target_yaw;
    int add_size, sign, move_to_next_row_time = 0;

    int if_edge = 0, if_c_edge = 0, if_c_all_edge = 0, if_half_get_c_edge = 0;
    rt_tick_t move_time = 0;

    int move_back_time = 0;
    rt_tick_t sensor_tick, cur_tick;
    rt_uint8_t label;
    rt_uint16_t dis;
    float cur_pitch;
    rt_tick_t odd_pre_move_time = 0, even_pre_move_time = 0;

    int if_odd_a_shrt_dis = 0;
    int if_even_a_shrt_dis = 0;
    int if_odd_line = -1; 

    int ret_msg;

    int half_washed_line_num = -1;
    float cur_roll;
    reset_once_clean_info();
    reset_cur_line_to_next_line_info();
    int if_reduce_forward_time = 0, if_cur_time_reduce = 0;
    int short_line_count = 0;    
    int sf = 0, cf = 0;      
    int if_vison = 0;


    rt_tick_t start_time, cur_time;
    start_time = rt_tick_get();
    LOG_I("start clean clk = %d", start_time);
    rt_uint32_t pre_wp_sp;
    if_uturn_slips_pre = 0;

    float current_yaw;
    const int DIRTY_INTERVIAL_TIME = 10*1000;
    if (wf.wash_times%2==0)
    {
        sum_vision_time = 0;
    }

    struct wash_record_info floor_record_info = {0};
    init_wash_record_info(&floor_record_info, &wf, slope_label);
    int row_id = 1;
    pre_topography = 0;
    if_cross_slope = 0;
	struct wash_row_info one_row_info;
    pre_time = 270000;
    

W_L: 
    LOG_I("start , time=%d", wf.wash_times);
    cur_yaw = get_current_yaw();
    LOG_D("cur_yaw=%f", cur_line_start_yaw);
    LOG_D("cur_scurrent_yaw=%f", cur_yaw);

    rotate_yaw_init(cur_line_start_yaw, 2);

    LOG_I("cur_lyaw=%f", get_current_yaw());
    reset_cur_line_to_next_line_info();
    if_reduce_forward_time = 0, if_cur_time_reduce = 0, slope_forward_time = 0, cur_line_forawrd_time = 0;
    if_curline_get_drain = 0;
    wf_cur_line_edge = INIT_VALUE;

    if_odd_line = wash_floor_task_info.total_moved_row%2; 
    LOG_D("if_odd_line=%d, wash_floor_task_info.total_moved_row=%d", if_odd_line, wash_floor_task_info.total_moved_row);

    low_power_detect(clean_info.off_value);

    pre_wp_sp = move_wp_speed_on_floor_get();
    LOG_D("before update, %d", pre_wp_sp);
    // restrict_value_range(&wf_conf_para);
    LOG_D("set pump, %d", test_param->forward_wp_speed);
    move_wp_speed_on_floor_set(test_param->forward_wp_speed);
    if (abs(test_param->forward_wp_speed - pre_wp_sp) > 40)
    {
        LOG_D("set water pump=%d", test_param->forward_wp_speed);
        // rt_thread_mdelay(3*1000);
        rt_thread_mdelay(1*1000);
    } else
    {
        rt_thread_mdelay(500);
    }
    

    if ((from_judge_moved_lines_num != -1)&& (remained_lines_num == -1))
    {
        from_judge_moved_lines_num += 1;
        if (from_judge_moved_lines_num > (ADJUST_C_COUNT-1))
        {
            from_judge_moved_lines_num = -1;
        }
    }
    LOG_D("from_judge_moved_lines_num=%d, if_c_all_edge=%d, if_c_edge=%d", from_judge_moved_lines_num, if_c_all_edge, if_c_edge);

    if (half_washed_line_num != -1)
    {
        half_washed_line_num += 1;
    }
    LOG_D("half_washed_line_num=%d", half_washed_line_num);


    if ((if_c_all_edge==1) && (from_judge_moved_lines_num==-1)) 
    {
        move_time = (int)move_tick_time/2;
        LOG_D("half line judge c, move_time=%d, mv_speed=%d", move_time, test_param->forward_mtr_speed);
        
        pid_info.start_yaw = get_current_yaw();
        pid_info.move_time = move_time;
        pid_info.move_speed = test_param->forward_mtr_speed;
        LOG_D("motor speed=%d", pid_info.move_speed);
        LOG_I("half line judge c, CMD_ACTION_FLOOR_MOVE_FORWARD_PID");

        action_on_floor(CMD_ACTION_FLOOR_MOVE_FORWARD_PID, &pid_info, NULL);

        if (if_odd_line==1)
        {
            delta = 90.0f;
        } else {
            delta = -90.0f;
        }
        move_rotate_on_floor(delta, MOVE_CONTROL_RIGHT_SAFEZONE);
        rt_thread_mdelay(500);

        half_judge_c_edge(&remained_lines_num);
        LOG_I("remained_lines_num=%d", remained_lines_num);

        from_judge_moved_lines_num = 0;
        rt_thread_mdelay(500);
    
        if (remained_lines_num != -1)
        {
            half_washed_line_num = 0;
            if (remained_lines_num < 1)
            {
                if_half_get_c_edge = 1;
            }
        } 
        if (remained_lines_num == -1)
        {
            half_washed_line_num = -1;
            if_half_get_c_edge = 0;
        }
               
        move_rotate_on_floor_using_target(cur_line_start_yaw, MOVE_CONTROL_RIGHT_SAFEZONE); 
        rt_thread_mdelay(500);    
    } else
    {
        move_time = 0;
    }
    LOG_D("remained_lines_num=%d, half_washed_line_num=%d", remained_lines_num, half_washed_line_num);
    
    p2a_msg.cmd = CMD_ACTION_FLOOR_MOVE_FORWARD_AND_GET_DIS;
    forward_action_ret_info.dirty_info = &one_row_info;
    p2a_msg.p_arg_out = &forward_action_ret_info;
    forward_action_info.if_change_speed = 1;
    forward_action_info.params = test_param;
    forward_action_info.last_line_forward_time = pre_forward_time;
    LOG_I("CMD_ACTION_FLOOR_MOVE_FORWARD_AND_GET_DIS, if_change_speed=%d", forward_action_info.if_change_speed);
    wf_ctr_light(1);
    rt_thread_mdelay(50);
    forward_action_info.if_odd_row = if_odd_line;
    forward_action_info.find_slope_time = slope_label;
    LOG_D("forward_action_info.if_odd_row=%d, slope_label=%d", forward_action_info.if_odd_row, slope_label);

    if (if_odd_line == 1)
    {
        if (if_odd_a_shrt_dis == 1)
        {
            forward_action_info.time_out = (int)odd_pre_move_time * 0.7;
            if_cur_time_reduce = 1;
        } else
        {
            forward_action_info.time_out = MOVE_FORWARD_TIMEOUT;
        }
    } else
    {
        if (if_even_a_shrt_dis == 1)
        {
            forward_action_info.time_out = (int)even_pre_move_time * 0.7;
            if_cur_time_reduce = 1;
        } else
        {
            forward_action_info.time_out = MOVE_FORWARD_TIMEOUT;
        }
    }
    LOG_D("if_odd_a_shrt_dis=%d, if_even_a_shrt_dis=%d, forward_action_info.time_out=%d", if_odd_a_shrt_dis, if_even_a_shrt_dis, forward_action_info.time_out);
    forward_action_info.start_yaw = cur_line_start_yaw;
    p2a_msg.p_arg_in = &forward_action_info;
    
    wash_floor_send_msg_to_action(&p2a_msg);    
    start_tick_time = rt_tick_get();
    LOG_D("start_tick_time=%d", start_tick_time);
    msg_size = wash_floor_recv_msg_from_action(&action_msg_handle, RT_WAITING_FOREVER);

    if ((rt_tick_get() - forward_action_ret_info.act_get_dirty_time) < DIRTY_INTERVIAL_TIME)
    {
        if_current_edge_dirty = 0;
        LOG_D("current line edge get dirty.");
    }
    LOG_D("if_current_edge_dirty=%d", if_current_edge_dirty);
    
    LOG_D("end_tick %d, action_msg_handle.msg=%d", rt_tick_get(), action_msg_handle->msg);
    LOG_I("ret time=%d, dis=%d, end method=%d, have_leaves=%d", \
        forward_action_ret_info.forward_time, forward_action_ret_info.end_dis, 
        forward_action_ret_info.ret_idx, forward_action_ret_info.if_right_area_have_leaves);
    sum_vision_time += forward_action_ret_info.vision_clean_time;
    LOG_D("vision_time=%d, sum_vision_time=%d", forward_action_ret_info.vision_clean_time, sum_vision_time);

    if (slope_label == 1)
    {
        if (wash_floor_task_info.wash_times==0 && wash_floor_task_info.total_moved_row==1)
        {
            area_a = MOVE_SPEED * forward_action_ret_info.forward_time / 1000;
            LOG_D("area_a=%fm", area_a);
            pool_area = calculate_pool_area(area_a, area_c, area_h);
            LOG_D("pool_area=%fm2", pool_area);
            save_pool_size_type_by_area(pool_area);
            LOG_D("saved pool area.");
        }
    }
        
    
    update_wf_interval();

    move_tick_time = forward_action_ret_info.forward_time + move_time;
    LOG_I("start_tick_time=%d, move_tick_time=%d", start_tick_time, move_tick_time);
    LOG_D("forward_action_ret_info.line_state=%d", forward_action_ret_info.line_state);
    one_row_info.length = (int)forward_action_ret_info.forward_time * MOVE_SPEED;
    one_row_info.row_id = row_id;
    LOG_D("cur line length=%f, id=%d", one_row_info.length, row_id);

    if (msg_size != 0)
    {
        LOG_E("recv msg ERR");
        move_stop();
        return -1;
    }

    ret = find_slope_during_washing();
    if (ret == 1) 
    {
        if_odd_a_shrt_dis = 0;
        if_even_a_shrt_dis = 0;
        LOG_I("reset current time wash. wash_floor_task_info.wash_times=%d", wash_floor_task_info.wash_times);
        // 走到坡中间
        LOG_D("go to deep area");
        stright_to_deep_area(cur_line_start_yaw, 5.0f);
        LOG_D("go to deep area end");
        goto WASH_LINE;
    }
    // yaw纠不过来停止
    if (forward_action_ret_info.ret_idx == 4)
    {
        // 过地漏处理
        ret = dislodged_from_drain(if_odd_line, cur_line_start_yaw);
        LOG_I("dislodged_from_drain %d", ret);
        if (ret > 0)
        {
            LOG_I("cur line get drain");
            if_curline_get_drain = ret;
            LOG_D("if_curline_get_drain=%d", if_curline_get_drain);
        }
    }

    // 关灯
    LOG_D("turn off light");
    wf_ctr_light(0);
    rt_thread_mdelay(50); 

    // 当前行没检测到地漏，才洗坡
    if (if_curline_get_drain == 0)
    {
        int need_close_dis_sensor_time = pre_forward_time*2/3 - move_tick_time;
        LOG_D("need_close_dis_sensor_time=%d", need_close_dis_sensor_time);
        wf_cur_line_edge =  move_to_edge(&forward_action_ret_info, cur_line_start_yaw, &slope_forward_time, need_close_dis_sensor_time);
    }
    LOG_D("wf_cur_line_edge=%d, slope_forward_time=%d", wf_cur_line_edge, slope_forward_time);
    cur_line_forawrd_time = slope_forward_time + move_tick_time; // 当前行前行总时间
    update_short_line_info(cur_line_forawrd_time, &short_line_count);
    LOG_I("cur_line_forawrd_time=%d, short_line_count=%d", cur_line_forawrd_time, short_line_count);
    LOG_D("current_slope=%f", get_current_slope());
    pre_forward_time = cur_line_forawrd_time;

    if (forward_action_ret_info.ret_idx == 3) // action靠roll停止，则认为已检测到c边
    {
        cur_line_to_next_line_info.cur_line_roll_get_c = 1;
    }
    
    cur_roll = get_current_roll();
    if (fabs(cur_roll) > SLOPE_THRES_MAX)
    {
        cur_line_to_next_line_info.cur_line_roll_get_c = 1;
    }
    LOG_D("cur_roll=%f, cur_line_roll_get_c=%d", cur_roll, cur_line_to_next_line_info.cur_line_roll_get_c);

    if (if_odd_line == 1) // 奇数行
    {
        odd_pre_move_time = move_tick_time;
        if (fabs(cur_roll) > 75.0f) // 认为上墙
        {
            LOG_I("odd, on wall");
            // 墙上左转90度
            move_rotate_on_wall(90.0f, MOVE_CONTROL_RIGHT_SAFEZONE);
            LOG_D("turn left, roll=%f, pitch=%f, yaw=%f", \
                get_current_roll(), get_current_pitch(), get_current_yaw());
            // 后退下墙
            back_down_wall();
            LOG_D("back down wall, pitch=%f", get_current_pitch());
        }
        
    } else
    {
        even_pre_move_time = move_tick_time;
        if (fabs(cur_roll) > 75.0f) // 认为上墙
        {
            LOG_I("even, on wall");
            // 墙上右转90度
            move_rotate_on_wall(-90.0f, MOVE_CONTROL_RIGHT_SAFEZONE);
            LOG_D("turn right, roll=%f, pitch=%f, yaw=%f", \
                get_current_roll(), get_current_pitch(), get_current_yaw());
            // 后退下墙
            back_down_wall();
            LOG_D("back down wall, pitch=%f", get_current_pitch());
        }
    }
    // 更新pre奇/偶行行走时间 
    LOG_D("odd_pre_move_time=%d, even_pre_move_time=%d", odd_pre_move_time, even_pre_move_time);

    // 校正yaw角
    if (fabs(get_current_yaw() - cur_line_start_yaw) >= 3.0f)
    {
        move_rotate_on_floor_using_target(cur_line_start_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
        rt_thread_mdelay(500);
    }
    // 获取当前离墙边的距离
    LOG_D("cur_dis_from_edge %d", forward_action_ret_info.end_dis);
    cur_row_move_distance = move_tick_time * MOTOR_SPEED_MOVE_SPEED / RT_TICK_PER_SECOND;         // 转为秒（s）的单位
    LOG_D("cur_row_move_distance %d", cur_row_move_distance);

    // 当前行长度
    cur_row_length = cur_row_move_distance + (float)forward_action_ret_info.end_dis + 400; // 400为机身长度（mm）
    LOG_D("cur_row_length %f", cur_row_length);

    // 计算当前行顶点坐标
    if (wash_times%2 == 0) // 正向清洗
    {
        if (if_odd_line==1) // 奇数行
        {
            edge_point.y = pre_y - cur_row_length;
        } else
        {
            edge_point.y = pre_y + cur_row_length;
        }
        edge_point.x = start_x + MOTOR_SPEED_MOVE_SPEED * wash_floor_task_info.total_x_move_time/1000.0f;
    } else { // 反向清洗
        if (if_odd_line==0) // 偶数行
        {
            edge_point.y = pre_y - cur_row_length;
        } else // 奇数行
        {
            edge_point.y = pre_y + cur_row_length;
        }
        edge_point.x = start_x - MOTOR_SPEED_MOVE_SPEED * wash_floor_task_info.total_x_move_time/1000.0f;
    }
    point_count += 1;
    edge_point_count += 1;

    // 更新pre_y
    pre_y = edge_point.y;
    
    // get_current_dis(DIS_SENSOR_MODE, &cur_dis_from_edge);
    // // 获取融合距离
    cur_tick = rt_tick_get_millisecond();
    get_distance_fusion(&sensor_tick, &label, &dis);
    LOG_D("src dis=%d, cur_time=%d, sensor_time=%d", dis, cur_tick, sensor_tick);
    if (label == 1 && (abs(cur_tick - sensor_tick) < 4000)) // 数据可信且新
    {
        cur_dis_from_edge = dis;
    }
    
    // 更新清洗的buffer信息
    // if (robot_work_mode == CLEAN_FIRST)
    if (0)
    {
        update_clean_info(wash_floor_task_info.total_moved_row, &move_forward_cs_info);
    }
    

    //// 判断切向哪一行,只有clean first模式才跳行 ////
    int cnt;
    // if (robot_work_mode == CLEAN_FIRST)
    if (0)
    {
        clean_first_get_the_newline_cnt(&cnt);
    } else
    {
        cnt = 1;
        cur_idx += 1;
        idx += 1;
    }    
    LOG_D("robot_work_mode=%d, cnt=%d", robot_work_mode, cnt);

    add_size =  (ADGEST_ROWCOUNT < 5) ? 5 : ADGEST_ROWCOUNT;
    if (idx > (line_info_size - add_size)) // malloc的空间不够用了
    {
        ret = add_malloc();
        if (ret != 0)
        {
            LOG_E("add malloc error");
            return -1;
        }
        LOG_D("malloc success, size=%d, idx=%d", line_info_size, idx);
    }
    //////////判断切向哪一行结束////////////
    

    /////////切行的第一次旋转////////
    LOG_D("before meature rotate, slope=%f", get_current_slope());
    struct point_info point;

    // 当前行没有检测到地漏
    if (if_curline_get_drain == 0 && (wash_floor_task_info.wash_times > 1) && (close_clean_edge == 0))
    {
        // 边缘拟合+清洗边缘脏污
        wf_actions_on_clean_edge(&point);
    } else {
        // 只做边缘拟合
        LOG_D("get_next_row_point_no_stop(&point)");
        ret = get_next_row_point_no_stop(&point, cur_line_start_yaw);
        LOG_D("get_next_row_point ret=%d", ret);
        rt_thread_mdelay(500);
        float cur_yaw = get_current_yaw();
        LOG_D("cur_yaw = %f", cur_yaw);

        if_clean_edge = 0;  // 第一个circle关闭边缘清洁
    }
    
    if (if_odd_line == 1) // 奇数行
    {
        odd_ret = ret; // 成功是0， 否则为-1
    } else  // 偶数行
    {
        even_ret = ret;
    }
    
    sign = (cnt > 0) ? 1:-1;

    // 如果建图已经转过，则此次行为为纠偏作用
    target_yaw = 0.0f;
    if (if_odd_line==1) // 奇数行换行
    {
        target_yaw = calculate_yaw(cur_line_start_yaw, 90.0f*sign); 
    } else // 偶数行换行
    {
        target_yaw = calculate_yaw(cur_line_start_yaw, -90.0f*sign);
    }
    LOG_D("target_yaw=%f", target_yaw);
    move_rotate_on_floor_using_target(target_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
    rt_thread_mdelay(500);
    LOG_D("after meature rotate, slope=%f", get_current_slope());
    
    /////////完成切行的第一次旋转////////
    // 判断是否卡地漏，并脱困
    judge_and_escape_drain(target_yaw);

    // 如果到了第三边，退出
    if ((remained_lines_num == half_washed_line_num) && (half_washed_line_num != -1))
    {
        if_half_get_c_edge = 1;
    }
    
    if (if_c_all_edge == 1 && if_half_get_c_edge == 1)
    {
        LOG_I("arrive c edge, current time end");
        goto END;
    }
    // if ((deep_area_state == GROUND_GET_SLOPE) && (if_find_deep_area == 0))
    // {
    //     LOG_D("get deep area, current time end");
    //     // if_find_deep_area = 1;
    //     goto END;
    // }
    LOG_D("area_state=%d, if_find_deep_area=%d", deep_area_state, if_find_deep_area);
    if (short_line_count > SHORT_LINE_COUNT_THRES)
    {
        LOG_I("arrive c, judge from short dis conut.short_line_count=%d", short_line_count);
        goto END;
    }
    cur_clk_time = rt_tick_get();
    LOG_D("cur_clk = %d", cur_clk_time);
    if (cur_clk_time - start_clean_time > clean_info.once_clean_max_clean_time)
    {
        // 超时，退出本次清洗
        LOG_I("clean time over. current time end");
        goto END;
    }
    if (wash_floor_task_info.total_moved_row > (clean_info.max_clean_rows-2))
    {
        // 达到最大
        LOG_I("cleaned rows %d, current time end", wash_floor_task_info.total_moved_row);
        goto END;
    }

    // 绕地漏
    move_to_next_line_avoid_drain_(if_odd_line, test_param->forward_mtr_speed);
    // 切行直行前探测第三边距离
    // get_current_dis(DIS_SENSOR_MODE, &a_dis);
    // 用融合的距离
    cur_tick = rt_tick_get_millisecond();
    get_distance_fusion(&sensor_tick, &label, &dis);
    if (label == 1 && (abs(cur_tick - sensor_tick) < 4000)) // 数据可信且新
    {
        a_dis = dis;
    } else {
        a_dis = -1;
    }
    cur_line_to_next_line_info.a_dis = a_dis;
    LOG_I("a_dis=%d", a_dis);

    if (wash_times%2 == 0) // 正向清洗
    {
        edge_point.x = start_x+ MOTOR_SPEED_MOVE_SPEED * (wash_floor_task_info.total_x_move_time + GAP_BETWEEN_ROWS)/1000.0f;
        if (if_odd_line == 0) // 如果在偶数行
        {
            if (dis_pt_info_num > 2)
            {
                edge_point.y = pre_y - (cur_dis_from_edge + 200) + point.y;
            } else {
                edge_point.y = pre_y;
            }
        } else // 如果在奇数行
        {
            if (dis_pt_info_num > 2) 
            {
                edge_point.y = pre_y + (cur_dis_from_edge + 200) + point.y;
            } else {
                edge_point.y = pre_y;
            }
        }
        
    } else // 反向清洗
    {
        edge_point.x = start_x - MOTOR_SPEED_MOVE_SPEED * (wash_floor_task_info.total_x_move_time + GAP_BETWEEN_ROWS)/1000.0f;
        if (if_odd_line == 0) // 如果在偶数行
        {
            if (dis_pt_info_num > 2)
            {
                edge_point.y = pre_y + (cur_dis_from_edge + 200) + point.y;
            } else {
                edge_point.y = pre_y;
            }
        } else // 如果在奇数行
        {
            if (dis_pt_info_num > 2) 
            {
                edge_point.y = pre_y - (cur_dis_from_edge + 200) + point.y;
            } else {
                edge_point.y = pre_y;
            }
        }
    }
    point_count += 1;
    edge_point_count += 1;
    // 存入点

    // 更新pre_y
    rt_thread_mdelay(500*1);
    pre_y = edge_point.y;
    
    // 做地磁矫正
    if ((a_dis < 0 || a_dis > CEDGE_THRES) && (GERO_ADGEST_YAW == 1))
    {
        if ((if_odd_line==0)&&(cnt>0)) // 偶数行&&cnt>0
        {
            // wash_floor_adjust_yaw_use_gero(&cur_line_start_yaw);
            wash_floor_adjust_yaw_use_geo_dynamic(&cur_line_start_yaw);
        }
    }

    // 換行直行cnt格
    pid_info.move_speed = WASH_FLOOR_MOTOR_SPEED;
    pid_info.move_time = move_time_to_next_line();

    if (if_curline_get_drain==1)
    {
        pid_info.move_time = 1000;
        LOG_D("drain, same direction, forward %d", pid_info.move_time);
    } else if (if_curline_get_drain==2)
    {
        pid_info.move_time *= 2;
        LOG_D("drain, diff direction, forward %d", pid_info.move_time);
    } else if (if_uturn_slips_pre == 1) // 上一行机器换行时滑落
    {
        pid_info.move_time = 5*1000;
        LOG_D("last line slips,forward %d", pid_info.move_time);
    } else
    {
        LOG_D("no drain");
        // 如果是smart mode第一遍洗，跳两行
        pid_info.move_time = (int)pid_info.move_time*line_wa_interval;
        LOG_D("across lines. mode=%d, pid_info.move_time=%d", robot_work_mode, pid_info.move_time);
        LOG_D("if_cur_time_reduce=%d", if_cur_time_reduce);

        if (if_cur_time_reduce == 1)
        {
            pid_info.move_time = GAP_BETWEEN_ROWS*abs(cnt)*2;
        }
    }
    if (if_clean_edge==1)
    {
        pid_info.move_time += 2000; // 有边缘脏污清洁动作时，换行时间增加2s。防止清洁动作多次旋转不同轴，机器回到原路径
    }
    
    // 如果已经探测到边，换行不前进
    LOG_D("if_odd_a_shrt_dis=%d, if_even_a_shrt_dis=%d", if_odd_a_shrt_dis, if_even_a_shrt_dis);
    if (cur_line_to_next_line_info.cur_line_roll_get_c == 1 || cur_pitch < -SLOPE_THRES_MAX)
    {
        pid_info.move_time = 0;
    }
    
    if (a_dis < (CEDGE_THRES-200) && a_dis > 0)
    {
        pid_info.move_time = 1000;
        cnt = 1;
    } 
    one_row_info.width = (int)pid_info.move_time * MOVE_SPEED;
    LOG_D("cur_row width=%d", one_row_info.width);

    wash_floor_task_info.total_x_move_time += pid_info.move_time;  // ms
    LOG_I("move time=%d, total_x_time=%d", pid_info.move_time, wash_floor_task_info.total_x_move_time);

    move_forward_with_speed_and_time(pid_info.move_speed, pid_info.move_time);
    rt_thread_mdelay(1000); 

    // 更新当前行的清洁信息
    update_record_clean_info(&row_id, &one_row_info, &floor_record_info);

    // 寻找深水区
    find_deep_area();

    // 再次探测第三边
    cur_tick = rt_tick_get_millisecond();
    get_distance_fusion(&sensor_tick, &label, &dis);
    if (label == 1 && (abs(cur_tick - sensor_tick) < 4000)) // 数据可信且新
    {
        b_dis = dis;
    } else {
        b_dis = -1;
    }
    cur_line_to_next_line_info.b_dis = b_dis;
    LOG_I("b_dis=%d", b_dis);
    
    // 判断第三边
    if_edge = judge_c_edge(JUDGE_C_TIME);
    LOG_D("if_edge = %d", if_edge);
    if (if_edge)
    {
        if_c_edge = 1;
    }
    
    if (odd_info.if_arrvied_c==1 &&even_info.if_arrvied_c==1)
    {
        if_c_all_edge = 1;
    } else
    {
        if_c_all_edge = 0;
    }
    
    // 判断下一个同向行要不要减小行走距离
    if_reduce_forward_time = judge_next_same_forward_derection_time();
    if (if_reduce_forward_time == 1)
    {
        if (if_odd_line == 1)
        {
            if_odd_a_shrt_dis = 1;
        } else
        {
            if_even_a_shrt_dis = 1;
        }
    } else
    {
        if (if_odd_line == 1)
        {
            if_odd_a_shrt_dis = 0;
        } else
        {
            if_even_a_shrt_dis = 0;
        }
    }
    LOG_D("after charge, if_odd_a_shrt_dis=%d, if_even_a_shrt_dis=%d", if_odd_a_shrt_dis, if_even_a_shrt_dis);
    
    
    // 切行直行已经走过，action不需要再走
    move_to_next_row_time = 0;
    LOG_D("move_to_next_row_time=%d", move_to_next_row_time);
    // 切行：包括旋转90度+后退到墙边
    struct move_to_next_row_info move_to_next_row;
    // 根据拟合出的点求后退时间
    int calc_time;
    if (if_odd_line == 1) //奇数行
    {
        if (odd_ret == 0)
        {
            calc_time = ((int)(float)(point.y-215)/MOTOR_SPEED_MOVE_SPEED)*1000;
            move_back_time = (calc_time > 0) ? calc_time : 0;
        }
    } else //偶数行
    {
        if (even_ret == 0)
        {
            calc_time = ((int)(float)(point.y-215)/MOTOR_SPEED_MOVE_SPEED)*1000;
            move_back_time = (calc_time > 0) ? calc_time : 0;;
        }
    }
    LOG_D("move back time=%d", move_back_time);

    // mode控制旋转方向，为0右转，为1左转
    if (sign > 0)
    {
        move_to_next_row.mode = wash_floor_task_info.total_moved_row%2; // 正常换向
    } else
    {
        move_to_next_row.mode = (wash_floor_task_info.total_moved_row+1)%2; // 按相反方向换向
    }
    
    move_to_next_row.move_time = move_to_next_row_time;
    move_to_next_row.if_clean_dirt = if_clean_edge;
    LOG_D("if_clean_edge=%d", if_clean_edge);
    
#if (IF_UPDATE_YAW==RT_TRUE)
    move_back_time += 10*1000; // 增加10s后退时间，以便怼到墙
#endif
    
    // 换行后退时，为了吸污增加5s前进时间
    if (if_clean_edge == 1)
    {
        move_to_next_row.back_time = move_back_time + 5*1000;
    } else
    {
        move_to_next_row.back_time = move_back_time;

    }
    LOG_D("before new line rotate, slope=%f", get_current_slope());


    LOG_I("CMD_ACTION_FLOOR_MOVE_TO_NEXT_LINE");
    ret_msg = action_on_floor(CMD_ACTION_FLOOR_MOVE_TO_NEXT_LINE, &move_to_next_row, NULL);

    LOG_D("move to next line %d, if_uturn_slips=%d", ret_msg, if_uturn_slips);
    if_uturn_slips_pre = if_uturn_slips;
    rt_thread_mdelay(1000);   
    LOG_D("after new line rotate, slope=%f", get_current_slope());
    wash_floor_task_info.total_moved_row += 1;   
    

    // 调整机器姿态
#if (IF_UPDATE_YAW==RT_TRUE)   // 每次换行更新起始角
    cur_line_start_yaw = get_current_yaw();
#else
    cur_line_start_yaw = calculate_yaw(cur_line_start_yaw, 180.0f);   // 下一行始终在上一行的基础上反转180度 
    current_yaw = get_current_yaw();
    delta = compare_yaws(get_current_yaw(), cur_line_start_yaw);
    LOG_D("delta=%f, current_yaw=%f", delta, current_yaw);
    if (fabs(delta) >= 3.0f)
    {
        LOG_D("newline start, before rotate yaw=%f", get_current_yaw());
        move_rotate_on_floor(delta, MOVE_CONTROL_RIGHT_SAFEZONE);
        LOG_D("newline start, after rotate yaw=%f", get_current_yaw());
        rt_thread_mdelay(500);
    }
#endif
    LOG_D("end one line, start_yaw=%f", cur_line_start_yaw);
    // 清洗新行
    goto WASH_LINE;

END:
    // factory_mode_led_show(COLOR_YELLOW);	//结束灯效
    floor_record_info.info_lists[row_id%EF_RCD_LIST_LEN] = one_row_info;
    floor_record_info.end_time = rt_tick_get_millisecond();
    floor_record_info.total_rows_num = row_id;
    get_battery_soc(&floor_record_info.end_soc);
    LOG_D("get c, write info");
    ef_rcd_write(&floor_record_info);
    memset(floor_record_info.info_lists, 0, EF_RCD_LIST_LEN * sizeof(struct wash_row_info));

    move_stop();
    return 0;
}


// 换到下一个反向清洗的初始位置。dis为机器y方向的改变量
static int wash_floor_to_next_clean_time(point_info_t new_start_point)
{
    int ret_msg;
    struct move_info pid_info; 
    RT_UNUSED(pid_info);
    float change_dis = 0.0f;
    
    float target_yaw, cur_yaw;
    struct move_forward_action_return_info ret_info;
    ret_info.forward_time = 0;
    LOG_I("wash_floor_to_next_clean_time, move_rows_num=%d", wash_floor_task_info.total_moved_row);
    LOG_D("wash_floor_task_info.wash_times=%d", wash_floor_task_info.wash_times);

    if (wash_floor_task_info.total_moved_row%2) // 当前在奇数行
    {
        // 走到偶数行
        cur_line_start_yaw = calculate_yaw(cur_line_start_yaw, 180.0f);
        cur_yaw = get_current_yaw(); // delta;
        move_rotate_on_floor(90.0f, MOVE_CONTROL_RIGHT_SAFEZONE);
        rt_thread_mdelay(500);
        LOG_D("odd line, target_yaw=%f, before_rotate=%f, after_rotate=%f", cur_time_start_yaw, cur_yaw, get_current_yaw());
        
        // 前进到墙边
        LOG_D("move to next time start");
        forward_action_info.if_change_speed = 0;
        forward_action_info.start_yaw = cur_time_start_yaw;
        forward_action_info.params = test_param;
        forward_action_info.time_out = MOVE_FORWARD_TIMEOUT;
        forward_action_info.find_slope_time = 1;

        ret_msg = action_on_floor(CMD_ACTION_FLOOR_MOVE_FORWARD_AND_GET_DIS, &forward_action_info, &ret_info);


        move_tick_time = ret_info.forward_time;
        LOG_D("%d, forward time=%d", ret_msg, move_tick_time);
        float cur_row_move_distance = ret_info.forward_time * MOTOR_SPEED_MOVE_SPEED / 1000;         // 转为秒（s）的单位
        LOG_D("cur_row_move_distance %d", cur_row_move_distance);
        LOG_D("cur_dis_from_edge %d", ret_info.end_dis); // 打印当前离墙边的距离

        // 机器y会变化的总长度
        change_dis = cur_row_move_distance + (float)ret_info.end_dis + 400; // 400mm是机身长度
        LOG_D("cur_row_length %f", change_dis);
        rt_thread_mdelay(500);
    }
#ifdef STOP_LINE_AT_B
    // 机器面对B边，停下
    LOG_D("STOP_LINE_AT_B. rotate target_yaw=%f", cur_line_start_yaw);
    move_rotate_on_floor_using_target(cur_line_start_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
    rt_thread_mdelay(1*1000);

    stop_at_waterline();
    return 0;
#endif
    
#ifdef STOP_LINE_AT_A
    int judge_stop_data = wash_floor_task_info.wash_times - stop_at_line_time*4;
    LOG_D("stop_at_line_time=%d, judge_stop_data=%d", stop_at_line_time, judge_stop_data);
    if ((judge_stop_data%3==0)&&(judge_stop_data!=0))  // 洗完两次来回
    {
        LOG_D("STOP_LINE_AT_A. ");
        if (wash_floor_task_info.total_moved_row%2 == 0)
        {
            LOG_D("left rotate 90.0f");
            move_rotate_on_floor(90.0f, MOVE_CONTROL_RIGHT_SAFEZONE);
        }
        rt_thread_mdelay(1000);
        LOG_D("after rotate, yaw=%f", get_current_yaw());

        //上水线，停水线2min
        struct stop_on_waterline_info wws_info = {
        .power_off_flag = 0,        // no power off
        .on_waterline_time = 2 *60,  // 2min
        .in_water_time = 5,       // 5s
        .repeat_count = 1};       // repeat 4 times
        goto_waterline_and_stop(&wws_info);
        rt_thread_mdelay(2*1000);

        cur_yaw = get_current_yaw();
        target_yaw = calculate_yaw(cur_yaw, 180.0f);
        LOG_D("cur_yaw=%f", cur_yaw);
        move_rotate_large_angle(target_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
        rt_thread_mdelay(500);
#if (IF_UPDATE_YAW==RT_TRUE)
        LOG_D("back updata yaw");
        // 如果矩形池子，后退更新yaw角
        float rotate_yaw = 0.0f;
        int slope_;
        actions_on_floor_backward_update_yaw(&rotate_yaw, &slope_);
        cur_time_start_yaw = get_current_yaw();
#else
        cur_time_start_yaw = calculate_yaw(cur_time_start_yaw, 180.0f);
#endif // (IF_UPDATE_YAW==RT_TRUE)
        LOG_I("new cur_time_start_yaw=%f", cur_time_start_yaw);
        stop_at_line_time += 1;
        return 0;
    } else
    {
        target_yaw = calculate_yaw(cur_time_start_yaw, 180.0f);
        LOG_D("odd time clean end. rotate to next time target yaw=%f", target_yaw);
        move_rotate_on_floor_using_target(target_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
        rt_thread_mdelay(500);
        LOG_D("after rotate, curyaw=%f", get_current_yaw());        
#if (IF_UPDATE_YAW==RT_TRUE)
        LOG_D("back updata yaw");
        // 如果矩形池子，后退更新yaw角
        float rotate_yaw = 0.0f;
        int slope_;
        actions_on_floor_backward_update_yaw(&rotate_yaw, &slope_);
        cur_time_start_yaw = get_current_yaw();
#else
        cur_time_start_yaw = calculate_yaw(cur_time_start_yaw, 180.0f);
#endif
        return 0;
    }
    
#endif // def STOP_LINE_AT_A
 
#ifdef RECYCLING_CLEAN
    // 循环清洗
    // 不洗陡坡
    if_clean_steep_incline = 0;
    target_yaw = calculate_yaw(cur_time_start_yaw, 180.0f);
    LOG_D("odd time clean end. rotate to next time target yaw=%f", target_yaw);
    move_rotate_on_floor_using_target(target_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
    rt_thread_mdelay(500);
    LOG_D("after rotate, curyaw=%f", get_current_yaw());        
#if (IF_UPDATE_YAW==RT_TRUE)
    LOG_D("back updata yaw");
    // 如果矩形池子，后退更新yaw角
    float rotate_yaw = 0.0f;
    int slope_;
    actions_on_floor_backward_update_yaw(&rotate_yaw, &slope_);
#endif  // #if (IF_UPDATE_YAW==RT_TRUE)
#else   // #ifdef RECYCLING_CLEAN
    if (wash_floor_task_info.wash_times%2==0) // 向深水区方向的清洗结束
    {
        target_yaw = calculate_yaw(cur_time_start_yaw, 180.0f);
        LOG_D("odd time clean end. rotate to next time target yaw=%f", target_yaw);
        // move_rotate_on_floor_using_target(target_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
        move_rotate_large_angle(target_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
        rt_thread_mdelay(500);
        LOG_D("after rotate, curyaw=%f", get_current_yaw());        
#if (IF_UPDATE_YAW==RT_TRUE)
        LOG_D("back updata yaw");
        // 如果矩形池子，后退更新yaw角
        float rotate_yaw = 0.0f;
        int slope_;
        actions_on_floor_backward_update_yaw(&rotate_yaw, &slope_);
#endif
    } else // 向浅水区方向的清洗结束
    {
        LOG_D("even time clean end.");
        target_yaw = calculate_yaw(cur_time_start_yaw, 180.0f);
        LOG_D("odd time clean end. rotate to next time target yaw=%f", target_yaw);
        move_rotate_on_floor_using_target(target_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
        rt_thread_mdelay(500);
        LOG_D("after rotate, curyaw=%f", get_current_yaw());

        // 头冲墙停到c边中间
        int forward_time = (int)move_tick_time/2;
        if ((forward_time < 500) || (forward_time > 180*1000))
        {
            forward_time = 10*1000;
        }
        LOG_D("move to middle, forward_time=%d, speed=%d", forward_time, test_param->forward_mtr_speed);
        move_forward_with_pid_and_time(test_param->forward_mtr_speed, target_yaw, forward_time);
        rt_thread_mdelay(1000);

        // 左转90度头冲c边
        move_rotate_on_floor(90.0f, MOVE_CONTROL_RIGHT_SAFEZONE);
        rt_thread_mdelay(1000);
    }
#endif

#if (IF_UPDATE_YAW == RT_TRUE)
    // 如果后退更新yaw，下一次的start_yaw根据机器姿态更新
    cur_time_start_yaw = get_current_yaw();
#else
    // 下一次清洗的初始角度为上一次的基础上反转180度 
    cur_time_start_yaw = calculate_yaw(cur_time_start_yaw, 180.0f);   // 下一次清洗的初始角度为上一次的基础上反转180度 
#endif
    LOG_I("new cur_time_start_yaw=%f", cur_time_start_yaw);

    // 更新start_point坐标
    if (wash_floor_task_info.wash_times%2 == 0) // 偶数次清洗
    {
        new_start_point->x = edge_point.x;
        new_start_point->y = pre_y + change_dis;
        pre_y += change_dis;
    } else // 奇数次清洗
    {
        new_start_point->x = edge_point.x;
        new_start_point->y = pre_y - change_dis;
        pre_y -= change_dis;
    }
    if (if_clean_steep_incline==0)
    {
        // 不洗陡坡
        if_find_deep_area = 1;
    }
    LOG_D("if_find_deep_area=%d", if_find_deep_area);
    
    //如果是找到了深水区，则需要洗深水区陡坡
    if ((robot_work_mode != WEEKLY_MODE)&&(if_find_deep_area==0))   // weekly mode不洗陡坡
    {
        LOG_D("work mode=%d", robot_work_mode);
        if ((deep_area_state > GROUND_FLAT)||(slope_label==1))   
        {
            LOG_I("first time wash, goto deep area");
            LOG_D("deep_area_state=%d, wash_times=%d", deep_area_state, wash_floor_task_info.wash_times);
            int forward_time = 2*1000;
            move_to_deep_area(forward_time);
            LOG_D("rotate to next time start yaw");
            move_rotate_on_floor_using_target(cur_time_start_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
            rt_thread_mdelay(1*1000);

            LOG_I("before yaw=%f", get_current_yaw());
            LOG_I("start wash slope");
            actions_on_slope_wash_around(NULL, NULL);
            LOG_I("end wash slope");
            
            move_rotate_on_floor_using_target(cur_time_start_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
            LOG_D("new start yaw=%f", get_current_yaw());
            rt_thread_mdelay(1*1000);
        }
    }
    
    // 重置初始找坡状态机
    deep_area_state = STATE_INITIAL;
    state_count = 0;
    if_find_deep_area = 1;
    LOG_D("close find deep area");

    return 0;
}


static int pentagonal_path_find_slope(int wash_times)
{
    float yaw;
    struct slope_type slope;
    LOG_D("slope_label==%d |%d", slope_label,wash_times);
    if (wash_times%2==0)
    {
        int find_slope_time = 0;
#if (VERSION_TYPE!=E_AUSTRALIA)
#if (POOL_BOTTOM_IS_SLOPE == RT_TRUE)

        if(slope_label != 0)
        {
            LOG_I("start pentagonal path find slope");
            actions_on_floor_search_slope(RT_NULL, &find_slope_time);  
        }
        else
        {
            move_rotate_on_floor(90.0f, 20000);
            rt_thread_mdelay(500);
            move_rotate_on_floor(90.0f, 20000);
            rt_thread_mdelay(500);
            move_backward_with_speed(2000);
            for(int i = 0; i < 5 ; i++)
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
            yaw = get_current_yaw();
            move_forward_with_pid_and_time(3000, yaw, 5000);
            rt_thread_mdelay(1000);

        }
         
#else   // (POOL_BOTTOM_IS_SLOPE == RT_TRUE)
        if (wash_times == 0)
        {
            LOG_D("first time, back and search wall.");
            actions_on_floor_backward_search_wall(RT_NULL, &find_slope_time);
        }
#endif  // (POOL_BOTTOM_IS_SLOPE == RT_TRUE)
        // 重置start_yaw
        yaw = get_current_yaw();
#else   // (VERSION_TYPE!=E_AUSTRALIA)
        // 走到起始点,并初始化yaw
        austra_goto_start_point_act(&yaw);
        LOG_D("aus start_yaw=%f", yaw);
#endif  // (VERSION_TYPE!=E_AUSTRALIA)
        if(slope_label != 0)
        {
            slope_label = find_slope_time;
            LOG_I("slope_label=%d", find_slope_time);
        }
        
        wash_floor_info_start_yaw_set(&wash_floor_task_info, yaw);
        LOG_I("reset start yaw=%f", yaw);
        if (slope_label == 1) // 找到坡
        {
            // 走到深水区
            LOG_D("go to deep area");
            stright_to_deep_area(yaw, DELTA_SLOPE);
            LOG_D("go to deep area end");
        }
#if (VERSION_TYPE==E_NORMAL)
        else {
            LOG_D("no slope");
            yaw = reset_imu(15*1000, 1000);
            wash_floor_info_start_yaw_set(&wash_floor_task_info, yaw);
        }
#endif
    }
    
    return 0;
}

static int reset_area_and_slope_info(int wash_times)
{
    // 重置找深水区状态
    deep_area_state = STATE_INITIAL;
    state_count = 0;
    LOG_I("init deep_area_state");

    // 重置标志
    LOG_D("wash_times=%d", wash_times);
    if (wash_times%2==0)
    {
        LOG_I("init slope state");
        if_find_deep_area = 0;
    }

    if(wash_times == 0)
        slope_label = -1;

    LOG_D("slope_label=%d, if_find_deep_area=%d", slope_label, if_find_deep_area);

    return 0;
}


// 洗地
int wash_floor_thread(enum work_mode_type work_mode)
{
    struct point_info tank_start_point = {0.0f, 0.0f, 0.0f}; // 机器初始位置设为坐标原点
    rt_uint32_t week_time;  // 休眠时长
    RT_UNUSED(stop_at_line_time);
    LOG_I("start wash floor");

    robot_work_mode = work_mode; // TODO:读取work mode
    LOG_D("robot_work_mode=%d", robot_work_mode);
    // 工作模式判断
    // if (robot_work_mode == WATERLINE_MODE) // 如果是洗墙模式，退出洗地
    // {
    //     LOG_W("E_WASH_WALL_MODE, exit wash floor");
    //     move_stop();
    //     return -1;
    // }
	
    get_model_info(&cur_model_type, &weekly_max_clean_time);
    LOG_I("model type %d, week time=%d", cur_model_type.model_mode, weekly_max_clean_time);
    
    // 获取清洗参数
    update_params_by_mode_and_circle(robot_work_mode, 0);
    test_param = get_wf_params_info();
    print_params(test_param);
    clean_info.line_interval = test_param->adjacent_line_interval;
    LOG_D("line_interval=%f", clean_info.line_interval);

    get_wf_paras(robot_work_mode, &wf_conf_para, &clean_info);
    LOG_I("get paras, wash times %d %d, off value %d, max rows %d", clean_info.min_wash_times, clean_info.wash_times, 
            clean_info.off_value, clean_info.max_clean_rows);
    
    // 开水泵 
    get_battery_voltage(&vol);
    LOG_I("current voltage: %dmV, if_clean_steep_incline=%d", vol, if_clean_steep_incline);
    LOG_D("set pump speed=%d", test_param->forward_wp_speed);
    move_wp_speed_on_floor_set(test_param->forward_wp_speed);
    // rt_thread_mdelay(3*1000);
    rt_thread_mdelay(1*1000);

    // 开激光
    laser_switch_on();
    LOG_D("turn on laser.");
    
    // 去掉地磁标定，用动态地磁数据
    arg_out.out_count = 0;
    LOG_D("arg_out.out_count = %d", arg_out.out_count);

    // 初始化洗地
    wash_floor_info_reset(&wash_floor_task_info, 0.0f);
    // 开辟内存
    init_malloc();

    // 开始清洗
    while ((wash_floor_task_info.wash_times < clean_info.wash_times) && (vol > clean_info.off_value))
    {
        int clean_clrcle = wash_floor_task_info.wash_times/2;
        update_params_by_mode_and_circle(robot_work_mode, clean_clrcle);
        test_param = get_wf_params_info();
        line_wa_interval = test_param->adjacent_line_interval;
        LOG_D("line_wa_interval=%f", line_wa_interval);
        print_params(test_param);
#if (VERSION_TYPE==E_NORMAL)
        update_clean_info_data(test_param, &clean_info);
#endif
        if(sum_vision_time >= 5)
            if_clean_steep_incline = test_param->if_clean_steep_incline;
        else
            if_clean_steep_incline = 0;  
#if ((VERSION_TYPE==E_AUSTRALIA) || (VERSION_TYPE==E_EXHIBITION))
        if_clean_steep_incline = 0;
#endif
        LOG_D("wash_floor_task_info.wash_times=%d, start_yaw=%f", 
                wash_floor_task_info.wash_times, wash_floor_task_info.start_yaw);
        // 开激光
        laser_switch_on();
        LOG_D("turn on laser.");
        LOG_D("set pump speed=%d", test_param->forward_wp_speed);
        move_wp_speed_on_floor_set(test_param->forward_wp_speed);
        rt_thread_mdelay(1*1000);

        // 重置标置位
        reset_area_and_slope_info(wash_floor_task_info.wash_times);
        // 从浅水区向深水区清洗时，才找坡
        pentagonal_path_find_slope(wash_floor_task_info.wash_times);

        LOG_D("start wash_floor_clean_once");
        wash_floor_clean_once(&tank_start_point, wash_floor_task_info.wash_times); // 清洗，直到碰到第三边
        LOG_D("end wash_floor_clean_once");

        // 更新机器位置和坐标
        LOG_D("start wash_floor_to_next_clean_time");
        wash_floor_to_next_clean_time(&tank_start_point);
        LOG_D("end wash_floor_to_next_clean_time");
        
        // 更新wash floor info
        wash_floor_info_reset_once(&wash_floor_task_info, cur_time_start_yaw);
        
        //获取当前电压
        get_battery_voltage(&vol);
        LOG_D("current voltage: %dmV", vol);
        LOG_D("last circle totle vision_time=%d", sum_vision_time);
#if (VERSION_TYPE==E_NORMAL)
        if (eeprom_model_get() < MODEL_L20 && (wash_floor_task_info.wash_times%2 == 0))
        {
            if (sum_vision_time == 0)
            {
                LOG_D("vision clean, finish wash");
                break;
            }
        }
        else if ((wash_floor_task_info.wash_times > (clean_info.min_wash_times - 1))&&(wash_floor_task_info.wash_times%2 == 0))
        {
            if (sum_vision_time == 0)
            {
                {
                    LOG_D("clean, finish wash");
                    break;
                }
            }
        }
#endif
    }
    free(wf);

   
    // 停止灯效
    // 停止
    LOG_I("wash floor end");
    wash_floor_off_cur_time();
    
    get_battery_voltage(&vol);
    LOG_D("current voltage: %dmV, wash_time=%d", vol, wash_floor_task_info.wash_times);
    int off_label = (wash_floor_task_info.wash_times%2) ? slope_label: 0;
    LOG_D("off_label=%d", off_label);
#if (VERSION_TYPE==E_EXHIBITION_P10)
    off_label = 0;
#endif

    if (wash_floor_task_info.wash_times%2==1)
    {
        LOG_D("rotate to edge");
        float cur_yaw = get_current_yaw();
        float target_yaw = calculate_yaw(cur_yaw, 180.0f);
        LOG_D("cur_yaw=%f", cur_yaw);
        move_rotate_large_angle(target_yaw, MOVE_CONTROL_RIGHT_SAFEZONE);
        rt_thread_mdelay(500);
        LOG_D("cur_yaw=%f", get_current_yaw());
    }

    if (robot_work_mode == ALL_MODE)
    {
        LOG_D("ALL_MODE, finish wash floor.");
        if (vol < clean_info.off_value)
        {
            actions_on_floor_goto_shallow_turn_off(&off_label, NULL);
        }
        else
        {
            fsmSetEvent(&pln_fsm, PLN_EVENT_ALL_MODE_START_WASH_WALL);
        }
        return 0;
    } else if (robot_work_mode == WEEKLY_MODE)
    {
        LOG_D("WEEKLY_MODE, in week");
        if (vol < clean_info.off_value)
        {
            // 关机
            actions_on_floor_goto_shallow_turn_off(&off_label, NULL);
            return 0;
        }
        
        // 单位为min
        week_time = get_weekly_time();
        LOG_D("get week time %d", week_time);
        // 休眠时间转为s
        week_time = week_time*60;   
        LOG_I("week time = %d s", week_time);

        eeprom_type_t p_eeprom = get_eeprom_info();
        p_eeprom->weely_cycles += 1;
        LOG_D("weekly cycles=%d", p_eeprom->weely_cycles);
        if (p_eeprom->weely_cycles >= weekly_max_clean_time)
        {
            // 关机
            LOG_I("weekly time get max,weekly wash done.");
        } else
        {
            // weekly标置位写入eeprom
            eeprom_work_mode_set(WEEKLY_MODE);
            LOG_I("set week time");
            json_update_shutdown();
//            set_battery_power_on_time(week_time);
			set_robot_weeklymode_sleep_on_time(week_time);
			rt_thread_mdelay(2000);
        }
        return 0;
    } else if (robot_work_mode == SMART_MODE)
    {
        LOG_D("SMART_MODE, end wash");
        actions_on_floor_goto_shallow_turn_off(&off_label, NULL);
    } else
    {
        LOG_I("mode %d, wash floor end.", robot_work_mode);
        actions_on_floor_goto_shallow_turn_off(&off_label, NULL);
        return 0;
    }
    
    return 0;
}

int8_t washFloorModeEnter(void)
{
    LOG_I("pln fsm %s\n", __FUNCTION__); 
    
    return 0;
}


int8_t washFloorModeRun(void)
{
    system_info_t info = system_info_get();
    
    wash_floor_thread(info->work_mode);
    
    // if(1)
    // {
    //     fsmSetEvent(&pln_fsm, PLN_EVENT_WASHFLOOR_TEST_MAP);
    // }
    // else
    // {
    //     fsmSetEvent(&pln_fsm, PLN_EVENT_WASHFLOOR_FAST_CLEAN);
    // }
    

	return 0;
}


int8_t washFloorModeExit(void)
{
	LOG_I("pln fsm %s\n", __FUNCTION__);
        
	return 0;
}


void test_geo(int argc, char *argv[])
{
    if (argc < 3)
    {
        rt_kprintf("usage: test_geo <mode> <count> \n ");
        rt_kprintf("\t 1: get current roll pitch yaw \n");
        rt_kprintf("\t 2: get cucrrent geo \n ");
        return;
    }

    int order = atoi(argv[1]);
    int count = 0, geo_count = 0;

    switch (order)
    {
    case 1:
        count = atoi(argv[2]);
        while (count > 0)
        {
            LOG_D("roll=%f, pitch=%f, yaw=%f", get_current_roll(), get_current_pitch(), get_current_yaw());
            rt_thread_mdelay(100);
            count -= 1;
        }
        break;
    case 2:
        geo_count = atoi(argv[2]);
        LOG_D("start get current geo");
        LOG_D("yaw=%f", get_current_yaw());
        struct mag_info geo_info;
        while (geo_count > 0)
        {
            magnetic_raw_data_get(&geo_info);
            LOG_D("%f, %f, %f", geo_info.raw_mag_x, geo_info.raw_mag_y, geo_info.raw_mag_z);
            rt_thread_mdelay(100);
            geo_count -= 1;
        }
        LOG_D("finish get current geo");
        break;

    default:
        break;
    }
}
MSH_CMD_EXPORT(test_geo, test geo test);
