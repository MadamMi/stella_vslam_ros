#ifndef __PLN_MODE_CONTROL_H
#define __PLN_MODE_CONTROL_H

#include "fsmCore.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define PLN_FSM_TABLE_LEN	8


/*  mode  */
typedef enum{
    RUN_MODE_STANDBY = 0,    //0
    RUN_MODE_WASHFLOOR,      //1
    RUN_MODE_WASHWALL,       //2
    RUN_MODE_TEST_MAP,       //3
    RUN_MODE_FAST_CLEAN,     //4
    RUN_MODE_NORMAL_CLEAN,   //5

    
    //add mode here
    PLN_FSM_MODE_COUNT
}plnModeTypeDef;

/*  event  */
typedef enum{
    PLN_EVENT_START_SMART_MODE = 1,         //1
    PLN_EVENT_START_WEEKLY_MODE,            //2
    PLN_EVENT_START_WATERLINE_MODE,         //3
    PLN_EVENT_START_ALL_MODE,               //4
    PLN_EVENT_WASHFLOOR_TEST_MAP,           //5
    PLN_EVENT_WASHFLOOR_FAST_CLEAN,         //6
    PLN_EVENT_WASHFLOOR_NORMAL_CLEAN,       //7
    PLN_EVENT_ALL_MODE_START_WASH_WALL,     //8

}plnEventTypeDef;

extern fsmFuncTypeDef pln_fsmFuncTable[PLN_FSM_MODE_COUNT];
extern fsmTableTypeDef pln_fsmTable[PLN_FSM_TABLE_LEN];
extern int8_t plnFsmEventManage(void);
extern fsmTypeDef pln_fsm;

#ifdef __cplusplus
}
#endif

#endif

