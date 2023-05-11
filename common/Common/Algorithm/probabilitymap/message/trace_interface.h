#ifndef TRACE_INTERFACE_H
#define TRACE_INTERFACE_H
#include <stdint.h>
#ifdef __cplusplus
extern "C"{
#endif 
typedef enum TraceLevel_t {
    FAILURE = 1,
    FAULT = 2,
    ERROR = 3,
    BUG = 4,
    WARN = 5,
    NOTICE = 6,
    INFO = 7,
    DEBUG = 8,
} TraceLevel;

typedef enum ModuleType_t {
    SENSOR = 1,
    CHASSIS = 2,
    APPLICATION=3,
} ModuleType;

/*
    Fault : It is a condition that causes the software to fail to perform its required function.
    Error : Refers to difference between Actual Output and Expected output.
    Failure : It is the inability of a system or component to perform required function according to its specification.
    IEEE Definitions

    Failure: External behavior is incorrect
    Fault: Discrepancy in code that causes a failure.
    Error: Human mistake that caused fault
    Note:

    Error is terminology of Developer.
    Bug is terminology of Tester
*/
/*
    1:Failure
    2:Fault
    3:Error
    4:Bug
    5:告警级(WARN)
    6:注意级(NOTICE)
    7:通知级(INFO)
    8:调试级(DEBUG)
*/
    
/**
 * @brief 触发或者清除FAULT
 * @param level　故障等级
 * @param type 模块所属类别（传感器、底盘、应用程序）
 * @param code　故障码，详见附件文档．
 * @param module_id　模块编号，详见附件文档
 * @param sub_id 安装多个同类传感器（如雷达），区分各个传感器，默认为0
 * @param fault_message　故障信息，非必须（使用C语言打印格式)
 */
int TraceInit();
void TraceDestroy();
void TraceTrigger(int level, int type, uint32_t code, uint16_t module_id, uint16_t sub_id, const char* format, ...);
void TraceClear(int level, int type, uint32_t code, uint16_t module_id, uint16_t sub_id, const char* format, ...);
char* GetCurrentTime(char *t);
void TraceSetSubId(uint16_t sub_id);
#ifdef __cplusplus
}
#endif

#endif //TRACE_INTERFACE_H
