#ifndef MODULE_INTERFACE_H
#define MODULE_INTERFACE_H

#include "trace_interface.h"

#define MODULE_TYPE 0x03
#define MODULE_ID 0x0b
#define SUB_ID 0x01
#include <map>

static std::map<int, int> errorcode_with_level;

inline void PushErrorCode(int error_code, int level) {
  errorcode_with_level.insert(std::pair<int, int>(error_code, level));
}

inline void PopErrorCode(int error_code, int level) {
  std::map<int, int>::iterator it = errorcode_with_level.find(error_code);
  if (it != errorcode_with_level.end()) {
    errorcode_with_level.erase(it);
  }
}


inline bool IsGotError() {
  return errorcode_with_level.size() == 0 ? false: true;
}

inline bool IsGotError(int error_code) {
  std::map<int, int>::iterator it = errorcode_with_level.find(error_code);
  if (it != errorcode_with_level.end()) {
    return true;
  }
  else {
    return false;
  }
}


#define TraceCheckIn(form, info...) do\
{\
    char t[32] = {0};\
    GetCurrentTime(t);\
    TraceChek(0, MODULE_TYPE, MODULE_ID, SUB_ID, "[DEBUG][%s][%s:%d]" form, t, __func__, __LINE__, ##info);\
}while(0)

#define TraceCheckFailed(form, info...) do\
{\
    char t[32] = {0};\
    GetCurrentTime(t);\
    TraceChek(1, MODULE_TYPE, MODULE_ID, SUB_ID, "[DEBUG][%s][%s:%d]" form, t, __func__, __LINE__, ##info);\
}while(0)

#define TraceDebug(code, form, info...)                                     \
  do {                                                                      \
    char t[32] = {0};                                                       \
    GetCurrentTime(t);                                                      \
    TraceTrigger(8, MODULE_TYPE, code, MODULE_ID, SUB_ID,                   \
                 "[DEBUG][%s][%s:%d]" form, t, __func__, __LINE__, ##info); \
  } while (0)

#define TraceInfo(code, form, info...)                                     \
  do {                                                                     \
    char t[32] = {0};                                                      \
    GetCurrentTime(t);                                                     \
    TraceTrigger(7, MODULE_TYPE, code, MODULE_ID, SUB_ID,                  \
                 "[INFO][%s][%s:%d]" form, t, __func__, __LINE__, ##info); \
  } while (0)

#define TraceNotice(code, form, info...)                                     \
  do {                                                                       \
    char t[32] = {0};                                                        \
    GetCurrentTime(t);                                                       \
    TraceTrigger(6, MODULE_TYPE, code, MODULE_ID, SUB_ID,                    \
                 "[NOTICE][%s][%s:%d]" form, t, __func__, __LINE__, ##info); \
  } while (0)

#define TraceWarn(code, form, info...)                                     \
  do {                                                                     \
    char t[32] = {0};                                                      \
    GetCurrentTime(t);                                                     \
    TraceTrigger(5, MODULE_TYPE, code, MODULE_ID, SUB_ID,                  \
                 "[WARN][%s][%s:%d]" form, t, __func__, __LINE__, ##info); \
    PushErrorCode(code, 5);                                                \
  } while (0)

#define TraceBug(code, form, info...)                                     \
  do {                                                                    \
    char t[32] = {0};                                                     \
    GetCurrentTime(t);                                                    \
    TraceTrigger(4, MODULE_TYPE, code, MODULE_ID, SUB_ID,                 \
                 "[BUG][%s][%s:%d]" form, t, __func__, __LINE__, ##info); \
    PushErrorCode(code, 4);                                               \
  } while (0)

#define TraceError(code, form, info...)                                     \
  do {                                                                      \
    char t[32] = {0};                                                       \
    GetCurrentTime(t);                                                      \
    TraceTrigger(3, MODULE_TYPE, code, MODULE_ID, SUB_ID,                   \
                 "[ERROR][%s][%s:%d]" form, t, __func__, __LINE__, ##info); \
    PushErrorCode(code, 3);                                                 \
  } while (0)

#define TraceFault(code, form, info...)                                     \
  do {                                                                      \
    char t[32] = {0};                                                       \
    GetCurrentTime(t);                                                      \
    TraceTrigger(2, MODULE_TYPE, code, MODULE_ID, SUB_ID,                   \
                 "[FAULT][%s][%s:%d]" form, t, __func__, __LINE__, ##info); \
    PushErrorCode(code, 2);                                                 \
  } while (0)

#define TraceFailure(code, form, info...)                                     \
  do {                                                                        \
    char t[32] = {0};                                                         \
    GetCurrentTime(t);                                                        \
    TraceTrigger(1, MODULE_TYPE, code, MODULE_ID, SUB_ID,                     \
                 "[FAILURE][%s][%s:%d]" form, t, __func__, __LINE__, ##info); \
    PushErrorCode(code, 1);                                                   \
  } while (0)

#define TraceWarnClear(code, form, info...)                              \
  do {                                                                   \
    char t[32] = {0};                                                    \
    GetCurrentTime(t);                                                   \
    TraceClear(5, MODULE_TYPE, code, MODULE_ID, SUB_ID,                  \
               "[WARN][%s][%s:%d]" form, t, __func__, __LINE__, ##info); \
    PopErrorCode(code, 5);                                               \
  } while (0)

#define TraceBugClear(code, form, info...)                              \
  do {                                                                  \
    char t[32] = {0};                                                   \
    GetCurrentTime(t);                                                  \
    TraceClear(4, MODULE_TYPE, code, MODULE_ID, SUB_ID,                 \
               "[BUG][%s][%s:%d]" form, t, __func__, __LINE__, ##info); \
    PopErrorCode(code, 4);                                              \
  } while (0)

#define TraceErrorClear(code, form, info...)                              \
  do {                                                                    \
    char t[32] = {0};                                                     \
    GetCurrentTime(t);                                                    \
    TraceClear(3, MODULE_TYPE, code, MODULE_ID, SUB_ID,                   \
               "[ERROR][%s][%s:%d]" form, t, __func__, __LINE__, ##info); \
    PopErrorCode(code, 3);                                                \
  } while (0)

#define TraceFaultClear(code, form, info...)                              \
  do {                                                                    \
    char t[32] = {0};                                                     \
    GetCurrentTime(t);                                                    \
    TraceClear(2, MODULE_TYPE, code, MODULE_ID, SUB_ID,                   \
               "[FAULT][%s][%s:%d]" form, t, __func__, __LINE__, ##info); \
    PopErrorCode(code, 2);                                                \
  } while (0)

#define TraceFailureClear(code, form, info...)                              \
  do {                                                                      \
    char t[32] = {0};                                                       \
    GetCurrentTime(t);                                                      \
    TraceClear(1, MODULE_TYPE, code, MODULE_ID, SUB_ID,                     \
               "[FAILURE][%s][%s:%d]" form, t, __func__, __LINE__, ##info); \
    PopErrorCode(code, 1);                                                  \
  } while (0)

// 20200722
//之前定义的接口中，没有考虑到SUB_ID动态决定的情况
//增加SUB_ID设置接口，如果需要设置SUB_ID,请调用这个接口。（没有SUB_ID的同事可以忽略这个接口，SUB_ID在多个相机或者多个雷达等多传感器，且需要在车端运行多个实例时指定）
#define TraceSetSubID(sub_id) \
  do {                        \
    char t[32] = {0};         \
    GetCurrentTime(t);        \
    TraceSetSubId(sub_id);    \
  } while (0)

inline void ResetErrorCode() {
  std::cout << "ResetErrorCode============================" << std::endl;
  if (errorcode_with_level.size()) {
    std::map<int, int>::iterator it;
    for (it = errorcode_with_level.begin(); it != errorcode_with_level.end(); it++){
      if (it->second == WARN) {
        TraceWarnClear(it->first, "Reset Error Code [0x%x]", it->second);
      } else if (it->second == BUG) {
        TraceBugClear(it->first, "Reset Error Code [0x%x]", it->second);
      } else if (it->second == ERROR) {
        TraceErrorClear(it->first, "Reset Error Code [0x%x]", it->second);
      } else if (it->second == FAULT) {
        TraceFaultClear(it->first, "Reset Error Code [0x%x]", it->second);
      } else if (it->second == FAILURE) {
        TraceFailureClear(it->first, "Reset Error Code [0x%x]", it->second);
      }
    }
  }
}


#endif
