/*
TraceInfon.hh

This C++ header file defines the NML Messages for TraceInfoINFO
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:04 CST 2013	Created by rcsdesign.
*/

// Prevent Multiple Inclusion
#ifndef TraceInfoN_HH
#define TraceInfoN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Define the integer type ids.
#define TRACEINFO_MSG_TYPE 19203

#define TRACE_CODE_LEN (50)
#define PROC_NAME_LEN (50)
#define INFO_LEN (1000)

class TRACEINFO_MSG : public NMLmsgEx
{
public:

    //Constructor
    TRACEINFO_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    
    //SYS_EMESG  = 0;//致命级
    //SYS_ALERT  = 1;//警戒级
    //SYS_CRIT   = 2;//临界级
    //SYS_ERR    = 3;//错误级
    //SYS_WARN   = 4;//告警级
    //SYS_NOTICE = 5;//注意级
    //SYS_INFO   = 6;//通知级
    //SYS_DEBUG  = 7;//调试级
    UINT8    trace_level;//故障等级
    
    char     trace_code[TRACE_CODE_LEN];//故障码
    char     proc_name[PROC_NAME_LEN];//进程或模块名
    char     info[INFO_LEN];//故障信息
};

// Declare NML format function
extern int TraceInfoFormat(NMLTYPE, void *, CMS *);

#endif 	// TraceInfoN_HH
