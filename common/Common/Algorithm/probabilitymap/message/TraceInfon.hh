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
    
    //SYS_EMESG  = 0;//������
    //SYS_ALERT  = 1;//���伶
    //SYS_CRIT   = 2;//�ٽ缶
    //SYS_ERR    = 3;//����
    //SYS_WARN   = 4;//�澯��
    //SYS_NOTICE = 5;//ע�⼶
    //SYS_INFO   = 6;//֪ͨ��
    //SYS_DEBUG  = 7;//���Լ�
    UINT8    trace_level;//���ϵȼ�
    
    char     trace_code[TRACE_CODE_LEN];//������
    char     proc_name[PROC_NAME_LEN];//���̻�ģ����
    char     info[INFO_LEN];//������Ϣ
};

// Declare NML format function
extern int TraceInfoFormat(NMLTYPE, void *, CMS *);

#endif 	// TraceInfoN_HH
