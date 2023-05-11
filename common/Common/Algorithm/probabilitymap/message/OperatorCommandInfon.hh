/*
OperatorCommandInfon.hh

This C++ header file defines the NML Messages for OperatorCommandInfon
Template Version 1.1

MODIFICATIONS:
Wed June 04 10:22:05 CST 2014	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef OPERATORCOMMANDN_HH
#define OPERATORCOMMANDN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "NMLmsgExn.hh"
#include "usertype.hh"

// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define OPERATORCOMMAND_MSG_TYPE 1666
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


//UGV_MODE_CMD 和UGV_ACTION_CMD，每帧中检测取值不在范围内，当作错误帧丢弃。


/*local coordinate*/

class OPERATORCOMMAND_MSG : public NMLmsgEx
{
public:

	//Constructor
	OPERATORCOMMAND_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
	int MessageID;                      // 消息ID
	// int MessageSeqNum;               // 消息序列号
	int TimeFlag;                       // 时间，单位ms
	int DestinationID;		            // 地址码，指定接收设备 
	int Ugv_mode;	                    // 第一层控制命令，包括不同导航模式（自主，半自主，主从） ，参照  UGV_MODE_CMD
	int Ugv_action;	                    // 第二层控制命令，不同导航模式下对应的动作 ，参照 UGV_ACTION_CMD
	int CommandData[4];                 // 控制数据，半自主
  LOCAL_COORDINATE position_frame;
};
// Declare NML format function
extern int OperatorCommandFormat(NMLTYPE, void *, CMS *);

#endif 	// OPERATORCOMMANDINFON_HH
