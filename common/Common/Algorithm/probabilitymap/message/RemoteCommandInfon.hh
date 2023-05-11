/*
RemoteCommandInfon.hh

This C++ header file defines the NML Messages for REMOTECOMMANDINFO
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef REMOTECOMMANDN_HH
#define REMOTECOMMANDN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define REMOTECOMMAND_MSG_TYPE 1601
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

//遥控指令通信数据包结构定义
// Define the NML Message Classes
// 全局路径点
/*struct GLOBAL_WAYPOINT
{
	int Longitude;
	int Laltitude;
	int Altitude;
	short int Roll;
	short int Pitch;
	short int Yaw;	
	short int Speed;
};
*/
class REMOTECOMMAND_MSG : public NMLmsgEx
{
public:

	//Constructor
	REMOTECOMMAND_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
	int DestinationID;		        // 地址码，指定接收设备 
	int Command_Level1;	                // 第一层控制命令，包括不同导航模式（自主，半自主，主从）以及传送全局任务 
  int Command_Level2;	                // 第二层控制命令，不同导航模式下对应的动作
	int CommandData[20];                    // 控制数据
	int CommandDataNumber;                  // 控制数据个数
//	DECLARE_NML_DYNAMIC_LENGTH_ARRAY(GLOBAL_WAYPOINT,GlobalPath,65536);  //全局点
//	int GlobalWaypointNumber;               // 全局点个数
//	int GlobalTaskNumber;                   // 当前发送的全局任务编号，-1表示没有全局任务

};

// Declare NML format function
extern int RemoteCommandFormat(NMLTYPE, void *, CMS *);

#endif 	// REMOTECOMMANDINFON_HH
