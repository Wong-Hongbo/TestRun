/*
PayloadCommandInfon.hh

This C++ header file defines the NML Messages for PayloadCommandInfo
Template Version 1.1

MODIFICATIONS:
March 19 19:23:05 CST 2014	Created by hutingbo.

*/

// Prevent Multiple Inclusion
#ifndef PAYLOADCOMMANDN_HH
#define PAYLOADCOMMANDN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define PAYLOADCOMMAND_MSG_TYPE 1602
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

//载荷指令通信格式定义
// Define the NML Message Classes

/*
Command_Modules[0]    :IRRoadDetector
Command_Modules[1]    :LaneMarkingDetector
Command_Modules[2]    :LocalThDModeling
Command_Modules[3]    :NegativeDetector
Command_Modules[4]    :PositiveDetector
Command_Modules[5]    :RemoteCommand
Command_Modules[6]    :RoadDetector
Command_Modules[7]    :RoadSceneModeling
Command_Modules[8]    :SceneEnhance
Command_Modules[9]    :SceneUnderstanding
Command_Modules[10]  :SurfaceProcessor
Command_Modules[11]  :VehicleDetector
Command_Modules[12]
Command_Modules[13]
Command_Modules[14]
Command_Modules[15]
Command_Modules[16]
Command_Modules[17]
Command_Modules[18]
Command_Modules[19]

*/
class PAYLOADCOMMAND_MSG : public NMLmsgEx
{
public:
	//Constructor
	PAYLOADCOMMAND_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
	char Command_IPCameras[6];	        // 针对IP相机的控制命令,0:无命令，1：打开，2：关闭
	char Command_Computers[10];            // 针对工控机的控制，0：无命令，1：关闭，2：重启
	char Command_Modules[20];              // 针对软件模块的控制命令，0：无命令，1：运行，2：暂停，3：退出
	char Command_Servers[10];              // 针对工控机Server的控制命令：0：无命令，1：重启
	char Command_Sensors[10];              // 针对传感器的控制命令：0：无命令，1：开启，2：暂停，3：重启，4:退出
	unsigned char Command_RemoteShow;               // 无人平台上一台机器中同时运行虚拟视景合成，数据采集与显示两个程序，
                                                        // 该命令决定显示哪个界面,0:无命令，1：视景合成，2：数据采集 
};

// Declare NML format function
extern int PayloadCommandFormat(NMLTYPE, void *, CMS *);

#endif 	// REMOTECOMMANDINFON_HH
