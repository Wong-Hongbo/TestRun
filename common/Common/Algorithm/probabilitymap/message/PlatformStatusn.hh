/*
PlatformStatusn.hh

This C++ header file defines the NML Messages for PLATFORMSTATUSN
Template Version 1.1

MODIFICATIONS:
March 19 19:33:05 CST 2014	Created by hutingbo.

*/

// Prevent Multiple Inclusion
#ifndef PLATFORMSTATUSN_HH
#define PLATFORMSTATUSN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
// Trying to merge the type ids often results in redefinn the ID twice..
// Define the integer type ids.
#define PLATFORMSTATUS_MSG_TYPE 1603

class PLATFORMSTATUS_MSG : public NMLmsgEx
{
public:

	//Constructor
	PLATFORMSTATUS_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
	// Place custom variables here.
	int MessageID;                          // 消息ID
    //    int MessageSeqNum;                      // 消息序列号
        int TimeFlag;                           // 时间，单位ms
	unsigned short	ResourceID;		// 地址码，标识发送设备
 	char   IPCameraState[6];       // UGV上IP相机状态，0：正常，1：关闭
	char   ComputerState[10];      // UGV上各工控机状态，0：正常，1：关机
	char   ModuleState[20];	       // UGV各软件模块的工作状态
	char   SensorState[10];        // UGV传感器状态，0：正常，1:关闭，2：出错，3：暂停

	int             GlobalTaskNumber;       // 当前车辆正在执行的全局任务编号，-1表示没有全局任务

	unsigned char	Fuel;	        	// 油量，0-100
	unsigned char	Shift;		        // 档位，值域：{PARK=0,BACKWARD=1,NEURAL=2,FORWARD=3}	
	unsigned short	Throttle;	        // 油门，值域：0~65536，对应实际的油门相对总行程的比例
	unsigned char	Brake;     	        // 刹车，值域：0(zero)~100(full)
	short 		Steer;                  // 前轮转角，值域：-3000 ~  +3000，单位：0.01°，左转为正
	unsigned char   NavMode;           	// DIRECT_ACTUATOR=0,REMOTE_PILOT=1,AUTO_PILOT=2;
	unsigned char   LightState[4];          // 车灯状态，0-3分别为左前车灯，右前车灯，左后车灯，右后车灯，值域：{0：关闭，1：打开}
	
	int		SpeedX;     	        // 横向速度，单位：cm/s
	int		SpeedY;     	        // 纵向速度，单位：cm/s
	int		SpeedZ;     	        // 上下速度，单位：cm/s
	int		GaussX;	        	// 位置：X坐标，单位：cm，向东为正
	int		GaussY;	        	// 位置：Y坐标，单位：cm，向北为正
	int             Height;                 // 高度，向上为正，单位厘米
	int		Heading;	        // 姿态：航向角，单位：0.01°,向东为零度，逆時针0-36000
	int		Roll;	        	// 姿态：侧滚角，单位：0.01°,向右为零度，逆時针-9000-9000
	int             Pitch;                  // 姿态：俯仰角，单位：0.01°,向前水平为零度，逆時针-9000-9000	

};

// Declare NML format function
extern int PlatformStatusFormat(NMLTYPE, void *, CMS *);

#endif 	// PLATFORMSTATUSN_HH
