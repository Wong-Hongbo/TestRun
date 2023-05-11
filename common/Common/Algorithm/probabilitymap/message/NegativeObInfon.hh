/*
NegativeObInfon.hh

This C++ header file defines the NML Messages for NEGATIVEOBINFO
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef NEGATIVEOBINFON_HH
#define NEGATIVEOBINFON_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"

// Define the integer type ids.
#define NEGATIVEOBINFO_MSG_TYPE 5203
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.
#define MAX_NEGOBSTACLE_OBJ 20
// 负障碍
struct NEGATIVE_OBSTACLE_OBJ
{
    INT32 NegObID;
    INT32 CenterX;    //cm为单位，惯导坐标系
    INT32 CenterY;
    INT32 AxisX;      //长轴，cm为单位
    INT32 AxisY;      //短轴，cm为单位
    INT32 Slope;      // 斜率，0.01度，逆时针为正
};
// Define the NML Message Classes

class NEGATIVEOBINFO_MSG : public NMLmsgEx
{
public:

	//Constructor
	NEGATIVEOBINFO_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    int NegObNum;
    NEGATIVE_OBSTACLE_OBJ NegObstacle[MAX_NEGOBSTACLE_OBJ];

    INT32 MapWidth;
    INT32 MapHeight;
    INT32 GridWidth;
    INT32 GridHeight;
    INT32 VehicleGridX;
    INT32 VehicleGridY;
    UINT8 LocalObsMap[325*150];  // 20cm*20cm, forward 50m, back 15m, left and right 15m
                                 // grid property: 0 (traversable),1 (positive ob), 2(negative ob),
                                 // 3(unknown),4(cliff),5(occlusion)
};

// Declare NML format function
extern int NegativeObInfoFormat(NMLTYPE, void *, CMS *);

#endif 	// NEGATIVEOBINFON_HH
