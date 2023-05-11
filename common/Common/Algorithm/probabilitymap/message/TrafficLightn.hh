/*
TrafficLightn.hh

This C++ header file defines the NML Messages for TRAFFICLIGHTINFO
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:04 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef TRAFFICLIGHTN_HH
#define TRAFFICLIGHTN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"


// Define the integer type ids.
#define TRAFFICLIGHT_MSG_TYPE 9203
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

//////交通灯
typedef struct{
	unsigned char           pos_flag;
	int                     x;
	int                     y;
	unsigned char           forward_light_type;  /*直行灯信息*/
  	unsigned char           left_light_type;     /*左转灯信息*/
  	unsigned char           right_light_type;    /*右转灯信息*/
  	unsigned char           uturn_light_type;    /*U Turn灯信息*/
}TRAFFIC_LIGHT;

/* 
pos_flag
  0: 没有位置信息
  1: 位置信息有效

light_type
  0: 没有红绿灯
  1：红灯
  2：黄灯
  3：绿灯
*/

// Define the NML Message Classes

class TRAFFICLIGHT_MSG : public NMLmsgEx
{
public:

    //Constructor
    TRAFFICLIGHT_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.

    int TrafficLightNum;      //给最近的1个      
    TRAFFIC_LIGHT TrafficLight;
};

// Declare NML format function
extern int TrafficLightFormat(NMLTYPE, void *, CMS *);

#endif 	// TRAFFICLIGHTN_HH
