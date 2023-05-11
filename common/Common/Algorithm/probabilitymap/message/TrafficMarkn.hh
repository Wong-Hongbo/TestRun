/*
TrafficMarkn.hh

This C++ header file defines the NML Messages for TRAFFICMARKINFO
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:04 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef TRAFFICMARKN_HH
#define TRAFFICMARKN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"


// Define the integer type ids.
#define TRAFFICMARK_MSG_TYPE 9303
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.
#define MAX_ROAD_MARKING 10

// 车体坐标系： x右，y前，z上
typedef struct{
	int                     bottomleftx;            // x坐标，厘米为单位,全局坐标系
	int                     bottomlefty;            // y坐标，厘米为单位,全局坐标系
	int                     bottomrightx;           // x坐标，厘米为单位,全局坐标系
	int                     bottomrighty;           // y坐标，厘米为单位,全局坐标系
	int                     topleftx;               // x坐标，厘米为单位,全局坐标系
	int                     toplefty;               // y坐标，厘米为单位,全局坐标系
	int                     toprightx;              // x坐标，厘米为单位,全局坐标系
	int                     toprighty;              // y坐标，厘米为单位,全局坐标系
    int                     direction;        // 停止线的方向,单位0.01度,与东向的夹角，逆时针0-36000
	int                     width;            // 地标的宽度
	unsigned char           sign_type;        // 地标的类型
}ROAD_MARKING;

/*
     (topleftx,toplefty)   (toprightx,toprighty)
               || || || || || ||  \ 
               || || || || || ||   | -> (width) (type)
               || || || || || ||  /
(bottomleftx,bottomlefty)   (bottomrightx,bottomrighty)
    type = 1,停止线；type = 2, 斑马线；type = 3,直行箭头；type = 4,左转箭头；
    type = 5,右转箭头；type = 6,直行左转箭头；type = 7,直行右转箭头；
    type = 8,左转掉头箭头；type = 9,掉头箭头；type = 10,菱形；type = 11，泊车位；
*/

// Define the NML Message Classes

class TRAFFICMARK_MSG : public NMLmsgEx
{
public:

    //Constructor
    TRAFFICMARK_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    int RoadMarkingNum;      //只给最近的6个地标          
    ROAD_MARKING RoadMarking[MAX_ROAD_MARKING];
};

// Declare NML format function
extern int TrafficMarkFormat(NMLTYPE, void *, CMS *);

#endif 	// TRAFFICMARKN_HH
