/*
SyncLidarDatan.hh

This C++ header file defines the NML Messages for SyncLadarData
Template Version 1.1

MODIFICATIONS:
Sun April 12 00:22:05 CST 2020	Created by MDY.

*/

// Prevent Multiple Inclusion
#ifndef SYNCLIDARDATAN_HH
#define SYNCLIDARDATAN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"
#include "LocalPosen.hh"
#include "HDLadarDatan.hh"

// Define the integer type ids.
// AA-BB-CC
// AA:10-preception, BB:00-raw data(01-sensor status), 01-SyncLidarData
#define  SYNCLIDARDATA_MSG_TYPE 100101
#define  SENSOR_MAX_NUM 16


typedef enum {
    MiddleMiddleTop = 0,
    MiddleFrontBottom = 1,
    RightFrontBottom = 2,
    RightMiddleBottom = 3,
    RightRearBottom = 4,
    MiddleRearBottom = 5,
    LeftRearBottom = 6,
    LeftMiddleBottom = 7,
    LeftFrontBottom = 8,
    MiddleFrontMiddle = 9,
    RightFrontMiddle = 10,
    RightMiddleMiddle = 11,
    RightRearMiddle = 12,
    MiddleRearMiddle = 13,
    LeftRearMiddle = 14,
    LeftMiddleMiddle = 15,
    LeftFrontMiddle = 16,
    MiddleFrontTop = 17,
    RightFrontTop = 18,
    RightMiddleTop = 19,
    RightRearTop = 20,
    MiddleRearTop = 21,
    LeftRearTop = 22,
    LeftMiddleTop = 23,
    LeftFrontTop = 24
} LidarPosition;

// Define the NML Message Classes
class SYNCLIDARDATA_MSG : public NMLmsgEx
{
public:
    //Constructor
    SYNCLIDARDATA_MSG();

    // CMS Update Function
    void update(CMS *);

    PointCoordinate64 lidarData[SENSOR_MAX_NUM*16][PACKETNUM*6];
    short ringNumList[SENSOR_MAX_NUM];

    /*
     * 0:MiddleMiddleTop
     *
     * 安装下部四周
     * 1:MiddleFrontBottom    2:RightFrontBottom    3:RightMiddleBottom    4:RightRearBottom
     * 5:MiddleRearBottom     6:LeftRearBottom      7:LeftMiddleBottom     8:LeftFrontBottom
     *
     * 安装中部四周
     * 9:MiddleFrontMiddle   10:RightFrontMiddle   11:RightMiddleMiddle   12:RightRearMiddle
     *13:MiddleRearMiddle    14:LeftRearMiddle     15:LeftMiddleMiddle    16:LeftFrontMiddle
     *
     * 安装顶部四周
     *17:MiddleFrontTop      18:RightFrontTop      19:RightMiddleTop      20:RightRearTop
     *21:MiddleRearTop       22:LeftRearTop        23:LeftMiddleTop       24:LeftFrontTop
     */
    short sensorPositionList[SENSOR_MAX_NUM];
    short sensorNum;

    double startTime;   //ms
    double endTime;     //ms
};

// Declare NML format function
extern int SyncLidarDataFormat(NMLTYPE, void *, CMS *);

#endif 	// SYNCLIDARDATAN_HH
