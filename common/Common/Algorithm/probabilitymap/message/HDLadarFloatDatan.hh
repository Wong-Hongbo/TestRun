/*
HDLadarDatan.hh

This C++ header file defines the NML Messages for HDLadarData
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef HDLADARFLOATDATAN_HH
#define HDLADARFLOATDATAN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"
#include "LocalPosen.hh"


// Define the integer type ids.
#define HDLADARFLOATDATA_MSG_TYPE 11008
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.
// 64线雷达点，单位厘米
typedef struct
{
    float x;      //车体坐标系, 单位m
    float y;
    float z;
//    int angleH;   //角度，单位为0.01度
//    int angleV;
    unsigned char Intensity;
}PointCoordinateFloat64;


// Define the NML Message Classes
class HDLADARFLOATDATA_MSG : public NMLmsgEx
{
public:
    //Constructor
    HDLADARFLOATDATA_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.

    PointCoordinateFloat64 HDData[64][PACKETNUM*6];
    int ringNum;
    double startTime;   //ms
    double endTime;     //ms
};

// Declare NML format function
extern int HDLadarFloatDataFormat(NMLTYPE, void *, CMS *);

#endif 	// HDLADARFLOATDATAN_HH
