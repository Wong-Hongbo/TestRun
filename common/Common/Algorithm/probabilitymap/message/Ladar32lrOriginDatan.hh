/*
Ladar64OriginDatan.hh

This C++ header file defines the NML Messages for HDLadarData
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef LADAR32lrORIGINDATAN_HH
#define LADAR32lrORIGINDATAN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"


// Define the integer type ids.
#define LADAR32LRORIGINDATA_MSG_TYPE 11014
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.
// Define the NML Message Classes
class LADAR32LRORIGINDATA_MSG : public NMLmsgEx
{
public:
    //Constructor
    LADAR32LRORIGINDATA_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    double LocalPoseTime[2];
    double GlobalPosTime[2];

    int RecFlag1;
    int RecFlag2; // 1-left, 2-right. 3-left and right

    char Ladar32LeftData[PACKETNUM_LADAR32][1206];
    char Ladar32RightData[PACKETNUM_LADAR32][1206];

    int  LeftPacketNum;
    int  RightPacketNum;
    PositionData PositionLeft;
    PositionData PositionRight;
};

// Declare NML format function
extern int Ladar32LROriginDataFormat(NMLTYPE, void *, CMS *);

#endif 	// HDLADARDATAN_HH

