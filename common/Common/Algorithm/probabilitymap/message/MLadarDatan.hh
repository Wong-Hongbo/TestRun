/*
MLadarDatan.hh

This C++ header file defines the NML Messages for MLadarData
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef MLADARDATAN_HH
#define MLADARDATAN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"

// Define the integer type ids.
#define MLADARDATA_MSG_TYPE 11004

#define MAX_LADAR_POINTS 2200
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.
typedef struct
{
    INT16 x;          // cm
    INT16 y;
    INT16 z;
    INT16 Intensity;  // 0-255
//    INT16 length;     // unit: cm
    UINT16 angleH;    // 0.01degree
    UINT16 angleV;    // 0.01degree
}CPointInfo;

typedef struct
{
    CPointInfo line[32];
}CLineInfo;

// Define the NML Message Classes
class MLADARDATA_MSG : public NMLmsgEx
{
public:
	//Constructor
	MLADARDATA_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    double LocalPoseTime[2];
    double GlobalPosTime[2];

    PositionData PositionL;
    PositionData PositionR;

    INT32     TimeDiff;
    CLineInfo PointInfoL[MAX_LADAR_POINTS];
    UINT32    TotalNumberL;
    CLineInfo PointInfoR[MAX_LADAR_POINTS];
    UINT32    TotalNumberR;
    UINT8     WorkflagL;
    UINT8     WorkflagR;
};

// Declare NML format function
extern int MLadarDataFormat(NMLTYPE, void *, CMS *);

#endif 	// MLADARDATAN_HH
