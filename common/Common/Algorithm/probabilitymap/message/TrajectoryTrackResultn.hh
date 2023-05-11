/*
TrajectoryTrackResultn.hh

This C++ header file defines the NML Messages for LOCALPATHINFO
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef TRAJECTORYTRACKRESULTN_HH
#define TRAJECTORYTRACKRESULTN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"


// Define the integer type ids.
#define TRAJECTORYTRACKRESULT_MSG_TYPE 1801
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

//#ifndef _VEHICLE_COMMAND_
//#define _VEHICLE_COMMAND_
//typedef enum{
//    ES=200,ST,AD_SPEED,AD_DISTANCE,AD_POINT,BK_SPEED,BK_POINT,IG,FO
//}VEHICLE_COMMAND;
//#endif

// Define the NML Message Classes

typedef struct
{
    float expVelocity;
    float expCurvature;
    int vehicle_command;

}TrackResult;

class TRAJECTORYTRACKRESULT_MSG : public NMLmsgEx
{
public:

    //Constructor
    TRAJECTORYTRACKRESULT_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    TrackResult trackResult;
};

// Declare NML format function
extern int TrajectoryTrackResultFormat(NMLTYPE, void *, CMS *);

#endif 	// TRAJECTORYtRACKrESULTN_H
