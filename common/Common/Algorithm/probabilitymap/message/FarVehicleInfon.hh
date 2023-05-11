/*
FarVehicleInfo.hh

This C++ header file defines the NML Messages for FARVEHICLEDETECTIONINFO
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:04 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef FARVEHICLEINFO_HH
#define FARVEHICLEINFO_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"
#include "VehicleInfon.hh"
#include "LaneMarkingInfon.hh"

// Define the integer type ids.
#define FARVEHICLEINFO_MSG_TYPE 2503
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

#define MAX_FARVEHICLE_OBJ 100
#define MAX_FAR_LANE_OBJ   10

// Define the NML Message Classes

class FARVEHICLEINFO_MSG : public NMLmsgEx
{
public:

    //Constructor
    FARVEHICLEINFO_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    INT32 VehicleNum;
    VEHICLE_OBJ VehicleObj[MAX_FARVEHICLE_OBJ];
    //0: unknown, 1: vehicle,      2: pedestrain,
    //3: bicycle, 4: small object, 5: big object, 6: traffic cones
    INT32 VehicleAttr[MAX_FARVEHICLE_OBJ];
    INT32 LaneNum;                             // lane num
    LANE_OBJ LaneObj[MAX_FAR_LANE_OBJ];
    INT32 CurrentLane;
};

// Declare NML format function
extern int FarVehicleInfoFormat(NMLTYPE, void *, CMS *);

#endif 	// FARVEHICLEINFO_HH
