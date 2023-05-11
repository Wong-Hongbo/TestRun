/*
PositiveObstaclen.hh

This C++ header file defines the NML Messages for POSITIVEOBSTACLE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef POSITIVEOBSTACLEN_HH
#define POSITIVEOBSTACLEN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions

#include "NMLmsgExn.hh"
// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.
#include "CommonDefinitionX.hh"
#include "GlobalPositionInfon.hh"

// Define the integer type ids.
#define POSITIVEOBSTACLE_MSG_TYPE 4103
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

// Define the NML Message Classes

class POSITIVEOBSTACLE_MSG : public NMLmsgEx
{
public:
    //Constructor
    POSITIVEOBSTACLE_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    INT32 MapWidth;
    INT32 MapHeight;
    INT32 GridWidth;
    INT32 GridHeight;
    INT32 VehicleGridX;
    INT32 VehicleGridY;

    UINT8 PositiveObstacle[POSITIVEOBMAP_SIZE];   // 20cm*20cm, forward 50m, back 15m, left and right 15m
                                                  // grid property: 0 (traversable),1 (positive ob), 2(negative ob),
                                                  // 3(unknown),4(cliff),5(occlusion),6(water)
};

// Declare NML format function
extern int PositiveObstacleFormat(NMLTYPE, void *, CMS *);

#endif 	// POSITIVEOBSTACLEN_HH
