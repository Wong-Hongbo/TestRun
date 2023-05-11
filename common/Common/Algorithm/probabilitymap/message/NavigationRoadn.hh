/*
NAVIGATIONROADn.hh

This C++ header file defines the NML Messages for NAVIGATIONROAD
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:04 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef NAVIGATIONROADN_HH
#define NAVIGATIONROADN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.
#include "NMLmsgExn.hh"


// Define the integer type ids.
#define NAVIGATIONROAD_MSG_TYPE 1203
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

#define MAX_LANE_NUM (10)
#define MAX_SEGMENT_NUM (50)

struct RoadSegment
{
    int     id;                       //road_id
    int     lane[MAX_LANE_NUM];       //该road下所有的lane
    int     reverse;
};

class NAVIGATIONROAD_MSG : public NMLmsgEx
{
public:

    //Constructor
    NAVIGATIONROAD_MSG();

    // CMS Update Function
    void update(CMS *);

    int segment_num;
    RoadSegment   road_segment[MAX_SEGMENT_NUM];

};

// Declare NML format function
extern int NavigationRoadformat(NMLTYPE, void *, CMS *);

#endif 	// NAVIGATIONROADN_HH
