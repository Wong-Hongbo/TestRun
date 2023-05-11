/*
RoadCurbEdgen.hh

This C++ header file defines the NML Messages for RoadCurbEdge
Template Version 1.1

MODIFICATIONS:
Wed Feb 26 08:40:05 CST 2014	Created by hutingbo.

*/

// Prevent Multiple Inclusion
#ifndef ROADCURBEDGEN_HH
#define ROADCURBEDGEN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions

// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.
#include "CommonDefinitionX.hh"
#include "GlobalPositionInfon.hh"

// Define the integer type ids.
#define ROADCURBEDGE_MSG_TYPE 4106
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

// Define the NML Message Classes
typedef struct
{
    short x;
    short y;
}POINT_IPC;

typedef struct
{
    long type;
    POINT_IPC point[50];
}LINE_IPC;

class ROADCURBEDGE_MSG : public NMLmsgEx
{
public:

    //Constructor
    ROADCURBEDGE_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    INT32 MessageID;
    //  INT32 MessageSeqNum;
    INT32 TimeFlag;

    PositionData Position;

    short   left_curb_point_num;
    short   right_curb_point_num;
    LINE_IPC left_curb;
    LINE_IPC right_curb;

    short   left_horiz_point_num;
    short   right_horiz_point_num;
    LINE_IPC left_horiz;
    LINE_IPC right_horiz;
};

// Declare NML format function
extern int RoadCurbEdgeFormat(NMLTYPE, void *, CMS *);

#endif 	// ROADCURBEDGEN_HH
