/*
IRRoadInfon.hh

This C++ header file defines the NML Messages for IRROADINFO
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:04 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef IRROADINFON_HH
#define IRROADINFON_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"

// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define IRROADINFO_MSG_TYPE 5303
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

struct IRROAD_OBJ
{
    INT32 RoadID;
    INT32 LeftPtNum;
    INT32 RightPtNum;
    INT32 LeftEdgePtX[50];
    INT32 LeftEdgePtY[50];
    INT32 RightEdgePtX[50];
    INT32 RightEdgePtY[50];
    // format
};
// Define the NML Message Classes

class IRROADINFO_MSG : public NMLmsgEx
{
public:

	//Constructor
    IRROADINFO_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 IRRoadNum;                             // road num
    IRROAD_OBJ IRRoadObj[MAX_IRROAD_OBJ];

};

// Declare NML format function
extern int IRRoadInfoFormat(NMLTYPE, void *, CMS *);

#endif 	// IRROADINFON_HH
