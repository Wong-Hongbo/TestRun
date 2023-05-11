/*
PlanningCmdn.hh
This C++ header file defines the NML Messages for PLANNINGCMD
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.


*/

// Prevent Multiple Inclusion
#ifndef PLANNINGCMDN_HH
#define PLANNINGCMDN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include    "GlobalPositionInfon.hh"
#include    "LocalPathInfon.hh"
#include "NMLmsgExn.hh"

// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define PLANNINGCMD_MSG_TYPE 170400
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

// Define the NML Message Classes

class PLANNINGCMD_MSG : public NMLmsgEx
{
public:

	//Constructor
    PLANNINGCMD_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    int MessageID;
   // int MessageSeqNum;
    int TimeFlag;
    // inert coordinate
    PositionData Position;
};

// Declare NML format function
extern int PlanningCmdFormat(NMLTYPE, void *, CMS *);

#endif 	// PLANNINGCMDN_HH
