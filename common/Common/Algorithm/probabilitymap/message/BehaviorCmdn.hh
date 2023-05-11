/*
BehaviorCmdn.hh
This C++ header file defines the NML Messages for BEHAVIORCMD
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.


*/

// Prevent Multiple Inclusion
#ifndef BEHAVIORCMDN_HH
#define BEHAVIORCMDN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include    "GlobalPositionInfon.hh"
#include "NMLmsgExn.hh"

// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define BEHAVIORCMD_MSG_TYPE 1204
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

// Define the NML Message Classes

class BEHAVIORCMD_MSG : public NMLmsgEx
{
public:

	//Constructor
    BEHAVIORCMD_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
};

// Declare NML format function
extern int BehaviorCmdFormat(NMLTYPE, void *, CMS *);

#endif 	// BEHAVIORCMDN_HH
