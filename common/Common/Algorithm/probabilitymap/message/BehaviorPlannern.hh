/*
BEHAVIORPLANNERN.hh

This C++ header file defines the NML Messages used for command and status by BEHAVIORPLANNER_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:01 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef BEHAVIORPLANNERN_HH
#define BEHAVIORPLANNERN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define BEHAVIORPLANNER_STATUS_TYPE 1202
#define BEHAVIORPLANNER_CMD_TYPE 1201
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class BEHAVIORPLANNER_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
	BEHAVIORPLANNER_STATUS();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 BPlanState;

};

// Command Classes

class BEHAVIORPLANNER_CMD : public NMLmsgEx
{
public:

	//Constructor
	BEHAVIORPLANNER_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 BPlanCommand;

};

// Declare NML format function
extern int BehaviorPlannerFormat(NMLTYPE, void *, CMS *);

#endif 	// BEHAVIORPLANNERN_HH
