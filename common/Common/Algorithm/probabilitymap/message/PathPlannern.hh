/*
PathPlannern.hh

This C++ header file defines the NML Messages used for command and status by PATHPLANNER_MODULE
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:53 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef PATHPLANNERN_HH
#define PATHPLANNERN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions

// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define PATHPLANNER_STATUS_TYPE 4000
#define PATHPLANNER_HALT_TYPE 4001
#define PATHPLANNER_INIT_TYPE 4002
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class PATHPLANNER_STATUS : public RCS_STAT_MSG
{
public:

	// Normal Constructor
	PATHPLANNER_STATUS();

	// Constructor used by derived classes
	PATHPLANNER_STATUS(NMLTYPE t, size_t s) :  RCS_STAT_MSG(t,s) {};

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Command Classes

class PATHPLANNER_HALT : public RCS_CMD_MSG
{
public:

	//Constructor
	PATHPLANNER_HALT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

class PATHPLANNER_INIT : public RCS_CMD_MSG
{
public:

	//Constructor
	PATHPLANNER_INIT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Declare NML format function
extern int PathPlannerFormat(NMLTYPE, void *, CMS *);

#endif 	// PATHPLANNERN_HH
