/*
LocalPoseSendern.hh

This C++ header file defines the NML Messages used for command and status by LOCALPOSESENDER_MODULE
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:53 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef LOCALPOSESENDERN_HH
#define LOCALPOSESENDERN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions

// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define LOCALPOSESENDER_STATUS_TYPE 8000
#define LOCALPOSESENDER_HALT_TYPE 8001
#define LOCALPOSESENDER_INIT_TYPE 8002
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class LOCALPOSESENDER_STATUS : public RCS_STAT_MSG
{
public:

	// Normal Constructor
	LOCALPOSESENDER_STATUS();

	// Constructor used by derived classes
	LOCALPOSESENDER_STATUS(NMLTYPE t, size_t s) :  RCS_STAT_MSG(t,s) {};

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Command Classes

class LOCALPOSESENDER_HALT : public RCS_CMD_MSG
{
public:

	//Constructor
	LOCALPOSESENDER_HALT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

class LOCALPOSESENDER_INIT : public RCS_CMD_MSG
{
public:

	//Constructor
	LOCALPOSESENDER_INIT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Declare NML format function
extern int LocalPoseSenderFormat(NMLTYPE, void *, CMS *);

#endif 	// LOCALPOSESENDERN_HH
