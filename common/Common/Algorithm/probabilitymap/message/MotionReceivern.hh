/*
MotionReceivern.hh

This C++ header file defines the NML Messages used for command and status by MOTIONRECEIVER_MODULE
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:53 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef MOTIONRECEIVERN_HH
#define MOTIONRECEIVERN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions

// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define MOTIONRECEIVER_STATUS_TYPE 10000
#define MOTIONRECEIVER_HALT_TYPE 10001
#define MOTIONRECEIVER_INIT_TYPE 10002
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class MOTIONRECEIVER_STATUS : public RCS_STAT_MSG
{
public:

	// Normal Constructor
	MOTIONRECEIVER_STATUS();

	// Constructor used by derived classes
	MOTIONRECEIVER_STATUS(NMLTYPE t, size_t s) :  RCS_STAT_MSG(t,s) {};

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Command Classes

class MOTIONRECEIVER_HALT : public RCS_CMD_MSG
{
public:

	//Constructor
	MOTIONRECEIVER_HALT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

class MOTIONRECEIVER_INIT : public RCS_CMD_MSG
{
public:

	//Constructor
	MOTIONRECEIVER_INIT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Declare NML format function
extern int MotionReceiverFormat(NMLTYPE, void *, CMS *);

#endif 	// MOTIONRECEIVERN_HH
