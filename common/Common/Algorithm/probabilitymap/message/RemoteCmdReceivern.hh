/*
RemoteCmdReceivern.hh

This C++ header file defines the NML Messages used for command and status by REMOTECMDRECEIVER_MODULE
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:53 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef REMOTECMDRECEIVERN_HH
#define REMOTECMDRECEIVERN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions

// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define REMOTECMDRECEIVER_STATUS_TYPE 9000
#define REMOTECMDRECEIVER_HALT_TYPE 9001
#define REMOTECMDRECEIVER_INIT_TYPE 9002
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class REMOTECMDRECEIVER_STATUS : public RCS_STAT_MSG
{
public:

	// Normal Constructor
	REMOTECMDRECEIVER_STATUS();

	// Constructor used by derived classes
	REMOTECMDRECEIVER_STATUS(NMLTYPE t, size_t s) :  RCS_STAT_MSG(t,s) {};

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Command Classes

class REMOTECMDRECEIVER_HALT : public RCS_CMD_MSG
{
public:

	//Constructor
	REMOTECMDRECEIVER_HALT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

class REMOTECMDRECEIVER_INIT : public RCS_CMD_MSG
{
public:

	//Constructor
	REMOTECMDRECEIVER_INIT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Declare NML format function
extern int RemoteCmdReceiverFormat(NMLTYPE, void *, CMS *);

#endif 	// REMOTECMDRECEIVERN_HH
