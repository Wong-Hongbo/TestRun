/*
Transingn.hh

This C++ header file defines the NML Messages used for command and status by TRANSING_MODULE
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:53 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef TRANSINGN_HH
#define TRANSINGN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions

// Predefined type files
#include "usertype.hh"


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define TRANSING_STATUS_TYPE 19000
#define TRANSING_NEURAL_TYPE 19001
#define TRANSING_BACKWARD_TYPE 19002
#define TRANSING_FORWARD_TYPE 19003
#define TRANSING_HALT_TYPE 19004
#define TRANSING_INIT_TYPE 19005
#define TRANSING_PARK_TYPE 19006
#define TRANSING_MOTORINIT_TYPE 19007
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

class TRANS_STATUS {
public:
    UINT32            timeflag;
    CMD_SOURCE        source;
    UINT32            confidence;
    TRANS_POSITION    desired_trans;
    TRANS_POSITION    actual_trans;
    LOGIC_STATE       is_initialized;
    LOGIC_STATE       motor_enable_switch;
    LOGIC_STATE       motor_status;
    LOGIC_STATE       CAN_status;
};

// Define the NML Message Classes

// Status Class
class TRANSING_STATUS : public RCS_STAT_MSG
{
public:

	// Normal Constructor
	TRANSING_STATUS();

	// Constructor used by derived classes
	TRANSING_STATUS(NMLTYPE t, size_t s) :  RCS_STAT_MSG(t,s) {};

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    TRANS_STATUS data;
};

// Command Classes

class TRANSING_MOTORINIT : public RCS_CMD_MSG
{
public:

    //Constructor
    TRANSING_MOTORINIT();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    CMD_SOURCE        source;
};

class TRANSING_NEURAL : public RCS_CMD_MSG
{
public:

	//Constructor
	TRANSING_NEURAL();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    CMD_SOURCE        source;
};

class TRANSING_BACKWARD : public RCS_CMD_MSG
{
public:

	//Constructor
	TRANSING_BACKWARD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    CMD_SOURCE        source;

};

class TRANSING_FORWARD : public RCS_CMD_MSG
{
public:

	//Constructor
	TRANSING_FORWARD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
   CMD_SOURCE        source;
};

class TRANSING_HALT : public RCS_CMD_MSG
{
public:

	//Constructor
	TRANSING_HALT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
   CMD_SOURCE        source;
};

class TRANSING_INIT : public RCS_CMD_MSG
{
public:

	//Constructor
	TRANSING_INIT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
   CMD_SOURCE        source;
};

class TRANSING_PARK : public RCS_CMD_MSG
{
public:

	//Constructor
	TRANSING_PARK();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
   CMD_SOURCE        source;
};

// Declare NML format function
extern int TransingFormat(NMLTYPE, void *, CMS *);

#endif 	// TRANSINGN_HH
