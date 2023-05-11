/*
Brakingn.hh

This C++ header file defines the NML Messages used for command and status by BRAKING_MODULE
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:50 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef BRAKINGN_HH
#define BRAKINGN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "usertype.hh"
// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define BRAKING_STATUS_TYPE 18000
#define BRAKING_CLEAN_BRAKE_TYPE 18001
#define BRAKING_EMERGENCY_BRAKE_TYPE 18002
#define BRAKING_HALT_TYPE 18003
#define BRAKING_INIT_TYPE 18004
#define BRAKING_SET_BRAKE_TYPE 18005
#define BRAKING_MOTORINIT_TYPE 18006
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.



class BRAKE_STATUS {
public:
    unsigned int      timeflag;
    CMD_SOURCE        source;
    double            desired_brake;
    unsigned int      confidence;
    double            expected;
    double            actual_brake;
    int               emergency_flag;
    LOGIC_STATE       brake_light;
    LOGIC_STATE       is_initialized;
    LOGIC_STATE       motor_enable_switch;
    LOGIC_STATE       motor_status;
    LOGIC_STATE       CAN_status;
};

// Define the NML Message Classes

// Status Class
class BRAKING_STATUS : public RCS_STAT_MSG
{
public:

	// Normal Constructor
	BRAKING_STATUS();

	// Constructor used by derived classes
	BRAKING_STATUS(NMLTYPE t, size_t s) :  RCS_STAT_MSG(t,s) {};

	// CMS Update Function
    void update(CMS *);

	// Place custom variables here.
    BRAKE_STATUS status_data;

};



// Command Classes

class BRAKING_MOTORINIT : public RCS_CMD_MSG
{
public:

    //Constructor
    BRAKING_MOTORINIT();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    CMD_SOURCE        source;
};

class BRAKING_CLEAN_BRAKE : public RCS_CMD_MSG
{
public:

	//Constructor
	BRAKING_CLEAN_BRAKE();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    UINT32  MessageCount;
    double desired_brake;
    CMD_SOURCE	    source;


};

class BRAKING_EMERGENCY_BRAKE : public RCS_CMD_MSG
{
public:

	//Constructor
	BRAKING_EMERGENCY_BRAKE();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    double desired_brake;
    CMD_SOURCE	    source;
};

class BRAKING_HALT : public RCS_CMD_MSG
{
public:

	//Constructor
	BRAKING_HALT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    double desired_brake;
    CMD_SOURCE	    source;
};

class BRAKING_INIT : public RCS_CMD_MSG
{
public:

	//Constructor
	BRAKING_INIT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    double desired_brake;
    CMD_SOURCE	    source;

};

class BRAKING_SET_BRAKE : public RCS_CMD_MSG
{
public:

	//Constructor
	BRAKING_SET_BRAKE();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    double      desired_brake;
    CMD_SOURCE	source;
};

// Declare NML format function
extern int BrakingFormat(NMLTYPE, void *, CMS *);

#endif 	// BRAKINGN_HH
