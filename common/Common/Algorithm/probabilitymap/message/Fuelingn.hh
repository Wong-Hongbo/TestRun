/*
Fuelingn.hh

This C++ header file defines the NML Messages used for command and status by FUELING_MODULE
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:52 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef FUELINGN_HH
#define FUELINGN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "usertype.hh"
// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define FUELING_STATUS_TYPE 17000
#define FUELING_CLEAN_FUEL_TYPE 17001
#define FUELING_HALT_TYPE 17002
#define FUELING_INIT_TYPE 17003
#define FUELING_SET_FUEL_TYPE 17004
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

class FUEL_STATUS {
public:
    unsigned int	timeflag;
    CMD_SOURCE	    source;
    unsigned int	confidence;
    double			desired;
    double			actual;
    LOGIC_STATE     is_initialized;
    LOGIC_STATE     motor_enable_switch;
    LOGIC_STATE     motor_status;
    LOGIC_STATE     CAN_status;
};

// Define the NML Message Classes

// Status Class
class FUELING_STATUS : public RCS_STAT_MSG
{
public:

	// Normal Constructor
	FUELING_STATUS();

	// Constructor used by derived classes
	FUELING_STATUS(NMLTYPE t, size_t s) :  RCS_STAT_MSG(t,s) {};

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    FUEL_STATUS status_data;
};

// Command Classes

class FUELING_CLEAN_FUEL : public RCS_CMD_MSG
{
public:

	//Constructor
	FUELING_CLEAN_FUEL();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    CMD_SOURCE	    source;
    double desired_fuel;
};

class FUELING_HALT : public RCS_CMD_MSG
{
public:

	//Constructor
	FUELING_HALT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    CMD_SOURCE	    source;
    double desired_fuel;
};

class FUELING_INIT : public RCS_CMD_MSG
{
public:

	//Constructor
	FUELING_INIT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    CMD_SOURCE	    source;
    double desired_fuel;
};

class FUELING_SET_FUEL : public RCS_CMD_MSG
{
public:

	//Constructor
	FUELING_SET_FUEL();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    CMD_SOURCE	    source;
    double desired_fuel;
};

// Declare NML format function
extern int FuelingFormat(NMLTYPE, void *, CMS *);

#endif 	// FUELINGN_HH
