/*
LogicSetingn.hh

This C++ header file defines the NML Messages used for command and status by LOGICSETING_MODULE
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:53 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef LOGICSETINGN_HH
#define LOGICSETINGN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions

// Predefined type files
#include "usertype.hh"


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define LOGICSETING_STATUS_TYPE 20000
#define LOGICSETING_HALT_TYPE 20001
#define LOGICSETING_INIT_TYPE 20003


//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes
class LOGIC_STATUS {
public:
    unsigned int      timeflag;
    LOGIC_STATE       head_light;
    LOGIC_STATE       left_light;
    LOGIC_STATE       right_light;
    LOGIC_STATE       horn;
    LOGIC_STATE       power;
    LOGIC_STATE       engine;
};
// Status Class
class LOGICSETING_STATUS : public RCS_STAT_MSG
{
public:

	// Normal Constructor
	LOGICSETING_STATUS();

	// Constructor used by derived classes
	LOGICSETING_STATUS(NMLTYPE t, size_t s) :  RCS_STAT_MSG(t,s) {};

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    LOGIC_STATUS status_data;
};

// Command Classes

class LOGICSETING_HALT : public RCS_CMD_MSG
{
public:

	//Constructor
    LOGICSETING_HALT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

class LOGICSETING_INIT : public RCS_CMD_MSG
{
public:

	//Constructor
    LOGICSETING_INIT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Declare NML format function
extern int LogicSetingFormat(NMLTYPE, void *, CMS *);

#endif 	// LOGICSETINGN_HH
