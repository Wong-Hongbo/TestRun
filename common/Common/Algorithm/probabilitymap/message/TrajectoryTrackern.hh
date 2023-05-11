/*
TrajectoryTrackern.hh

This C++ header file defines the NML Messages used for command and status by TRAJECTORYTRACKER_MODULE
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:53 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef TRAJECTORYTRACKERN_HH
#define TRAJECTORYTRACKERN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "usertype.hh"
// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define TRAJECTORYTRACKER_STATUS_TYPE 6000
#define TRAJECTORYTRACKER_GOBYPATH_TYPE 6001
#define TRAJECTORYTRACKER_HALT_TYPE 6002
#define TRAJECTORYTRACKER_INIT_TYPE 6003
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.



// Define the NML Message Classes
typedef struct{
    UINT32            timeflag;
    CMD_SOURCE        source;
}TRACKING_STATUS_DATA;


// Status Class
class TRAJECTORYTRACKER_STATUS : public RCS_STAT_MSG
{
public:

	// Normal Constructor
	TRAJECTORYTRACKER_STATUS();

	// Constructor used by derived classes
	TRAJECTORYTRACKER_STATUS(NMLTYPE t, size_t s) :  RCS_STAT_MSG(t,s) {};

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    TRACKING_STATUS_DATA status_data;
};

// Command Classes

class TRAJECTORYTRACKER_GOBYPATH : public RCS_CMD_MSG
{
public:

	//Constructor
	TRAJECTORYTRACKER_GOBYPATH();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
};

class TRAJECTORYTRACKER_HALT : public RCS_CMD_MSG
{
public:

	//Constructor
	TRAJECTORYTRACKER_HALT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

class TRAJECTORYTRACKER_INIT : public RCS_CMD_MSG
{
public:

	//Constructor
	TRAJECTORYTRACKER_INIT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Declare NML format function
extern int TrajectoryTrackerFormat(NMLTYPE, void *, CMS *);

#endif 	// TRAJECTORYTRACKERN_HH
