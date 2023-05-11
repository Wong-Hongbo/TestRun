/*
SpeedStatesn.hh

This C++ header file defines the NML Messages for SPEEDSTATES
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:53 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef SPEEDSTATESN_HH
#define SPEEDSTATESN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "usertype.hh"
// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define SPEEDSTATES_MSG_TYPE 148000
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

typedef struct {
    UINT32      	timeflag;
    LOGIC_STATE     is_speed_control;
    CMD_SOURCE	    source;
    UINT32      	confidence;
    double			desired_speed;
    double			actual_speed;
    double			desired_acc;
    double			actual_acc;
    TRANS_POSITION  desired_trans_pos;
    double          desired_fuel;
    double          desired_brake;
    TRANS_POSITION  actual_trans_pos;
    double          actual_fuel;
    double          actual_brake;
    INT32           emergency_flag;
    INT32          hardswitch_on;
}SPEED_DATA;

class SPEEDSTATES_MSG : public NMLmsg
{
public:

	//Constructor
	SPEEDSTATES_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    SPEED_DATA  data;
};

// Declare NML format function
extern int SpeedStatesFormat(NMLTYPE, void *, CMS *);

#endif 	// SPEEDSTATESN_HH
