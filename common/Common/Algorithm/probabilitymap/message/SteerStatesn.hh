/*
SteerStatesn.hh

This C++ header file defines the NML Messages for STEERSTATES
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:53 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef STEERSTATESN_HH
#define STEERSTATESN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "usertype.hh"
// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define STEERSTATES_MSG_TYPE 149000
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes
class STEER_STATE {
public:
    unsigned int	timeflag;
    LOGIC_STATE     is_steer_control;
    CMD_SOURCE	    source;
    unsigned int	confidence;
    double			desired_curvature;
    double			actual_curvature;
    double          actual_steer;
    double          desired_steer;
};

class STEERSTATES_MSG : public NMLmsg
{
public:

	//Constructor
	STEERSTATES_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    STEER_STATE data;
};

// Declare NML format function
extern int SteerStatesFormat(NMLTYPE, void *, CMS *);

#endif 	// STEERSTATESN_HH
