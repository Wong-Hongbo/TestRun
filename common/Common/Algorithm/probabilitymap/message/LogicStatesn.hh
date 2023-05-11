/*
LogicStatesn.hh

This C++ header file defines the NML Messages for LOGICSTATES
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:53 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef LOGICSTATESN_HH
#define LOGICSTATESN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "usertype.hh"
// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define LOGICSTATES_MSG_TYPE 150000
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

class LOGICSTATES_MSG : public NMLmsg
{
public:

	//Constructor
	LOGICSTATES_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    bool head_light;
    bool left_turn_light;
    bool right_turn_light;
    bool horn;
    bool engine;
    bool power;
    bool starter;
    bool headlight;
    bool alternator;
    bool handbrake;

};

// Declare NML format function
extern int LogicStatesFormat(NMLTYPE, void *, CMS *);

#endif 	// LOGICSTATESN_HH
