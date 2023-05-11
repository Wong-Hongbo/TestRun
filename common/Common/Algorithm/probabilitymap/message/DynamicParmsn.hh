/*
DynamicParmsn.hh

This C++ header file defines the NML Messages for DYNAMICPARMS
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:52 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef DYNAMICPARMSN_HH
#define DYNAMICPARMSN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions

// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define DYNAMICPARMS_MSG_TYPE 146000
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

class DYNAMICPARMS_MSG : public NMLmsg
{
public:

	//Constructor
	DYNAMICPARMS_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Declare NML format function
extern int DynamicParmsFormat(NMLTYPE, void *, CMS *);

#endif 	// DYNAMICPARMSN_HH
