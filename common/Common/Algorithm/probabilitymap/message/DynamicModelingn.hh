/*
DynamicModelingn.hh

This C++ header file defines the NML Messages used for command and status by DYNAMICMODELING_MODULE
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:52 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef DYNAMICMODELINGN_HH
#define DYNAMICMODELINGN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions

// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define DYNAMICMODELING_STATUS_TYPE 11000
#define DYNAMICMODELING_HALT_TYPE 11001
#define DYNAMICMODELING_INIT_TYPE 11002
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class DYNAMICMODELING_STATUS : public RCS_STAT_MSG
{
public:

	// Normal Constructor
	DYNAMICMODELING_STATUS();

	// Constructor used by derived classes
	DYNAMICMODELING_STATUS(NMLTYPE t, size_t s) :  RCS_STAT_MSG(t,s) {};

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Command Classes

class DYNAMICMODELING_HALT : public RCS_CMD_MSG
{
public:

	//Constructor
	DYNAMICMODELING_HALT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

class DYNAMICMODELING_INIT : public RCS_CMD_MSG
{
public:

	//Constructor
	DYNAMICMODELING_INIT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Declare NML format function
extern int DynamicModelingFormat(NMLTYPE, void *, CMS *);

#endif 	// DYNAMICMODELINGN_HH
