/*
SafetyModelingn.hh

This C++ header file defines the NML Messages used for command and status by SAFETYMODELING_MODULE
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:53 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef SAFETYMODELINGN_HH
#define SAFETYMODELINGN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions

// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define SAFETYMODELING_STATUS_TYPE 13000
#define SAFETYMODELING_HALT_TYPE 13001
#define SAFETYMODELING_INIT_TYPE 13002
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class SAFETYMODELING_STATUS : public RCS_STAT_MSG
{
public:

	// Normal Constructor
	SAFETYMODELING_STATUS();

	// Constructor used by derived classes
	SAFETYMODELING_STATUS(NMLTYPE t, size_t s) :  RCS_STAT_MSG(t,s) {};

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Command Classes

class SAFETYMODELING_HALT : public RCS_CMD_MSG
{
public:

	//Constructor
	SAFETYMODELING_HALT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

class SAFETYMODELING_INIT : public RCS_CMD_MSG
{
public:

	//Constructor
	SAFETYMODELING_INIT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Declare NML format function
extern int SafetyModelingFormat(NMLTYPE, void *, CMS *);

#endif 	// SAFETYMODELINGN_HH
