/*
LocalThDModelingn.hh

This C++ header file defines the NML Messages used for command and status by LOCALTHDMODELING_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:02 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef LOCALTHDMODELINGN_HH
#define LOCALTHDMODELINGN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

#define LOCALTHDMODELING_STATUS_TYPE 5402
#define LOCALTHDMODELING_CMD_TYPE    5401
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class LOCALTHDMODELING_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
	LOCALTHDMODELING_STATUS();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 LThDModelingState;

};

// Command Classes

class LOCALTHDMODELING_CMD : public NMLmsgEx
{
public:

	//Constructor
	LOCALTHDMODELING_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 LThDModelingCommand;

};

// Declare NML format function
extern int LocalThDModelingFormat(NMLTYPE, void *, CMS *);

#endif 	// LOCALTHDMODELINGN_HH
