/*
RoadSceneModelingn.hh

This C++ header file defines the NML Messages used for command and status by ROADSCENEMODELING_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:02 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef ROADSCENEMODELINGN_HH
#define ROADSCENEMODELINGN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define ROADSCENEMODELING_STATUS_TYPE 2402
#define ROADSCENEMODELING_CMD_TYPE 2401
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class ROADSCENEMODELING_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
	ROADSCENEMODELING_STATUS();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 RSModelingState;

};

// Command Classes

class ROADSCENEMODELING_CMD : public NMLmsgEx
{
public:

	//Constructor
	ROADSCENEMODELING_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 RSModelingCommand;

};

// Declare NML format function
extern int RoadSceneModelingFormat(NMLTYPE, void *, CMS *);

#endif 	// ROADSCENEMODELINGN_HH
