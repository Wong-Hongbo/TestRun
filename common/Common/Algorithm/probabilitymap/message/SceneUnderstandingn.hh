/*
SceneUnderstandingn.hh

This C++ header file defines the NML Messages used for command and status by SCENEUNDERSTANDING_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:02 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef SCENEUNDERSTANDINGN_HH
#define SCENEUNDERSTANDINGN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define SCENEUNDERSTANDING_STATUS_TYPE 2302
#define SCENEUNDERSTANDING_CMD_TYPE 2301
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class SCENEUNDERSTANDING_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
	SCENEUNDERSTANDING_STATUS();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 SceneUnderstandingState;
};

// Command Classes

class SCENEUNDERSTANDING_CMD : public NMLmsgEx
{
public:

	//Constructor
	SCENEUNDERSTANDING_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 SceneUnderstandingCommand;
};

// Declare NML format function
extern int SceneUnderstandingFormat(NMLTYPE, void *, CMS *);

#endif 	// SCENEUNDERSTANDINGN_HH
