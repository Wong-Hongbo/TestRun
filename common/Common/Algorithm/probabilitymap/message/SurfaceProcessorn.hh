/*
SurfaceProcessorn.hh

This C++ header file defines the NML Messages used for command and status by SURFACEPROCESSOR_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:02 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef SURFACEPROCESSORN_HH
#define SURFACEPROCESSORN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define SURFACEPROCESSOR_STATUS_TYPE 5102
#define SURFACEPROCESSOR_CMD_TYPE    5101
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class SURFACEPROCESSOR_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
	SURFACEPROCESSOR_STATUS();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 SurfaceProcessState;
};

// Command Classes

class SURFACEPROCESSOR_CMD : public NMLmsgEx
{
public:

	//Constructor
	SURFACEPROCESSOR_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 SurfaceProcessCommand;
};

// Declare NML format function
extern int SurfaceProcessorFormat(NMLTYPE, void *, CMS *);

#endif 	// SURFACEPROCESSORN_HH
