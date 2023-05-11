/*
PositiveDetectorn.hh

This C++ header file defines the NML Messages used for command and status by POSITIVEDETECTOR_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:02 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef POSITIVEDETECTORN_HH
#define POSITIVEDETECTORN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define POSITIVEDETECTOR_STATUS_TYPE 4102
#define POSITIVEDETECTOR_CMD_TYPE    4101
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class POSITIVEDETECTOR_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
	POSITIVEDETECTOR_STATUS();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 TerrainProcessState;

};

// Command Classes

class POSITIVEDETECTOR_CMD : public NMLmsgEx
{
public:

	//Constructor
	POSITIVEDETECTOR_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 TerrainProcessCommand;
};

// Declare NML format function
extern int PositiveDetectorFormat(NMLTYPE, void *, CMS *);

#endif 	// POSITIVEDETECTORN_HH
