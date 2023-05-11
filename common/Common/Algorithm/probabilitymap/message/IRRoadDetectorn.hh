/*
IRRoadDetectorn.hh

This C++ header file defines the NML Messages used for command and status by IRROADDETECTOR_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:02 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef IRROADDETECTORN_HH
#define IRROADDETECTORN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define IRROADDETECTOR_STATUS_TYPE 5302
#define IRROADDETECTOR_CMD_TYPE    5301
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class IRROADDETECTOR_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
    IRROADDETECTOR_STATUS();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 IRRoadState;
};

// Command Classes

class IRROADDETECTOR_CMD : public NMLmsgEx
{
public:

	//Constructor
    IRROADDETECTOR_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 IRRoadCommand;

};

// Declare NML format function
extern int IRRoadDetectorFormat(NMLTYPE, void *, CMS *);

#endif 	// IRROADDETECTORN_HH
