/*
RoadDetectorn.hh

This C++ header file defines the NML Messages used for command and status by ROADDETECTOR_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:02 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef ROADDETECTORN_HH
#define ROADDETECTORN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define ROADDETECTOR_STATUS_TYPE 2202
#define ROADDETECTOR_CMD_TYPE 2201
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class ROADDETECTOR_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
	ROADDETECTOR_STATUS();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 RdState;
};

// Command Classes

class ROADDETECTOR_CMD : public NMLmsgEx
{
public:

	//Constructor
	ROADDETECTOR_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 RdDetectCommand;

};

// Declare NML format function
extern int RoadDetectorFormat(NMLTYPE, void *, CMS *);

#endif 	// ROADDETECTORN_HH
