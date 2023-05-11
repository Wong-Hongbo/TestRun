/*
LaneMarkingDetectorn.hh

This C++ header file defines the NML Messages used for command and status by LANEMARKINGDETECTOR_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:02 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef LANEMARKINGDETECTORN_HH
#define LANEMARKINGDETECTORN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define LANEMARKINGDETECTOR_STATUS_TYPE 2102
#define LANEMARKINGDETECTOR_CMD_TYPE 2101
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class LANEMARKINGDETECTOR_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
	LANEMARKINGDETECTOR_STATUS();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 LaneDetectorState;

};

// Command Classes

class LANEMARKINGDETECTOR_CMD : public NMLmsgEx
{
public:

	//Constructor
	LANEMARKINGDETECTOR_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 LaneDetectorCommand;
};

// Declare NML format function
extern int LaneMarkingDetectorFormat(NMLTYPE, void *, CMS *);

#endif 	// LANEMARKINGDETECTORN_HH
