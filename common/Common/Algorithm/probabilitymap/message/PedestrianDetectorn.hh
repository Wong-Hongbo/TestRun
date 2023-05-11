/*
PedestrianDetectorn.hh

This C++ header file defines the NML Messages used for command and status by PEDESTRIANDETECTOR_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:02 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef PEDESTRIANDETECTORN_HH
#define PEDESTRIANDETECTORN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define PEDESTRIANDETECTOR_STATUS_TYPE 8101
#define PEDESTRIANDETECTOR_CMD_TYPE 8102
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class PEDESTRIANDETECTOR_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
	PEDESTRIANDETECTOR_STATUS();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 PdState;
};

// Command Classes

class PEDESTRIANDETECTOR_CMD : public NMLmsgEx
{
public:

	//Constructor
	PEDESTRIANDETECTOR_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 PdCommand;

};

// Declare NML format function
extern int PedestrianDetectorFormat(NMLTYPE, void *, CMS *);

#endif 	// PEDESTRIANDETECTORN_HH
