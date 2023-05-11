/*
NegativeDetectorn.hh

This C++ header file defines the NML Messages used for command and status by NEGATIVEDETECTOR_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:02 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef NEGATIVEDETECTORN_HH
#define NEGATIVEDETECTORN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define NEGATIVEDETECTOR_STATUS_TYPE 5202
#define NEGATIVEDETECTOR_CMD_TYPE    5201
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class NEGATIVEDETECTOR_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
	NEGATIVEDETECTOR_STATUS();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 NegDetectionState;

};

// Command Classes

class NEGATIVEDETECTOR_CMD : public NMLmsgEx
{
public:

	//Constructor
	NEGATIVEDETECTOR_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 NegDetectionCommand;

};

// Declare NML format function
extern int NegativeDetectorFormat(NMLTYPE, void *, CMS *);

#endif 	// NEGATIVEDETECTORN_HH
