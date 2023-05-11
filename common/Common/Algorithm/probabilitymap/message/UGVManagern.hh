/*
UGVManagern.hh

This C++ header file defines the NML Messages used for command and status by UGVMANAGER_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:02 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef UGVMANAGERN_HH
#define UGVMANAGERN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define UGVMANAGER_STATUS_TYPE 1102
#define UGVMANAGER_CMD_TYPE 1101
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class UGVMANAGER_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
	UGVMANAGER_STATUS();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    int UGVManagerState;
};

// Command Classes

class UGVMANAGER_CMD : public NMLmsgEx
{
public:

	//Constructor
	UGVMANAGER_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    int UGVManagerCommand;
};

// Declare NML format function
extern int UGVManagerFormat(NMLTYPE, void *, CMS *);

#endif 	// UGVMANAGERN_HH
