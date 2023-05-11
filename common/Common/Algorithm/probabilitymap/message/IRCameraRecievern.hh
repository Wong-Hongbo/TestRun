/*
IRCameraRecievern.hh

This C++ header file defines the NML Messages used for command and status by IRCAMERARECIEVER_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:02 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef IRCAMERARECIEVERN_HH
#define IRCAMERARECIEVERN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define IRCAMERARECIEVER_STATUS_TYPE 11052
#define IRCAMERARECIEVER_CMD_TYPE 11051
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class IRCAMERARECIEVER_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
    IRCAMERARECIEVER_STATUS();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 IRCameraState;
};

// Command Classes

class IRCAMERARECIEVER_CMD : public NMLmsgEx
{
public:

	//Constructor
    IRCAMERARECIEVER_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 IRCameraCommand;
};

// Declare NML format function
extern int IRCameraRecieverFormat(NMLTYPE, void *, CMS *);

#endif 	// IRCAMERARECIEVERN_HH
