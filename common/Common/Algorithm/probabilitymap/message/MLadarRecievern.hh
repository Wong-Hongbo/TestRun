/*
MLadarRecievern.hh

This C++ header file defines the NML Messages used for command and status by MLADARRECIEVER_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:02 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef MLADARRECIEVERN_HH
#define MLADARRECIEVERN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define MLADARRECIEVER_STATUS_TYPE 13000
#define MLADARRECIEVER_CMD_TYPE 13001
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class MLADARRECIEVER_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
	MLADARRECIEVER_STATUS();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 MLadarState;

};

// Command Classes

class MLADARRECIEVER_CMD : public NMLmsgEx
{
public:

	//Constructor
	MLADARRECIEVER_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 MLadarCommand;

};

// Declare NML format function
extern int MLadarRecieverFormat(NMLTYPE, void *, CMS *);

#endif 	// MLADARRECIEVERN_HH
