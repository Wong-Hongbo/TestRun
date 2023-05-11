/*
HDLadarRecievern.hh

This C++ header file defines the NML Messages used for command and status by HDLADARRECIEVER_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:02 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef HDLADARRECIEVERN_HH
#define HDLADARRECIEVERN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define HDLADARRECIEVER_STATUS_TYPE 11032
#define HDLADARRECIEVER_CMD_TYPE 11031
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class HDLADARRECIEVER_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
	HDLADARRECIEVER_STATUS();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 HDLadarState;

};

// Command Classes

class HDLADARRECIEVER_CMD : public NMLmsgEx
{
public:

	//Constructor
	HDLADARRECIEVER_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 HDLadarCommand;

};

// Declare NML format function
extern int HDLadarRecieverFormat(NMLTYPE, void *, CMS *);

#endif 	// HDLADARRECIEVERN_HH
