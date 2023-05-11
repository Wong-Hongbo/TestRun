/*
CCameraRecievern.hh

This C++ header file defines the NML Messages used for command and status by CCAMERARECIEVER_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:01 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef CCAMERARECIEVERN_HH
#define CCAMERARECIEVERN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define CCAMERARECIEVER_STATUS_TYPE 11042
#define CCAMERARECIEVER_CMD_TYPE 11041
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class CCAMERARECIEVER_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
	CCAMERARECIEVER_STATUS();

	// Constructor used by derived classes
    //CCAMERARECIEVER_STATUS(NMLTYPE t, size_t s) :  RCS_STAT_MSG(t,s) {};

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 CCameraState;

};

// Command Classes

class CCAMERARECIEVER_CMD : public NMLmsgEx
{
public:

	//Constructor
	CCAMERARECIEVER_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 CCameraCommand;

};

// Declare NML format function
extern int CCameraRecieverFormat(NMLTYPE, void *, CMS *);

#endif 	// CCAMERARECIEVERN_HH
