/*
DataRecordern.hh

This C++ header file defines the NML Messages used for command and status by DATARECORDER_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:02 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef DATARECORDERN_HH
#define DATARECORDERN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define DATARECORDER_STATUS_TYPE 3202
#define DATARECORDER_CMD_TYPE 3201
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class DATARECORDER_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
	DATARECORDER_STATUS();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 DRState;

};

// Command Classes

class DATARECORDER_CMD : public NMLmsgEx
{
public:

	//Constructor
	DATARECORDER_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 DRCommand;

};

// Declare NML format function
extern int DataRecorderFormat(NMLTYPE, void *, CMS *);

#endif 	// DATARECORDERN_HH
