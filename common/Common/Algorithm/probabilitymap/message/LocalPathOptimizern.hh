/*
LocalPathOptimizern.hh

This C++ header file defines the NML Messages used for command and status by LOCALPATHOPTIMIZER_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:02 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef LOCALPATHOPTIMIZERN_HH
#define LOCALPATHOPTIMIZERN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define LOCALPATHOPTIMIZER_STATUS_TYPE 1302
#define LOCALPATHOPTIMIZER_CMD_TYPE 1301
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class LOCALPATHOPTIMIZER_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
	LOCALPATHOPTIMIZER_STATUS();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 LPathOptState;

};

// Command Classes

class LOCALPATHOPTIMIZER_CMD : public NMLmsgEx
{
public:

	//Constructor
	LOCALPATHOPTIMIZER_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 LPathOptCommand;

};

// Declare NML format function
extern int LocalPathOptimizerFormat(NMLTYPE, void *, CMS *);

#endif 	// LOCALPATHOPTIMIZERN_HH
