/*
GlobalMapInquiryn.hh

This C++ header file defines the NML Messages used for command and status by GLOBALMAPINQUIRY_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:02 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef GLOBALMAPINQUIRYN_HH
#define GLOBALMAPINQUIRYN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define GLOBALMAPINQUIRY_STATUS_TYPE 1402
#define GLOBALMAPINQUIRY_CMD_TYPE 1401
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class GLOBALMAPINQUIRY_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
	GLOBALMAPINQUIRY_STATUS();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 GMapInquiryState;

};

// Command Classes

class GLOBALMAPINQUIRY_CMD : public NMLmsgEx
{
public:

	//Constructor
	GLOBALMAPINQUIRY_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 GMapInquiryCommand;

};

// Declare NML format function
extern int GlobalMapInquiryFormat(NMLTYPE, void *, CMS *);

#endif 	// GLOBALMAPINQUIRYN_HH
