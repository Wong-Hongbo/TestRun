/*
GlobalMapInfon.hh

This C++ header file defines the NML Messages for GLOBALMAPINFO
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:04 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef GLOBALMAPINFON_HH
#define GLOBALMAPINFON_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define GLOBALMAPINFO_MSG_TYPE 1403
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

class GLOBALMAPINFO_MSG : public NMLmsgEx
{
public:

	//Constructor
	GLOBALMAPINFO_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Declare NML format function
extern int GlobalMapInfoFormat(NMLTYPE, void *, CMS *);

#endif 	// GLOBALMAPINFON_HH
