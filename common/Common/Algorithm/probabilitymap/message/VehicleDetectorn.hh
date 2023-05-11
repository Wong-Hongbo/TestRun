/*
VehicleDetectorn.hh

This C++ header file defines the NML Messages used for command and status by VEHICLEDETECTOR_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:02 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef VEHICLEDETECTORN_HH
#define VEHICLEDETECTORN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define VEHICLEDETECTOR_STATUS_TYPE 8202
#define VEHICLEDETECTOR_CMD_TYPE 8201
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class VEHICLEDETECTOR_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
	VEHICLEDETECTOR_STATUS();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    int VehicleDetectorState;
};

// Command Classes

class VEHICLEDETECTOR_CMD : public NMLmsgEx
{
public:

	//Constructor
	VEHICLEDETECTOR_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    int VehicleDetectorCommand;
};

// Declare NML format function
extern int VehicleDetectorFormat(NMLTYPE, void *, CMS *);

#endif 	// VEHICLEDETECTORN_HH
