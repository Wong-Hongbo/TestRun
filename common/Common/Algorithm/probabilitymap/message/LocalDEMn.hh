/*
LocalDEMn.hh

This C++ header file defines the NML Messages for LOCALDEM
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef LOCALDEMN_HH
#define LOCALDEMN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"


// Define the integer type ids.
#define LOCALDEM_MSG_TYPE 4203
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

// Define the NML Message Classes

class LOCALDEM_MSG : public NMLmsgEx
{
public:

	//Constructor
	LOCALDEM_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 MapWidth;
    INT32 MapHeight;
    INT32 GridWidth;
    INT32 GridHeight;
    INT32 VehicleGridX;
    INT32 VehicleGridY;


    UINT8 LocalDEM[TERRAIN_MAP_SIZE];
    INT32 VehicleHeight;  //cm
};

// Declare NML format function
extern int LocalDEMFormat(NMLTYPE, void *, CMS *);

#endif 	// LOCALDEMN_HH
