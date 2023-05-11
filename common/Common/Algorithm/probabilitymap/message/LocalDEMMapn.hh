/*
LocalDEMMapn.hh

This C++ header file defines the NML Messages for LOCALDEMMAP
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef LOCALDEMMAPN_HH
#define LOCALDEMMAPN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"

//ADD INDEX
#include "LocalDEMn.hh"


// Define the integer type ids.
#define LOCALDEMMAP_MSG_TYPE 5403

// Define the NML Message Classes

class LOCALDEMMAP_MSG : public NMLmsgEx
{
public:

	//Constructor
	LOCALDEMMAP_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 MapWidth;
    INT32 MapHeight;
    INT32 GridWidth;
    INT32 GridHeight;
    INT32 VehicleGridX;
    INT32 VehicleGridY;

    UINT8 LocalDEMMap[LOCALDEM_MAP_SIZE];
    //ADD INDEX
    UINT32 index_localdem;

};

// Declare NML format function
extern int LocalDEMMapFormat(NMLTYPE, void *, CMS *);

#endif 	// LOCALDEMMAPN_HH
