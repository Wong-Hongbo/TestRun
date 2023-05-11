/*
SurfaceMapn.hh

This C++ header file defines the NML Messages for SURFACEMAP
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef SURFACEMAPN_HH
#define SURFACEMAPN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"

// Define the integer type ids.
#define SURFACEMAP_MSG_TYPE 5103
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.
// Define the NML Message Classes

class SURFACEMAP_MSG : public NMLmsgEx
{
public:

	//Constructor
	SURFACEMAP_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 MapWidth;
    INT32 MapHeight;
    INT32 GridWidth;
    INT32 GridHeight;
    INT32 VehicleGridX;
    INT32 VehicleGridY;

    UINT8 SurfaceMap[SURFACE_MAP_SIZE];

};

// Declare NML format function
extern int SurfaceMapFormat(NMLTYPE, void *, CMS *);

#endif 	// SURFACEMAPN_HH
