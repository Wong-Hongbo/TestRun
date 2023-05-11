
/*
EnvPerceptionMapn.hh

This C++ header file defines the NML Messages for ENVPERCEPTIONMAP
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef ENVPERCEPTIONMAPN_HH
#define ENVPERCEPTIONMAPN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "NMLmsgExn.hh"
#include "CommonDefinitionX.hh"
#include "GlobalPositionInfon.hh"


// Define the integer type ids.
#define ENVPERCEPTIONMAP_MSG_TYPE 5405
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

// Define the NML Message Classes

class ENVPERCEPTIONMAP_MSG : public NMLmsgEx
{
public:

	//Constructor
	ENVPERCEPTIONMAP_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 MapWidth;
    INT32 MapHeight;
    // 0-road 1-water 2-grass 3-tree,bosk 4-curb 5-pedestrian,car 6-negative obstacle 7-unknown
    UINT8 AttributeMap[ENV_MAP_SIZE];
    // height,
    UINT8 DEMMap[ENV_MAP_SIZE];
    UINT8 TranversableMap[ENV_MAP_SIZE];

};

// Declare NML format function
extern int EnvPerceptionMapFormat(NMLTYPE, void *, CMS *);

#endif 	// ENVPERCEPTIONMAPN_HH
