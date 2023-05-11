/*
LocalSurfaceMapn.hh

This C++ header file defines the NML Messages for LOCALSURFACEMAP
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef LOCALATTRIBUTEMAPN_HH
#define LOCALATTRIBUTEMAPN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"

//ADD INDEX
#include "SurfaceMapn.hh"
#include "PositiveObstaclen.hh"

// Define the integer type ids.
#define LOCALATTRIBUTEMAP_MSG_TYPE 5404
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

// Define the NML Message Classes

// LocalAttributeMap grid attribute definition
// 低4位
// 0: 水泥， 1： 砂石, 2: 泥土， 3：草地， 4：未知

// 高4位
// 0： 可通行， 1： 悬挂， 2： 水坑， 3： 危险， 4： 灌木丛， 5： 树， 6： 路牙
// 7： 护栏，   8： 房子， 9： 行人， 10: 车辆，11：负障碍，12：正障碍，13：遮挡

class LOCALATTRIBUTEMAP_MSG : public NMLmsgEx
{
public:

	//Constructor
    LOCALATTRIBUTEMAP_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 MapWidth;
    INT32 MapHeight;
    INT32 GridWidth;
    INT32 GridHeight;
    INT32 VehicleGridX;
    INT32 VehicleGridY;

    UINT8 LocalAttributeMap[LOCAL_ATTRIBUTEMAP_SIZE];
    //ADD INDEX
    UINT32 index_surfacemap;
    UINT32 index_positiveobstacle;

};


// Declare NML format function
extern int LocalAttributeMapFormat(NMLTYPE, void *, CMS *);

#endif 	// LOCALATTRIBUTEMAPN_HH
