/*
EntityMapn.hh

This C++ header file defines the NML Messages for ENTITYMAP
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:04 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef ENTITYMAPN_HH
#define ENTITYMAPN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"
#include "PedestrianInfon.hh"
#include "LaneMarkingInfon.hh"
#include "VehicleInfon.hh"
#include "TrafficLightn.hh"
#include "TrafficMarkn.hh"
#include "TrafficSignn.hh"

// Define the integer type ids.
#define ENTITYMAP_MSG_TYPE 2403
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

// Define the NML Message Classes
class ENTITYMAP_MSG : public NMLmsgEx
{
public:
	//Constructor
	ENTITYMAP_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 PdNum;
    PEDESTRIAN_OBJ PdObj[MAX_PEDESTRIAN_OBJ];
    INT32 PdLane[MAX_PEDESTRIAN_OBJ];                    // 行人所在车道线

    INT32 LaneNum;
    LANE_OBJ LaneObj[MAX_LANE_OBJ];
    INT32 LaneGenerateType[MAX_LANE_OBJ];
    INT32 CurrentLane;                                   // -100: on the left of all lanes, 100: on the right of all lanes
                                                         // -100<CurrentLane<100, vehicle is between lane CurrentLane and CurrentLane+1
    INT32 VehicleNum;
    VEHICLE_OBJ VehicleObj[MAX_VEHICLE_OBJ];
    INT32 VehicleLane[MAX_VEHICLE_OBJ];

    int TrafficLightNum;      //
    TRAFFIC_LIGHT TrafficLight;

    ROAD_MARKING ParkingRegion;
    UINT8  ParkingRegionExist;

    INT32 RoadMarkingNum;
    ROAD_MARKING RoadMarking[MAX_ROAD_MARKING];

    INT32 TrafficSignNum;      //
    TRAFFIC_SIGN TrafficSign[MAX_TRAFFIC_SIGN];

    UINT32 index_lanemarkinginfo;
    UINT32 index_sceneunderstandinginfo;
    UINT32 index_roadinfo;
    UINT32 index_irroadinfo;
    UINT32 index_negobinfo;
    UINT32 index_pedestrianinfo;
    UINT32 index_vehicleinfo;
};

// Declare NML format function
extern int EntityMapFormat(NMLTYPE, void *, CMS *);

#endif 	// ENTITYMAPN_HH
