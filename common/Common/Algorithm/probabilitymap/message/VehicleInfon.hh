/*
VehicleInfon.hh

This C++ header file defines the NML Messages for VEHICLEINFO
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef VEHICLEINFON_HH
#define VEHICLEINFON_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"

#define VEHICLEINFO_MSG_TYPE 8203
// Define the NML Message Classes
//
#define MAX_LOCAL_VEHICLE_OBJ 100
typedef struct
{
    INT32 VehicleID;
    INT32 CenterX;         // cm
    INT32 CenterY;
    INT32 Speed;           // cm/s
    INT32 SpeedDirection;  // 0.01degree
    INT32 Height;          // cm
    INT32 Type;            //0:vehicle 1:ped

    INT32 Convexhull_VertexNumber;
    INT32 ConvexhullX[50];
    INT32 ConvexhullY[50];
} VEHICLE_OBJ;

class VEHICLEINFO_MSG : public NMLmsgEx
{
public:

	//Constructor
	VEHICLEINFO_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 VehicleNum;
    VEHICLE_OBJ VehicleObj[MAX_LOCAL_VEHICLE_OBJ];
    INT32 ObjectType[MAX_LOCAL_VEHICLE_OBJ];        //0: unknown, 1: vehicle, 2: pedestrain, 3: bicycle,4 : small object
                                                    //5: big object, 6: confirmed static
};

// Declare NML format function
extern int VehicleInfoFormat(NMLTYPE, void *, CMS *);

#endif 	// VEHICLEINFON_HH
