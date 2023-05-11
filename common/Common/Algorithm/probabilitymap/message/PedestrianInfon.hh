/*
PedestrianInfon.hh

This C++ header file defines the NML Messages for PEDESTRIANINFO
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef PEDESTRIANINFON_HH
#define PEDESTRIANINFON_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"

// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define PEDESTRIANINFO_MSG_TYPE 8103
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.
#define MAX_PEDESTRIAN_OBJ 30

struct PEDESTRIAN_OBJ
{
    INT32 PedestrianID;
    INT32 CenterX;    //cm为单位，惯导坐标系
    INT32 CenterY;
    INT32 Radius;     //cm为单位
    INT32 Speed;      // cm/s
    INT32 Direction;  // 0.01度，惯导坐标系
    INT32 Height;     // height of the pedestrian
};


// Define the NML Message Classes

class PEDESTRIANINFO_MSG : public NMLmsgEx
{
public:

	//Constructor
	PEDESTRIANINFO_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    int PdNum;                             // pd num
    PEDESTRIAN_OBJ PdObj[MAX_PEDESTRIAN_OBJ];
    int                          ObjType[MAX_PEDESTRIAN_OBJ];   //0: unknown, 1:pedestrian,  2:car, 3: facing car,  4: bicycle

};

// Declare NML format function
extern int PedestrianInfoFormat(NMLTYPE, void *, CMS *);

#endif 	// PEDESTRIANINFON_HH
