/*
LadarEightDatan.hh

This C++ header file defines the NML Messages for HDLadarData
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef LADAREIGHTDATAN_HH
#define LADAREIGHTDATAN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"

// Define the integer type ids.
#define LADAREIGHTDATA_MSG_TYPE 11005
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.
struct _DataHeader
{
	UINT32	magicWord;
	UINT32	sizeLastMes;
	UINT32	sizeThisMes;
	UINT8	reserved;
	UINT8	deviceID;
	UINT16	dataType;
	UINT8	ntpTime[8];
};

struct _DataScanHeader
{
    UINT16	scanNum;
    UINT16	scanState;
    UINT16	syncPhaOf;
    UINT8	startTime[8];
    UINT8	endTime[8];

    UINT16	anglePR;
    INT16	startAn;
    INT16	endAn;
    UINT16	scanPointNum;

    INT16	yaw;
    INT16	pitch;
    INT16	roll;
    INT16	posX;
    INT16	posY;
    INT16	posZ;

    UINT16	flags;
};

struct _DataScanPoint
{
    UINT8   state;	//	Echo+Layer
    UINT8   flags;
    INT16   angle;
    UINT16  distance;
    UINT16  echoPW;
    UINT16  reserved;
    
};
struct _DataLaserPoint
{
    INT16   angle;
    UINT16  distance;
    UINT16  echoPW;
    float   X;
    float   Y;
    float   Z;
};
struct _DataScan
{
    struct _DataScanHeader	header;
    struct _DataScanPoint	data[MAX_SCAN_DATA_NUM];//注意不要越界
};
struct _DataLaserScan
{
    UINT16	scanPointNum;
    struct _DataLaserPoint	data[MAX_SCAN_DATA_NUM];//注意不要越界
};
struct _Point2D
{
	INT16	posX;
	INT16	posY;
	//int posX;
	//int posY;
};

struct _Size2D 
{
	UINT16	sizeX;
	UINT16	sizeY;
};

struct _ObjectData 
{
	UINT16	objectID;
	UINT16	objectAge;
	UINT16	predAge;
	UINT16	relaTime;

	struct _Point2D	refPoint;
	struct _Point2D	refPoingSig;
	struct _Point2D	closestPoint;
	struct _Point2D	boxCenter;
	UINT16	boxWidth;
	UINT16	boxLength;

	struct _Point2D	objectBC;
	struct _Size2D	objectBS;
	INT16	objectBO;
	struct _Point2D	absVelocity;
	struct _Size2D	absVelocitySig;
	struct _Point2D	relaVelocity;
    //0: unclassified
    //1: unknown small
    //2: unknown big
    //3: pedestrian
    //4: bike
    //5: car
    //6: truck
    //7...: reserved
   UINT16	classfication;
	UINT16	classAge;
	UINT16	classConfidence;
	UINT16	contourPN;

    struct _Point2D pPointData[1000];		//	每个障碍物最多100个边缘点
};
struct _ObjectHeader 
{
	UINT8	scanTime[8];
	UINT16	objectNum;
	//	目标数据指针
	struct _ObjectData	pObjectData[100];//最多100个障碍物
};

// Define the NML Message Classes
class LADAREIGHTDATA_MSG : public NMLmsgEx
{
public:
    //Constructor
    LADAREIGHTDATA_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    //

    UINT8 Workflag;
    _DataScan ScanDataA;
    _DataScan ScanDataB;
    _ObjectHeader ObjectList;
    };

// Declare NML format function
extern int LadarEightDataFormat(NMLTYPE, void *, CMS *);

#endif 	// LADAREIGHTDATAN_HH
