/*
RadarDatan.hh

This C++ header file defines the NML Messages for RadarData
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef RADARDATAN_HH
#define RADARDATAN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"


// Define the integer type ids.
#define RADARDATA_MSG_TYPE 11006
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

#define ANZ_BUFFERS		128				//
#define TARGET_NUM	ANZ_BUFFERS

enum SMVehicle_exit_type
{
    SMVehicle_Not_Exit = 0,
    SMVehicle_Far_Away = 1,
    SMVehicle_Close_To = 2,
    SMVehicle_Static   = 3,
    SMVehicle_Unlike = 9,
    SMVehicle_MustBe = 10,
    SMVehicle_MayBe = 11
};
/*
typedef struct
{
    double Range;
    double RangeRate;
    double Angle;
    double PosX;
    double PosY;
    double Speed;
    double SpeedX;
    double SpeedY;
    double Width;
    double Length;
    double amp;  //esr:-10db -- 21db; rsds: -24db -- 40db
    int PUU;
    int PVV;
    int PUU2;
    int PVV2;
    int Used;
    int exist;
}SMVehicle;

//Delphi ESR毫米波雷达目标数据
typedef struct
{
    double range;
    double rangeRate;
    double angle;
    double width;
    double amp;
}OneTarget;	//	用于存储单个目标信息
typedef struct
{
    OneTarget Target[TARGET_NUM];
    int Refresh;
    unsigned long  ID;
}TargetData;


// Define the NML Message Classes
class RADARDATA_MSG : public NMLmsgEx
{
public:
    //Constructor
    RADARDATA_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    //
    //TargetData
    TargetData Target;
    SMVehicle  SMVTarget[TARGET_NUM];
    int SMVTargetNum;

    double TimeRecv;
    int ID;    //0-forward, 1-left, 2-right
};

// Declare NML format function
extern int RadarDataFormat(NMLTYPE, void *, CMS *);
*/


/**************************new version********************************/
typedef struct
{
    UINT8 reserved;
    UINT8 id;
    UINT8 exist;  //
    UINT8 type;   // 0-point 1-car 2-truck 3-pedestrian 4-motorcycle 5-bicycie 6-wide
    UINT8 width;  // 10cm
    UINT8 length; // 10cm
    INT16 range;  // 1cm
    INT16 speed;  // 1cm/s
    INT16 accel;  // 1cm/ss
    INT16 angle;  // 0.01deg / 1cm
    INT16 amp;    // 0.01deg / 1cm
} radar_target_t;

typedef struct
{
    UINT32 time;
    UINT8  radar_id;    //0-7   forward,....  left, right  and so on;
    UINT8  objs_count; // targets count
    UINT16  reserved;
    radar_target_t targets[TARGET_NUM];
}radar_packet_t;


// Define the NML Message Classes
class RADARDATA_MSG : public NMLmsgEx
{
public:
    //Constructor
    RADARDATA_MSG();

    // CMS Update Function
    void update(CMS *);
    // Place custom variables here.

    //TargetData
    radar_packet_t Target;
    double TimeRecv;
};

// Declare NML format function
extern int RadarDataFormat(NMLTYPE, void *, CMS *);


#endif 	// RADARDATAN_HH
