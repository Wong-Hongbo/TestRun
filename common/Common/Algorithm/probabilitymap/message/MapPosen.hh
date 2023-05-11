
// Prevent Multiple Inclusion
#ifndef MAPPOSEN_HH
#define MAPPOSEN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"


// Define the integer type ids.
#define MAP_POSE_MSG_TYPE 9188
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

// matching pose 
class MAP_POSE_MSG : public NMLmsgEx
{
public:

    //Constructor
    MAP_POSE_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    double x;   // 地图对应Gauss，单位：m
    double y;   //
    double z;   //
    double roll;  // 弧度
    double pitch;   // 弧度
    double yaw; // 弧度
    double time;
    double score;   //位置得分（0-1）
    double pose_cov[6][6];
};

// Declare NML format function
extern int MapPoseFormat(NMLTYPE, void *, CMS *);

#endif 	// MAPPOSEN_HH
