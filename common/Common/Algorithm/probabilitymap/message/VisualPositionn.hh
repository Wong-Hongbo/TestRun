
// Prevent Multiple Inclusion
#ifndef VISUALPOSITIONN_HH
#define VISUALPOSITIONN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"

// Define the integer type ids.
#define VISUAL_POSITION_MSG_TYPE 20630
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

class VISUAL_POSITION_MSG : public NMLmsgEx
{
public:
    //Constructor
    VISUAL_POSITION_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    double global_x;   // 全局位置，Gauss或者UTM
    double global_y;
    double height;    // m
    double roll;      // 弧度
    double pitch;     //弧度
    double yaw;       // 弧度
    double longitude;  // 经度
    double latitude;   // 纬度
    char   works_well;

};

// Declare NML format function
extern int VISUALPositionFormat(NMLTYPE, void *, CMS *);

#endif 	// VISUALPOSITIONN_HH
