
// Prevent Multiple Inclusion
#ifndef MAPPOSITIONN_HH
#define MAPPOSITIONN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"


// Define the integer type ids.
#define MAP_POSITION_MSG_TYPE 9903
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

class MAP_POSITION_MSG : public NMLmsgEx
{
public:
    //Constructor
    MAP_POSITION_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    double global_x;   // 全局位置，Gauss或者UTM
    double global_y;
    double azimuth;
    double longitude;  // 经度
    double latitude;   // 纬度
    double height; // m
    char   works_well;
    double roll;      //3d posture  add for yhl ,using 3d localizer
    double pitch;     //3d posture  add for yhl ,using 3d localizer


    /*
     * score: 0.5以上，定位状态较好
     * score: 0.3-0.5,定位状态一般，需要提示waring
     * score: 0.3以下，定位较差，需要停车检测
    */

    double score;              //  match score add for yhl
    int    level;              //  0-1000 地面以上根据高度依次增加，1001-2000地面以下，默认情况下level=0 为地面
    int    envstatewarning;    //  提示环境变化等级：0：检测环境无变化  1：检测环境场景轻微变化 2：检测到环境中发生较大变化

    /*localizer odom*/
    double x;
    double y;
    double z;
    double yaw;
    double Alltime;

    bool request_init;      // 当为ture时，请求初始化任务 ，当为false时，不请求初始化任务

};

// Declare NML format function
extern int MapPositionFormat(NMLTYPE, void *, CMS *);

#endif 	// TRAFFICSIGNN_HH
