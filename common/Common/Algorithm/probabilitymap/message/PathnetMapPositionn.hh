
// Prevent Multiple Inclusion
#ifndef PATHNETMAPPOSITION_HH
#define PATHNETMAPPOSITION_HH

// Include Files
#include "rcs.hh" 	  // Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"


// Define the integer type ids.
#define PATHNET_MAP_POSITION_MSG_TYPE 17599

class PATHNET_MAP_POSITION_MSG : public NMLmsgEx
{
public:

    //Constructor
    PATHNET_MAP_POSITION_MSG();

    // CMS Update Function
    void update(CMS *);

    char PathnetPCDname[1024];
    int  state;//0:start,1:arrive

    // Place custom variables here.
//    double longitude;  // 经度，单位为度
//    double latitude;   // 纬度，单位为度
//    double height;     // 单位为米
//    double azimuth;    // 单位为度
//    int    command;
};

// Declare NML format function
extern int PathnetMapPositionFormat(NMLTYPE, void *, CMS *);

#endif 	
