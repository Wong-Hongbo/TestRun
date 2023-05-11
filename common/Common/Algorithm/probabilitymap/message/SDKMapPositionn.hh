
// Prevent Multiple Inclusion
#ifndef SDKMAPPOSITIONN_HH
#define SDKMAPPOSITIONN_HH

// Include Files
#include "rcs.hh" 	  // Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"


// Define the integer type ids.
#define SDK_MAP_POSITION_MSG_TYPE 9906

class SDK_MAP_POSITION_MSG : public NMLmsgEx
{
public:

    //Constructor
    SDK_MAP_POSITION_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    double longitude;  // 经度，单位为度
    double latitude;   // 纬度，单位为度
    double height;     // 单位为米
    double azimuth;    // 单位为度
    int    command;
};

// Declare NML format function
extern int SDKMapPositionFormat(NMLTYPE, void *, CMS *);

#endif 	
