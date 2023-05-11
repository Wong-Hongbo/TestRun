/*


*/

// Prevent Multiple Inclusion
#ifndef SINGLELIDARDATAN_HH
#define SINGLELIDARDATAN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"
#include "LocalPosen.hh"


// Define the integer type ids.
#define SINGLELIDARDATA_MSG_TYPE 11009
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.
// 1线雷达点，单位厘米
typedef struct
{
    short x;         //车体坐标系, cm
    short y;
    short z;
    double angleH;       //角度，单位为度
    double realDistance; // 真实距离，单位厘米
    double intensity;
}SingleLidarPoint;

// Define the NML Message Classes
class SINGLELIDARDATA_MSG : public NMLmsgEx
{
public:
    //Constructor
    SINGLELIDARDATA_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    int ptNum;
    SingleLidarPoint lidarPoint[1000];
};

// Declare NML format function
extern int SingleLidarDataFormat(NMLTYPE, void *, CMS *);

#endif 	//
