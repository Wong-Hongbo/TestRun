// Prevent Multiple Inclusion
#ifndef MAP_POINTCLOUD_HH
#define MAP_POINTCLOUD_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
//#include "GlobalPositionInfon.hh"
//#include "LocalPosen.hh"

// Define the integer type ids.
#define MAP_POINTCLOUD_MSG_TYPE 9907
#define PCD_FILE_PATH_SIZE   1024

#define PC_NO_DATA  0  // no map data
#define PC_SUCCESS  1  // got map successfully
//int
typedef struct
{
    int x;      //点云坐标，单位cm
    int y;
    int z;
    unsigned char intensity;
}MapPoint3d;

typedef struct {
    double POS_X;
    double POS_Y;
    double POS_Z;
    char filename[PCD_FILE_PATH_SIZE];
    int status;
}MapPointCloudStatus;

// Define the NML Message Classes
class MAP_POINTCLOUD_MSG : public NMLmsgEx
{
public:

    //Constructor
    MAP_POINTCLOUD_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    //double base_x;  //基准点的x，y， 单位cm
    //double base_y;  //单位cm
    //int pointNum;
    //MapPoint3d points[100000];
    double longitude;  // 经度，单位为度
    double latitude;   // 纬度，单位为度
    double height;     // 单位为米
    double azimuth;    // 单位为度
    MapPointCloudStatus cloud_status;
};

// Declare NML format function
extern int MapPointCloudFormat(NMLTYPE, void *, CMS *);

#endif 	// HDLADARDATAN_HH
