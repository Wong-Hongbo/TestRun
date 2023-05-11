#ifndef  __TRAFFICMAPNEW__
#define __TRAFFICMAPNEW__

#include "rcs.hh"
#include "NMLmsgExn.hh"
#include "CommonDefinitionX.hh"


#define TRAFFICMAP_MSG_TYPE 19203


#define TRAFFIC_MAP_ROAD_NUM    20    //最多发20条路
#define TRAFFIC_MAP_DRIVEWAY_NUM  30    //最多发30个车道
#define TRAFFIC_MAP_DRIVEWAY_PT_NUM 200  //每条车道最多发200个点
#define TRAFFIC_MAP_LANEMARKING_NUM  30    //最多发30个车道线
#define TRAFFIC_MAP_LANEMARKING_PT_NUM 200   //每条车道线最多发200个点


struct TrafficMapPosition
{
    INT32 gauss_x;
    INT32 gauss_y;
    INT32 azimuth;
};

//如果全局路中发了路点序列，这里就不发具体点坐标了
struct TrafficRoad
{
    INT32 road_id;    //road_id
    INT32 go_straight;    //直行
    INT32 turn_left;    //左转
    INT32 turn_right;    //右转
    INT32 u_turn;    //调头
};


//车道信息
struct TrafficDriveway
{
    INT32  driveway_id;    //车道id
    INT32  road_id;    //车道所属的road id
    UINT16 driveway_attr;    //车道属性，0：机动车道；1：非机动车道；2：物流车道
    UINT16 driveway_pt_num;    //点的个数
    INT32  driveway_pt_x[TRAFFIC_MAP_DRIVEWAY_PT_NUM];
    INT32  driveway_pt_y[TRAFFIC_MAP_DRIVEWAY_PT_NUM];
};

struct TrafficLaneMarking
{
    INT32 lanemarking_id;    //标志线id
    INT32 road_id;    //车道线所属的road id
    UINT16 lanemarking_attr;    //标志线属性，0：白实线；1：黄实线；2：白虚线；3：黄虚线
    UINT16 lanemarking_pt_num;    //点的个数
    INT32  lanemarking_pt_x[TRAFFIC_MAP_LANEMARKING_PT_NUM];
    INT32  lanemarking_pt_y[TRAFFIC_MAP_LANEMARKING_PT_NUM];
};

class TRAFFICMAP_MSG: public NMLmsgEx
{
public:

    //Constructor
    TRAFFICMAP_MSG();

    // CMS Update Function
    void update(CMS *);

    int current_road;

    TrafficMapPosition traffic_map_position;
    
    int road_num;    //有效路的条数

    TrafficRoad    traffic_road[TRAFFIC_MAP_ROAD_NUM];

    int driveway_num;    //有效车道条数

    TrafficDriveway    traffic_driveway[TRAFFIC_MAP_DRIVEWAY_NUM];

    int lanemarking_num;    //有效标志线条数

    TrafficLaneMarking traffic_lanemarking[TRAFFIC_MAP_LANEMARKING_NUM];

    //待添加马路牙子等其他环境信息
};

// Declare NML format function
extern int TrafficMapFormat(NMLTYPE, void *, CMS *);


#endif
