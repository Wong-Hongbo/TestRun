#ifndef  __TRAFFICMAPNEW__
#define __TRAFFICMAPNEW__

#include "rcs.hh"
#include "NMLmsgExn.hh"
#include "CommonDefinitionX.hh"


#define TRAFFICMAP_MSG_TYPE 19203


#define TRAFFIC_MAP_ROAD_NUM    20    //��෢20��·
#define TRAFFIC_MAP_DRIVEWAY_NUM  30    //��෢30������
#define TRAFFIC_MAP_DRIVEWAY_PT_NUM 200  //ÿ��������෢200����
#define TRAFFIC_MAP_LANEMARKING_NUM  30    //��෢30��������
#define TRAFFIC_MAP_LANEMARKING_PT_NUM 200   //ÿ����������෢200����


struct TrafficMapPosition
{
    INT32 gauss_x;
    INT32 gauss_y;
    INT32 azimuth;
};

//���ȫ��·�з���·�����У�����Ͳ��������������
struct TrafficRoad
{
    INT32 road_id;    //road_id
    INT32 go_straight;    //ֱ��
    INT32 turn_left;    //��ת
    INT32 turn_right;    //��ת
    INT32 u_turn;    //��ͷ
};


//������Ϣ
struct TrafficDriveway
{
    INT32  driveway_id;    //����id
    INT32  road_id;    //����������road id
    UINT16 driveway_attr;    //�������ԣ�0������������1���ǻ���������2����������
    UINT16 driveway_pt_num;    //��ĸ���
    INT32  driveway_pt_x[TRAFFIC_MAP_DRIVEWAY_PT_NUM];
    INT32  driveway_pt_y[TRAFFIC_MAP_DRIVEWAY_PT_NUM];
};

struct TrafficLaneMarking
{
    INT32 lanemarking_id;    //��־��id
    INT32 road_id;    //������������road id
    UINT16 lanemarking_attr;    //��־�����ԣ�0����ʵ�ߣ�1����ʵ�ߣ�2�������ߣ�3��������
    UINT16 lanemarking_pt_num;    //��ĸ���
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
    
    int road_num;    //��Ч·������

    TrafficRoad    traffic_road[TRAFFIC_MAP_ROAD_NUM];

    int driveway_num;    //��Ч��������

    TrafficDriveway    traffic_driveway[TRAFFIC_MAP_DRIVEWAY_NUM];

    int lanemarking_num;    //��Ч��־������

    TrafficLaneMarking traffic_lanemarking[TRAFFIC_MAP_LANEMARKING_NUM];

    //�������·���ӵ�����������Ϣ
};

// Declare NML format function
extern int TrafficMapFormat(NMLTYPE, void *, CMS *);


#endif
