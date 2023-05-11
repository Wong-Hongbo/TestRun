/*
TrafficMapn.hh

This C++ header file defines the NML Messages for TRAFFICMAPINFO
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:04 CST 2013	Created by rcsdesign.
*/

// Prevent Multiple Inclusion
#ifndef TRAFFICMAPN_HH
#define TRAFFICMAPN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"
#include "TrafficSignn.hh"
#include "TrafficMarkn.hh"

// Define the integer type ids.
#define TRAFFICMAP_MSG_TYPE 9203
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.
//////Traffic Map
///
///
#define TRAFFIC_MAP_LANEMARKING_MAX_PTNUM 200
#define TRAFFIC_MAP_ROAD_MAX_LANE_NUM 10
#define TRAFFIC_MAP_MAX_ROADMARKINGNUM 15    //����·����Ŀ
#define TRAFFIC_MAP_MAX_KEYPOINTNUM 40    //���Ĺؼ�����Ŀ
#define TRAFFIC_MAP_MAX_LANEMARKINGNUM 20    //���ĳ�������Ŀ
#define TRAFFIC_MAP_MAX_ROADNUM  30    //���ĳ�����Ŀ
#define TRAFFIC_MAP_MAX_OBSTACLE 5    //�������
#define TRAFFIC_MAP_OBSTACLE_MAX_PTNUM 100


typedef struct{
    ROAD_MARKING  Marking;
           INT16  RoadID;  //
           INT16  LaneID;  //��ֵ0
}TRAFFIC_MAP_ROADMARKING;

typedef struct{
    int x;
    int y;
    INT16 RoadID;  // -18000---18000
    INT16 LaneID;  // ��ߵ�һ���ߵ�ID
    unsigned char type;// 0����ǰ·�ĳ��ڵ� 1�������ڵ�  2���Ҳ���ڵ� 3��ֱ����ڵ� 4��U turn�ؼ���
}TRAFFIC_MAP_KEYPOINT;

typedef struct{
    INT16 RoadID; //
    INT16 LaneID; // 0 1 2 3
    INT16 LaneRefer;
    INT16 LaneAttr; // 0 ��ʵ��1 ��ʵ��2 ���飬3 ����
    INT16 LanePtNum;
    INT32 LanePtX[TRAFFIC_MAP_LANEMARKING_MAX_PTNUM];
    INT32 LanePtY[TRAFFIC_MAP_LANEMARKING_MAX_PTNUM];
}TRAFFIC_MAP_LANEMARKING;

typedef struct{
    INT16 RoadID;    
    INT16 RoadAttr;  //
    INT16 RoadType;
    INT16 LaneID[TRAFFIC_MAP_ROAD_MAX_LANE_NUM];    //����·�ĳ����ߵ�id
    INT16 LaneNum;    //�����ߵ���Ŀ
}TRAFFIC_MAP_ROAD;    //·����Ϣ

typedef struct{
	INT16 RoadID;
	INT32 StartX;
	INT32 StartY;
	INT32 EndX;
	INT32 EndY;
	INT32 StartDirect;
	INT32 EndDirect;
	INT32 CurvePtNum;
	INT32 CurveX[100];
	INT32 CurveY[100];
}TRAFFIC_MAP_GUIDELINE;

typedef struct{
    INT32 gauss_x;
    INT32 gauss_y;
    INT32 azimuth;
    bool works_well;
}TRAFFIC_MAP_POSITION;    //����λ����Ϣ
// Define the NML Message Classes

typedef struct
{
    INT16 Attr;
    INT16 PtNum;
    INT32 PtX[TRAFFIC_MAP_OBSTACLE_MAX_PTNUM];
    INT32 PtY[TRAFFIC_MAP_OBSTACLE_MAX_PTNUM];
}TRAFFIC_MAP_OBSTACLE;

class TRAFFICMAP_MSG : public NMLmsgEx
{
public:

    //Constructor
    TRAFFICMAP_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
          
    TRAFFIC_MAP_ROADMARKING RoadMarking[TRAFFIC_MAP_MAX_ROADMARKINGNUM];
    int RoadMarkingNum;
    TRAFFIC_MAP_KEYPOINT KeyPoint[TRAFFIC_MAP_MAX_KEYPOINTNUM];
    int KeyPointNum;
    TRAFFIC_MAP_LANEMARKING LaneMarking[TRAFFIC_MAP_MAX_LANEMARKINGNUM];    //������
    int LaneMarkingNum;
    TRAFFIC_MAP_ROAD Road[TRAFFIC_MAP_MAX_ROADNUM];    //·
    int RoadNum;
    TRAFFIC_MAP_GUIDELINE  GuideLine;
    int GuideLineNum;
    TRAFFIC_MAP_POSITION MapPosition;
    int ObstacleNum;
    TRAFFIC_MAP_OBSTACLE Obstacle[TRAFFIC_MAP_MAX_OBSTACLE];    //�ϰ���
};

// Declare NML format function
extern int TrafficMapFormat(NMLTYPE, void *, CMS *);

#endif 	// TRAFFICMAPN_HH
