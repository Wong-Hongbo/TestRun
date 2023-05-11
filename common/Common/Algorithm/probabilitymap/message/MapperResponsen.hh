#ifndef MAPPERRESPONSE_HH
#define MAPPERRESPONSE_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"
#include "LocalPosen.hh"

// Define the integer type ids.
#define MAPPERRESPONSE_MSG_TYPE 15927
#define MAP2D_SIZE 800*800 //10000*10000  grid size 20cm
//#define UINT8	unsigned char

typedef struct
{
    double minx;     // 单位cm
    double miny;
    double maxx;
    double maxy;
    int gridsize;     // cm
} RangeData;

typedef struct
{
    double x;     // 单位 m
    double y;
    double yaw;     // rad
} PositionInfo;

typedef struct
{
    int  task;      // task 0: 无任务状态； 1:建图任务已开始（正在建图） 2：建图任务已结束 3: 保存地图任务已完成
                    //4：不保存地图任务已完成 6:生成地图已开始（正在生成地图） 7：生成地图已结束； 8：获取地图已结束
    int  status;    // 1：成功；  0：失败；  2：进行中； 针对task1（3：地图闭环中；    4：闭环成功；      5：闭环失败；）
    char status_msg[1024];
    int  progress;
}taskStatus;

typedef struct
{
    taskStatus status;
} MapperStatus;

typedef struct
{
    // 获取建图回传信息
    PositionInfo pose;             // 获取实时位置信息
    UINT8        Map2DImage[MAP2D_SIZE];
 // 2D地图 totalmap
    RangeData rangeinfo;           // 地图原地信息 range.txt
} MapperTile;

typedef struct
{
    // 获取建图回传信息
//    PositionInfo pose;             // 获取实时位置信息
    char resultpath[1024];
    RangeData rangeinfo;           // 地图原地信息 range.txt
    char pcdpath[1024];
} MapperResult;

// Define the NML Message Classes
class MAPPERRESPONSE_MSG : public NMLmsgEx
{
public:
    //Constructor
    MAPPERRESPONSE_MSG();
    // CMS Update Function
    void update(CMS *);
/**
 *  MsgCode: 1 ,应答start 开始建图
            MsgContent: MapperStatus （成功则表示开始建图，同时开始上报MsgCod6,瓦片地图。
                                        失败则表示无法开始建图）
    MsgCode: 2 ,应答stop 结束建图
            MsgContent: MapperStatus
    MsgCode: 3 ,应答save 建图
            MsgContent: MapperStatus
    MsgCode: 8 ,应答Not Save 不保存
            MsgContent: MapperStatus
    收到MsgCode: 9 ,应答生成地图
            MsgContent: MapperStatus （成功则表示开始生成地图，同时开始上报MsgCod7,进度。
                                        失败则表示无法开始生成地图）

    MsgCode: 10,应答增加闭环点
            MsgContent: MapperStatus
    MsgCode: 11,应答匹配闭环点
            MsgContent: MapperStatus
    MsgCode: 12,应答获取建图任务状态
            MsgContent: MapperStatus

    MsgCode: 5 , 应答 Result Info
            MsgContent: MapperResult

    MsgCode: 6 , report tile
            MsgContent: MapperTile
    MsgCode: 7 , report status
            MsgContent: MapperStatus
    MsgCode：13,暂停建图
            MsgContent：MapperStatus
    MsgCode：14,恢复建图
            MsgContent：MapperStatus
    MsgCode: 15,地图切换完成
            MsgContent: MapperStatus

**/
    UINT8 MsgCode;

    UINT8 MsgContent[800*900];
    int seq_num;
};

// Declare NML format function
extern int MapperResponseFormat(NMLTYPE, void *, CMS *);

#endif 	// MAPPERRESPONSE_HH
