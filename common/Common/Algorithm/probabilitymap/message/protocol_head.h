#ifndef PROTOCOL_HEAD_H_
#define PROTOCOL_HEAD_H_

// version 0.8.2 增加注释
// version 0.8.1 增加注释
// version 0.8
//1. general msg 基础定义
//Request
struct  CommonBase {
    unsigned char head; //0x7e
    unsigned char version;
    int req_id; // 命令id 
    int seq_id; // 流水号
    long seq_time; //ms 发送时间
};

//Response:
struct CommonResponse {
    CommonBase base; // req_id, 对应所回应消息的req_id
                    // seq_id, 对应所回应消息的seq_id
                    // seq_time, 发送消息的时间
    int result; //0: success; -1: error; else: error code
    char result_msg[128]; // error msg
};
//general msg end-----------------------

//2. Localisation Init 
//使用LocalisationInitResponsen.hh　和　LocalisationInitRequestn.hh中定义的ｍｓｇ

/**
 * 发送方发送内容：
        //0 定位成功
        MsgContent: LocalControl
        //1 use gps mode
        MsgContent: LocalControl
        //2 get stations(response stations),
        MsgContent: LocalControl
        //3 manual mode
        MsgContent: LocalControl
        //4 定位失败
        MsgContent: LocalControl
        //5 获取底图和雷达图
        MsgContent: LocalControl
        //6 检查定位结果
        MsgContent: LocalControl
        //7 获取大底图(回应底图)
        MsgContent: LocalControl
        //11 暂停
        MsgContent: LocalControl
        //12 恢复
        MsgContent: LocalControl
        //13 停止
        MsgContent: LocalControl
        //14 获取定位初始化进度状态l
        MsgContent: LocalControl
        //8 定位成功 + pose
        MsgContent: LocalControlWithPose
        //9 manual mode(no gps, with pose)
        MsgContent: LocalControlWithPose
        //10 station mode(with position) 
        MsgContent: LocalControlWithPosition

 * 接收方回应内容：
        //0 定位成功
        MsgContent: LocalControlResponse
        //1 use gps mode
        MsgContent: LocalControlResponse
        //3 manual mode
        MsgContent: LocalControlResponse
        //4 定位失败
        MsgContent: LocalControlResponse
        //8 定位成功 + pose
        MsgContent: LocalControlResponse
        //9 manual mode(no gps, with pose)
        MsgContent: LocalControlResponse
        //10 station mode(with position)
        MsgContent: LocalControlResponse
        //11 暂停
        MsgContent: LocalControlResponse
        //12 恢复
        MsgContent: LocalControlResponse
        //13 停止
        MsgContent: LocalControlResponse
        //5 获取底图和雷达图
        MsgContent: LocalMatchResult
        //6 检查定位结果
        MsgContent: LocalMatchResult
        //7 获取大底图(回应底图)
        MsgContent: LocalMatchResult
        //2 get stations(response stations),
        MsgContent: LocalStations
        //14 获取定位初始化进度状态
        MsgContent: LocalProgressStatus
**/

// request:
struct  LocalisationPose{
    int x;   // cm gauss
    int y;
};

struct LocalisationPoseWithYaw {
    int x;   // cm
    int y;
    int yaw;  // degree(0~36000)
};


struct LocalControl {
    CommonBase base;// req_id:
                //0 定位成功
                //1 use gps mode
                //2 get stations(response stations),
                //3 manual mode
                //4 定位失败
                //5 获取底图和雷达图
                //6 检查定位结果
                //7 获取大底图(回应底图)
                //11 暂停
                //12 恢复
                //13 停止
                //14 获取定位初始化进度状态
                
};


struct LocalControlWithPose {
    CommonBase base;// req_id:
                //8 定位成功 + pose
                //9 manual mode(no gps, with pose)
    LocalisationPose pose;
};

struct LocalControlWithPosition {
    CommonBase base;// req_id:
                //10 station mode(with position)
    LocalisationPoseWithYaw pose;
};
// response:
#define INITIALIZE_SIZE 100

struct POSE_T
{
    int x;
    int y;
    int yaw;
};

struct RangeData_T {
    double minx;
    double miny;
    double maxx;
    double maxy;
    int gridsize;
};

struct MatchResult
{
    POSE_T result_pose;
    char mappath[1024];
    char pointcloudpath[1024];
    RangeData_T rangeinfo;
    int score;
};

struct STATION {
    char name[200];
    int id;
    POSE_T pose;
};

struct InitStationLists {
    unsigned char station_num;
    STATION station[INITIALIZE_SIZE];
};

struct LocalControlResponse {
    CommonResponse response;
                // req_id:
                //0 定位成功
                //1 use gps mode
                //3 manual mode
                //4 定位失败
                //8 定位成功 + pose
                //9 manual mode(no gps, with pose)
                //10 station mode(with position)
                //11 暂停
                //12 恢复
                //13 停止
                
};

struct LocalMatchResult {
    CommonResponse response;//5 获取底图和雷达图
                    //6 检查定位结果
                    //7 获取大底图(回应底图)
    MatchResult result;
};
                
struct LocalStations {
    CommonResponse response;//2 get stations(response stations),
    InitStationLists stations;
};

// 	enum status 
//     {
//         NORMAL = 0; 
//         START_LOCAL = 1;
//         STOP_LOCAL = 2;
//         PAUSE_LOCAL = 3;
//         RESUME_LOCAL = 4;
//     }

struct LocalProgressStatus {
    CommonResponse response;//14 获取定位初始化进度状态
    int status;
};
// Localisation Init end----------------------------

//3. control command
// 使用ControlCommandn.hh中定义的msg
/**
 * 发送方发送内容：
    req_id: 0x1 ,暂停工作
            struct ControlCommand
    req_id: 0x2 ,恢复工作 
            struct ControlCommand
    req_id: 0x3 ,重新初始化 （如果使用地图，此时重新加载地图）,进入工作状态
            struct ControlCommand
    req_id: 0x4 ,加载地图
            struct LoadMap
    req_id: 0x5 ,虚拟墙已更新
            struct UpdateVirtualWall

    

 * 接收方回应内容：
    req_id: 0x1 ,暂停工作
            struct ControlResult
    req_id: 0x2 ,恢复工作
            struct ControlResult
    req_id: 0x3 ,重新初始化
            struct ControlResult
    req_id: 0x4 ,加载地图
            struct ControlResult
    req_id: 0x5 ,虚拟墙已更新
            struct ControlResult
**/

struct ControlCommand {
    CommonBase base; //req_id:
//      0x1 ,暂停工作
//      0x2 ,恢复工作
//      0x3 ,重新初始化
};

struct LoadMap {
    CommonBase base; // req_id: 0x4 ,加载地图
    char map_name[500];
};

struct UpdateVirtualWall {
    CommonBase base; // req_id: 0x5 ,虚拟墙已更新
    char virtualwall_path[500];
};

struct ControlResult {
    CommonResponse response;
};

// control command end --------------------------




#endif
