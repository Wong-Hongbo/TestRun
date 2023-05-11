/************************************************************************/
/*                                                                      */
/*                                  公共处理类                          */
/*                                                                      */
/*                                                                      */
/************************************************************************/

#ifndef __COMMON__
#define __COMMON__
#include <iostream>
#include <sstream>
#include <vector>
#include <mutex>
#include <thread>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <cstring>
#include <cstdio>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <string>
#include <sys/time.h>
#include <time.h>
#include <pwd.h>
#include <sys/prctl.h>


#include <boost/serialization/singleton.hpp>   //ubuntu
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include "CommonDefinitionX.hh"


#define UGVPublicNetMask 0xAC1764FF   //172.23.100.255

namespace com
{


#define SetToNULL(p) p = NULL

/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                                 日志输出打印                         */
/*                                                                      */
/*                                                                      */
/************************************************************************/

//获取当前系统时间
extern char *GetCurrentTime(char *t);

//获取当前系统时间对应的毫秒
extern double getCurrentMillSecond();


class TraceLog
{
public:
    TraceLog();
    ~TraceLog();
    /**
     * @brief setTraceLog:该函数用于初始化LOG
     * @param const char *f:程序名
     * @param  const char *p:LOG文件夹路径
     * @param int level:LOG等级
     * @param int fflush_mode:刷新模式——1：实时刷新，其他：异步缓存刷新
     * @param int reserved_day:日志保存时间
     * @param int divide_log:LOG按等级区分文件
     * @return 成功标志位
     */
    bool setTraceLog(const char *f, const char *p, int level, int fflush_mode, int reserved_day, int divide_log);
    FILE* getTraceLog(int log_mode);
    int getDivideLog();
    int getTraceLevel();
    int getTraceFlush();
    bool m_main_thread_state;
    std::vector<std::string> m_log_list;
    std::vector<std::string> m_log_info_list;
    std::vector<std::string> m_log_error_list;
    std::vector<std::string> m_log_debug_list;
    std::mutex m_mtx_log;

private:
    static void *SaveLogCallback(void *arg);
    FILE *m_logfp;
    FILE *m_log_info_fp;
    FILE *m_log_error_fp;
    FILE *m_log_debug_fp;
    int m_divide_log;
    int m_level;
    int m_fflush_mode;
    std::thread m_savelog_thread;

};


typedef boost::serialization::singleton<com::TraceLog> GlobeLogFp; //ubuntu
#define SINGLETON_LOG com::GlobeLogFp::get_mutable_instance() //ubuntu


//这里使用了logfp是文件句柄指针
#define LogInfo(form, info...) do\
{\
    char t[32] = {0};\
    com::GetCurrentTime(t);\
    if(SINGLETON_LOG.getDivideLog() == 1)\
    {\
        if (SINGLETON_LOG.getTraceLog(1) != NULL)\
        {\
            if (SINGLETON_LOG.getTraceFlush() == 1)\
            {\
                fprintf(SINGLETON_LOG.getTraceLog(1), "[%s][INFO]" form, t, ##info);\
                fflush(SINGLETON_LOG.getTraceLog(1));\
            }\
            else\
            {\
                char log_data[2048];\
                sprintf(log_data, "[%s][INFO]" form, t, ##info);\
                std::string log_data_str = log_data;\
                SINGLETON_LOG.m_mtx_log.lock();\
                SINGLETON_LOG.m_log_info_list.push_back(log_data_str);\
                SINGLETON_LOG.m_mtx_log.unlock();\
            }\
        }\
        else\
        {\
            printf("[%s][INFO]" form, t, ##info);\
        }\
    }\
    else\
    {\
        if (SINGLETON_LOG.getTraceLog(0) != NULL)\
        {\
            if (SINGLETON_LOG.getTraceFlush() == 1)\
            {\
                fprintf(SINGLETON_LOG.getTraceLog(0), "[%s][INFO]" form, t, ##info);\
                fflush(SINGLETON_LOG.getTraceLog(0));\
            }\
            else\
            {\
                char log_data[2048];\
                sprintf(log_data, "[%s][INFO]" form, t, ##info);\
                std::string log_data_str = log_data;\
                SINGLETON_LOG.m_mtx_log.lock();\
                SINGLETON_LOG.m_log_list.push_back(log_data_str);\
                SINGLETON_LOG.m_mtx_log.unlock();\
            }\
        }\
        else\
        {\
            printf("[%s][INFO]" form, t, ##info);\
        }\
    }\
}while(0)


#define LogError(form, info...) do\
{\
    char t[32] = {0};\
    com::GetCurrentTime(t);\
    if(SINGLETON_LOG.getDivideLog() == 1)\
    {\
        if (SINGLETON_LOG.getTraceLog(2) != NULL)\
        {\
            if (SINGLETON_LOG.getTraceFlush() == 1)\
            {\
                fprintf(SINGLETON_LOG.getTraceLog(2), "[%s][ERROR]" form, t, ##info);\
                fflush(SINGLETON_LOG.getTraceLog(2));\
            }\
            else\
            {\
                char log_data[2048];\
                sprintf(log_data, "[%s][ERROR]" form, t, ##info);\
                std::string log_data_str = log_data;\
                SINGLETON_LOG.m_mtx_log.lock();\
                SINGLETON_LOG.m_log_error_list.push_back(log_data_str);\
                SINGLETON_LOG.m_mtx_log.unlock();\
            }\
        }\
        else\
        {\
            printf("[%s][ERROR]" form, t, ##info);\
        }\
    }\
    else\
    {\
        if (SINGLETON_LOG.getTraceLog(0) != NULL)\
        {\
            if (SINGLETON_LOG.getTraceFlush() == 1)\
            {\
                fprintf(SINGLETON_LOG.getTraceLog(0), "[%s][ERROR]" form, t, ##info);\
                fflush(SINGLETON_LOG.getTraceLog(0));\
            }\
            else\
            {\
                char log_data[2048];\
                sprintf(log_data, "[%s][ERROR]" form, t, ##info);\
                std::string log_data_str = log_data;\
                SINGLETON_LOG.m_mtx_log.lock();\
                SINGLETON_LOG.m_log_list.push_back(log_data_str);\
                SINGLETON_LOG.m_mtx_log.unlock();\
            }\
        }\
        else\
        {\
            printf("[%s][ERROR]" form, t, ##info);\
        }\
    }\
}while(0)

#define LogDebug(level, form, info...) do\
{\
    if (SINGLETON_LOG.getTraceLevel() >= level)\
    {\
        char t[32] = {0};\
        com::GetCurrentTime(t);\
        if(SINGLETON_LOG.getDivideLog() == 1)\
        {\
            if (SINGLETON_LOG.getTraceLog(3) != NULL)\
            {\
                if (SINGLETON_LOG.getTraceFlush() == 1)\
                {\
                    fprintf(SINGLETON_LOG.getTraceLog(3), "[%s][DEBUG]" form, t, ##info);\
                    fflush(SINGLETON_LOG.getTraceLog(3));\
                }\
                else\
                {\
                    char log_data[2048];\
                    sprintf(log_data, "[%s][DEBUG]" form, t, ##info);\
                    std::string log_data_str = log_data;\
                    SINGLETON_LOG.m_mtx_log.lock();\
                    SINGLETON_LOG.m_log_debug_list.push_back(log_data_str);\
                    SINGLETON_LOG.m_mtx_log.unlock();\
                }\
            }\
            else\
            {\
                printf("[%s][DEBUG]" form, t, ##info);\
            }\
        }\
        else\
        {\
            if (SINGLETON_LOG.getTraceLog(0) != NULL)\
            {\
                if (SINGLETON_LOG.getTraceFlush() == 1)\
                {\
                    fprintf(SINGLETON_LOG.getTraceLog(0), "[%s][DEBUG]" form, t, ##info);\
                    fflush(SINGLETON_LOG.getTraceLog(0));\
                }\
                else\
                {\
                    char log_data[2048];\
                    sprintf(log_data, "[%s][DEBUG]" form, t, ##info);\
                    std::string log_data_str = log_data;\
                    SINGLETON_LOG.m_mtx_log.lock();\
                    SINGLETON_LOG.m_log_list.push_back(log_data_str);\
                    SINGLETON_LOG.m_mtx_log.unlock();\
                }\
            }\
            else\
            {\
                printf("[%s][DEBUG]" form, t, ##info);\
            }\
        }\
    }\
}while(0)


/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                          运行时间计时                                */
/*                                                                      */
/*                                                                      */
/************************************************************************/
class timer
{
public:
    timer();
    void begin();
    double elapsed(); //返回的是微妙
    int calElapsedMin();  //返回使用的分钟
    int calElapsedSec();  //返回使用的秒
    int calElapsedMill();    //返回的是毫秒
    void getRecordTime(struct timeval& _t);

private:
    struct timeval _start_time;
};


/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                    程序信号操作                                      */
/*                                                                      */
/*                                                                      */
/************************************************************************/
extern bool setExitCode();
extern bool getExitCode();
extern bool checkExitCode();


/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                       socket     操作                                */
/*                                                                      */
/*                                                                      */
/************************************************************************/

#define PORT 2020    //通讯端口
#define BACKLOG 20
#define MAXFILEPATH 256

#define STRCMPFALSERETURN(a, b) do\
{\
    if (strcmp(a, b) != 0)\
    {\
        return false;\
    }\
}while(0)

#define STRCMPFALSECONTINUE(a, b) do\
{\
    if (strcmp(a, b) != 0)\
    {\
        sleep(3);\
        continue;\
    }\
}while(0)


/*
 自定义应用层协议
 第一个字节表示命令类型，
 接下来4个字节表示数据长度，
 再接下来就是具体数据
*/
enum LENGTH
{
    header_length = 1,
    data_length = 4,
    max_data_length = 1000
};

class message
{
public:
    message(int fd);

    void reset(int fd);

    bool recvMsg();

    bool sendMsg(const char* mode, const char* data);

    const char* mode() const;

    int length() const;

    const char* data() const;

private:
    void clear();

private:
    char m_mode[header_length + 1];
    int  m_length;
    char m_data[max_data_length + 1];
    int  m_fd;
};



//信号量

union semun
{
    int  val;
    struct semid_ds* buf;
    unsigned short* array;
};


class sem
{
public:
    //create
    bool init(int Key, int val = 1);

    bool create(int key);

    bool get(int key);
    //lock
    void lock();
    //unlock
    void unlock();

    bool del();

    void wait();

    void post();
private:
    int m_semid;

    //FILE *logfp;
};


/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                             file操作                                 */
/*                                                                      */
/*                                                                      */
/************************************************************************/

class file
{
public:
    file();
    ~file();
    //创建文件
    bool Create(const char* str, char* filename);
    //打开文件
    bool Open(const char* filename);
    //读取文件
    int ReadData(char* data);
    //写入文件
    bool WriteData(const char* data);
    //检查文件
    size_t FileSize();
private:
    char _fileName[MAXFILEPATH];
    size_t _fileSize;
    int  _fileDes;
};


/************************************************************************/
/*                                                                      */
/*                           读取ini配置文件                            */
/*                                                                      */
/*                                                                      */
/************************************************************************/
class iniConf
{
public:
    iniConf();
    iniConf(const char* fileConf);
    ~iniConf();

    bool openFile(const char* fileConf);

    int getValue(std::string Key, int Default, char Sep = '.');

    std::string getValue(std::string Key, std::string Default, char Sep = '.');

private:
    boost::property_tree::ptree m_pt;

};


typedef boost::serialization::singleton<com::iniConf> GlobeIniConf; //ubuntu
#define SINGLETON_INICONF GlobeIniConf::get_mutable_instance() //ubuntu


#define UGVGLOBALPOINTNUM 500
#define UGVSTATUSHEAD (0xA6)
#define UGVMISSIONHEAD (0xA7)
#define UGVMAXCLIENT 200
#define MAXEPOLLSIZE (5+UGVMAXCLIENT+1)


enum UGVGLOBALPOINTTYPE
{
    UGV_TYPE_FOLLOW = 0,
    UGV_TYPE_AUTO = 1
};


//基于经纬度的坐标点
struct ugvGlobalPoint
{
    double x;
    double y;
    UINT32 speed;
    UINT8 type;
};


//车体上报的状态信息
struct ugvTransmitStatus
{
    UINT8 head;
    UINT32 primaryKey;
    INT64 time;    //时间标记
    UINT8 fuel;    //油量0 - 100
    UINT8 shift;    //档位
    UINT8 throttle;    //油门
    UINT8 brake;    //刹车
    INT16 steer;    //前轮转角
    UINT8 navMode;    //
    INT32 speedX;    //横向速度cm /s
    INT32 speedY;    //纵向速度cm/s
    INT32 speedZ;    //上下速度cm/s
    INT32 gaussX;    //x坐标，cm，向东为正
    INT32 gaussY;    //y坐标，cm，向北为正
    INT32 height;    //高度，向上为正cm
    INT32 heading;    //航向角，单位0.01度向东为0度，逆时针0-36000
    INT32 roll;    //侧滚角，单位0.01度，向右为零度，逆时针-9000-9000
    INT32 pitch;    //俯仰角，单位0.01度，向前水平为零度，逆时针-9000-9000
    UINT8 sysSts;    //系统状态0,error 1 ok
public:
    ugvTransmitStatus()
    {
        head = UGVSTATUSHEAD;
    }

    bool check()
    {
        return head == UGVSTATUSHEAD;
    }
};


//下发给车体的任务
struct ugvTransmitMission
{
    UINT8 head;
    UINT32 primaryKey;
    INT64 time;    //时间标记

    INT32 pointNum;
    ugvGlobalPoint globalPoints[UGVGLOBALPOINTNUM];

public:

    ugvTransmitMission()
    {
        head = UGVMISSIONHEAD;
    }

    bool check()
    {
        return head == UGVMISSIONHEAD;
    }
};


class ugvTransmitMsg
{
public:
    ugvTransmitMsg(int readLen, int writeLen);

    ~ugvTransmitMsg();

    void reset(int fd);

    int write(void *data, int timeout = 500);  //mill second

    int read(void *data, int timeout = 500);  //mill second

    int getReadLength();

    int getWriteLength();

private:
    int m_fd;

    int m_readLength;

    int m_writeLength;

};

struct Computer_status
{
    char hostname[10];

    UINT8 cpuUseRate; //0 - 100
    UINT8 memUseRate; //0 - 100
    //网络流量kb/s
    INT32 netTranferKB;
    //Io read kb/s
    INT32 ioReadKB;

    //Io write kb/s
    INT32 ioWriteKB;
};

#define SENSORCOLLECT
#ifdef SENSORCOLLECT


struct SensorDataCollectCmd
{
    UINT8 Cmd_NewSave;        // 0-no new save, if this process is saving info, and Cmd_NewSave=1, start a new save
    UINT8 Cmd_SaveType;     // 0-zhengchang 1-xiulu 2-chalu 3-shoufeizhan 4-duche 5-weizhi
    UINT8 Cmd_StartOnline;    // 0-no command, 1-work, 2-sleep
    UINT8 Cmd_SaveData;     // 0-no command, 1-save, 2-save stop
    UINT8 Cmd_StartOffline; // 0-no command, 1-work offline, 2-sleep offline

    UINT8 Cmd_SaveColorImage;  //0-not save,1-save
    UINT8 Cmd_SaveLadar64;    //0-not save,1-save
    UINT8 Cmd_SaveLadar32;    //0-not save,1-save
    UINT8 Cmd_SaveLadar8;    //0-not save,1-save
    UINT8 Cmd_SaveRadar;    //0-not save,1-save
    UINT8 Cmd_SaveIRImage;    //0-not save,1-save

    SensorDataCollectCmd()
    {
        reset();
    }

    void reset()
    {
        Cmd_NewSave        = 0;
        Cmd_SaveType       = 0;
        Cmd_StartOnline    = 0;
        Cmd_SaveData       = 0;
        Cmd_StartOffline   = 0;
        Cmd_SaveColorImage = 1;
        Cmd_SaveLadar64 = 1;
        Cmd_SaveLadar32 = 1;
        Cmd_SaveLadar8 = 1;
        Cmd_SaveRadar = 1;
        Cmd_SaveIRImage = 1;
    }

    void print()
    {
        printf("Cmd_NewSave :%d.\n", Cmd_NewSave);
        printf("Cmd_SaveType :%d.\n", Cmd_SaveType);
        printf("Cmd_StartOnline :%d.\n", Cmd_StartOnline);
        printf("Cmd_SaveData :%d.\n", Cmd_SaveData);
        printf("Cmd_StartOffline :%d.\n", Cmd_StartOffline);

        printf("Cmd_SaveColorImage :%d.\n", Cmd_SaveColorImage);
        printf("Cmd_SaveLadar64 :%d.\n", Cmd_SaveLadar64);
        printf("Cmd_SaveLadar32 :%d.\n", Cmd_SaveLadar32);
        printf("Cmd_SaveLadar8 :%d.\n", Cmd_SaveLadar8);
        printf("Cmd_SaveRadar :%d.\n", Cmd_SaveRadar);
        printf("Cmd_SaveIRImage :%d.\n", Cmd_SaveIRImage);

    }

};

#define SensorPathLength 128

struct SensorDataManagerCmd: public SensorDataCollectCmd
{
    char CurrentSaveBase[SensorPathLength];
    char CurrentSaveDay[SensorPathLength];
    char CurrentSavePath[SensorPathLength];

    char CurrentColorImageLPath[SensorPathLength];
    char CurrentColorImageRPath[SensorPathLength];
    char CurrentLadar64Path[SensorPathLength];
    char CurrentLadar32Path[SensorPathLength];
    char CurrentLadar8Path[SensorPathLength];
    char CurrentRadarPath[SensorPathLength];
    char CurrentIRImagePath[SensorPathLength];
    char CurrentLocalPosePath[SensorPathLength];
    char CurrentGlobalPositionPath[SensorPathLength];
    char CurrentTimeStampPath[SensorPathLength];
    char CurrentCalibrationPath[SensorPathLength];

    void getCmd(SensorDataCollectCmd* cmd)
    {
        Cmd_NewSave        = cmd->Cmd_NewSave;
        Cmd_SaveType       = cmd->Cmd_SaveType;
        Cmd_StartOnline    = cmd->Cmd_StartOnline;
        Cmd_SaveData       = cmd->Cmd_SaveData;
        Cmd_StartOffline   = cmd->Cmd_StartOffline;
        Cmd_SaveColorImage = cmd->Cmd_SaveColorImage;
        Cmd_SaveLadar64    = cmd->Cmd_SaveLadar64;
        Cmd_SaveLadar32    = cmd->Cmd_SaveLadar32;
        Cmd_SaveLadar8     = cmd->Cmd_SaveLadar8;
        Cmd_SaveRadar      = cmd->Cmd_SaveRadar;
        Cmd_SaveIRImage    = cmd->Cmd_SaveIRImage;
    }

    void getCmd(SensorDataManagerCmd *cmd)
    {
        memcpy(this, cmd, sizeof(SensorDataManagerCmd));
        /*
        Cmd_NewSave        = cmd->Cmd_NewSave;
        Cmd_SaveType       = cmd->Cmd_SaveType;
        Cmd_StartOnline    = cmd->Cmd_StartOnline;
        Cmd_SaveData       = cmd->Cmd_SaveData;
        Cmd_StartOffline   = cmd->Cmd_StartOffline;
        Cmd_SaveColorImage = cmd->Cmd_SaveColorImage;
        Cmd_SaveLadar64    = cmd->Cmd_SaveLadar64;
        Cmd_SaveLadar32    = cmd->Cmd_SaveLadar32;
        Cmd_SaveLadar8     = cmd->Cmd_SaveLadar8;
        Cmd_SaveRadar      = cmd->Cmd_SaveRadar;
        Cmd_SaveIRImage    = cmd->Cmd_SaveIRImage;
        strcpy(CurrentSaveBase, cmd->CurrentSaveBase);
        strcpy(CurrentSaveDay, cmd->CurrentSaveDay);
        strcpy(CurrentSavePath, cmd->CurrentSavePath);

        strcpy(CurrentColorImageLPath, cmd->CurrentColorImageLPath);
        strcpy(CurrentColorImageRPath, cmd->CurrentColorImageRPath);
        strcpy(CurrentLadar64Path, cmd->CurrentLadar64Path);
        strcpy(CurrentLadar32Path, cmd->CurrentLadar32Path);
        strcpy(CurrentLadar8Path, cmd->CurrentLadar8Path);
        strcpy(CurrentRadarPath, cmd->CurrentRadarPath);
        strcpy(CurrentIRImagePath, cmd->CurrentIRImagePath);
        strcpy(CurrentLocalPosePath, cmd->CurrentLocalPosePath);
        strcpy(CurrentGlobalPositionPath, cmd->CurrentGlobalPositionPath);
        strcpy(CurrentTimeStampPath, cmd->CurrentTimeStampPath);
        strcpy(CurrentCalibrationPath, cmd->CurrentCalibrationPath);
        */
    }


    void setCurrentPath()
    {
        strcpy(CurrentColorImageLPath, CurrentSavePath);
        strcpy(CurrentColorImageRPath, CurrentSavePath);
        strcpy(CurrentLadar64Path, CurrentSavePath);
        strcpy(CurrentLadar32Path, CurrentSavePath);
        strcpy(CurrentLadar8Path, CurrentSavePath);
        strcpy(CurrentRadarPath, CurrentSavePath);
        strcpy(CurrentIRImagePath, CurrentSavePath);
        strcpy(CurrentLocalPosePath, CurrentSavePath);
        strcpy(CurrentGlobalPositionPath, CurrentSavePath);
        strcpy(CurrentTimeStampPath, CurrentSavePath);
        strcpy(CurrentCalibrationPath, CurrentSavePath);

        strcat(CurrentColorImageLPath, "/");
        strcat(CurrentColorImageRPath, "/");
        strcat(CurrentLadar64Path, "/");
        strcat(CurrentLadar32Path, "/");
        strcat(CurrentLadar8Path, "/");
        strcat(CurrentRadarPath, "/");
        strcat(CurrentIRImagePath, "/");
        strcat(CurrentLocalPosePath, "/");
        strcat(CurrentGlobalPositionPath, "/");
        strcat(CurrentTimeStampPath, "/");
        strcat(CurrentCalibrationPath, "/");
    }
};

struct SensorContainer
{
    char name[32];
    pid_t pid;

    SensorContainer()
    {
        reset();
    }

    SensorContainer(const char* n, int p)
    {
        strcpy(name ,n);
        pid = p;
    }
    void reset()
    {
        memset(name ,0 , sizeof(name));
        pid = 0;
    }
};

#endif
////***********    ugv 数据采集**************//


struct UgvDataCollectCmd
{
    UINT8 Cmd_NewSave;        // 0-no new save, if this process is saving info, and Cmd_NewSave=1, start a new save
    UINT8 Cmd_SaveType;     // 0-zhengchang 1-xiulu 2-chalu 3-shoufeizhan 4-duche 5-weizhi
    UINT8 Cmd_StartOnline;    // 0-no command, 1-work, 2-sleep
    UINT8 Cmd_SaveData;     // 0-no command, 1-save, 2-save stop
    UINT8 Cmd_StartOffline; // 0-no command, 1-work offline, 2-sleep offline

    UINT8 Cmd_SaveColorImage;  //0-not save,1-save
    UINT8 Cmd_SaveLadar64;    //0-not save,1-save
    UINT8 Cmd_SaveLadar32;    //0-not save,1-save
    UINT8 Cmd_SaveLadar8;    //0-not save,1-save
    UINT8 Cmd_SaveRadar;    //0-not save,1-save
    UINT8 Cmd_SaveIRImage;    //0-not save,1-save

    UINT8 Cmd_SaveLocalAttributeMap;    //0-not save,1-save
    UINT8 Cmd_SaveEntityMap;    //0-not save,1-save
    UINT8 Cmd_SaveLocalPathInfo;    //0-not save,1-save

    UgvDataCollectCmd()
    {
        reset();
    }

    void reset()
    {
        Cmd_NewSave        = 0;
        Cmd_SaveType       = 0;
        Cmd_StartOnline    = 0;
        Cmd_SaveData       = 0;
        Cmd_StartOffline   = 0;
        Cmd_SaveColorImage = 1;
        Cmd_SaveLadar64 = 1;
        Cmd_SaveLadar32 = 1;
        Cmd_SaveLadar8 = 1;
        Cmd_SaveRadar = 1;
        Cmd_SaveIRImage = 1;
        Cmd_SaveLocalAttributeMap = 1;
        Cmd_SaveEntityMap = 1;
        Cmd_SaveLocalPathInfo = 1;
    }

    void print()
    {
        printf("Cmd_NewSave :%d.\n", Cmd_NewSave);
        printf("Cmd_SaveType :%d.\n", Cmd_SaveType);
        printf("Cmd_StartOnline :%d.\n", Cmd_StartOnline);
        printf("Cmd_SaveData :%d.\n", Cmd_SaveData);
        printf("Cmd_StartOffline :%d.\n", Cmd_StartOffline);

        printf("Cmd_SaveColorImage :%d.\n", Cmd_SaveColorImage);
        printf("Cmd_SaveLadar64 :%d.\n", Cmd_SaveLadar64);
        printf("Cmd_SaveLadar32 :%d.\n", Cmd_SaveLadar32);
        printf("Cmd_SaveLadar8 :%d.\n", Cmd_SaveLadar8);
        printf("Cmd_SaveRadar :%d.\n", Cmd_SaveRadar);
        printf("Cmd_SaveIRImage :%d.\n", Cmd_SaveIRImage);
        printf("Cmd_SaveLocalAttributeMap :%d.\n", Cmd_SaveLocalAttributeMap);
        printf("Cmd_SaveEntityMap :%d.\n", Cmd_SaveEntityMap);
        printf("Cmd_SaveLocalPathInfo: %d.\n", Cmd_SaveLocalPathInfo);

    }

};

#define UgvPathLength 128

struct UgvDataManagerCmd: public UgvDataCollectCmd
{
    char CurrentSaveBase[UgvPathLength];     //存放数据的根目录
    char CurrentSaveDay[UgvPathLength];
    char CurrentSaveSec[UgvPathLength];

    char CurrentSaveSensor[UgvPathLength];
    char CurrentSaveModule[UgvPathLength];
    char CurrentSaveReport[UgvPathLength];

    char CurrentColorImageLPath[UgvPathLength];
    char CurrentColorImageRPath[UgvPathLength];
    char CurrentLadar64Path[UgvPathLength];
    char CurrentLadar32Path[UgvPathLength];
    char CurrentLadar8Path[UgvPathLength];
    char CurrentRadarPath[UgvPathLength];
    char CurrentIRImagePath[UgvPathLength];
    char CurrentLocalPosePath[UgvPathLength];
    char CurrentGlobalPositionPath[UgvPathLength];
    char CurrentTimeStampPath[UgvPathLength];
    char CurrentCalibrationPath[UgvPathLength];

    char CurrentLocalAttributeMapPath[UgvPathLength];
    char CurrentEntityMapPath[UgvPathLength];
    char CurrentLocalPathInfoPath[UgvPathLength];

    void getCmd(UgvDataCollectCmd* cmd)
    {
        Cmd_NewSave        = cmd->Cmd_NewSave;
        Cmd_SaveType       = cmd->Cmd_SaveType;
        Cmd_StartOnline    = cmd->Cmd_StartOnline;
        Cmd_SaveData       = cmd->Cmd_SaveData;
        Cmd_StartOffline   = cmd->Cmd_StartOffline;
        Cmd_SaveColorImage = cmd->Cmd_SaveColorImage;
        Cmd_SaveLadar64    = cmd->Cmd_SaveLadar64;
        Cmd_SaveLadar32    = cmd->Cmd_SaveLadar32;
        Cmd_SaveLadar8     = cmd->Cmd_SaveLadar8;
        Cmd_SaveRadar      = cmd->Cmd_SaveRadar;
        Cmd_SaveIRImage    = cmd->Cmd_SaveIRImage;
        Cmd_SaveLocalAttributeMap = cmd->Cmd_SaveLocalAttributeMap;
        Cmd_SaveEntityMap         = cmd->Cmd_SaveEntityMap;
        Cmd_SaveLocalPathInfo     = cmd->Cmd_SaveLocalPathInfo;
    }

    void getCmd(UgvDataManagerCmd *cmd)
    {
        memcpy(this, cmd, sizeof(UgvDataManagerCmd));
    }


    void setCurrentPath()
    {
        strcpy(CurrentColorImageLPath, CurrentSaveSensor);
        strcpy(CurrentColorImageRPath, CurrentSaveSensor);
        strcpy(CurrentLadar64Path, CurrentSaveSensor);
        strcpy(CurrentLadar32Path, CurrentSaveSensor);
        strcpy(CurrentLadar8Path, CurrentSaveSensor);
        strcpy(CurrentRadarPath, CurrentSaveSensor);
        strcpy(CurrentIRImagePath, CurrentSaveSensor);
        strcpy(CurrentLocalPosePath, CurrentSaveSensor);
        strcpy(CurrentGlobalPositionPath, CurrentSaveSensor);
        strcpy(CurrentTimeStampPath, CurrentSaveSensor);
        strcpy(CurrentCalibrationPath, CurrentSaveSensor);
        strcpy(CurrentLocalAttributeMapPath, CurrentSaveModule);
        strcpy(CurrentEntityMapPath, CurrentSaveModule);
        strcpy(CurrentLocalPathInfoPath, CurrentSaveModule);

    }
};

struct UgvContainer
{
    char name[32];
    pid_t pid;

    UgvContainer()
    {
        reset();
    }

    UgvContainer(const char* n, int p)
    {
        strcpy(name ,n);
        pid = p;
    }
    void reset()
    {
        memset(name ,0 , sizeof(name));
        pid = 0;
    }
};

}
#endif
