/************************************************************************/
/*                                                                                                                          */
/*                                      ����������                                                        */
/*                                                                                                                          */
/*                                                                                                                          */
/************************************************************************/
    
#ifndef __COMMON__
#define __COMMON__

#include <iostream>
#include <queue>
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

//#include <boost/pool/detail/singleton.hpp>
#include <boost/serialization/singleton.hpp>   //ubuntu
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include "CommonDefinitionX.hh"

#ifndef PIN
#define PIN
#endif

#ifndef POUT
#define POUT
#endif

#define CameraUdpPort 7018          //���ͨѶ�㲥�˿�
#define UGVMonitorPort 7019         //UGV����ͨѶ�˿�
#define UGVTransmitCenterPort 7020
#define UGVTransmitClientPort 7021
#define ManMachineTransferSendPort 7022
#define ManMachineTransferRecvPort 7023
#define ComputerStatusRecvPort 7024
#define SensorDataCollectCmdPort 7025

//Ĭ�϶�Ӧ�����ڴ���ź�����ֵ
#define AVTCameraImageKey 0x123
#define SensorDataManagerCmdKey 0x124
#define UgvDataCollectCmdKey 0x125

#define UGVTransmitCenterStatusKey 0x200
#define UGVTransmitCenterMissionKey 0x201
#define UGVTransmitClientStatusKey 0x202
#define UGVTransmitClientMissionKey 0x203


#define UGVPublicNetMask 0xAC1764FF   //172.23.100.255

namespace com
{


#define SetToNULL(p) p = NULL
//#define DeletePoint(p) do\
//{\
//    if (p != NULL)\
//    {\
//        delete p;\
//        p = NULL;\
//    }\
//}while(0)


/************************************************************************/
/*                                                                                                                          */
/*                                                                                                                          */
/*                                             ��־�����ӡ                                            */
/*                                                                                                                          */
/*                                                                                                                          */
/************************************************************************/

//��ȡ��ǰϵͳʱ��
extern char* getCurrentTime(char* t);

//��ȡ��ǰϵͳʱ���Ӧ�ĺ���
extern double getCurrentMillSecond();

class TraceLog
{
public:
    TraceLog();
    ~TraceLog();

    bool setTraceLog(const char* f);

    FILE* getTraceLog();

    int getTraceLevel();

    int getTraceFlush();

private:
    FILE *m_logfp;

    int m_level;

    int m_flush;
};

//typedef boost::details::pool::singleton_default<com::TraceLog> GlobeLogFp;
//#define SINGLETON_LOG com::GlobeLogFp::instance()

typedef boost::serialization::singleton<com::TraceLog> GlobeLogFp; //ubuntu
#define SINGLETON_LOG com::GlobeLogFp::get_mutable_instance() //ubuntu


//����ʹ����logfp���ļ����ָ��
#define LogInfo(form, info...) do\
{\
    char t[32] = {0};\
    com::getCurrentTime(t);\
    if (SINGLETON_LOG.getTraceLog() != NULL)\
    {\
        fprintf(SINGLETON_LOG.getTraceLog(), "[%s][NORMAL]" form, t, ##info);\
        if (SINGLETON_LOG.getTraceFlush() == 1)\
        {\
            fflush(SINGLETON_LOG.getTraceLog());\
        }\
    }\
    else\
    {\
        printf("[%s][NORMAL]" form, t, ##info);\
    }\
}while(0)
    
//error
#define LogError(form, info...) do\
{\
    char t[32] = {0};\
    com::getCurrentTime(t);\
    if (SINGLETON_LOG.getTraceLog() != NULL)\
    {\
        fprintf(SINGLETON_LOG.getTraceLog(), "[%s][ERROR]%s:%d, " form, t, __FILE__, __LINE__, ##info);\
        if (SINGLETON_LOG.getTraceFlush() == 1)\
        {\
            fflush(SINGLETON_LOG.getTraceLog());\
        }\
    }\
    else\
    {\
        printf("[%s][ERROR]%s:%d, " form, t, __FILE__, __LINE__, ##info);\
    }\
}while(0)

//debug
#define LogDebug(level, form, info...) do\
{\
    if (SINGLETON_LOG.getTraceLevel() >= level)\
    {\
        char t[32] = {0};\
        if (SINGLETON_LOG.getTraceLog() != NULL)\
        {\
            com::getCurrentTime(t);\
            fprintf(SINGLETON_LOG.getTraceLog(), "[%s][DEBUG]" form, t, ##info);\
            if (SINGLETON_LOG.getTraceFlush() == 1)\
            {\
                fflush(SINGLETON_LOG.getTraceLog());\
            }\
        }\
        else\
        {\
            com::getCurrentTime(t);\
            printf("[%s][DEBUG]" form, t, ##info);\
        }\
    }\
}while(0)


/************************************************************************/
/*                                                                                                                          */
/*                                                                                                                          */
/*                                             ����ʱ���ʱ                                           */
/*                                                                                                                          */
/*                                                                                                                          */
/************************************************************************/
class timer
{
public:
    timer();

    void begin();

    double elapsed(); //���ص���΢��

    int calElapsedMin();  //����ʹ�õķ���

    int calElapsedSec();  //����ʹ�õ���

    int calElapsedMill();    //���ص��Ǻ���

    void getRecordTime(struct timeval& _t);

private:
    struct timeval _start_time;
};




/************************************************************************/
/*                                                                                                                          */
/*                                                                                                                          */
/*                                             �����źŲ���                                            */
/*                                                                                                                          */
/*                                                                                                                          */
/************************************************************************/

extern bool setExitCode();

extern bool getExitCode();

extern bool checkExitCode();

/************************************************************************/
/*                                                                                                                          */
/*                                                                                                                          */
/*                                                   socket     ����                                              */
/*                                                                                                                          */
/*                                                                                                                          */
/************************************************************************/

#define PORT 2020    //ͨѶ�˿�
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
 �Զ���Ӧ�ò�Э��
 ��һ���ֽڱ�ʾ�������ͣ�
 ������4���ֽڱ�ʾ���ݳ��ȣ�
 �ٽ��������Ǿ�������
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



//�ź���

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
/*                                                                                                                          */
/*                                                                                                                          */
/*                                                   file         ����                                              */
/*                                                                                                                          */
/*                                                                                                                          */
/************************************************************************/

class file
{
public:
    file();
    ~file();
    //�����ļ�
    bool Create(const char* str, char* filename);
    //���ļ�
    bool Open(const char* filename);
    //��ȡ�ļ�
    int ReadData(char* data);
    //д���ļ�
    bool WriteData(const char* data);
    //����ļ�
    size_t FileSize();
private:
    char _fileName[MAXFILEPATH];
    size_t _fileSize;
    int  _fileDes;
};

/************************************************************************/
/*                                                                                                                          */
/*                                                                                                                          */
/*                                             ��ȡini�����ļ�                                        */
/*                                                                                                                          */
/*                                                                                                                          */
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

//typedef boost::details::pool::singleton_default<com::iniConf> GlobeIniConf;
//#define SINGLETON_INICONF GlobeIniConf::instance()
typedef boost::serialization::singleton<com::iniConf> GlobeIniConf; //ubuntu
#define SINGLETON_INICONF GlobeIniConf::get_mutable_instance() //ubuntu



struct Avt_shmType
{
    INT32 Width;
    INT32 Height;
    INT32 ImageWriteType;    //enum AVT_IMAGE_WRITE_TYPE
    INT32 PixelChannel;
    UINT8 ImageBufferL[COLORIMAGE_SIZE];
    UINT8 ImageBufferR[COLORIMAGE_SIZE];

    INT64 ImageTimeL;
    INT64 ImageTimeR;
    UINT8 HaveNewImage;
    struct Avt_shmType* next;

    Avt_shmType()
    {
        init();
    }

    void init()
    {
        Width = 0;
        Height = 0;
        ImageWriteType = 0;
        PixelChannel = 0;
        HaveNewImage = 0;
        memset(ImageBufferL, 0, sizeof(ImageBufferL));
        memset(ImageBufferR, 0, sizeof(ImageBufferR));
    }
};

struct CameraParam
{
    int FrameRate;    //֡��
    int ImgWidth;    //ͼ����
    int ImgHeight;    //ͼ��߶�
    int ImgBeginX;    //ͼ��ʼX����
    int ImgBeginY;    //ͼ��ʼY����
    int HardTriggerFlag;    //Ӳ�������
    int RestartTimeOut;    //����ʱ���趨ms
    int ExpectedExposureValue;    //�����ع�ֵ
    int ExpectedCameraNum;    //�������ӵ������
    int WriteMode;     //дbufģʽAVT_IMAGE_WRITE_TYPE

    int ShmKey;    //�����ڴ��ֵ
    int SemKey;    //�ź�����ֵ
    int ReduceRate;    //�����ع��ʱ��ƫ��������λΪ΢��
    int BlackFlag;    //�Ƿ�һ�ź�ͼƬ

    int left;    //�ع����ݵ�����
    int right;    //�ع����ݵ�����
    int top;    //�ع����ݵ�����
    int bottom;    //�ع����ݵ�����
};

struct InnerMatrix
{
    double M1[3][3];
    double M2[3][3];
    double R[3][3];
    double T[3];
    double D1[5];
    double D2[5];
    double RotateLeft[3][3];
    double TranslateLeft[3];
    double OriginLeft[3];
    double RotateRight[3][3];
    double TranslateRight[3];
    double OriginRight[3];
    
};


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


//���ھ�γ�ȵ������
struct ugvGlobalPoint
{
    double x;
    double y;
    UINT32 speed;
    UINT8 type;
};


//�����ϱ���״̬��Ϣ
struct ugvTransmitStatus
{
    UINT8 head;
    UINT32 primaryKey;
    INT64 time;    //ʱ����

    UINT8 fuel;    //����0 - 100
    UINT8 shift;    //��λ
    UINT8 throttle;    //����
    UINT8 brake;    //ɲ��
    INT16 steer;    //ǰ��ת��
    UINT8 navMode;    //

    INT32 speedX;    //�����ٶ�cm /s
    INT32 speedY;    //�����ٶ�cm/s
    INT32 speedZ;    //�����ٶ�cm/s

    INT32 gaussX;    //x���꣬cm����Ϊ��
    INT32 gaussY;    //y���꣬cm����Ϊ��
    INT32 height;    //�߶ȣ�����Ϊ��cm
    INT32 heading;    //����ǣ���λ0.01����Ϊ0�ȣ���ʱ��0-36000
    INT32 roll;    //����ǣ���λ0.01�ȣ�����Ϊ��ȣ���ʱ��-9000-9000
    INT32 pitch;    //�����ǣ���λ0.01�ȣ���ǰˮƽΪ��ȣ���ʱ��-9000-9000

    UINT8 sysSts;    //ϵͳ״̬0,error 1 ok
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


//�·������������
struct ugvTransmitMission
{
    UINT8 head;
    UINT32 primaryKey;
    INT64 time;    //ʱ����

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
    //��������kb/s
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
////***********    ugv ���ݲɼ�**************//


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
    char CurrentSaveBase[UgvPathLength];     //������ݵĸ�Ŀ¼
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
