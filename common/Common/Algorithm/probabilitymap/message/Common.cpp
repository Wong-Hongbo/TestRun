#include "Common.h"
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <vector>


using namespace std;

namespace com
{
/*
TraceLevel::TraceLevel()
{
    m_level = 0;
}

bool TraceLevel::setTraceLevel(int level)
{
    m_level = level;
    return true;
}

int TraceLevel::getTraceLevel()
{
    return m_level;
}
*/
TraceLog::TraceLog()
{
    m_logfp = NULL;
    m_level = 0;
    m_flush = 0;
}

TraceLog::~TraceLog()
{
    if (m_logfp != NULL)
    {
        fclose(m_logfp);
    }
}

int day_diff(int year_start, int month_start, int day_start
             , int year_end, int month_end, int day_end)
{
    int y2, m2, d2;
    int y1, m1, d1;

    m1 = (month_start + 9) % 12;
    y1 = year_start - m1/10;
    d1 = 365*y1 + y1/4 - y1/100 + y1/400 + (m1*306 + 5)/10 + (day_start - 1);

    m2 = (month_end + 9) % 12;
    y2 = year_end - m2/10;
    d2 = 365*y2 + y2/4 - y2/100 + y2/400 + (m2*306 + 5)/10 + (day_end - 1);

    return abs(d2 - d1);
}

int removeFileList(char *basePath)
{
    DIR *dir;
    struct dirent *ptr;
    char path[1000];

    if ((dir=opendir(basePath)) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }

    while ((ptr=readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
            continue;
        else if(ptr->d_type == 8)    ///file
        {
            sprintf(path, "%s/%s", basePath, ptr->d_name);
            timeval tv;
            gettimeofday(&tv,NULL);
            tm *curTime=localtime(&tv.tv_sec);
            string str=ptr->d_name;
            int index=str.find(".log");
            if(index>=10 )
            {
                string str2=str.substr(index-10, index);
                int year=0, month=0, day=0;
                sscanf(str2.c_str(), "%d-%d-%d", &year, &month, &day);
                if(day_diff(year, month, day, curTime->tm_year+1900,
                            curTime->tm_mon+1, curTime->tm_mday)>2)
                {
                    printf("delete useless log file: %s\n",path);
                    remove(path);
                }
            }
            else
            {
                printf("delete useless log file: %s\n",path);
                remove(path);
            }
        }
    }
    closedir(dir);
    return 1;
}

bool TraceLog::setTraceLog(const char * f)
{
    char *p;
    char logname[200] = {0};
    if (f != NULL)
    {
        //根据环境变量目录信息设置日志文件路径
        p = getenv("LOG_TRACE_FOLDER");
        if (p != NULL)
        {
            strcpy(logname, p);
        }
        else
        {
            strcpy(logname, "/home/ugv/log/");
        }
        if(access(logname, 0)==-1) mkdir(logname, 0744);
        // delete useless log files
        removeFileList(logname);

        // "/home/ugv/log/Sensor"
        strcat(logname, f);
        strcat(logname, "_");
        char t[32] = {0};
        com::getCurrentTime(t);
        char d[32] = {0};
        memcpy(d, t, 10);
        strcat(logname, d);
        strcat(logname, ".log");
        m_logfp = fopen(logname, "a");
        //if (NULL != m_logfp)
        //{
        p = getenv("LOG_TRACE_LEVEL");
        if (p != NULL)
        {
            m_level = atoi(p);
        }

        p = getenv("LOG_TRACE_FLUSH");
        if (p != NULL)
        {
            m_flush = atoi(p);
        }
        //}
    }
    else
    {
        return false;
    }

    return true;
}

FILE* TraceLog::getTraceLog()
{
    return m_logfp;
}

int TraceLog::getTraceLevel()
{
    return m_level;
}

int TraceLog::getTraceFlush()
{
    return m_flush;
}

char* getCurrentTime(char *t)
{
    memset(t, 0, strlen(t));
    struct timeval _tv;
    struct tm *_now;
    time_t _time;

    gettimeofday(&_tv, NULL);
    _time = _tv.tv_sec;
    _now = localtime(&_time);

    sprintf(t, "%4d-%02d-%02d %02d:%02d:%02d.%06ld",
            1900+_now->tm_year,
            1+_now->tm_mon,
            _now->tm_mday,
            _now->tm_hour,
            _now->tm_min,
            _now->tm_sec,
            _tv.tv_usec);
    return t;
}

double getCurrentMillSecond()
{
    struct timeval _tv;
    gettimeofday(&_tv, NULL);
    return _tv.tv_sec*1000 + _tv.tv_usec/1000;
}
timer::timer()
{
    gettimeofday(&_start_time, NULL);
}

void timer::begin()
{
    gettimeofday(&_start_time, NULL);
}

double timer::elapsed()
{
    struct timeval _end_time;
    gettimeofday(&_end_time, NULL);
    double timeuse = 1000000 * (_end_time.tv_sec - _start_time.tv_sec ) + _end_time.tv_usec - _start_time.tv_usec;
    //timeuse /= 1000000;
    return timeuse;
}

int timer::calElapsedMin()
{
    struct timeval _end_time;
    gettimeofday(&_end_time, NULL);
    return ((_end_time.tv_sec - _start_time.tv_sec ) + (_end_time.tv_usec - _start_time.tv_usec) / 1000000)/60;
}

int timer::calElapsedSec()
{
    struct timeval _end_time;
    gettimeofday(&_end_time, NULL);
    return _end_time.tv_sec - _start_time.tv_sec  + (_end_time.tv_usec - _start_time.tv_usec) / 1000000;
}

int timer::calElapsedMill()
{
    struct timeval _end_time;
    gettimeofday(&_end_time, NULL);
    return 1000 * (_end_time.tv_sec - _start_time.tv_sec)  + (_end_time.tv_usec - _start_time.tv_usec) / 1000;
}

void timer::getRecordTime(struct timeval& _t)
{
    _t.tv_sec = _start_time.tv_sec;
    _t.tv_usec = _start_time.tv_usec;
}
bool setExitCode()
{
    sigset_t newmask;
    sigset_t oldmask;

    //清除所有信号的阻塞标志
    sigemptyset(&newmask);
    sigaddset(&newmask, SIGINT);
    sigaddset(&newmask, SIGTERM);

    //更改进程的信号屏蔽字
    if (sigprocmask(SIG_BLOCK, &newmask, &oldmask) < 0)
    {
        return false;
    }
    return true;
}

bool getExitCode()
{
    sigset_t pendmask;
    //取阻塞信号
    if (sigpending(&pendmask) < 0)
    {
        return false;
    }

    //判断某个信号是否被阻塞
    if (sigismember(&pendmask, SIGINT) || sigismember(&pendmask, SIGTERM))
    {
        return true;
    }
    return false;
}

/************************************************************************/
/*                                                                                                                          */
/*                                                                                                                          */
/*                                                   socket     操作                                              */
/*                                                                                                                          */
/*                                                                                                                          */
/************************************************************************/


message::message(int fd)
{
    clear();
    m_fd = fd;
}

void message::reset(int fd)
{
    m_fd = fd;
}

bool message::recvMsg()
{
    clear();
    char buf[header_length + data_length + max_data_length] = {0};
    if (recv(m_fd, buf, sizeof(buf), 0) <= 0)
    {
        return false;
    }
    if (strlen(buf) < header_length + data_length)
    {
        return false;
    }

    memcpy(m_mode, buf, header_length);
    char len[data_length + 1] = {0};
    memcpy(len, buf + header_length, data_length);
    m_length = atoi(len);
    m_length = m_length > max_data_length ? max_data_length : m_length;
    if (m_length > 0)
    {
        memcpy(m_data, buf + header_length + data_length, m_length);
    }
    return true;
}

bool message::sendMsg(const char* mode,const char* data)
{
    clear();
    char buf[header_length + data_length + max_data_length] = {0};
    if (mode[0] == 0 || data[0] == 0)
    {
        return false;
    }
    if (sizeof(buf) <= strlen(mode) + strlen(data))
    {
        return false;
    }

    strcpy(m_mode, mode);
    m_length = strlen(data);
    strcpy(m_data, data);
    strcpy(buf, m_mode);
    char len[data_length + 1] = {0};
    sprintf(len, "%4d", m_length);
    strcat(buf, len);
    strcat(buf, m_data);

    if (send(m_fd, buf, sizeof(buf), 0) <= 0)
    {
        return false;
    }
    return true;
}

const char* message::mode() const
{
    return m_mode;
}

int message::length() const
{
    return m_length;
}

const char* message::data() const
{
    return m_data;
}

void message::clear()
{
    memset(m_mode, 0, sizeof(m_mode));
    m_length = 0;
    memset(m_data, 0, sizeof(m_data));
}

//信号量

bool sem::init(int Key, int val)
{
    m_semid = semget(Key, 1, IPC_CREAT|0660);
    if (m_semid < 0)
    {
        LogError("semget() failed!\n");
        return false;
    }
    union semun sem_union;

    if (val < 0)
    {
        return false;
    }
    sem_union.val = val;
    if (semctl(m_semid, 0, SETVAL, sem_union) < 0)
    {
        LogError("semctl() falied!\n ");
        return false;
    }
    //int ret = semctl(m_semid, 0, GETVAL, sem_union);
    //printf("after setVal ,val = %d\n", ret);
    return true;
}

bool sem::create(int key)
{
    m_semid = semget(key, 1, IPC_CREAT|0660);
    if (m_semid < 0)
    {
        LogError("semget() failed!\n");
        return false;
    }
    union semun sem_union;

    sem_union.val = 1;
    if (semctl(m_semid, 0, SETVAL, sem_union) < 0)
    {
        LogError("semctl() falied!\n ");
        return false;
    }
    return true;
}

bool sem::get(int key)
{
    m_semid = semget(key, 1, IPC_CREAT|0660);
    if (m_semid < 0)
    {
        LogError("semget() failed!\n");
        return false;
    }
    return true;
}

void sem::lock()
{
    struct sembuf sem_b;
    sem_b.sem_num = 0;
    sem_b.sem_op  = -1;
    sem_b.sem_flg = SEM_UNDO;

    if (semop(m_semid, &sem_b, 1) < 0)
    {
        LogError("P semop failed.\n");
    }
    //union semun sem_union;
    //int ret = semctl(m_semid, 0, GETVAL, sem_union);
    //printf("after setVal ,val = %d\n", ret);
}

void sem::unlock()
{
    struct sembuf sem_b;
    sem_b.sem_num = 0;
    sem_b.sem_op = 1;
    sem_b.sem_flg = SEM_UNDO;
    if (semop(m_semid, &sem_b, 1) < 0)
    {
        LogError("V semop failed.\n");
    }
    //union semun sem_union;
    //int ret = semctl(m_semid, 0, GETVAL, sem_union);
    //printf("after setVal ,val = %d\n", ret);
}

//先得到值，然后减去现有值加1
void sem::wait()
{   
    struct sembuf sem_b;
    sem_b.sem_num = 0;
    sem_b.sem_op  = -1;
    sem_b.sem_flg = SEM_UNDO;

    if (semop(m_semid, &sem_b, 1) < 0)
    {
        LogError("P semop failed.\n");
    }
}

void sem::post()
{
    union semun sem_union;
    struct sembuf sem_b;
    if (semctl(m_semid, 0, GETVAL, sem_union) < 0)
    {
        LogError("semctl() falied!\n ");
        return ;
    }
    printf("val is %d.\n", sem_union.val);
    if (sem_union.val != 0)
    {
        sem_b.sem_op = 1 - sem_union.val;
    }
    else
    {
        sem_b.sem_op = 1;
    }
    
    sem_b.sem_num = 0;
    //sem_b.sem_op  = -1;
    sem_b.sem_flg = SEM_UNDO;

    if (semop(m_semid, &sem_b, 1) < 0)
    {
        LogError("P semop failed.\n");
    }
}

bool sem::del()
{
    union semun sem_union;
    
    if (semctl(m_semid, 0, IPC_RMID, sem_union) < 0)
    {
        LogError("Delete semaphore failed!\n");
        return false;
    }
    return true;

}

/************************************************************************/
/*                                                                                                                          */
/*                                                                                                                          */
/*                                                   file         操作                                              */
/*                                                                                                                          */
/*                                                                                                                          */
/************************************************************************/


file::file()
{
    memset(_fileName, 0, sizeof(_fileName));
    _fileSize = 0;
    _fileDes = 0;
}

file::~file()
{
    close(_fileDes);
}

bool file::Create(const char* str, char* filename)
{
    if (str[0] == 0)
    {
        return false;
    }
    std::vector<std::string> v;
    boost::split(v, str, boost::is_any_of(";"));
    if (v.size() != 3)
    {
        return false;
    }

    std::string path = v[0];
    strcpy(_fileName, v[1].c_str());
    _fileSize = atoi(v[2].c_str());
    std::string totallyName = path + "/" + v[1];
    strcpy(filename, totallyName.c_str());

    //boost::filesystem::path p(totallyName.c_str());
    //文件存在先删除
    //if (boost::filesystem::exists(p))
    //{
    //    boost::filesystem::remove(p);
    //}

    remove(totallyName.c_str());
    
    _fileDes = open(totallyName.c_str(), O_WRONLY|O_CREAT|O_APPEND, S_IRUSR|S_IWUSR|S_IRGRP);//O_SYNC O_EXCL
    if (_fileDes == -1)
    {
        return false;
    }

    return true;
}

bool file::Open(const char * filename)
{
    if (filename[0] == 0)
    {
        return false;
    }

    _fileDes = open(filename, O_RDONLY);
    if (_fileDes == -1)
    {
        return false;
    }
    return true;
}

bool file::WriteData(const char* data)
{
    if (data[0] == 0)
    {
        return false;
    }
    int res = write(_fileDes, data, strlen(data));
    if (res == -1)
    {
        return false;
    }
    return true;
}

int file::ReadData(char* data)
{
    memset(data, 0, strlen(data));
    //0表示文件末尾
    //-1表示出错
    size_t read_bytes = read(_fileDes, data, max_data_length);
    if (read_bytes > 0 && read_bytes < strlen(data))
    {
        printf("read_bytes is %d. strlen(data) is %ld\n", read_bytes, strlen(data));
        memset(data + read_bytes, 0, strlen(data) - read_bytes);
    }
    return read_bytes;
}

size_t file::FileSize()
{
    return _fileSize;
}

/************************************************************************/
/*                                                                                                                          */
/*                                                                                                                          */
/*                                             读取ini配置文件                                        */
/*                                                                                                                          */
/*                                                                                                                          */
/************************************************************************/

iniConf::iniConf()
{
}

iniConf::iniConf(const char* fileConf)
{
    openFile(fileConf);
}

iniConf::~iniConf()
{
}

bool iniConf::openFile(const char* fileConf)
{
    if (NULL == fileConf)
    {
        return false;
    }
    try
    {
        boost::property_tree::read_ini(fileConf, m_pt);
    }catch(exception& e)
    {
        cout<<"iniConf::openFile -- parse file failed :"<<e.what()<<endl;
        return false;
    }
    
    return true;
}

int iniConf::getValue(std::string Key, int Default, char Sep)
{
    try
    {
        return m_pt.get(boost::property_tree::ptree::path_type(Key, Sep), Default);
    }catch(exception& e)
    {
        cout<<"iniConf::getvalue -- get value feiled:"<<e.what()<<endl;
        return Default;
    }
}

std::string iniConf::getValue(std::string Key, std::string Default, char Sep)
{
    try
    {
        return m_pt.get(boost::property_tree::ptree::path_type(Key, Sep), Default);
    }catch(exception& e)
    {
        cout<<"iniConf::getvalue -- get value feiled:"<<e.what()<<endl;
        return Default;
    }
}


ugvTransmitMsg::ugvTransmitMsg(int readLen, int writeLen)
{
    m_readLength = readLen;
    m_writeLength = writeLen;
}

ugvTransmitMsg::~ugvTransmitMsg()
{

}

void ugvTransmitMsg::reset(int fd)
{
    m_fd = fd;

}

int ugvTransmitMsg::getReadLength()
{
    return m_readLength;
}

int ugvTransmitMsg::getWriteLength()
{
    return m_writeLength;
}


//to check
int ugvTransmitMsg::write(void *data, int timeout)
{
    //check
    if (NULL == data)
    {
        return 0;
    }
    
    
    //send data
    int ret = 0;
    int writelen = 0;
    int msglen = getWriteLength();

    //get current time
    struct timeval _tv1, _tv2;
    gettimeofday(&_tv1, NULL);
    gettimeofday(&_tv2, NULL);
    INT64 _currentMill;
    INT64 _beginMill;
    _beginMill =  _tv1.tv_sec*1000 + _tv1.tv_usec/1000;

    while (msglen > 0)
    {
        //judge timeout
        gettimeofday(&_tv2, NULL);
        _currentMill = _tv2.tv_sec*1000 + _tv2.tv_usec/1000;
        if (_currentMill - _beginMill > timeout)
        {
            printf("timeout!\n");
            return -1;
        }
        
        ret = send(m_fd, (void*)data + writelen, msglen, 0);
        if (ret == -1)
        {
            if (errno != EAGAIN)
            {
                //printf("send error!, %d, %s\n", errno, strerror(errno));
                return -1;
            }
        }
        //连接断开
        else if (ret == 0)
        {
            return -1;
        }
        else
        {
            writelen+=ret;
            msglen-=ret;
            //printf("writelen is %d, msglen is %d.\n", writelen, msglen);
        }
    }

    return writelen;
}


int ugvTransmitMsg::read(void *data, int timeout)
{
    //check
    if (NULL == data)
    {
        return -1;
    }

    //get current time
    struct timeval _tv1, _tv2;
    gettimeofday(&_tv1, NULL);
    gettimeofday(&_tv2, NULL);
    INT64 _currentMill;
    INT64 _beginMill;
    _beginMill =  _tv1.tv_sec*1000 + _tv1.tv_usec/1000;
    
    //read data
    int ret = 0;
    int readlen = 0;
    int msglen = getReadLength();
    while (msglen > 0)
    {
        //judge timeout
        gettimeofday(&_tv2, NULL);
        _currentMill = _tv2.tv_sec*1000 + _tv2.tv_usec/1000;
        if (_currentMill - _beginMill > timeout)
        {
            printf("timeout!\n");
            return -1;
        }
        
        ret = recv(m_fd, (void*)data + readlen, msglen, 0);
        //printf("ret is %d.\n", ret);
        if (ret == -1)
        {
            //printf("recv error!, %d, %s\n", errno, strerror(errno));
            if (errno != EAGAIN)
            {
                return -1;
            }
            //EAGAIN表示要继续读
        }
        //连接断开
        else if (ret == 0)
        {
            return -1;
        }
        else
        {
            readlen+=ret;
            msglen-=ret;
            //printf("readlen is %d, msglen is %d.\n", readlen, msglen);

        }
    }

    return readlen;
}


}

