#ifndef MAPPERREQUEST_HH
#define MAPPERREQUEST_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "NMLmsgExn.hh"


//#define UINT8	unsigned char

// Define the integer type ids.
#define MAPPERREQUEST_MSG_TYPE 15928

typedef struct
{
    int mode;  // 0：正常建图    1：增量建图   2：多楼层建图
} MapperMode;

// Define the NML Message Classes
class MAPPERREQUEST_MSG : public NMLmsgEx
{
public:
    //Constructor
    MAPPERREQUEST_MSG();
    // CMS Update Function
    void update(CMS *);
/**
    MsgCode: 1 ,start 开始建图
            MsgContent: MapperMode
    MsgCode: 2 ,stop 结束建图
    MsgCode: 3 ,save 建图
    MsgCode: 4 ,upload 上传建图
    MsgCode: 5 ,Result Info
    MsgCode: 6 ,Cancel upload 撤销上传
    MsgCode: 8 ,Not Save 不保存
    MsgCode: 9 ,生成地图
    MsgCode: 10,增加闭环点
    MsgCode: 11,匹配闭环点
    MsgCode: 12,获取建图任务状态
**/
    UINT8 MsgCode;
    long time;
    UINT8 MsgContent[4096];
    int seq_num;
};

// Declare NML format function
extern int MapperRequestFormat(NMLTYPE, void *, CMS *);

#endif 	// MAPPERREQUEST_HH
