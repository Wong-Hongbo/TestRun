#ifndef CONTROL_COMMAND_HH
#define CONTROL_COMMAND_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "NMLmsgExn.hh"

// Define the integer type ids.
#define CONTROL_COMMAND_MSG_TYPE 15936

// Define the NML Message Classes
class CONTROL_COMMAND_MSG : public NMLmsgEx
{
public:
    CONTROL_COMMAND_MSG();
    void update(CMS *);
    unsigned char MsgContent[1024];
/**
 * 发送方发送内容：
    req_id: 0x1 ,暂停工作
            struct ControlCommand
    req_id: 0x2 ,恢复工作
            struct ControlCommand
    req_id: 0x3 ,重新初始化
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
};

// Declare NML format function
extern int ControlCommandFormat(NMLTYPE, void *, CMS *);

#endif 	// CONTROL_COMMAND_HH

