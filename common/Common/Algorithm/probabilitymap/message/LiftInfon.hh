// Prevent Multiple Inclusion
#ifndef LIFTINFO_HH
#define LIFTINFO_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions

// Define the integer type ids.
#define LIFT_CMD_MSG_TYPE 1115
#define LIFT_STA_MSG_TYPE 1116
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


/*
 *
 * 电梯id为楼栋内的电梯序号, 一栋楼5个电梯, 则依次编号 1 2 3 4 5
 * 门禁id为楼栋内的门禁序号, 一栋楼5个门禁, 则依次编号 1 2 3 4 5
 *
 */


class LIFT_CMD_MSG : public NMLmsg
{
public:

        //Constructor
    LIFT_CMD_MSG();

        // CMS Update Function
        void update(CMS *);

    // Place custom variables here.
    double timestamp; // 时间戳, 单位: ms
    unsigned int seq; // 命令标识, 用于在 LIFT_STA_MSG 反馈时指明最后一条响应的 LIFT_CMD_MSG
    unsigned int cmd; // 0-无效 1-锁定电梯 2-释放电梯 3-打开门禁 4-释放门禁
    unsigned int id;  // cmd为电梯命令时指明电梯id，cmd为门禁命令时指明门禁id
    int       floor;  // cmd为1时指明呼梯楼层: 地下1层为-1, 1层为1, 2层为2, 0为取消呼梯,关闭电梯门
 };



class LIFT_STA_MSG : public NMLmsg
{
public:

        //Constructor
    LIFT_STA_MSG();

        // CMS Update Function
        void update(CMS *);

    // Place custom variables here.
    double timestamp;               // 时间戳, 单位: ms, LocalPose时间
    unsigned int seq;               // 收到的最后一条 LIFT_CMD_MSG 的seq

    unsigned int lift_id;           // 电梯识别标识
    unsigned int lift_error;        // 电梯故障状态: 0-正常 1-电梯通讯无响应 2-电梯通讯错误 3-无效目标电梯id 4-无效目标楼层
    unsigned int lift_state;        // 电梯状态: 0-停止 1-上行 2-下行 3-满载 4-停止服务 5-电梯已被占用
    unsigned int lift_door_state;   // 电梯门状态: 0-关闭 1-打开
    int          lift_floor;        // 电梯所在楼层: 0为电梯未锁定, 地下1层为-1, 1层为1, 2层为2
    int          lift_target_floor; // 目标楼层: 无目标楼层时为0, 地下1层为-1, 4层为4
                                    // 电梯所在楼层 和 目标楼层相等且电梯门为打开状态, 即判定为电梯到达

    unsigned int access_door_id;    // 门禁识别标识
    unsigned int access_door_error; // 门禁故障状态: 0-正常
    unsigned int access_door_state; // 门禁门状态: 0-关闭 1-打开
};

// Declare NML format function
extern int LiftFormat(NMLTYPE, void *, CMS *);

#endif 	// LIFTINFO_HH
