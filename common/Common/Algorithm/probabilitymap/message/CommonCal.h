/************************************************************************/
/*                                                                                                                          */
/*                                      公共数据处理类                                              */
/*                                                                                                                          */
/*                                                                                                                          */
/************************************************************************/

#ifndef __COMMONCAL__
#define __COMMONCAL__

#include "rcs.hh"
#include "LocalPosen.hh"
#include "GlobalPositionInfon.hh"
#include "Common.h"
#include <math.h>

using namespace com;

class CommonCal
{
public:
    CommonCal();

    ~CommonCal();

    /*********************************************************************************/
    //输入:time        各传感器的LocalPoseTime
    //            localPose 读到的LOCALPOSE_MSG
    //输出:posData   根据time在LOCALPOSE_MSG中匹配上最新的LOCAL_POS_DATA
    //
    //返回值: 1 success, -1 failed
    //说明:LOCALPOSE_MSG中存储这最近的50帧数据，根据time匹配历史帧
    /*********************************************************************************/
    
    int calLocalPose(double time, LOCALPOSE_MSG* localPose, LOCAL_POS_DATA* posData);

    /*********************************************************************************/
    //输入:time             各传感器的GlobalPositon gps_millisecond
    //            positionData  读到的GLOBALPOSITIONINFO_MSG
    //输出:posData   根据time在GLOBALPOSITIONINFO_MSG中匹配上最新的PositionData
    //
    //返回值: 1 success, -1 failed
    //说明:GLOBALPOSITIONINFO_MSG 中存储这最近的50帧数据，根据time匹配历史帧
    /*********************************************************************************/
    int calPositionData(double time, GLOBALPOSITIONINFO_MSG* positionData, PositionData* posData);

        
    /*********************************************************************************/
    //输入:LocalPosData        参考的源数据
    //
    //输出:position   修改后的惯导
    //
    //返回值: 1 success, -1 failed
    //说明:将LocalPoseData中的时间、速度、航向角等赋值给惯导结构体
    /*********************************************************************************/    
    //int setPosition(LOCAL_POS_DATA* LocalPosData, PositionData* position);

private:

    /*********************************************************************************/
    //输入:rate     比率
    //            beData  前一帧LocalPose
    //            afData   后一帧LocalPose
    //输出:relData 根据差值计算后的真实LocalPose
    //
    //返回值: 1 success, -1 failed
    //说明:根据前后帧的LocalPose以及比率，算出当前真实的LocalPose
    /*********************************************************************************/    
    int calRealPose(float rate, LOCAL_POS_DATA* beData, LOCAL_POS_DATA* afData, LOCAL_POS_DATA* relData);

    /*********************************************************************************/
    //输入:rate     比率
    //            beData  前一帧PositionData
    //            afData   后一帧PositionData
    //输出:relData 根据差值计算后的真实PositionData
    //
    //返回值: 1 success, -1 failed
    //说明:根据前后帧的LocalPose以及比率，算出当前真实的LocalPose
    /*********************************************************************************/    
    int calRealPosition(float rate, PositionData* beData, PositionData* afData, PositionData* relData);
    
};
#endif

