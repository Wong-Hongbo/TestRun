/************************************************************************/
/*                                                                                                                          */
/*                                      �������ݴ�����                                              */
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
    //����:time        ����������LocalPoseTime
    //            localPose ������LOCALPOSE_MSG
    //���:posData   ����time��LOCALPOSE_MSG��ƥ�������µ�LOCAL_POS_DATA
    //
    //����ֵ: 1 success, -1 failed
    //˵��:LOCALPOSE_MSG�д洢�������50֡���ݣ�����timeƥ����ʷ֡
    /*********************************************************************************/
    
    int calLocalPose(double time, LOCALPOSE_MSG* localPose, LOCAL_POS_DATA* posData);

    /*********************************************************************************/
    //����:time             ����������GlobalPositon gps_millisecond
    //            positionData  ������GLOBALPOSITIONINFO_MSG
    //���:posData   ����time��GLOBALPOSITIONINFO_MSG��ƥ�������µ�PositionData
    //
    //����ֵ: 1 success, -1 failed
    //˵��:GLOBALPOSITIONINFO_MSG �д洢�������50֡���ݣ�����timeƥ����ʷ֡
    /*********************************************************************************/
    int calPositionData(double time, GLOBALPOSITIONINFO_MSG* positionData, PositionData* posData);

        
    /*********************************************************************************/
    //����:LocalPosData        �ο���Դ����
    //
    //���:position   �޸ĺ�Ĺߵ�
    //
    //����ֵ: 1 success, -1 failed
    //˵��:��LocalPoseData�е�ʱ�䡢�ٶȡ�����ǵȸ�ֵ���ߵ��ṹ��
    /*********************************************************************************/    
    //int setPosition(LOCAL_POS_DATA* LocalPosData, PositionData* position);

private:

    /*********************************************************************************/
    //����:rate     ����
    //            beData  ǰһ֡LocalPose
    //            afData   ��һ֡LocalPose
    //���:relData ���ݲ�ֵ��������ʵLocalPose
    //
    //����ֵ: 1 success, -1 failed
    //˵��:����ǰ��֡��LocalPose�Լ����ʣ������ǰ��ʵ��LocalPose
    /*********************************************************************************/    
    int calRealPose(float rate, LOCAL_POS_DATA* beData, LOCAL_POS_DATA* afData, LOCAL_POS_DATA* relData);

    /*********************************************************************************/
    //����:rate     ����
    //            beData  ǰһ֡PositionData
    //            afData   ��һ֡PositionData
    //���:relData ���ݲ�ֵ��������ʵPositionData
    //
    //����ֵ: 1 success, -1 failed
    //˵��:����ǰ��֡��LocalPose�Լ����ʣ������ǰ��ʵ��LocalPose
    /*********************************************************************************/    
    int calRealPosition(float rate, PositionData* beData, PositionData* afData, PositionData* relData);
    
};
#endif

