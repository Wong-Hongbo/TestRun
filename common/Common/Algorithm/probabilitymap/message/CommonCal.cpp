#include "CommonCal.h"


CommonCal::CommonCal()
{
}


CommonCal::~CommonCal()
{
}


int CommonCal::calLocalPose(double time, LOCALPOSE_MSG* localPose, LOCAL_POS_DATA* posData)
{
    //LogDebug(2, "CommonCal::calLocalPose, time is %f, min.time is %lf, max.time is %lf.\n",
    //    time,
    //    localPose->LocalPoseData[LOCALPOSE_DATA_SIZE-1].time,
    //    localPose->LocalPoseData[0].time);   
    double beTime ;//前一个时间
    double afTime;//后一个时间

    float rate = 0.0;
    //比最近的时间大
    if (time >= localPose->LocalPoseData[0].time || time <= localPose->LocalPoseData[LOCALPOSE_DATA_SIZE-1].time)
    {
        //LogDebug(3, "time >= localPose->LocalPoseData[0].time.\n");
        memcpy(posData, &(localPose->LocalPoseData[0]), sizeof(LOCAL_POS_DATA));
        //setPosition(posData, position);
        return 1;
    }
    //比记录的最早时间还小
    //else if (time <= localPose->LocalPoseData[LOCALPOSE_DATA_SIZE-1].time)
    //{
    //    //LogDebug(3, "time <= localPose->LocalPoseData[49].time.\n");
    //    memcpy(posData, &(localPose->LocalPoseData[LOCALPOSE_DATA_SIZE-1]), sizeof(LOCAL_POS_DATA));
    //    //setPosition(posData, position);
    //    return 1;
    //}
    //在数组中间找合适的时间
    else
    {
        //LogDebug(3, "time < localPose->LocalPoseData[0].time && time >= localPose->LocalPoseData[49].time .\n");
        for (int i = 0; i < LOCALPOSE_DATA_SIZE - 1 ; ++i)
        {
            if (time < localPose->LocalPoseData[i].time && time >= localPose->LocalPoseData[i+1].time)
            {
                
                beTime = localPose->LocalPoseData[i+1].time;
                afTime = localPose->LocalPoseData[i].time;
                if (beTime != afTime)
                {
                    //计算差值
                    rate = (float)(time - beTime) / (afTime - beTime);
                }
                else
                {
                    rate = 1;
                }
               
                
                //LogDebug(2, "time is %lf, beTime is %lf, afTime is %lf.\n", time, beTime, afTime);
                calRealPose(rate, &(localPose->LocalPoseData[i+1]), &(localPose->LocalPoseData[i]), posData);
                //setPosition(posData, position);
                return 1;
            }
        }
    }
    

    return 0;
}

int CommonCal::calPositionData(double time, GLOBALPOSITIONINFO_MSG* positionData, PositionData* posData)
{
    //LogDebug(2, "CommonCal::calPositionData, time is %f, min.time is %lf, max.time is %lf.\n",
    //    time,
    //    positionData->positionData[GLOBALPOS_ARRAYSIZE-1].gps_millisecond,
    //    positionData->positionData[0].gps_millisecond);   
    
    double beTime ;//前一个时间
    double afTime;//后一个时间

    float rate = 0.0;
    //比最近的时间大
    if (time >= positionData->positionData[0].gps_millisecond)
    {
        //LogDebug(3, "time >= positionData->positionData[0].gps_millisecond.\n");
        memcpy(posData, &(positionData->positionData[0]), sizeof(PositionData));
        //setPosition(posData, position);
        return 1;
    }
    //比记录的最早时间还小
    else if (time <= positionData->positionData[LOCALPOSE_DATA_SIZE-1].gps_millisecond)
    {
        //LogDebug(3, "time <= positionData->positionData[49].gps_millisecond.\n");
        memcpy(posData, &(positionData->positionData[LOCALPOSE_DATA_SIZE-1]), sizeof(PositionData));
        //setPosition(posData, position);
        return 1;
    }
    //在数组中间找合适的时间
    else
    {
        //LogDebug(3, "time < positionData->positionData[0].gps_millisecond && time >= positionData->positionData[49].gps_millisecond.\n");
        for (int i = 0; i < LOCALPOSE_DATA_SIZE - 1 ; ++i)
        {
            if (time < positionData->positionData[i].gps_millisecond && time >= positionData->positionData[i+1].gps_millisecond)
            {
                
                beTime = positionData->positionData[i+1].gps_millisecond;
                afTime = positionData->positionData[i].gps_millisecond;
               
                //计算差值
                rate = (float)(time - beTime) / (afTime - beTime);
                //LogDebug(2, "time is %f, beTime is %lf, afTime is %lf.\n", time, beTime, afTime);
                calRealPosition(rate, &(positionData->positionData[i+1]), &(positionData->positionData[i]), posData);
                //setPosition(posData, position);
                return 1;
            }
        }
    }
    

    return 0;

}

int CommonCal::calRealPose(float rate, LOCAL_POS_DATA* beData, LOCAL_POS_DATA* afData, LOCAL_POS_DATA* relData)
{
    if (beData == NULL || afData == NULL || relData == NULL)
    {
        return -1;
    }

    //LogInfo("rate is %f.\n", rate);
    relData->header      = beData->header     + rate * (afData->header     - beData->header);
    relData->time        = beData->time       + rate * (afData->time       - beData->time);
    relData->dr_x        = beData->dr_x       + rate * (afData->dr_x       - beData->dr_x);
    relData->dr_y        = beData->dr_y       + rate * (afData->dr_y       - beData->dr_y);
    relData->dr_z        = beData->dr_z       + rate * (afData->dr_z       - beData->dr_z);
    relData->dr_heading  = beData->dr_heading + rate * (afData->dr_heading - beData->dr_heading);
    relData->dr_roll     = beData->dr_roll    + rate * (afData->dr_roll    - beData->dr_roll);
    relData->dr_pitch    = beData->dr_pitch   + rate * (afData->dr_pitch   - beData->dr_pitch);
    relData->lf_speed    = beData->lf_speed   + rate * (afData->lf_speed   - beData->lf_speed);
    relData->rf_speed    = beData->rf_speed   + rate * (afData->rf_speed   - beData->rf_speed);
    relData->lr_speed    = beData->lr_speed   + rate * (afData->lr_speed   - beData->lr_speed);
    relData->rr_speed    = beData->rr_speed   + rate * (afData->rr_speed   - beData->rr_speed);
    relData->rot_x       = beData->rot_x      + rate * (afData->rot_x      - beData->rot_x);
    relData->rot_y       = beData->rot_y      + rate * (afData->rot_y      - beData->rot_y);
    relData->rot_z       = beData->rot_z      + rate * (afData->rot_z      - beData->rot_z);
    relData->acc_x       = beData->acc_x      + rate * (afData->acc_x      - beData->acc_x);
    relData->acc_y       = beData->acc_y      + rate * (afData->acc_y      - beData->acc_y);
    relData->acc_z       = beData->acc_z      + rate * (afData->acc_z      - beData->acc_z);
    relData->steer       = beData->steer      + rate * (afData->steer      - beData->steer);
    relData->brake       = beData->brake      + rate * (afData->brake      - beData->brake);
    relData->fuel        = beData->fuel       + rate * (afData->fuel       - beData->fuel);
  
    //当航向角跨越360度时特殊处理
    if (abs(afData->dr_heading - beData->dr_heading) >= 18000)
    {
        //比如后一帧为30度，前一帧为350度
        if (afData->dr_heading <= beData->dr_heading)
        {
            relData->dr_heading  = beData->dr_heading + rate * (afData->dr_heading + 36000 - beData->dr_heading);
        }
        //比如前一帧为30度，后一帧为350度
        else
        {
            relData->dr_heading  = beData->dr_heading + 36000 + rate * (afData->dr_heading - beData->dr_heading - 36000);
        }

        relData->dr_heading = relData->dr_heading%36000;
        //LogDebug(6,"beData->dr_heading:%d, afData->dr_heading:%d, relData->dr_heading:%d.\n",
        //    beData->dr_heading,
        //    afData->dr_heading,
        //    relData->dr_heading);
    }

    //LogDebug(2, "rate is %f, beData->time is %lf, afData->time is %lf, relData->time is %lf.\n", 
    //    rate,
    //    beData->time,
    //    afData->time,
    //    relData->time);
    return 1;
}


int CommonCal::calRealPosition(float rate, PositionData* beData, PositionData* afData, PositionData* relData)
{
    if (beData == NULL || afData == NULL || relData == NULL)
    {
        return -1;
    }

    relData->gps_flag            = afData->gps_flag;
    relData->gps_week            = beData->gps_week           + rate * (afData->gps_week             - beData->gps_week);
    relData->gps_millisecond     = beData->gps_millisecond    + rate * (afData->gps_millisecond      - beData->gps_millisecond);
    relData->llhPos[0]           = beData->llhPos[0]          + rate * (afData->llhPos[0]            - beData->llhPos[0]);
    relData->llhPos[1]           = beData->llhPos[1]          + rate * (afData->llhPos[1]            - beData->llhPos[1]);
    relData->gaussPos[0]         = beData->gaussPos[0]        + rate * (afData->gaussPos[0]          - beData->gaussPos[0]);
    relData->gaussPos[1]         = beData->gaussPos[1]        + rate * (afData->gaussPos[1]          - beData->gaussPos[1]);
    relData->height              = beData->height             + rate * (afData->height               - beData->height);
    relData->pitch               = beData->pitch              + rate * (afData->pitch                - beData->pitch);
    relData->roll                = beData->roll               + rate * (afData->roll                 - beData->roll);
    relData->azimuth             = beData->azimuth            + rate * (afData->azimuth              - beData->azimuth);
    relData->northVelocity       = beData->northVelocity      + rate * (afData->northVelocity        - beData->northVelocity);
    relData->eastVelocity        = beData->eastVelocity       + rate * (afData->eastVelocity         - beData->eastVelocity);
    relData->upVelocity          = beData->upVelocity         + rate * (afData->upVelocity           - beData->upVelocity);
    relData->positionStatus      = beData->positionStatus     + rate * (afData->positionStatus       - beData->positionStatus);
    relData->motion_flag         = beData->motion_flag        + rate * (afData->motion_flag          - beData->motion_flag);
    relData->motion_week         = beData->motion_week        + rate * (afData->motion_week          - beData->motion_week);
    relData->motion_millisecond  = beData->motion_millisecond + rate * (afData->motion_millisecond   - beData->motion_millisecond);
    relData->drPos[0]            = beData->drPos[0]           + rate * (afData->drPos[0]             - beData->drPos[0]);
    relData->drPos[1]            = beData->drPos[1]           + rate * (afData->drPos[1]             - beData->drPos[1]);
    relData->drHeight            = beData->drHeight           + rate * (afData->drHeight             - beData->drHeight);
    relData->drAzimuth           = beData->drAzimuth          + rate * (afData->drAzimuth            - beData->drAzimuth);
    relData->drNorthVelocity     = beData->drNorthVelocity    + rate * (afData->drNorthVelocity      - beData->drNorthVelocity);
    relData->drEastVelocity      = beData->drEastVelocity     + rate * (afData->drEastVelocity       - beData->drEastVelocity);
    relData->drUpVelocity        = beData->drUpVelocity       + rate * (afData->drUpVelocity         - beData->drUpVelocity);

    //当航向角跨越360度时特殊处理
    if (abs(afData->azimuth - beData->azimuth) >= 18000)
    {
        //比如后一帧为30度，前一帧为350度
        if (afData->azimuth <= beData->azimuth)
        {
            relData->azimuth  = beData->azimuth + rate * (afData->azimuth + 36000 - beData->azimuth);
        }
        //比如前一帧为30度，后一帧为350度
        else
        {
            relData->azimuth  = beData->azimuth + 36000 + rate * (afData->azimuth - beData->azimuth - 36000);
        }

        relData->azimuth = relData->azimuth%36000;
        //LogDebug(6,"beData->azimuth:%d, afData->azimuth:%d, relData->azimuth:%d.\n",
        //    beData->azimuth,
        //    afData->azimuth,
        //    relData->azimuth);
    }

    //LogDebug(2, "rate is %f, beData->time is %lf, afData->time is %lf, relData->time is %lf.\n", 
    //   rate,
    //    beData->gps_millisecond,
    //    afData->gps_millisecond,
    //    relData->gps_millisecond);

    return 1;

}

/*
int CommonCal::setPosition(LOCAL_POS_DATA* LocalPosData, PositionData* position)
{
    position->gps_millisecond = LocalPosData->time;
    position->gaussPos[0]     = LocalPosData->dr_x;
    position->gaussPos[1]     = LocalPosData->dr_y;
    position->height          = LocalPosData->dr_z;
    position->pitch           = LocalPosData->dr_pitch;
    position->roll            = LocalPosData->dr_roll;
    position->azimuth         = LocalPosData->dr_heading;
   
    position->northVelocity   = cos((LocalPosData->dr_heading*PI/(100.0*180))) * (LocalPosData->lf_speed + LocalPosData->rf_speed)/2;
    position->eastVelocity    = sin((LocalPosData->dr_heading*PI/(100.0*180))) * (LocalPosData->lf_speed + LocalPosData->rf_speed)/2;
    
    return 1;
}
*/

