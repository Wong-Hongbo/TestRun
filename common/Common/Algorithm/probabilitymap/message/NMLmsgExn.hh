/*
NMLmsgEx.hh
This C++ header file defines the NMLmsgEx Messages derived from
NMLmsg .

MODIFICATIONS:
Mon Mar 31 21:37  2014	Created by wza.


*/

// Prevent Multiple Inclusion
#ifndef NMLMSGEX_HH
#define NMLMSGEX_HH

// Include Files
#include "nml.hh"
#include "nmlmsg.hh"
#include "CommonDefinitionX.hh"

#define NMLmsgEx_TYPE 40709
// Define the NML Message Classes

typedef struct{
    INT32		z_x;
    INT32		z_y;
    INT32		g_x;
    INT32		g_y;
    INT32		heading;		/*	UINT32		heading; */
    INT32		pitch;
    INT32		roll;
    INT32		height;
    UINT8		class_id;
    UINT8		reserved[3];
} LOCAL_COORDINATE;

class LOCAL_POS_DATA
{
public:
    UINT32       header;                   // 数据头
    double   		time;			            // millisecond

    INT32			dr_x;                       //centimeter
    INT32			dr_y;                       //centimeter
    INT32			dr_z;                       //centimeter

    INT32			dr_heading;		    //0.01degree
    INT32			dr_roll;		            //0.01degree
    INT32			dr_pitch;		            //0.01degree

    INT32			lf_speed;		            //left_front wheel speed,cm/s
    INT32			rf_speed;		            //right_front wheel speed,cm/s
    INT32			lr_speed;		            //left_rear wheel speed,cm/s
    INT32			rr_speed;		            //right_rear wheel speed,cm/s

    INT32			rot_x;                      //imu三轴陀螺速度 0.01degree/s
    INT32			rot_y;                      //imu三轴陀螺速度 0.01degree/s
    INT32			rot_z;                      //imu三轴陀螺速度 0.01degree/s

    INT32			acc_x;                      //imu三轴加速度  0.01m/s^2
    INT32			acc_y;                      //imu三轴加速度  0.01m/s^2
    INT32			acc_z;                      //imu三轴加速度  0.01m/s^2

    INT32           steer;                      //-3000(right)~3000(left) degree  0.01degree/s
    INT32           brake;                     //0(zero)~100(full)
    INT32           fuel;                         //0(zero)~100(full)
    TRANS_POSITION  trans;                      //PARK=0,BACKWARD=1,NEURAL=2,FORWARD=3;
    LC_STATE        VehicleState;
    CMD_SOURCE      mode;                       //DIRECT_ACTUATOR=0,REMOTE_PILOT=1,AUTO_PILOT=2;

    INT32			drStatus;				    //dr运行状态
    INT32			errorStatus;				//错误状态
    INT32           emergency_flag;
    INT32           hardswitch_on;
};

struct PositionData{
    INT16         gps_flag;               //gps info is updated
    INT16         gps_week;
    double        gps_millisecond;        //millisecond in a week
    INT32		  llhPos[2];			  //经纬度，llpos[i]/SC_POS 单位为度
    INT32		  gaussPos[2];			  //高斯投影位置,cm，
    INT32         height;                 //cm
    INT32         pitch;                  //欧拉角，单位为0.01度
    INT32         roll;                   //欧拉角，单位为0.01度
    INT32         azimuth;                //欧拉角，单位为0.01度,向东为零度，逆时针0-360

    INT32         northVelocity;          //north速度，单位为cm/s
    INT32         eastVelocity;
    INT32         upVelocity;
    INT32         positionStatus;		  //系统运行状态

    INT16         motion_flag;            // motion info is updated
    INT16         motion_week;
    INT32         motion_millisecond;
    INT32         drPos[2];
    INT32         drHeight;
    INT32         drAzimuth;
    INT32         drNorthVelocity;
    INT32         drEastVelocity;
    INT32         drUpVelocity;
    INT32         reserved[2];
};

class NMLmsgEx : public NMLmsg
{
public:

	//Constructor
	NMLmsgEx(NMLTYPE t,long s);
	NMLmsgEx();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 MessageID;
    //INT32 FrameNo;
    INT32 MessageSeqNum;

    double LocalPoseTime;       // LocalPose的时间of message
    double GlobalPosTime;       // LocalPose的时间of message

    LOCAL_POS_DATA  LocalPose;
    PositionData    Position;
};

extern int NMLmsgExFormat(NMLTYPE, void *, CMS *);

#endif 	// NMLMSGEX_HH
