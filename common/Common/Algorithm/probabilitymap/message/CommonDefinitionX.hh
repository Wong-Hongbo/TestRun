/*
 *
*	New hh File starts here.
*/
#ifndef COMMONDEFINITION_HH
#define COMMONDEFINITION_HH

//*********************************************************************************************/
//*
//*
//*                                                        ¹«¹²¶¨ÒåÍ·ÎÄ¼þ
//*
//*
//*                      
//*
//*                  ÐÞ¸Ä´ËÍ·ÎÄ¼þÇë°´¸ñÊ½Ìí¼ÓÐÞ¸ÄÈË¡¢Ê±¼ä¡¢ÓÃÍ¾
//*
//*
//*********************************************************************************************/

#ifndef __INT64__
#define __INT64__
typedef  signed     long        INT64;
#endif

#ifndef __UINT64__
#define __UINT64__
typedef  unsigned   long        UINT64;
#endif

#ifndef __INT32__
#define __INT32__
typedef  signed     int         INT32;
#endif

#ifndef __UINT32__
#define __UINT32__
typedef  unsigned   int         UINT32;
#endif

#ifndef __INT16__
#define __INT16__
typedef  signed     short       INT16;
#endif

#ifndef __UINT16__
#define __UINT16__
typedef  unsigned   short       UINT16;
#endif

#ifndef __INT8__
#define __INT8__
typedef  signed     char        INT8;
#endif

#ifndef __UINT8__
#define __UINT8__
typedef  unsigned   char        UINT8;
#endif


//#ifndef PI
//#define PI 3.14159265358979
//#endif

//#ifndef Pi
//#define Pi 3.14159265358979
//#endif

//#ifndef pi
//#define pi 3.14159265358979
//#endif

/*
#ifndef bool
  #define bool  int
  #define true  1
  #define false 0
#endif
*/

#define RING_NUM 64

#define SC_ANG              (100.0)
#define SC_H                (100.0)
#define SC_POS              (10000000.0)
#define SC_T                (1000.0)
#define SC_TPVEL            (1000.0)

#define ENHANCED_IMAGE_SIZE 2000*1000
#define COLORIMAGE_SIZE 7000*1000

#define MAX_VEHICLE_OBJ 100
#define MAX_PEDESTRIAN_OBJ 30
#define MAX_ROAD_OBJ 5
//#define MAX_INTERSECTION_OBJ 3

#define MAX_LANE_OBJ 10

#define MAX_IRROAD_OBJ 5

// 正障碍检测输出: 局部坐标系，前方80米，后方20米，左右25米
#define POSITIVEMAP_WIDTH   250
#define POSITIVEMAP_HEIGHT  500
#define POSITIVEMAP_VEHICLEX   125
#define POSITIVEMAP_VEHICLEY   400
#define POSITIVEOBMAP_SIZE POSITIVEMAP_WIDTH*POSITIVEMAP_HEIGHT
#define GRID_SIZE 20

// other map, size and vehicle position
#define LOCALMAP_WIDTH      150
#define LOCALMAP_HEIGHT     325
#define LOCALMAP_VEHICLEX   75
#define LOCALMAP_VEHICLEY   250
// Terrain Map: 局部坐标系，前方30米，后方15米，左右15米
#define TERRAIN_MAP_SIZE LOCALMAP_WIDTH*LOCALMAP_HEIGHT
// Surface Map: 局部坐标系，前方50米，后方15米，左右15米
#define SURFACE_MAP_SIZE LOCALMAP_WIDTH*LOCALMAP_HEIGHT

// 环境感知输出
// DEM Map: 局部坐标系，前方50米，后方15米，左右15米
#ifndef LOCALDEM_MAP_SIZE
#define LOCALDEM_MAP_SIZE LOCALMAP_WIDTH*LOCALMAP_HEIGHT
#endif
// Obstacle Map: 局部坐标系，前方50米，后方15米，左右15米
#define LOCAL_OBSTACLEMAP_SIZE TERRAIN_MAP_SIZE

// AttributeMap: 局部坐标系，前方80米，后方20米，左右25米
#define ATTRIBUTEMAP_WIDTH          300
#define ATTRIBUTEMAP_HEIGHT         300
#define ATTRIBUTEMAP_VEHICLEX       150
#define ATTRIBUTEMAP_VEHICLEY       150
#ifndef LOCAL_ATTRIBUTEMAP_SIZE
#define LOCAL_ATTRIBUTEMAP_SIZE ATTRIBUTEMAP_WIDTH*ATTRIBUTEMAP_HEIGHT
#endif
// Env map: 局部坐标系，前方50米，后方15米，左右15米
//#define ENV_MAP_SIZE 325*150

#define IR_IMAGE_SIZE 640*480
#define IR_IMAGE_SIZE_BAYER 1296*964
#define IR_IMGAE_SIZE_NIR 1296*966

#define PACKETNUM 350
//#define PACKETNUM 600    //s3

#define PACKETNUM_LADAR32 200

///////////////////////////////////////
#define GLOBALPOINTNUM  2000

//definition for local path planning
#define MAXPLANPOINTS 50
#define MYPLANPOINTS 100

#define MAX_SCAN_DATA_NUM 1800

// definition by sun zhenping
const int  DIRECT_ACTUATOR=0,REMOTE_PILOT=1,AUTO_PILOT=2;
#define CMD_SOURCE int

const int      OFF=0,ON=1;
#define LOGIC_STATE int

const int      PARK=0,BACKWARD=1,NEURAL=2,FORWARD=3;
#define TRANS_POSITION int

#undef TRANS_POSITION
const int L_POSITION=2,L2_POSITION=3,D_POSITION=4,N_POSITION=5,R_POSITION=6,P_POSITION=7;
#define TRANS_POSITION int


enum UGV_MODE_CMD
{
	UGV_MODE_CMD_AU=0,
    UGV_MODE_CMD_KP=1,
    UGV_MODE_CMD_DR=2,
    UGV_MODE_CMD_GC=3
};

// end of definition by sun zhenping

enum COMMAND_TYPE
{
	CMD_START_RECORDING,
	CMD_STOP_RECORDING,
	CMD_EXIT_RECORDING,
	//add other commands here:
	CMD_START_PROCESS,
	CMD_EXIT_PROCESS
};

enum LC_STATE
{
    FLAMEOUT=100, 
    INFLAME, 
    ADVANCE, 
    BACKOFF, 
    VEHICLEUNKNOWN
};

enum AVT_IMAGE_WRITE_TYPE
{
    WRITE_ALL = 0,
    WRITE_LEFT = 1,
    WRITE_RIGHT = 2
};

//
enum OBJECT_CLASSIFICATION_TYPE
{
    UNKNOWN_CLASS     =0,
    VEHICLE_CLASS     =1,
    PEDESTRIAN_CLASS  =2,
    BICYCLE_CLASS     =3,
    SMALL_OBJECT_CLASS=4,
    BIG_OBJECT_CLASS  =5,
    STATIC_OBJECT_CLASS=6
};

//#define DeletePoint(p) do\
//{\
//    if (p != NULL)\
//    {\
//        delete p;\
//        p = NULL;\
//    }\
//}while(0)


#endif 	// COMMONDEFINITION_HH
