/*
TrafficSignn.hh

This C++ header file defines the NML Messages for TRAFFICSIGNINFO
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:04 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef TRAFFICSIGNN_HH
#define TRAFFICSIGNN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"


// Define the integer type ids.
#define TRAFFICSIGN_MSG_TYPE 9103
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

#define MAX_TRAFFIC_SIGN 5

////////////////////////路标（交通标志）结构体，在世界坐标系下定义
typedef struct{
    unsigned char             pos_flag;       /*  交通标志位置信息 */
    int                       x;              // x坐标，厘米为单位,全局坐标系
    int                       y;              // y坐标，厘米为单位,全局坐标系
    unsigned char             sign_type;      /*  交通标志类型信息 */
}TRAFFIC_SIGN;
/*
pos_flag
  0: 没有位置信息
  1: 位置信息有效

sign_type:
  0：无路标        
  1：禁止左转
  2：禁止右转
  3：禁止直行
  4：禁止向左和向右
  5：禁止向左和直行
  6：禁止向右和直行
  7：禁止驶入
  8：停车
  9： 前方学校 （减速）       
  10：注意行人 （减速）       
  11：路面不平 （减速）      
  12：路面施工 （减速）
  13：直行
  14：向左转
  15：向右转
  16：直行和左转
  17：直行和右转
  18：向左和向右
  19：靠右行驶        
  20：靠左行驶        
  21：鸣喇叭 ------- 无    
  22：环岛行驶 ------- 无     
  23：人行横道        
  24：允许掉头        
  25：停车位          
  26：禁止掉头  
  27：限速10公里 （减速）
  28：解除限速10公里 ------- 无
  29：限速15公里
  30：限速20公里
  31：限速25公里
  32: 三角修车标志
  33：限速5公里
  34：限速30公里
  35：限速40公里
  36：限速50公里

*/
// Define the NML Message Classes

class TRAFFICSIGN_MSG : public NMLmsgEx
{
public:

    //Constructor
    TRAFFICSIGN_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    int TrafficSignNum;      //最多给出5个路标  
    TRAFFIC_SIGN TrafficSign[MAX_TRAFFIC_SIGN];
};

// Declare NML format function
extern int TrafficSignFormat(NMLTYPE, void *, CMS *);

#endif 	// TRAFFICSIGNN_HH
