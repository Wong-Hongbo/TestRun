
#ifndef DATA_ANALYSIS_H
#define DATA_ANALYSIS_H

#include <thread> // std::thread
#include <mutex> // std::mutex
# include <pcl/point_types.h>
# include <pcl/io/ply_io.h>
# include <pcl/visualization/pcl_visualizer.h>
#include <base/global_pose.pb.h>
#include <iostream>
#include <armadillo>
#include <memory>

#include "GlobalPositionInfon.hh"
#include "LocalPosen.hh"
#include "HDLadarDatan.hh"

#include "transform/GaussCoorConv.h"


#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

#include <iomanip>
#include "common/Common/Commonfig.hh"

#include "common/Common/ParamServer.hh"
#include "common/Common/Algorithm/AutoLoopDetect.hh"
#include "common/Common/MapDataFrame/MapDataStruct.hh"


#include "send_data.h"
class GlobalPose {
public:
  double time;
  GLOBALPOSITIONINFO_MSG gp;
};

class LocalPose {
public:
  double time;
  LOCALPOSE_MSG lp;
};

class DataAnalysis {
public:
  DataAnalysis();
  DataAnalysis(std::string path,int i);
  ~DataAnalysis();
  void Analysis();   // 解析函数
private:
  // 数据关联
  bool findLpAtStamp(double stamp, LocalPose &outLP);
  bool findGpAtStamp(double stamp, GlobalPose& outGP);

  // 线程函数
  void threadDecodeIns();        // GP 读取 Ins.txt
  void threadDecodeLidar();      // Lidar 读取
  void threadDecodeLocalPose();  // LP 读取 LPose.txt
  void threadReciveMap();  // 接收MapPosition

public:
  std::string mDataPath;   // 读取离线数据的路径


  SendData* send = nullptr;

private:

  bool endthread_ = false;
  // 参数
  int mImuSource = 0;      // 0 LocalPose/100;  1 LocalPose/10000;   2 VehicleStatus (分别对应LP车载，LP背包，VS车载的情况)
  int mLpGpAvailable = 0;  // 0 Lp Gp 都有效; 1 仅Lp有效; 2 仅Gp有效
  int mLidarScanCol = 1800;   // Lidar水平点分辨率
  double mImuScaler = 100.0;  // Imu的缩放比例
  int mRawLidarHz = 10.0;     // Lidar的数据频率

  // 线程
  std::thread* mThreadDecodeIns = nullptr;
  std::thread* mThreadDecodeLidar = nullptr;
  std::thread* mThreadDecodeLocalPose = nullptr;
  std::thread* mThreadReciveMap = nullptr;
  //内部容器  用作Lidar帧关联Lp Gp数据时间同步使用
  std::deque<GlobalPose> mGlobalPoseQue;
  std::deque<LocalPose> mLocalPoseQue;

  std::deque<GlobalPose> mGlobalPoseQue_temp;
  std::deque<LocalPose> mLocalPoseQue_temp;
  /*---------- 内部锁 ----------*/
  std::mutex mLocalPoseMutex;
  std::mutex mGlobalPoseMutex;
};

#endif
