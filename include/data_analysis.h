
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

#include "GlobalPositionInfon.hh"
#include "LocalPosen.hh"
#include "HDLadarDatan.hh"

#include "send_data.h"

struct VelodynePointXYZIRT
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
                                   (float, x, x) (float, y, y) (float, z, z)(float, intensity, intensity)
                                   (uint16_t, ring, ring) (float, time, time)
                                   )
struct ScanXYZIRT{
  double stime;
  double etime;
  pcl::PointCloud<VelodynePointXYZIRT>::Ptr cloud;
};

class GlobalPose {
public:
  double time;
  GLOBALPOSITIONINFO_MSG gp;
};

class LocalPose {
public:
  double time;
  LOCALPOSE_MSG lp;
//  LOCAL_POS_DATA lp;
};

struct SyncRawScan{
  ScanXYZIRT lidar;
//  LOCALPOSE_MSG lp;
  GLOBALPOSITIONINFO_MSG gp;
  HDLADARDATA_MSG lp;

  double time;
};

//using XSLidarPoint = pcl::PointXYZI;
using PointIRT = VelodynePointXYZIRT;
using PointI = pcl::PointXYZI;
using PointIRTPtr =  pcl::PointCloud<PointIRT>::Ptr;
//using LocalPose = LOCAL_POS_DATA;// LocalPosen
//using GlobalPose = PositionData;// GlobalPositionInfon
//using LidarData = xsproto::perception::LidarData;

class DataAnalysis {
public:
  DataAnalysis();
  DataAnalysis(std::string path);
  ~DataAnalysis();
  // 解析函数
  void Analysis();
private:

  // 数据关联
  bool findLpAtStamp(double stamp, LocalPose &outLP);
  bool findGpAtStamp(double stamp, GlobalPose& outGP);
  GlobalPose SlerpGPRPY(GlobalPose,GlobalPose,double);
  GlobalPose SlerpGPXYZ(GlobalPose,GlobalPose,double);
  LocalPose SlerpLPRPY(LocalPose,LocalPose,double);
  LocalPose SlerpLPXYZ(LocalPose,LocalPose,double);
  // 线程函数
  void threadDecodeIns();        // GP 读取 Ins.txt
  void threadDecodeLidar();      // Lidar 读取
  void threadDecodeLocalPose();  // LP 读取 LPose.txt

public:
  std::string mDataPath;   // 读取离线数据的路径
  // data
//  std::vector <GLOBALPOSITIONINFO_MSG> global_position_datas_{};
//  std::vector <LOCALPOSE_MSG > local_pose_datas_{};
//  std::deque<GLOBALPOSITIONINFO_MSG> global_position_datas_;
//  std::deque<LOCALPOSE_MSG> local_pose_datas_;

//  std::deque<SyncRawScan>* mLidarQue = nullptr;
//  std::deque<SyncRawScan> mLidarQue{};
  std::deque<HDLADARDATA_MSG> mLidarQue{};

private:

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
  // lidar数据
//  HDLADARDATA_MSG *lidar_data_{};

  //内部容器  用作Lidar帧关联Lp Gp数据时间同步使用
  std::deque<GlobalPose> mGlobalPoseQue;
  std::deque<LocalPose> mLocalPoseQue;

  /*---------- 内部锁 ----------*/
  std::mutex mLocalPoseMutex;
  std::mutex mGlobalPoseMutex;
//  std::mutex* mLidarMutex = nullptr;
  std::mutex mLidarMutex;
};

#endif
