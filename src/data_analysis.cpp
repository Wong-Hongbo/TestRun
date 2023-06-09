

#include "data_analysis.h"

#include <utility>

DataAnalysis::DataAnalysis() = default;

DataAnalysis::DataAnalysis(std::string data_path,int i,std::string map_path) {
  mDataPath = std::move(data_path);
  send = new SendData(i);
  mMapPath = std::move(map_path);
  NO_ = i;
}

DataAnalysis::~DataAnalysis() {
  if (send != nullptr) {
    delete send;
    send = nullptr;
  }
};

void DataAnalysis::Analysis() {
  std::cout << "DataAnalysis::Analysis()" << std::endl;

  // 开始启动GP解析线程
  mThreadDecodeIns = new std::thread(&DataAnalysis::threadDecodeIns, this);
  usleep(0.1 * 1000);
  // 开始启动LP解析线程
  mThreadDecodeLocalPose = new std::thread(&DataAnalysis::threadDecodeLocalPose, this);
  usleep(0.1 * 1000);
  // 开始启动RawLidar解析线程
  mThreadDecodeLidar = new std::thread(&DataAnalysis::threadDecodeLidar, this);
  usleep(0.1 * 1000);
  mThreadReciveMap = new std::thread(&DataAnalysis::threadReciveMap, this);
  usleep(0.1 * 1000);

  mThreadDecodeIns->join();
  mThreadDecodeLocalPose->join();
  mThreadDecodeLidar->join();
  mThreadReciveMap->join();
}


bool DataAnalysis::findGpAtStamp(double stamp, GlobalPose &outGP) {
  std::cout << "findGpAtStamp::" << std::to_string(stamp) << std::endl;
  GlobalPose prevGP, lattGP;
  prevGP.time = -1;
  lattGP.time = -1;

  if (mGlobalPoseQue.empty()) {
    std::cout << "findGpAtStamp::mGlobalPoseQue.size()" << mGlobalPoseQue.size() << std::endl;
    return false;
  } else {
    std::cout << "[DataTransmit] findGpAtStamp: Find!!" << std::endl;
    printf("[DataTransmit] findGpAtStamp: Tempt to find gp at stamp %lf [%lf ---- %lf] \n", stamp,
           mGlobalPoseQue.front().time, mGlobalPoseQue.back().time);

    mGlobalPoseMutex.lock();
    while (!mGlobalPoseQue.empty()) {
      if (mGlobalPoseQue.front().time < stamp) {
        prevGP = mGlobalPoseQue.front();
//        send->SendGlobalPosition(prevGP.gp);
//        std::cout<<fixed<<std::setprecision(15)<<"sendGP::"<<prevGP.time<<","<<prevGP.gp.longitude<<","<<prevGP.gp.laltitude<<std::endl;
        mGlobalPoseQue_temp.push_back(prevGP);
        mGlobalPoseQue.pop_front();
      } else {
        lattGP = mGlobalPoseQue.front();
//        send->SendGlobalPosition(lattGP.gp);
//        std::cout<<fixed<<std::setprecision(15)<<"sendGP::"<<lattGP.time<<","<<lattGP.gp.longitude<<","<<lattGP.gp.laltitude<<std::endl;

        break;
      }
    }
    mGlobalPoseMutex.unlock();

    std::cout << "prevGP.time::" << std::to_string(prevGP.time) << "lattGP.time::" << std::to_string(lattGP.time)
              << std::endl;

    /* Ins.txt的数据编码内容：（39维）
     * 0 %lf: LocalPose.time; 1 %hd: gps_flag; 2 %hd: gps_week; 3 %lf: gps_millisecond; 4 %.8lf: longitude; 5 %.8lf: laltitude; 6 %d:
     * Position.gaussPos[0]; 7 %d: Position.gaussPos[1]; 8 %d: Position.height; 9 %d: Position.azimuth; 10 %d: Position.pitch; 11 %d:
     * Position.roll; 12 %d: Position.northVelocity; 13 %d: Position.eastVelocity; 14 %d: Position.upVeloctity; 15 %d:
     * Position.positionStatus; 16 %lf: longitude_dev; 17 %lf: laltitude_dev; 18 %lf: azimuth_dev; 19 %d: Position.reserved[0]; 20 %d:
     * Position.reserved[1]; 21 %d: DataNumber; 22 %lf: SystemTimeDiff; 23 %lf: LocalPoseTime; 24 %hd: Position1.gps_flag; 25 %hd:
     * Position1.gps_week; 26 %lf: Position1.gps_millisecond; 27 %d: Position1.gaussPos[0]; 28 %d: Position1.gaussPos[1]; 29 %d:
     * Position1.height; 30 %d: Position1.azimuth; 31 %d: Position1.pitch; 32 %d: Position1.roll; 33 %d: Position1.northVelocity; 34 %d:
     * Position1.eastVelocity; 35 %d: Position1.upVeloctity; 36 %d: Position1.positionStatus; 37 %d: Position1.reserved[0];
     * 38 %d: Position1.reserved[1];
     */

    if (prevGP.time != -1 && lattGP.time != -1) {
      double ratio = (stamp - prevGP.time) / (lattGP.time - prevGP.time);
      outGP = prevGP;
      outGP.gp.Position.gaussPos[0] = prevGP.gp.Position.gaussPos[0] +
                                      static_cast<int>(ratio * (lattGP.gp.Position.gaussPos[0] -
                                                                prevGP.gp.Position.gaussPos[0]));
      outGP.gp.Position.gaussPos[1] = prevGP.gp.Position.gaussPos[1] +
                                      static_cast<int>( ratio * (lattGP.gp.Position.gaussPos[1] -
                                                                 prevGP.gp.Position.gaussPos[1]));
      outGP.gp.Position.height = prevGP.gp.Position.height +
                                 static_cast<int>( ratio * (lattGP.gp.Position.height - prevGP.gp.Position.height));
      outGP.gp.Position.azimuth = prevGP.gp.Position.azimuth +
                                  static_cast<int>( ratio * (lattGP.gp.Position.azimuth - prevGP.gp.Position.azimuth));
      outGP.gp.Position.pitch = prevGP.gp.Position.pitch +
                                static_cast<int>( ratio * (lattGP.gp.Position.pitch - prevGP.gp.Position.pitch));
      outGP.gp.Position.roll = prevGP.gp.Position.roll +
                               static_cast<int>( ratio * (lattGP.gp.Position.roll - prevGP.gp.Position.roll));
      outGP.gp.longitude_dev = ratio < 0.5 ? prevGP.gp.longitude_dev : lattGP.gp.longitude_dev;
      outGP.gp.laltitude_dev = ratio < 0.5 ? prevGP.gp.laltitude_dev : lattGP.gp.laltitude_dev;
      outGP.gp.azimuth_dev = ratio < 0.5 ? prevGP.gp.azimuth_dev : lattGP.gp.azimuth_dev;
      outGP.gp.Position.gps_flag = ratio < 0.5 ? prevGP.gp.Position.gps_flag : lattGP.gp.Position.gps_flag;
      outGP.time = stamp;
    } else {
      outGP = prevGP;
      std::cout << "GP false!" << std::endl;
      return false;
    }
    std::cout << "GP true!" << std::endl;
    return true;
  }
}

bool DataAnalysis::findLpAtStamp(double stamp, LocalPose &outLP) {
  std::cout << "findLpAtStamp::" << std::to_string(stamp) << std::endl;
  LocalPose prevLP, lattLP;
  prevLP.time = -1;
  lattLP.time = -1;

  if (mLocalPoseQue.empty()) {
    std::cout << "mLocalPoseQue.size()" << mLocalPoseQue.size() << std::endl;
    return false;
  } else {
    std::cout << "[DataTransmit] findLpAtStamp: Find!!" << std::endl;
    printf("[DataTransmit] findLpAtStamp: Tempt to find lp at stamp %lf [%lf ---- %lf] \n", stamp,
           mLocalPoseQue.front().time, mLocalPoseQue.back().time);

    mLocalPoseMutex.lock();
    while (!mLocalPoseQue.empty()) {
      if (mLocalPoseQue.front().time < stamp) {
        prevLP = mLocalPoseQue.front();
        mLocalPoseQue_temp.push_back(prevLP);
        mLocalPoseQue.pop_front();
      } else {
        lattLP = mLocalPoseQue.front();
        break;
      }
    }
    mLocalPoseMutex.unlock();

    std::cout << "prevLp.time::" << std::to_string(prevLP.time) << "lattLP.time::" << std::to_string(lattLP.time)
              << std::endl;

    if (prevLP.time != -1 && lattLP.time != -1) {
      double ratio = (stamp - prevLP.time) / (lattLP.time - prevLP.time);
      /* LPose.txt的数据编码内容：(31维)
       * 0 %lf: time; 1 %d: header; 2 %lf: time; 3 %d: dr_x; 4 %d: dr_y; 5 %d: dr_z; 6 %d:dr_heading;
       * 7 %d: dr_roll; 8 %d: dr_pitch; 9 %d: lf_speed; 10 %d: rf_speed; 11 %d: lr_speed; 12 %d: rr_speed;
       * 13 %d: gyr_x; 14 %d: gyr_y; 15 %d: gyr_z; 16 %d: acc_x; 17 %d: acc_y; 18 %d: acc_z; 19 %d:
       * steer; 20 %d: brake; 21 %d: fuel; 22 %d: trans; 23 %d: VehicleState; 24 %d: mode; 25 %d: drStatus; 26 %d:
       * errorStatus; 27 %d: emergency_flag; 28 %d: hardswitch_on; 29 %d: DataNumber; 30 %lf: SystemTimeDiff;
       */
      outLP = prevLP;
      outLP.lp.data.dr_x = prevLP.lp.data.dr_x +
                           static_cast<int>(ratio * (lattLP.lp.data.dr_x - prevLP.lp.data.dr_x));
      outLP.lp.data.dr_y = prevLP.lp.data.dr_y +
                           static_cast<int>(ratio * (lattLP.lp.data.dr_y - prevLP.lp.data.dr_y));
      outLP.lp.data.dr_z = prevLP.lp.data.dr_z +
                           static_cast<int>(ratio * (lattLP.lp.data.dr_z - prevLP.lp.data.dr_z));
      outLP.lp.data.dr_heading = prevLP.lp.data.dr_heading +
                                 static_cast<int>(ratio * (lattLP.lp.data.dr_heading - prevLP.lp.data.dr_heading));
      outLP.lp.data.dr_roll = prevLP.lp.data.dr_roll +
                              static_cast<int>(ratio * (lattLP.lp.data.dr_roll - prevLP.lp.data.dr_roll));
      outLP.lp.data.dr_pitch = prevLP.lp.data.dr_pitch +
                               static_cast<int>(ratio * (lattLP.lp.data.dr_pitch - prevLP.lp.data.dr_pitch));
      outLP.lp.data.lf_speed = prevLP.lp.data.lf_speed +
                               static_cast<int>(ratio * (lattLP.lp.data.lf_speed - prevLP.lp.data.lf_speed));
      outLP.lp.data.rf_speed = prevLP.lp.data.rf_speed +
                               static_cast<int>(ratio * (lattLP.lp.data.rf_speed - prevLP.lp.data.rf_speed));
      outLP.lp.data.lr_speed = prevLP.lp.data.lr_speed +
                               static_cast<int>(ratio * (lattLP.lp.data.lr_speed - prevLP.lp.data.lr_speed));
      outLP.lp.data.rr_speed = prevLP.lp.data.rr_speed +
                               static_cast<int>(ratio * (lattLP.lp.data.rr_speed - prevLP.lp.data.rr_speed));
      outLP.time = stamp;

    } else {
      if (prevLP.time != -1 && abs(prevLP.time - stamp) < 0.01) {
        outLP = prevLP; // 找上一最近时刻的LP
        std::cout << "LP prevLP true!" << std::endl;
        return true;
      }
      if (lattLP.time != -1 && abs(lattLP.time - stamp) < 0.01) {
        outLP = lattLP; // 找下一最近时刻的LP
        std::cout << "LP lattLP true!" << std::endl;
        return true;
      } else {
        std::cout << "LP false!" << std::endl;
        return false;
      }
    }
    std::cout << "LP true!" << std::endl;
    return true;
  }
}

// GP解析
void DataAnalysis::threadDecodeIns() {
  std::cout << "-- threadDecodeIns Start!" << std::endl;

//  std::string mDataPath{"/home/whb/Downloads/data/2022-12-20_offline_1682240507/2022-12-20-19-11-20"};
  std::string InsTxtPath = mDataPath + "/Ins/Ins.txt";
  std::cout << "InsTxtPath::" << InsTxtPath << std::endl;
  // 从 Ins.txt解析出GlobalPose信息插入到buffer中
  FILE *fp = fopen(InsTxtPath.c_str(), "r");
  if (fp == nullptr) std::cout << "threadDecodeLocalPose Failed to Open File:" << InsTxtPath << std::endl;
//  GLOBALPOSITIONINFO_MSG tmp_gp; // 待读取的GP
  GlobalPose tmp_gp;
  size_t count = 0;   // LP数据计数
  while (!feof(fp)) {
    double last_gptime = tmp_gp.time;
    /* Ins.txt的数据编码内容：（39维）
     * 0 %lf: LocalPose.time; 1 %hd: gps_flag; 2 %hd: gps_week; 3 %lf: gps_millisecond; 4 %.8lf: longitude; 5 %.8lf: laltitude; 6 %d:
     * Position.gaussPos[0]; 7 %d: Position.gaussPos[1]; 8 %d: Position.height; 9 %d: Position.azimuth; 10 %d: Position.pitch; 11 %d:
     * Position.roll; 12 %d: Position.northVelocity; 13 %d: Position.eastVelocity; 14 %d: Position.upVeloctity; 15 %d:
     * Position.positionStatus; 16 %lf: longitude_dev; 17 %lf: laltitude_dev; 18 %lf: azimuth_dev; 19 %d: Position.reserved[0]; 20 %d:
     * Position.reserved[1]; 21 %d: DataNumber; 22 %lf: SystemTimeDiff; 23 %lf: LocalPoseTime; 24 %hd: Position1.gps_flag; 25 %hd:
     * Position1.gps_week; 26 %lf: Position1.gps_millisecond; 27 %d: Position1.gaussPos[0]; 28 %d: Position1.gaussPos[1]; 29 %d:
     * Position1.height; 30 %d: Position1.azimuth; 31 %d: Position1.pitch; 32 %d: Position1.roll; 33 %d: Position1.northVelocity; 34 %d:
     * Position1.eastVelocity; 35 %d: Position1.upVeloctity; 36 %d: Position1.positionStatus; 37 %d: Position1.reserved[0];
     * 38 %d: Position1.reserved[1];
     */

    fscanf(fp,
           "%lf %hd %hd %lf %lf %lf %d %d %d %d %d %d %d %d %d %d %lf %lf %lf %d %d %d %lf %lf %hd %hd %lf "
           "%d %d %d %d %d %d %d %d %d %d %d %d",
           &tmp_gp.time, &tmp_gp.gp.Position.gps_flag, &tmp_gp.gp.Position.gps_week,
           &tmp_gp.gp.Position.gps_millisecond,
           &tmp_gp.gp.longitude, &tmp_gp.gp.laltitude,
           &tmp_gp.gp.Position.gaussPos[0], &tmp_gp.gp.Position.gaussPos[1],
           &tmp_gp.gp.Position.height, &tmp_gp.gp.Position.azimuth,
           &tmp_gp.gp.Position.pitch, &tmp_gp.gp.Position.roll,
           &tmp_gp.gp.Position.northVelocity, &tmp_gp.gp.Position.eastVelocity,
           &tmp_gp.gp.Position.upVelocity, &tmp_gp.gp.Position.positionStatus,
           &tmp_gp.gp.longitude_dev, &tmp_gp.gp.laltitude_dev,
           &tmp_gp.gp.azimuth_dev,
           &tmp_gp.gp.Position.reserved[0], &tmp_gp.gp.Position.reserved[1],
           &tmp_gp.gp.DataNumber, &tmp_gp.gp.SystemTimeDiff, &tmp_gp.gp.LocalPoseTime,
           &tmp_gp.gp.Position1.gps_flag, &tmp_gp.gp.Position1.gps_week,
           &tmp_gp.gp.Position1.gps_millisecond,
           &tmp_gp.gp.Position1.gaussPos[0], &tmp_gp.gp.Position1.gaussPos[1],
           &tmp_gp.gp.Position1.height, &tmp_gp.gp.Position1.azimuth,
           &tmp_gp.gp.Position1.pitch, &tmp_gp.gp.Position1.roll,
           &tmp_gp.gp.Position1.northVelocity, &tmp_gp.gp.Position1.eastVelocity,
           &tmp_gp.gp.Position1.upVelocity, &tmp_gp.gp.Position1.positionStatus,
           &tmp_gp.gp.Position1.reserved[0], &tmp_gp.gp.Position1.reserved[1]
    );
    tmp_gp.gp.LocalPose.time = tmp_gp.time;
    tmp_gp.time = tmp_gp.time / 1000.0;
    if (count == 0) // 第一帧打印Log
    {
      printf("GP1::%lf %hd %hd %lf %lf %lf %d %d %d %d %d %d %d %d %d %d %lf %lf %lf %d %d %d %lf %lf %hd %hd %lf "
             "%d %d %d %d %d %d %d %d %d %d %d %d \n",
             &tmp_gp.time, &tmp_gp.gp.Position.gps_flag, &tmp_gp.gp.Position.gps_week,
             &tmp_gp.gp.Position.gps_millisecond,
             &tmp_gp.gp.longitude, &tmp_gp.gp.laltitude,
             &tmp_gp.gp.Position.gaussPos[0], &tmp_gp.gp.Position.gaussPos[1],
             &tmp_gp.gp.Position.height, &tmp_gp.gp.Position.azimuth,
             &tmp_gp.gp.Position.pitch, &tmp_gp.gp.Position.roll,
             &tmp_gp.gp.Position.northVelocity, &tmp_gp.gp.Position.eastVelocity,
             &tmp_gp.gp.Position.upVelocity, &tmp_gp.gp.Position.positionStatus,
             &tmp_gp.gp.longitude_dev, &tmp_gp.gp.laltitude_dev,
             &tmp_gp.gp.azimuth_dev,
             &tmp_gp.gp.Position.reserved[0], &tmp_gp.gp.Position.reserved[1],
             &tmp_gp.gp.DataNumber, &tmp_gp.gp.SystemTimeDiff, &tmp_gp.gp.LocalPoseTime,
             &tmp_gp.gp.Position1.gps_flag, &tmp_gp.gp.Position1.gps_week,
             &tmp_gp.gp.Position1.gps_millisecond,
             &tmp_gp.gp.Position1.gaussPos[0], &tmp_gp.gp.Position1.gaussPos[1],
             &tmp_gp.gp.Position1.height, &tmp_gp.gp.Position1.azimuth,
             &tmp_gp.gp.Position1.pitch, &tmp_gp.gp.Position1.roll,
             &tmp_gp.gp.Position1.northVelocity, &tmp_gp.gp.Position1.eastVelocity,
             &tmp_gp.gp.Position1.upVelocity, &tmp_gp.gp.Position1.positionStatus,
             &tmp_gp.gp.Position1.reserved[0], &tmp_gp.gp.Position1.reserved[1]
      );
    }


//    if (last_gptime == tmp_gp.time) // 上一帧已经读取结束，此帧为无效数据
//    {
//      std::cout<<"无效！！"<<std::endl;
//      break;
//    }
//    global_position_datas_.emplace_back(tmp_gp);

    // 将GlobalPose插入队列
    mGlobalPoseMutex.lock();
    mGlobalPoseQue.push_back(tmp_gp);
    mGlobalPoseMutex.unlock();

    count++;
  }

  printf("GP2::%lf %hd %hd %lf %lf %lf %d %d %d %d %d %d %d %d %d %d %lf %lf %lf %d %d %d %lf %lf %hd %hd %lf "
         "%d %d %d %d %d %d %d %d %d %d %d %d \n",
         &tmp_gp.time, &tmp_gp.gp.Position.gps_flag, &tmp_gp.gp.Position.gps_week,
         &tmp_gp.gp.Position.gps_millisecond,
         &tmp_gp.gp.longitude, &tmp_gp.gp.laltitude,
         &tmp_gp.gp.Position.gaussPos[0], &tmp_gp.gp.Position.gaussPos[1],
         &tmp_gp.gp.Position.height, &tmp_gp.gp.Position.azimuth,
         &tmp_gp.gp.Position.pitch, &tmp_gp.gp.Position.roll,
         &tmp_gp.gp.Position.northVelocity, &tmp_gp.gp.Position.eastVelocity,
         &tmp_gp.gp.Position.upVelocity, &tmp_gp.gp.Position.positionStatus,
         &tmp_gp.gp.longitude_dev, &tmp_gp.gp.laltitude_dev,
         &tmp_gp.gp.azimuth_dev,
         &tmp_gp.gp.Position.reserved[0], &tmp_gp.gp.Position.reserved[1],
         &tmp_gp.gp.DataNumber, &tmp_gp.gp.SystemTimeDiff, &tmp_gp.gp.LocalPoseTime,
         &tmp_gp.gp.Position1.gps_flag, &tmp_gp.gp.Position1.gps_week,
         &tmp_gp.gp.Position1.gps_millisecond,
         &tmp_gp.gp.Position1.gaussPos[0], &tmp_gp.gp.Position1.gaussPos[1],
         &tmp_gp.gp.Position1.height, &tmp_gp.gp.Position1.azimuth,
         &tmp_gp.gp.Position1.pitch, &tmp_gp.gp.Position1.roll,
         &tmp_gp.gp.Position1.northVelocity, &tmp_gp.gp.Position1.eastVelocity,
         &tmp_gp.gp.Position1.upVelocity, &tmp_gp.gp.Position1.positionStatus,
         &tmp_gp.gp.Position1.reserved[0], &tmp_gp.gp.Position1.reserved[1]
  );
  fclose(fp);
}


void DataAnalysis::threadDecodeLidar() {
  std::cout << "-- threadDecodeLidar Start!" << std::endl;
  std::string TimeStampTxtPath = mDataPath + "/TimeStamp/LadarMiddleMiddleTop.txt";

  size_t count = 0; // Lidar数据计数
  // 从 TimeStamp txt解析出每一帧Lidar对应的时间戳， 从而取对应的bin文件名
  FILE *fp = fopen(TimeStampTxtPath.c_str(), "r");
  if (fp == nullptr) std::cout << "threadDecodeLidar Failed to Open File:" << TimeStampTxtPath << std::endl;
  double un_double;     // 不需要的double数据的占位值
  long long un_longint; // 不需要的long long数据的占位值
  double timestamp;

  while (!feof(fp)) {
    /* TimeStamp txt的数据编码内容：(6维)
     * 0 %llu: lptime; 1 %lf: xx time; 2 %lld: gauss x? ; 3 %lld: gauss y? ; 4 %lf: endTime? ; 5 %lf: start time?
     */
    fscanf(fp, "%lf %lf %lld %lld %lf %lf", &timestamp, &un_double, &un_longint, &un_longint, &un_double, &un_double);
    std::cout << "-- Current Scanf Lidar Timestamp:" << timestamp << std::endl;
    std::cout << "-- Current Scanf Lidar Timestamp:" << std::to_string(timestamp) << std::endl;

    char lidarBinPath[256];
    sprintf(lidarBinPath, "%s/LadarMiddleMiddleTop/Ladar-%015.f.bin", mDataPath.c_str(), timestamp);

    std::cout << "lidarbin" << lidarBinPath << std::endl;
    /* 读取当前帧对应的LidarBin文件 */
    FILE *fpBin;
    fpBin = fopen(lidarBinPath, "rb");
    if (fpBin == nullptr) {
      std::cout << "threadDecodeLidar Failed to Open Lidar Bin File: " << lidarBinPath << "! Next!" << std::endl;
      continue;
    }
    int ringNum;
    double startTime, endTime;
    fread(&(ringNum), sizeof(int), 1, fpBin);
    fread(&(startTime), sizeof(double), 1, fpBin);
    fread(&(endTime), sizeof(double), 1, fpBin);
    printf("[DataTransmit] First Decode Lidar Name %lf, Start Time: %lf, End Time:%lf, RingNum: %d\n",
           double(timestamp) / 1000.0, startTime, endTime, ringNum);
    // 处理时间戳信息（LPTime, startTime, endTime）
    double lptime = double(timestamp) / 1000.0;
    double scantime, scan_endTime;
    if (abs(startTime - lptime) < 1.0 && (endTime - startTime) <= 1.0 / mRawLidarHz + 0.02) { // 0.02是余量，lidar频率最低为10hz
      scantime = startTime;
      scan_endTime = endTime;
    } else {
      scantime = lptime - 0.1;
      scan_endTime = scantime + 1.0 / mRawLidarHz;
    }

    HDLADARDATA_MSG thisSyncLidar;
//    memcpy(thisSyncLidar.HDData, 0,sizeof(PointCoordinate64) * ringNum  * PACKETNUM);
    PointCoordinate64 ptRead;
    PointCoordinate64 pt[ringNum][PACKETNUM * 6];
//    PointCoordinate64 pt[64][PACKETNUM * 6];
    for (int i = 0; i < ringNum; i++) {
      for (int j = 0; j < PACKETNUM * 6; j++) {
        fread(&ptRead, sizeof(PointCoordinate64), 1, fpBin);
        if (!pcl_isfinite(ptRead.x) || !pcl_isfinite(ptRead.y) || !pcl_isfinite(ptRead.z)) {
          continue;
        }
        if (ptRead.x != 0 || ptRead.y != 0 || ptRead.z != 0) {
//          pt[i][j].x = ptRead.x;
//          pt[i][j].y = ptRead.y;
//          pt[i][j].z = ptRead.z;
//          pt[i][j].Intensity = ptRead.Intensity;

          thisSyncLidar.HDData[i][j].x = ptRead.x;
          thisSyncLidar.HDData[i][j].y = ptRead.y;
          thisSyncLidar.HDData[i][j].z = ptRead.z;
          thisSyncLidar.HDData[i][j].Intensity = ptRead.Intensity;
        }
      }
    }
    fclose(fpBin);

    /* lidar和Gp Lp数据关联同步（GP、LP数据插值到lidar时刻）*/
    LocalPose thisLP;
    GlobalPose thisGP;

    bool findLp = false;
    bool findGp = false;

    switch (mLpGpAvailable) // 根据Lp Gp是否存在选择不同的同步策略
    {
      case 0:
        if (!findLp)findLp = findLpAtStamp(scan_endTime, thisLP);
        if (!findGp)findGp = findGpAtStamp(scan_endTime, thisGP);

        if (!findLp || !findGp) {
          printf("[DataTransmit] inputLidarThread(): Lidar Sync LP GP Failed! LP: %d, GP: %d, lidarstamp: %lf \n",
                 findLp, findGp, scan_endTime);
          if (!findLp) {
            size_t qsize = mLocalPoseQue.size();
            printf("[DataTransmit] inputLidarThread(): Curr LP QueSize: %zu , back time : %lf \n", qsize,
                   qsize == 0 ? thisLP.time : mLocalPoseQue.back().time);
          }
          if (!findGp) {
            size_t qsize = mGlobalPoseQue.size();
            printf("[DataTransmit] inputLidarThread(): Curr GP QueSize: %zu , back time : %lf \n", qsize,
                   qsize == 0 ? thisGP.time : mGlobalPoseQue.back().time);
          }
          std::cout << "No Find LP && GP ::" << std::endl;
          continue;
        }

        if (mLocalPoseQue_temp.empty()&&mGlobalPoseQue_temp.empty()) {
          std::cout << "mLocalPoseQue_temp.size()" << mLocalPoseQue_temp.size() << std::endl;
        } else {
          while (!mLocalPoseQue_temp.empty() || !mGlobalPoseQue_temp.empty()) {
            if (!mLocalPoseQue_temp.empty()) {
              LocalPose lp_tmp = mLocalPoseQue_temp.front();
              send->SendLocalPose(lp_tmp.lp);
              std::cout<<fixed<<std::setprecision(15)<<"sendLP::"<<lp_tmp.time<<","<<lp_tmp.lp.data.dr_x<<","<<lp_tmp.lp.data.dr_y<<std::endl;

              mLocalPoseQue_temp.pop_front();
            }
            if (!mGlobalPoseQue_temp.empty()) {
              GlobalPose gp_tmp = mGlobalPoseQue_temp.front();
              std::cout<<fixed<<std::setprecision(15)<<"sendGP::"<<gp_tmp.time<<","<<gp_tmp.gp.longitude<<","<<gp_tmp.gp.laltitude<<std::endl;
              std::cout<<fixed<<std::setprecision(15)<<"sendGP::"<<gp_tmp.time<<","<<gp_tmp.gp.Position.gaussPos[0]
              <<","<<gp_tmp.gp.Position.gaussPos[1]<<","<<gp_tmp.gp.Position.height<<","<<gp_tmp.gp.Position.azimuth
                  <<","<<gp_tmp.gp.Position.pitch<<","<<gp_tmp.gp.Position.roll<<std::endl;

//              &tmp_gp.gp.Position.gaussPos[0], &tmp_gp.gp.Position.gaussPos[1],
//                  &tmp_gp.gp.Position.height, &tmp_gp.gp.Position.azimuth,
//                  &tmp_gp.gp.Position.pitch, &tmp_gp.gp.Position.roll,
              send->SendGlobalPosition(gp_tmp.gp);
              mGlobalPoseQue_temp.pop_front();
            }
          }
        }

        std::cout << "lp.time::" << std::to_string(thisLP.time) << "gp.time::" << std::to_string(thisGP.time)
                  << std::endl;

        break;
      case -1:
        break;
      default:
        printf(
            "[DataTransmit] inputLidarThread(): Sync LPGP Failed cause of Unexpected LpGpAvailable Params Setting: %d! \n",
            mLpGpAvailable);
    }
    // 同步完成写入数据（如果同步失败，isLidarValid为false，这一帧数据不会写到队列中）
//    thisSyncLidar.lp = thisLP.lp;

//    ringNum = 64;
    thisSyncLidar.ringNum = ringNum;
    thisSyncLidar.startTime = startTime;
    thisSyncLidar.endTime = endTime;
    thisSyncLidar.LocalPose = thisLP.lp.data;
    thisSyncLidar.Position = thisGP.gp.Position;
//    std::cout << "sizeof(PointCoordinate64)::" << sizeof(PointCoordinate64) << std::endl;
    memcpy(thisSyncLidar.HDData, pt, 6 * ringNum * PACKETNUM);
    std::cout << "ringNum" << ringNum << std::endl;

    std::cout << "同步成功" << std::endl;
    // 将提取到的Lidar Scan信息写入队列

//    GLOBALPOSITIONINFO_MSG thisGlobalPosition;
//    LOCALPOSE_MSG thisLocalPose;
//    send->SendGlobalPosition( thisGP.gp);
//    send->SendLocalPose( thisLP.lp);

    if(!init){
      int find_pose = findInit(timestamp);
      if(find_pose != -1){
        MAP_POSITION_MSG thisMapPosition;
        thisMapPosition.MessageSeqNum++;
        thisMapPosition.MessageID = MAP_POSITION_MSG_TYPE;
        thisMapPosition.Position = thisSyncLidar.Position;
        thisMapPosition.LocalPose = thisSyncLidar.LocalPose;

        pcl::PointCloud<PointType6D>::Ptr key_pose;
        key_pose.reset(new pcl::PointCloud<PointType6D>);
        std::cout << "pcl::" << mMapPath + "/keypose.pcd" << std::endl;
        if (pcl::io::loadPCDFile<PointType6D>(mMapPath + "/keypose.pcd", *key_pose) == -1) {
          PCL_ERROR ("Couldn't read keypose PCD file \n");
          return ;
        }

        int azimuth = int((key_pose->at(find_pose).yaw + M_PI / 2) * (18000 / M_PI)) % 36000;
        if (azimuth < 0) {
          azimuth += 36000;
        }
        thisMapPosition.azimuth = azimuth / 100;
        thisMapPosition.global_x = key_pose->at(find_pose).x * 1.0 + 6272000 / 100.0f;
        thisMapPosition.global_y = key_pose->at(find_pose).y * 1.0 - 59868500 / 100.0f;
        thisMapPosition.roll = key_pose->at(find_pose).roll;
        thisMapPosition.pitch = key_pose->at(find_pose).pitch;
        thisMapPosition.height = key_pose->at(find_pose).z;

        double longitude, latitude;
        GaussProjInvCal(thisMapPosition.global_x / 100.0 + BASE_X, thisMapPosition.global_y / 100.0 + BASE_Y,
                        &longitude, &latitude, 19);
        std::cout << "thisGP.gp.longitude::" << longitude << std::endl;
        std::cout << "thisGP.gp.laltitude::" << latitude << std::endl;
        thisMapPosition.longitude = thisGP.gp.longitude;
        thisMapPosition.latitude = thisGP.gp.longitude;

        thisMapPosition.works_well = 1;

        std::cout << "=============================================" << std::endl;

        thisMapPosition.envstatewarning = 199;
        send->SendMapPosition(thisMapPosition);
        int k = 0;
        while(k<10){
          send->SendHDLadarData(thisSyncLidar);
          sleep(1);
          k++;
          std::cout<<"等待初始化！！！"<<std::endl;
        }
        init = true;

      }else{
        send->SendHDLadarData(thisSyncLidar);
      }
    }else{
      send->SendHDLadarData(thisSyncLidar);
    }


    count++;
    printf("[DataTransmit] Load Lidar Counter %zu. \n", count);
    usleep(100000); // 雷达100ms
  }
  fclose(fp);
  endthread_ = true;
}


void DataAnalysis::threadDecodeLocalPose() {
  std::cout << "-- threadDecodeLocalPose Start!" << std::endl;
  std::string LPoseTxtPath = mDataPath + "/LocalPose/LPose.txt";
  // 从 LPose.txt解析出GlobalPose信息插入到buffer中
  FILE *fp = fopen(LPoseTxtPath.c_str(), "r");
  if (fp == nullptr) {
    std::cout << "threadDecodeLocalPose Failed to Open File:" << LPoseTxtPath << std::endl;
  }

  LocalPose tmp_lp{};   // 待读取的LP
  size_t count = 0;   // LP数据计数
//  double filename{}; // file 时间
  while (!feof(fp)) {
    double last_lptime = tmp_lp.time;
    /* LPose.txt的数据编码内容：(31维)
     * 0 %lf: time; 1 %d: header; 2 %lf: time; 3 %d: dr_x; 4 %d: dr_y; 5 %d: dr_z; 6 %d:dr_heading;
     * 7 %d: dr_roll; 8 %d: dr_pitch; 9 %d: lf_speed; 10 %d: rf_speed; 11 %d: lr_speed; 12 %d: rr_speed;
     * 13 %d: gyr_x; 14 %d: gyr_y; 15 %d: gyr_z; 16 %d: acc_x; 17 %d: acc_y; 18 %d: acc_z; 19 %d:
     * steer; 20 %d: brake; 21 %d: fuel; 22 %d: trans; 23 %d: VehicleState; 24 %d: mode; 25 %d: drStatus; 26 %d:
     * errorStatus; 27 %d: emergency_flag; 28 %d: hardswitch_on; 29 %d: DataNumber; 30 %lf: SystemTimeDiff;
     */
    fscanf(fp, "%lf %d %lf %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %lf",
           &tmp_lp.time, &tmp_lp.lp.data.header, &tmp_lp.lp.data.time, &tmp_lp.lp.data.dr_x, &tmp_lp.lp.data.dr_y,
           &tmp_lp.lp.data.dr_z, &tmp_lp.lp.data.dr_heading, &tmp_lp.lp.data.dr_roll, &tmp_lp.lp.data.dr_pitch,
           &tmp_lp.lp.data.lf_speed, &tmp_lp.lp.data.rf_speed, &tmp_lp.lp.data.lr_speed, &tmp_lp.lp.data.rr_speed,
           &tmp_lp.lp.data.rot_x, &tmp_lp.lp.data.rot_y, &tmp_lp.lp.data.rot_z, &tmp_lp.lp.data.acc_x,
           &tmp_lp.lp.data.acc_y, &tmp_lp.lp.data.acc_z, &tmp_lp.lp.data.steer, &tmp_lp.lp.data.brake,
           &tmp_lp.lp.data.fuel, &tmp_lp.lp.data.trans, &tmp_lp.lp.data.VehicleState, &tmp_lp.lp.data.mode,
           &tmp_lp.lp.data.drStatus, &tmp_lp.lp.data.errorStatus, &tmp_lp.lp.data.emergency_flag,
           &tmp_lp.lp.data.hardswitch_on, &tmp_lp.lp.DataNumber, &tmp_lp.lp.SystemTimeDiff);
    tmp_lp.time = tmp_lp.time / 1000.0;
    if (last_lptime == tmp_lp.time) {
      continue;// break
    }
//    tmp_lp_datas_.emplace_back(tmp_lp);

    if (count == 0) // 第一帧打印Log
    {
      printf("LP::%lf %d %lf %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %lf",
             &tmp_lp.time, &tmp_lp.lp.data.header, &tmp_lp.lp.data.time, &tmp_lp.lp.data.dr_x, &tmp_lp.lp.data.dr_y,
             &tmp_lp.lp.data.dr_z, &tmp_lp.lp.data.dr_heading, &tmp_lp.lp.data.dr_roll, &tmp_lp.lp.data.dr_pitch,
             &tmp_lp.lp.data.lf_speed, &tmp_lp.lp.data.rf_speed, &tmp_lp.lp.data.lr_speed, &tmp_lp.lp.data.rr_speed,
             &tmp_lp.lp.data.rot_x, &tmp_lp.lp.data.rot_y, &tmp_lp.lp.data.rot_z, &tmp_lp.lp.data.acc_x,
             &tmp_lp.lp.data.acc_y, &tmp_lp.lp.data.acc_z, &tmp_lp.lp.data.steer, &tmp_lp.lp.data.brake,
             &tmp_lp.lp.data.fuel, &tmp_lp.lp.data.trans, &tmp_lp.lp.data.VehicleState, &tmp_lp.lp.data.mode,
             &tmp_lp.lp.data.drStatus, &tmp_lp.lp.data.errorStatus, &tmp_lp.lp.data.emergency_flag,
             &tmp_lp.lp.data.hardswitch_on, &tmp_lp.lp.DataNumber, &tmp_lp.lp.SystemTimeDiff);
    }

    // 将LocalPose插入队列
    mLocalPoseMutex.lock();
    mLocalPoseQue.push_back(tmp_lp);
    mLocalPoseMutex.unlock();
    count++;
  }
  fclose(fp);

  printf("LP2::%lf %d %lf %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %lf",
         &tmp_lp.time, &tmp_lp.lp.data.header, &tmp_lp.lp.data.time, &tmp_lp.lp.data.dr_x, &tmp_lp.lp.data.dr_y,
         &tmp_lp.lp.data.dr_z, &tmp_lp.lp.data.dr_heading, &tmp_lp.lp.data.dr_roll, &tmp_lp.lp.data.dr_pitch,
         &tmp_lp.lp.data.lf_speed, &tmp_lp.lp.data.rf_speed, &tmp_lp.lp.data.lr_speed, &tmp_lp.lp.data.rr_speed,
         &tmp_lp.lp.data.rot_x, &tmp_lp.lp.data.rot_y, &tmp_lp.lp.data.rot_z, &tmp_lp.lp.data.acc_x,
         &tmp_lp.lp.data.acc_y, &tmp_lp.lp.data.acc_z, &tmp_lp.lp.data.steer, &tmp_lp.lp.data.brake,
         &tmp_lp.lp.data.fuel, &tmp_lp.lp.data.trans, &tmp_lp.lp.data.VehicleState, &tmp_lp.lp.data.mode,
         &tmp_lp.lp.data.drStatus, &tmp_lp.lp.data.errorStatus, &tmp_lp.lp.data.emergency_flag,
         &tmp_lp.lp.data.hardswitch_on, &tmp_lp.lp.DataNumber, &tmp_lp.lp.SystemTimeDiff);
}


void DataAnalysis::threadReciveMap() {

  while (!endthread_&& init) {
    MAP_POSITION_MSG map_position{};

    std::fstream f;
    f.open("./ReciveMap"+ std::to_string(NO_)+".txt", std::ios::app);

    if (send->ReceiveMapPosition(map_position) != 1) {

      f << std::to_string(map_position.global_x)<<","<<std::to_string(map_position.global_y)<<","<<map_position.score<<std::endl;

      std::cout << "Recive::x::" << map_position.global_x << std::endl;
      std::cout << "Recive::y::" << map_position.global_y << std::endl;
      std::cout << "Recive::score::" << map_position.score << std::endl;

    } else {
      std::cout << "Recive::null!!!" << std::endl;
    }
    f.close();
    usleep(10000);
  }


  //    data.x = map_localizer_data_->global_x;
//    data.y = map_localizer_data_->global_y;
//    data.z = map_localizer_data_->height;

//    data.yaw = map_localizer_data_->yaw;
//    data.timestamp = map_localizer_data_->LocalPose.time;
//    //    data.work_status = map_localizer_data_->works_well;
//    data.work_status = map_localizer_data_->request_init;
//    data.match_state = map_localizer_data_->works_well;
//    data.score = map_localizer_data_->score;
//    data.azimuth = map_localizer_data_->azimuth;
//    data.is_move = (map_localizer_data_->LocalPose.rr_speed == 0 &&map_localizer_data_->LocalPose.lr_speed == 0)?0:1;
//    data.is_auto = map_localizer_data_->LocalPose.mode;

//    data.init = true;
//    recent_data_ = data;
//    recent_data_time_ = GetSystemTime();

}

int DataAnalysis::findInit(double k){
//  std::string mMapPath = "/home/whb/Downloads/data/testrun/map/keyframe";
//  std::string num_pose_str = "/home/whb/Downloads/data/testrun/map/keyframe";

  pcl::PointCloud<PointType6D>::Ptr key_pose;
  key_pose.reset(new pcl::PointCloud<PointType6D>);
  std::cout << "pcl::" << mMapPath + "/keypose.pcd" << std::endl;
  if (pcl::io::loadPCDFile<PointType6D>(mMapPath + "/keypose.pcd", *key_pose) == -1) {
    PCL_ERROR ("Couldn't read PCD file \n");
    return -1;
  }
  pcl::PointCloud<PointType3D>::Ptr num_pose;
  num_pose.reset(new pcl::PointCloud<PointType3D>);
  for (int i = 0; i < key_pose->size()-1; ++i) {
    if (pcl::io::loadPCDFile<PointType3D>(mMapPath + "/"+std::to_string(i)+".pcd", *num_pose) == -1) {
      PCL_ERROR ("Couldn't read PCD file \n");
    }
//    long k ;
    std::cout<<fixed<<std::setprecision(15)<<"key_pose::"<<key_pose->at(i).time<<std::endl;
    if(key_pose->at(i).time>k){
      std::cout<<fixed<<std::setprecision(15)<<"k::"<<k<<std::endl;
      std::cout<<"no!!!"<<std::endl;
      return -1;
    }
    if(key_pose->at(i+1).time >= k ){
      std::cout<<fixed<<std::setprecision(15)<<"key_pose::"<<key_pose->at(i+1).time<<std::endl;
      std::cout<<"Find::i_true::"<<i<<std::endl;
      std::cout<<"Find::i_true::"<<i<<std::endl;
      std::cout<<"Find::i_true::"<<i<<std::endl;
      std::cout<<"Find::i_true::"<<i<<std::endl;
      std::cout<<"Find::i_true::"<<i<<std::endl;
      std::cout<<"Find::i_true::"<<i<<std::endl;
      return i;
    } else{
      std::cout<<"i::"<<i<<std::endl;
    }
  }

  return -1;
}