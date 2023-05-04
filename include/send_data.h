
#ifndef SEND_DATA_H
#define SEND_DATA_H

#include <iostream>
#include <thread>
#include <vector>

#include "GlobalPositionInfon.hh"
#include "LocalPosen.hh"
#include "HDLadarDatan.hh"


#include "data_analysis.h"

class SendData {
public:
  SendData();
  SendData(std::deque<HDLADARDATA_MSG>,int);
  ~SendData();

  void Send();
  void SendLocalPose(HDLADARDATA_MSG);
private:
  void InitNml(int);
//  void SendGlobalPosition();

private:
  NML *m_local_response_channel_;
  LOCALPOSE_MSG *m_local_response_data_;
  NML *m_global_response_channel_;
  GLOBALPOSITIONINFO_MSG *m_global_response_data_;


  // 线程
//  std::thread* mThreadSendGP = nullptr;
  std::thread* mThreadSendLP = nullptr;

  // data
  std::deque<HDLADARDATA_MSG> mLidarQue;
//  std::vector <GLOBALPOSITIONINFO_MSG> global_position_datas_{};
//  std::vector <LOCALPOSE_MSG > local_pose_datas_{};
};

#endif
