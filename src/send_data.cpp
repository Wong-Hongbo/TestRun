#include <functional>
#include <unistd.h>
#include <sys/stat.h>
#include <fstream>
#include "send_data.h"


SendData::SendData() {
  InitNml(0);

}

SendData::SendData(std::deque<HDLADARDATA_MSG> LidarQue,int num){
  InitNml(num);
  mLidarQue = LidarQue;
//  global_position_datas_ = std::move(global_position_datas);
//  local_pose_datas_ = std::move(local_pose_datas);
}

SendData::~SendData() {

}

void SendData::Send(){
//  mThreadSendLP = new std::thread( std::bind(&SendData::SendLocalPose, this));
//  mThreadSendLP->join();
}

void SendData::InitNml(int num) {

  std::string folder_nml = "./nml";
  if (access(folder_nml.c_str(), 0) == -1) {
    mkdir(folder_nml.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
  }
  std::string file_nml = folder_nml+"/UGVAuto_"+std::to_string(num)+".nml";
  std::fstream fs_file;
  fs_file.open(file_nml, std::ios::out | std::ios::trunc);
  std::string file{
      "\n"
      "# Buffers\n"
      "# Name                     Type       Host                    size       neut?          RPC#                    buffer#          MP     . . .\n"
      "# -----------------------以下为传感器buffer，只支持本地读写--------------#\n"
      "\n"
      "#车辆自身状态Buffer\n"
      "B GlobalPositionBuffer           SHMEM     localhost               20480      0        *     001           *         1001 bsem=11001  TCP=11001  xdr\n"
      "B LocalPose                      SHMEM     localhost               10240      0        *     002           *         1002 bsem=11002  TCP=11002  xdr\n"
      "\n"
      "\n"
      "#-------------------------------------------------------------------------------------------#\n"
      "#                                                                                           #\n"
      "#                                                                                           #\n"
      "#                                  下面是进程处理                                             #\n"
      "#                                                                                           #\n"
      "#                                                                                           #\n"
      "#-------------------------------------------------------------------------------------------#\n"
      "\n"
      "# Processes\n"
      "# Name                 Buffer                       Type         Host                    Ops     server?      timeout     master?     cnum\n"
      "\n"
      "\n"
      "# Data\n"
      "P TestRun            LocalPose                     LOCAL      localhost     W       0             0.1         1        "+std::to_string(100+3*(num+1))+"\n"
      "P TestRun            GlobalPositionBuffer          LOCAL      localhost     W       0             0.1         1        "+std::to_string(100+3*(num+1))+"\n"
      "\n"
      "# MP\n"
      "P MP            LocalPose                          LOCAL      localhost     R       0             0.1         1        "+std::to_string(100+3*(num+1)+1)+"\n"
      "P MP            GlobalPositionBuffer               LOCAL      localhost     R       0             0.1         1        "+std::to_string(100+3*(num+1)+1)+"\n"
      "\n"
      "# LP&GP\n"
      "P LP            LocalPose                          LOCAL      localhost     R       0             0.1         1        "+std::to_string(100+3*(num+1)+2)+"\n"
      "P LP            GlobalPositionBuffer               LOCAL      localhost     R       0             0.1         1        "+std::to_string(100+3*(num+1)+2)+"\n"
      ""
  };
  fs_file<<file;
  fs_file.close();

  m_local_response_channel_ = new NML(HDLadarDataFormat, "LadarMiddleMiddleTop1", "TestRun", "UGVAuto.nml");
//  m_local_response_data_ = new HDLADARDATA_MSG;
  if (!m_local_response_channel_->valid()) std::cout << "m_local_response_channel_ create Error!" << std::endl;

  m_global_response_channel_ = new NML(GlobalPositionInfoFormat, "GlobalPositionBuffer1", "TestRun",  "UGVAuto.nml");
  m_global_response_data_ = new GLOBALPOSITIONINFO_MSG;
  if (!m_global_response_channel_->valid()) std::cout << "m_global_response_channel_ create Error!" << std::endl;

}


// 所有匹配得到的结果都会存在成员变量pose_best_，由LocalizeSuccess()函数下发给定位程序
//void SendData::SendLocalPose() {
//  std::cout<<"mLidarQue.size"<<mLidarQue.size()<<std::endl;
//  for (int i = 0; i < mLidarQue.size(); ++i) {
//    std::cout << "LP::"<<mLidarQue.at(i).LocalPose.dr_x<<","<<mLidarQue.at(i).LocalPose.dr_y<<","<<mLidarQue.at(i).LocalPose.dr_z<<std::endl;
//    m_local_response_channel_->write(mLidarQue.at(i));
//    usleep(100000);
//  }
//}

void SendData::SendLocalPose(HDLADARDATA_MSG msg) {
  std::cout<<"mLidarQue.size"<<mLidarQue.size()<<std::endl;
  std::cout << "LP::"<<msg.LocalPose.dr_x<<","<<msg.LocalPose.dr_y<<","<<msg.LocalPose.dr_z<<std::endl;
  std::cout << "GP::"<<msg.Position.gaussPos[0]<<","<<msg.Position.gaussPos[0]<<","<<msg.Position.azimuth<<std::endl;
  m_local_response_channel_->write(msg);

}



//void SendData::SendGlobalPosition() {
//  for (std::deque<SyncRawScan>::const_iterator it = mLidarQue.begin(); it != mLidarQue.end(); ++it){
//    cout << "GP::"<<it->gp.Position.gaussPos[0]<<","<<it->gp.Position.gaussPos[1]<<","<<it->gp.Position.height<<std::endl;
////    GLOBALPOSITIONINFO_MSG *global_position_info;
////    memcpy(global_position_info,&it->gp,sizeof(GLOBALPOSITIONINFO_MSG));
//    m_global_response_channel_->write((GLOBALPOSITIONINFO_MSG &)it->lp);
////    MapperResponse_data
//    usleep(100000);
//  }
////  return rtn;
//}