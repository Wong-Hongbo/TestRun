#include <functional>
#include <unistd.h>
#include <sys/stat.h>
#include <fstream>
#include "send_data.h"


SendData::SendData() = default;

SendData::SendData(int num) {
  InitNml(num);
}

SendData::~SendData() = default;

void SendData::InitNml(int num) {
  m_ladar_response_channel_ = new NML(HDLadarDataFormat, std::string{"LadarMiddleMiddleTop"+std::to_string(num)}.data(),
                                      std::string{"TestRun"+std::to_string(num)}.data(), "UGVAuto.nml");
//  LOCALPOSE_MSG *m_local_response_data_ = new HDLADARDATA_MSG;
  if (!m_ladar_response_channel_->valid()) std::cout << "m_ladar_response_channel_ create Error!" << std::endl;

  m_map_response_channel_ = new NML(MapPositionFormat,  std::string{"MapPositionInit"+std::to_string(num)}.data(),
                                    std::string{"TestRun"+std::to_string(num)}.data(), "UGVAuto.nml");
//  MAP_POSITION_MSG * map_init_data_ = new MAP_POSITION_MSG;
  if (!m_map_response_channel_->valid()) std::cout << "m_map_response_channel create Error!" << std::endl;

  m_global_response_channel_ = new NML(GlobalPositionInfoFormat,
                                       std::string{"GlobalPositionBuffer"+std::to_string(num)}.data(),
                                       std::string{"TestRun"+std::to_string(num)}.data(), "UGVAuto.nml");
//  MAP_POSITION_MSG * map_init_data_ = new MAP_POSITION_MSG;
  if (!m_global_response_channel_->valid()) std::cout << "m_global_response_channel_ create Error!" << std::endl;

  m_local_response_channel_ = new NML(LocalPoseFormat, std::string{"LocalPose"+std::to_string(num)}.data(),
                                      std::string{"TestRun"+std::to_string(num)}.data(), "UGVAuto.nml");
//  MAP_POSITION_MSG * map_init_data_ = new MAP_POSITION_MSG;
  if (!m_local_response_channel_->valid()) std::cout << "m_local_response_channel_ create Error!" << std::endl;

  m_map_request_channel_ = new NML(MapPositionFormat, std::string{"FusionPositionScore"+std::to_string(num)}.data(),
                                   std::string{"TestRun"+std::to_string(num)}.data(), "UGVAuto.nml");
  map_localizer_data_ =(MAP_POSITION_MSG *)m_map_request_channel_->get_address();
  if (!m_map_request_channel_->valid()) std::cout << "m_map_request_channel_ create Error!" << std::endl;
}

void SendData::SendHDLadarData(HDLADARDATA_MSG msg) {
  if(m_ladar_response_channel_ != nullptr){
    m_ladar_response_channel_->write(msg);
  }
}

void SendData::SendMapPosition(MAP_POSITION_MSG msg) {
  if(m_map_response_channel_ != nullptr){
    m_map_response_channel_->write(msg);
  }
}
void SendData::SendGlobalPosition(GLOBALPOSITIONINFO_MSG msg){
  if(m_global_response_channel_ != nullptr){
    m_global_response_channel_->write(msg);
  }
}
void SendData::SendLocalPose(LOCALPOSE_MSG msg){
  if(m_local_response_channel_ != nullptr){
    m_local_response_channel_->write(msg);
  }
}


int SendData::ReceiveMapPosition(MAP_POSITION_MSG& r){
  if(m_map_request_channel_ != nullptr && MAP_POSITION_MSG_TYPE == m_map_request_channel_->read()){
//    MAP_POSITION_MSG r;
    r.LocalPose.time = map_localizer_data_->LocalPose.time;
    r.global_x = map_localizer_data_->global_x;
    r.global_y = map_localizer_data_->global_y;
    r.score = map_localizer_data_->score;
    std::cout<<"Read::ReceiveMapPosition"<<std::endl;
    return 0;
  }
  return 1;
}
