#include <functional>
#include <unistd.h>
#include <sys/stat.h>
#include <fstream>
#include "send_data.h"


SendData::SendData() = default;

SendData::SendData(int num) {
  InitNml(0);
}

SendData::~SendData() = default;

void SendData::InitNml(int num) {
  std::string folder_nml = "./nml";
  if (access(folder_nml.c_str(), 0) == -1) {
    mkdir(folder_nml.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
  }
  std::string file_nml = folder_nml + "/UGVAuto_" + std::to_string(num) + ".nml";
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
      "P TestRun            LocalPose                     LOCAL      localhost     W       0             0.1         1        " +
      std::to_string(100 + 3 * (num + 1)) + "\n"
                                            "P TestRun            GlobalPositionBuffer          LOCAL      localhost     W       0             0.1         1        " +
      std::to_string(100 + 3 * (num + 1)) + "\n"
                                            "\n"
                                            "# MP\n"
                                            "P MP            LocalPose                          LOCAL      localhost     R       0             0.1         1        " +
      std::to_string(100 + 3 * (num + 1) + 1) + "\n"
                                                "P MP            GlobalPositionBuffer               LOCAL      localhost     R       0             0.1         1        " +
      std::to_string(100 + 3 * (num + 1) + 1) + "\n"
                                                "\n"
                                                "# LP&GP\n"
                                                "P LP            LocalPose                          LOCAL      localhost     R       0             0.1         1        " +
      std::to_string(100 + 3 * (num + 1) + 2) + "\n"
                                                "P LP            GlobalPositionBuffer               LOCAL      localhost     R       0             0.1         1        " +
      std::to_string(100 + 3 * (num + 1) + 2) + "\n"
                                                ""
  };
  fs_file << file;
  fs_file.close();

  m_ladar_response_channel_ = new NML(HDLadarDataFormat, "LadarMiddleMiddleTop", "TestRun", "UGVAuto.nml");
//  LOCALPOSE_MSG *m_local_response_data_ = new HDLADARDATA_MSG;
  if (!m_ladar_response_channel_->valid()) std::cout << "m_ladar_response_channel_ create Error!" << std::endl;

  m_map_response_channel_ = new NML(MapPositionFormat, "MapPositionInit", "TestRun", "UGVAuto.nml");
//  MAP_POSITION_MSG * map_init_data_ = new MAP_POSITION_MSG;
  if (!m_map_response_channel_->valid()) std::cout << "m_map_response_channel create Error!" << std::endl;

  m_global_response_channel_ = new NML(GlobalPositionInfoFormat, "GlobalPositionBuffer", "TestRun", "UGVAuto.nml");
//  MAP_POSITION_MSG * map_init_data_ = new MAP_POSITION_MSG;
  if (!m_global_response_channel_->valid()) std::cout << "m_global_response_channel_ create Error!" << std::endl;

  m_local_response_channel_ = new NML(LocalPoseFormat, "LocalPose", "TestRun", "UGVAuto.nml");
//  MAP_POSITION_MSG * map_init_data_ = new MAP_POSITION_MSG;
  if (!m_local_response_channel_->valid()) std::cout << "m_local_response_channel_ create Error!" << std::endl;

  m_map_request_channel_ = new NML(MapPositionFormat, "FusionPositionScore", "TestRun", "UGVAuto.nml");
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
