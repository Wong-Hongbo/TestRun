
#ifndef SEND_DATA_H
#define SEND_DATA_H

#include <iostream>
#include <thread>
#include <vector>

#include "HDLadarDatan.hh"
#include "MapPositionn.hh"
//#include "


//#include "data_analysis.h"

class SendData {
public:
  SendData();
  SendData(int);
  ~SendData();

  void SendHDLadarData(HDLADARDATA_MSG);
  void SendMapPosition(MAP_POSITION_MSG);
  void SendGlobalPosition(GLOBALPOSITIONINFO_MSG);
  void SendLocalPose(LOCALPOSE_MSG);

  int ReceiveMapPosition(MAP_POSITION_MSG&);
private:
  void InitNml(int);

private:
  NML *m_ladar_response_channel_{};
  NML *m_map_response_channel_{};
  NML *m_global_response_channel_{};
  NML *m_local_response_channel_{};
  NML *m_map_request_channel_{};
  MAP_POSITION_MSG *map_localizer_data_{};

};

#endif
