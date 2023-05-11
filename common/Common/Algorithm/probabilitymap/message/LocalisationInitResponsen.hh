#ifndef LOCALISATIONINITRESPONSE_HH
#define LOCALISATIONINITRESPONSE_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "NMLmsgExn.hh"

#define LOCALISATIONINITRESPONSE_MSG_TYPE 15930

class LOCALISATIONINITRESPONSE_MSG :public NMLmsgEx {
public:
    LOCALISATIONINITRESPONSE_MSG();

    // CMS Update Function
    void update(CMS *);
    unsigned char MsgContent[4096];

    // UINT8 cmd;//0 定位成功
    //           //0 定位成功 + pose
    //           //1 use gps mode
    //           //2 get stations(response stations),
    //           //3 station mode(with position)
    //           //4 manual mode
    //           //5 定位失败

    //           //9 manual mode(no gps, with pose)
    // InitStationLists station_list;
    // MatchResult match_result;
};

extern int LocalisationInitResponseFormat(NMLTYPE, void *, CMS *);

#endif
