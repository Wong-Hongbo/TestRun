#ifndef LOCALISATIONINITREQUEST_HH
#define LOCALISATIONINITREQUEST_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "NMLmsgExn.hh"

#define LOCALISATIONINITREQUEST_MSG_TYPE 15929

class LOCALISATIONINITREQUEST_MSG : public NMLmsgEx {
public:
    // Constructor
    LOCALISATIONINITREQUEST_MSG();

    // CMS Update Function
    void update(CMS *);
    unsigned char MsgContent[1024];
    // UINT8 cmd;  //0 定位成功
    //             //0 定位成功 + pose
    //             //1 use gps mode
    //             //2 get stations(response stations),
    //             //3 station mode(with position)
    //             //4 manual mode
    //             //5 定位失败
    //             //6 获取底图和雷达图
    //             //7 检查定位结果
    //             //8 获取大底图(回应底图)
    //             //9 manual mode(no gps, with pose)
    // RequestPosition request_position;
    // ResultPose  result_pose;
};

// Declare NML format function
extern int LocalisationInitRequestFormat(NMLTYPE, void *, CMS *);

#endif // LOCALISATIONINITREQUEST_HH
