#ifndef HDLADARSTATUSN_HH
#define HDLADARSTATUSN_HH

#include "rcs.hh"
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

#define HDLADARSTATUS_MSG_TYPE 21002

#define HDLADARSTATUS_WIDTH (80)
#define HDLADARSTATUS_HEIGHT (80)

class HDLADARSTATUS_MSG : public NMLmsgEx
{
public:
    HDLADARSTATUS_MSG();
    
    void update(CMS *);
    INT32 Width;
    INT32 Height;
    UINT8 ImageFormat;    //0-BayerGB,1-BayerBG,2-Gray,3-rgb
    
    UINT8 ImageBuffer[HDLADARSTATUS_WIDTH*HDLADARSTATUS_HEIGHT];
};


extern int HDLadarStatusFormat(NMLTYPE, void *, CMS *);

#endif