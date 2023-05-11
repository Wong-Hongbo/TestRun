#ifndef MLADARSTATUSN_HH
#define MLADARSTATUSN_HH

#include "rcs.hh"
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

#define MLADARSTATUS_MSG_TYPE 21003

#define  MLADARSTATUS_WIDTH (80)
#define  MLADARSTATUS_HEIGHT (80)

class  MLADARSTATUS_MSG : public NMLmsgEx
{
public:
    MLADARSTATUS_MSG();
    
    void update(CMS *);
    INT32 Width;
    INT32 Height;
    UINT8 ImageFormat;    //0-BayerGB,1-BayerBG,2-Gray,3-rgb
    
    UINT8 ImageBuffer[MLADARSTATUS_WIDTH*MLADARSTATUS_HEIGHT];
};


extern int MLadarStatusFormat(NMLTYPE, void *, CMS *);

#endif