#ifndef COLORIMAGESTATUSN_HH
#define COLORIMAGESTATUSN_HH

#include "rcs.hh"
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

#define COLORIMAGESTATUS_MSG_TYPE 21001

#define IMAGESTATUS_WIDTH (100)
#define IMAGESTATUS_HEIGHT (50)

class COLORIMAGESTATUS_MSG : public NMLmsgEx
{
public:
    COLORIMAGESTATUS_MSG();
    
    void update(CMS *);
    INT32 Width;
    INT32 Height;
    UINT8 CameraNumber;    //0, left and right, 1: only left; 2: only right;
    UINT8 ImageFormat;    //0-BayerGB,1-BayerBG,2-Gray,3-rgb
    
    UINT8 ImageBufferL[IMAGESTATUS_WIDTH*IMAGESTATUS_HEIGHT];
    UINT8 ImageBufferR[IMAGESTATUS_WIDTH*IMAGESTATUS_HEIGHT];
};


extern int ColorImageStatusFormat(NMLTYPE, void *, CMS *);

#endif