#ifndef GRASSMAP_HH
#define GRASSMAP_HH


#include "rcs.hh"
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"


#define GRASSMAP_MSG_TYPE 5413

#define GRASSMAP_WIDTH (1000)
#define GRASSMAP_HEIGHT (500)


//0:²»ÊÇ²Ý
//1:ÊÇ²Ý
class GRASSMAP_MSG : public NMLmsgEx
{

public:
    GRASSMAP_MSG();

    void update(CMS *);

    INT32 MapWidth;
    INT32 MapHeight;

    UINT8 GrassMap[GRASSMAP_WIDTH*GRASSMAP_HEIGHT];

    // KK of left and right cameras
    double M1[3][3];
    double M2[3][3];
    // R T between left and right cameras
    double RotateLR[3][3];
    double TranslateLR[3];
    // D of LR
    double D1[5];
    double D2[5];
    // external parameters of left camera
    double RotateLeft[3][3];
    double TranslateLeft[3];
    double OriginLeft[3];
    // external parameters of right camera
    double RotateRight[3][3];
    double TranslateRight[3];
    double OriginRight[3];
};


extern int GrassMapFormat(NMLTYPE, void *, CMS *);

#endif