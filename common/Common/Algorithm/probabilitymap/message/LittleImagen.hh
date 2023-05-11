/*
ColorImagen.hh

This C++ header file defines the NML Messages for COLORIMAGE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:04 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef LITTLEIMAGEN_HH
#define LITTLEIMAGEN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"

// Define the integer type ids.
#define LITTLEIMAGE_MSG_TYPE 11009
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

#define LITTLEIMAGE_WIDTH 400
#define LITTLEIMAGE_HEIGHT 200
#define LITTLEIMAGE_SIZE LITTLEIMAGE_WIDTH*LITTLEIMAGE_HEIGHT

// Define the NML Message Classes

class LITTLEIMAGE_MSG : public NMLmsgEx
{
public:
	//Constructor
	LITTLEIMAGE_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    double LocalPoseTime[2];
    double GlobalPosTime[2];
    INT32 Width;
    INT32 Height;
    INT32 CameraNumber;  //0, left and right, 1: only left; 2: only right;
    INT32 ImageFormat;   //0-BayerGB,1-BayerBG,2-Gray,3-rgb
    INT32 PixelChannel;

    //DECLARE_NML_DYNAMIC_LENGTH_ARRAY(UINT8, ImageBufferL, COLORIMAGE_SIZE);
    //DECLARE_NML_DYNAMIC_LENGTH_ARRAY(UINT8, ImageBufferR, COLORIMAGE_SIZE);

    UINT8 ImageBufferL[LITTLEIMAGE_SIZE];
    UINT8 ImageBufferR[LITTLEIMAGE_SIZE];

 };

// Declare NML format function
extern int LittleImageFormat(NMLTYPE, void *, CMS *);

#endif 	// COLORIMAGEN_HH
