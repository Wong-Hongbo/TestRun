/*
ColorImagen.hh

This C++ header file defines the NML Messages for COLORIMAGE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:04 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef COLORIMAGEN_HH
#define COLORIMAGEN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"

// Define the integer type ids.
#define COLORIMAGE_MSG_TYPE 11002
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

// Define the NML Message Classes

class COLORIMAGE_MSG : public NMLmsgEx
{
public:
	//Constructor
	COLORIMAGE_MSG();

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

    UINT8 ImageBufferL[COLORIMAGE_SIZE];
    UINT8 ImageBufferR[COLORIMAGE_SIZE];

    INT32 TimeDifference;   // capture time difference
	
    int  CameraStatus;   // Store the status of the cameras  0:bad 1:good
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

// Declare NML format function
extern int ColorImageFormat(NMLTYPE, void *, CMS *);

#endif 	// COLORIMAGEN_HH
