/*
IRImagen.hh

This C++ header file defines the NML Messages for IRIMAGE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:04 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef IRIMAGEN_HH
#define IRIMAGEN_HH

// Include Files

#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"


// Define the integer type ids.
#define IRIMAGE_MSG_TYPE 11007
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

// Define the NML Message Classes

class IRIMAGE_MSG : public NMLmsgEx
{
public:

	//Constructor
    IRIMAGE_MSG();

	// CMS Update Function
	void update(CMS *);

    // Place custom variables here.
    double LocalPoseTime[2];
    double GlobalPosTime[2];
    INT32 IRWidth;
    INT32 IRHeight;
//    UINT8 IRImageBuffer[IR_IMAGE_SIZE];


    // IR Double Camera data added by xzp
    UINT8  IRD_Valid_Flag;
    UINT64 TimeStamp_Bayer;  // time stamp to determine image names
    UINT64 TimeStamp_NIR;

    UINT32 PixelType_Bayer;          ///< Pixel Format Type
    UINT32 PixelType_NIR;          ///< Pixel Format Type

    UINT32 SizeX_Bayer;              ///< Image width
    UINT32 SizeY_Bayer;              ///< Image height

    UINT32 SizeX_NIR;              ///< Image width
    UINT32 SizeY_NIR;              ///< Image height

    UINT32 iImageSize_Bayer;          ///< Number of bytes for image
    UINT32 iImageSize_NIR;          ///< Number of bytes for image

    UINT8  pImageBuffer_Bayer[IR_IMAGE_SIZE_BAYER];       ///< Buffer pointer
    UINT8  pImageBuffer_NIR[IR_IMGAE_SIZE_NIR];       ///< Buffer pointer

};

// Declare NML format function
extern int IRImageFormat(NMLTYPE, void *, CMS *);

#endif 	// IRIMAGEN_HH
