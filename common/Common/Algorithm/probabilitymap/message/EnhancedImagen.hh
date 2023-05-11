/*
EnhancedImagen.hh

This C++ header file defines the NML Messages for ENHANCEDIMAGE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:04 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef ENHANCEDIMAGEN_HH
#define ENHANCEDIMAGEN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"


// Define the integer type ids.
#define ENHANCEDIMAGE_MSG_TYPE 3303
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

// Define the NML Message Classes
//#define ENHANCED_IMAGE_SIZE 2000*1000

class ENHANCEDIMAGE_MSG : public NMLmsgEx
{
public:

	//Constructor
    ENHANCEDIMAGE_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 Width;
    INT32 Height;
    INT32 Channel;
    unsigned char EnhancedImageBuffer[ENHANCED_IMAGE_SIZE];

};

// Declare NML format function
extern int EnhancedImageFormat(NMLTYPE, void *, CMS *);

#endif 	// ENHANCEDIMAGEN_HH
