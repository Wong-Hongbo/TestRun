/*
Ladar64OriginDatan.hh

This C++ header file defines the NML Messages for HDLadarData
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef LADAR64ORIGINDATAN_HH
#define LADAR64ORIGINDATAN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"
#include "LocalPosen.hh"


// Define the integer type ids.
#define LADAR64ORIGINDATA_MSG_TYPE 11013
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.
// Define the NML Message Classes
class LADAR64ORIGINDATA_MSG : public NMLmsgEx
{
public:
    //Constructor
    LADAR64ORIGINDATA_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    char Ladar64Data[PACKETNUM][1206];
};

// Declare NML format function
extern int Ladar64OriginDataFormat(NMLTYPE, void *, CMS *);

#endif 	// HDLADARDATAN_HH
