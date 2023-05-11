
#ifndef __COMPUTERSTATUS__
#define __COMPUTERSTATUS__

#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"


// Define the integer type ids.
#define COMPUTERSTATUS_MSG_TYPE 11003


// Define the NML Message Classes
class COMPUTERSTATUS_MSG : public NMLmsgEx
{
public:
    //Constructor
    COMPUTERSTATUS_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.

    //统计间隔(ms)
    INT32 timeElapsed;
	
    //CPU 占用率
    float cpuUseRate;

	//内存占用率
	float memUseRate;

	//网络流量kb/s
	INT32 netTranferKB;

	//Io read kb/s
	INT32 ioReadKB;

    //Io write kb/s
	INT32 ioWriteKB;
	
};

// Declare NML format function
extern int COMPUTERSTATUSFormat(NMLTYPE, void *, CMS *);


#endif