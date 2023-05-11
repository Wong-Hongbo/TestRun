
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

    //ͳ�Ƽ��(ms)
    INT32 timeElapsed;
	
    //CPU ռ����
    float cpuUseRate;

	//�ڴ�ռ����
	float memUseRate;

	//��������kb/s
	INT32 netTranferKB;

	//Io read kb/s
	INT32 ioReadKB;

    //Io write kb/s
	INT32 ioWriteKB;
	
};

// Declare NML format function
extern int COMPUTERSTATUSFormat(NMLTYPE, void *, CMS *);


#endif