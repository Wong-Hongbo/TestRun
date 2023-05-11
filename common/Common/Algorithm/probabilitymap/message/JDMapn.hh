#ifndef  __JDMAP__
#define  __JDMAP__

#include "rcs.hh"


#define JDMAP_MSG_TYPE 10006

#define JDMAP_MSG_SIZE 500000

class JDMAP_MSG: public NMLmsg
{
public:

    //Constructor
    JDMAP_MSG();

    // CMS Update Function
    void update(CMS *);

    int len;
    char data[JDMAP_MSG_SIZE];
};

// Declare NML format function
extern int JDMapFormat(NMLTYPE, void *, CMS *);


#endif
