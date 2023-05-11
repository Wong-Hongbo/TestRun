#ifndef LOCALISATIONINITSTATE_HH
#define LOCALISATIONINITSTATE_HH

// Include Files
#include "NMLmsgExn.hh"
#include "rcs.hh"  // Common RCS definitions

#define LOCALISATIONINITSTATE_MSG_TYPE 15931

class LOCALISATIONINITSTATE_MSG : public NMLmsgEx {
 public:
  LOCALISATIONINITSTATE_MSG();

  // CMS Update Function
  void update(CMS *);
  
  // state
  int state; // 0 idle, 1 busy, -1 error
  // unsigned char msg[1024];
};

extern int LocalisationInitStateFormat(NMLTYPE, void *, CMS *);

#endif
