/*
 * nit.h: NIT section filter
 *
 * See the main source file 'vdr.c' for copyright information and
 * how to reach the author.
 *
 * $Id$
 */

#ifndef __NIT_H
#define __NIT_H

#include "filter.h"

#define MAXNITS 16
#define MAXNETWORKNAME 256

class cNitFilter : public cFilter {
private:

  class cNit {
  public:
    u_short networkId;
    char name[MAXNETWORKNAME];
    bool hasTransponder;
    };

  cSectionSyncer sectionSyncer;
  cNit nits[MAXNITS];
  u_short networkId;
  int numNits;
protected:
  virtual void Process(u_short Pid, u_char Tid, const u_char *Data, int Length);
public:
  cNitFilter(void);
  virtual void SetStatus(bool On);
  };

#endif //__NIT_H
