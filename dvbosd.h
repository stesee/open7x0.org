/*
 * dvbosd.h: Implementation of the DVB On Screen Display
 *
 * See the main source file 'vdr.c' for copyright information and
 * how to reach the author.
 *
 * $Id: dvbosd.h 1.18 2004/06/12 13:09:52 kls Exp $
 */

#ifndef __DVBOSD_H
#define __DVBOSD_H

#include "osd.h"

class cDvbOsdProvider : public cOsdProvider {
private:
  int osdDev;
//M7X0 BEGIN AK
// OSD is done via framebuffer-device so mmap can be used.
// Save settings in here.
	uint32_t *fbMem;
	bool fbInterlaced;
	int fbWidth;
	int fbHeight;
	int fbBpp;
//M7X0 END AK
public:
  cDvbOsdProvider(int OsdDev);
//M7X0 BEGIN AK
  virtual ~cDvbOsdProvider();
//M7X0 END AK
  virtual cOsd *CreateOsd(int Left, int Top);
  };

#endif //__DVBOSD_H
