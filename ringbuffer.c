/*
 * ringbuffer.c: A ring buffer
 *
 * See the main source file 'vdr.c' for copyright information and
 * how to reach the author.
 *
 * Parts of this file were inspired by the 'ringbuffy.c' from the
 * LinuxDVB driver (see linuxtv.org).
 *
 * $Id: ringbuffer.c 1.23 2005/12/30 15:42:08 kls Exp $
 */

#include "ringbuffer.h"
#include <stdlib.h>
#include <unistd.h>
#include "tools.h"

// --- cRingBuffer -----------------------------------------------------------

#define OVERFLOWREPORTDELTA 5 // seconds between reports
#define PERCENTAGEDELTA     10
#define PERCENTAGETHRESHOLD 70
//M7X0 BEGIN AK
cRingBuffer::cRingBuffer(int Size, bool Statistics, const char* Description)
{
  description = Description ? strdup(Description) : NULL;
//M7X0 ENDE AK
  size = Size;
  statistics = Statistics;
  getThreadTid = 0;
  maxFill = 0;
  lastPercent = 0;
  putTimeout = getTimeout = 0;
  lastOverflowReport = 0;
  overflowCount = overflowBytes = 0;
}

cRingBuffer::~cRingBuffer()
{
//M7X0 BEGIN AK
  if (statistics)
     dsyslog("buffer stats%s%s: %d (%d%%) used",description?" for ":"",
	  description?description:"", maxFill, maxFill * 100 / (size - 1));

	free(description);
//M7X0 END AK
}

void cRingBuffer::UpdatePercentage(int Fill)
{
  if (Fill > maxFill)
     maxFill = Fill;
  int percent = Fill * 100 / (Size() - 1) / PERCENTAGEDELTA * PERCENTAGEDELTA;
  if (percent != lastPercent) {
     if (percent >= PERCENTAGETHRESHOLD && percent > lastPercent || percent < PERCENTAGETHRESHOLD && lastPercent >= PERCENTAGETHRESHOLD) {
//M7X0 BEGIN AK
        dsyslog("buffer usage%s%s: %d%% (tid=%d)",description?" for ":"",
	  description?description:"", percent, getThreadTid);
//M7X0 END AK
        lastPercent = percent;
        }
     }
}

void cRingBuffer::WaitForPut(void)
{
  if (putTimeout)
     readyForPut.Wait(putTimeout);
}

void cRingBuffer::WaitForGet(void)
{
  if (getTimeout)
     readyForGet.Wait(getTimeout);
}

void cRingBuffer::EnablePut(void)
{
  if (putTimeout && Free() > Size() / 3)
     readyForPut.Signal();
}

void cRingBuffer::EnableGet(void)
{
  if (getTimeout && Available() > Size() / 3)
     readyForGet.Signal();
}

void cRingBuffer::SetTimeouts(int PutTimeout, int GetTimeout)
{
  putTimeout = PutTimeout;
  getTimeout = GetTimeout;
}

void cRingBuffer::ReportOverflow(int Bytes)
{
  overflowCount++;
  overflowBytes += Bytes;
  if (time(NULL) - lastOverflowReport > OVERFLOWREPORTDELTA) {
//M7X0 BEGIN AK
     esyslog("ERROR: %d ring buffer overflow%s (%d bytes dropped)%s%s", overflowCount, overflowCount > 1 ? "s" : "", overflowBytes, 
	  description?" on ":"", description?description:"");
//M7X0 END AK
     overflowCount = overflowBytes = 0;
     lastOverflowReport = time(NULL);
     }
}

// --- cRingBufferLinear -----------------------------------------------------

#ifdef DEBUGRINGBUFFERS
#define MAXRBLS 30
#define DEBUGRBLWIDTH 45

cRingBufferLinear *cRingBufferLinear::RBLS[MAXRBLS] = { NULL };

void cRingBufferLinear::AddDebugRBL(cRingBufferLinear *RBL)
{
  for (int i = 0; i < MAXRBLS; i++) {
      if (!RBLS[i]) {
         RBLS[i] = RBL;
         break;
         }
      }
}

void cRingBufferLinear::DelDebugRBL(cRingBufferLinear *RBL)
{
  for (int i = 0; i < MAXRBLS; i++) {
      if (RBLS[i] == RBL) {
         RBLS[i] = NULL;
         break;
         }
      }
}

void cRingBufferLinear::PrintDebugRBL(void)
{
  bool printed = false;
  for (int i = 0; i < MAXRBLS; i++) {
      cRingBufferLinear *p = RBLS[i];
      if (p) {
         printed = true;
         int lh = p->lastHead;
         int lt = p->lastTail;
         int h = lh * DEBUGRBLWIDTH / p->Size();
         int t = lt * DEBUGRBLWIDTH / p->Size();
         char buf[DEBUGRBLWIDTH + 10];
         memset(buf, '-', DEBUGRBLWIDTH);
         if (lt <= lh)
            memset(buf + t, '*', max(h - t, 1));
         else {
            memset(buf, '*', h);
            memset(buf + t, '*', DEBUGRBLWIDTH - t);
            }
         buf[t] = '<';
         buf[h] = '>';
         buf[DEBUGRBLWIDTH] = 0;
         printf("%2d %s %8d %8d %s\n", i, buf, p->lastPut, p->lastGet, p->description);
         }
      }
  if (printed)
     printf("\n");
  }
#endif
//M7X0 BEGIN AK
cRingBufferLinear::cRingBufferLinear(int Size, int Margin, bool Statistics, const char *Description)
:cRingBuffer(Size, Statistics,Description)
{
//Put to cRingBuffer to easier see which buffer report which thing
  //description = Description ? strdup(Description) : NULL;
//M7X0 END AK
  tail = head = margin = Margin;
  buffer = NULL;
  if (Size > 1) { // 'Size - 1' must not be 0!
     if (Margin <= Size / 2) {
        buffer = MALLOC(uchar, Size);
        if (!buffer)
           esyslog("ERROR: can't allocate ring buffer (size=%d)", Size);
        Clear();
        }
     else
        esyslog("ERROR: invalid margin for ring buffer (%d > %d)", Margin, Size / 2);
     }
  else
     esyslog("ERROR: invalid size for ring buffer (%d)", Size);
#ifdef DEBUGRINGBUFFERS
  lastHead = head;
  lastTail = tail;
  lastPut = lastGet = -1;
  AddDebugRBL(this);
#endif
}

cRingBufferLinear::~cRingBufferLinear()
{
#ifdef DEBUGRINGBUFFERS
  DelDebugRBL(this);
#endif
  free(buffer);
//M7X0 BEGIN AK
 // free(description);
//M7X0 END AK
}

int cRingBufferLinear::Available(void)
{
  int diff = head - tail;
  return (diff >= 0) ? diff : Size() + diff - margin;
}

void cRingBufferLinear::Clear(void)
{
  tail = head;
#ifdef DEBUGRINGBUFFERS
  lastHead = head;
  lastTail = tail;
  lastPut = lastGet = -1;
#endif
  maxFill = 0;
  EnablePut();
}

int cRingBufferLinear::Read(int FileHandle, int Max)
{
  int Tail = tail;
  int diff = Tail - head;
  int free = (diff > 0) ? diff - 1 : Size() - head;
  if (Tail <= margin)
     free--;
  int Count = 0;
  if (free > 0) {
     if (0 < Max && Max < free)
        free = Max;
     Count = safe_read(FileHandle, buffer + head, free);
     if (Count > 0) {
        int Head = head + Count;
        if (Head >= Size())
           Head = margin;
        head = Head;
        if (statistics) {
           int fill = head - Tail;
           if (fill < 0)
              fill = Size() + fill;
           else if (fill >= Size())
              fill = Size() - 1;
           UpdatePercentage(fill);
           }
        }
     }
#ifdef DEBUGRINGBUFFERS
  lastHead = head;
  lastPut = Count;
#endif
  EnableGet();
  if (free == 0)
     WaitForPut();
  return Count;
}

int cRingBufferLinear::Put(const uchar *Data, int Count)
{
  if (Count > 0) {
     int Tail = tail;
     int rest = Size() - head;
     int diff = Tail - head;
     int free = ((Tail < margin) ? rest : (diff > 0) ? diff : Size() + diff - margin) - 1;
     if (statistics) {
        int fill = Size() - free - 1 + Count;
        if (fill >= Size())
           fill = Size() - 1;
        UpdatePercentage(fill);
        }
     if (free > 0) {
        if (free < Count)
           Count = free;
        if (Count >= rest) {
           memcpy(buffer + head, Data, rest);
           if (Count - rest)
              memcpy(buffer + margin, Data + rest, Count - rest);
           head = margin + Count - rest;
           }
        else {
           memcpy(buffer + head, Data, Count);
           head += Count;
           }
        }
     else
        Count = 0;
#ifdef DEBUGRINGBUFFERS
     lastHead = head;
     lastPut = Count;
#endif
     EnableGet();
     if (Count == 0)
        WaitForPut();
     }
  return Count;
}

uchar *cRingBufferLinear::Get(int &Count)
{
  uchar *p = NULL;
  int Head = head;
  if (getThreadTid <= 0)
     getThreadTid = cThread::ThreadId();
  int rest = Size() - tail;
  if (rest < margin && Head < tail) {
     int t = margin - rest;
     memcpy(buffer + t, buffer + tail, rest);
     tail = t;
     rest = Head - tail;
     }
  int diff = Head - tail;
  int cont = (diff >= 0) ? diff : Size() + diff - margin;
  if (cont > rest)
     cont = rest;
  if (cont >= margin) {
     p = buffer + tail;
     Count = gotten = cont;
     }
  if (!p)
     WaitForGet();
  return p;
}

void cRingBufferLinear::Del(int Count)
{
  if (Count > gotten) {
     esyslog("ERROR: invalid Count in cRingBufferLinear::Del: %d (limited to %d)", Count, gotten);
     Count = gotten;
     }
  if (Count > 0) {
     int Tail = tail;
     Tail += Count;
     gotten -= Count;
     if (Tail >= Size())
        Tail = margin;
     tail = Tail;
     EnablePut();
     }
#ifdef DEBUGRINGBUFFERS
  lastTail = tail;
  lastGet = Count;
#endif
}

// --- cFrame ----------------------------------------------------------------

cFrame::cFrame(const uchar *Data, int Count, eFrameType Type, int Index)
{
  count = abs(Count);
  type = Type;
  index = Index;
  if (Count < 0)
     data = (uchar *)Data;
  else {
     data = MALLOC(uchar, count);
     if (data)
        memcpy(data, Data, count);
     else
        esyslog("ERROR: can't allocate frame buffer (count=%d)", count);
     }
  next = NULL;
}

cFrame::~cFrame()
{
  free(data);
}

// --- cRingBufferFrame ------------------------------------------------------

cRingBufferFrame::cRingBufferFrame(int Size, bool Statistics)
:cRingBuffer(Size, Statistics)
{
  head = NULL;
  currentFill = 0;
}

cRingBufferFrame::~cRingBufferFrame()
{
  Clear();
}

void cRingBufferFrame::Clear(void)
{
  Lock();
  cFrame *p;
  while ((p = Get()) != NULL)
        Drop(p);
  Unlock();
  EnablePut();
  EnableGet();
}

bool cRingBufferFrame::Put(cFrame *Frame)
{
  if (Frame->Count() <= Free()) {
     Lock();
     if (head) {
        Frame->next = head->next;
        head->next = Frame;
        head = Frame;
        }
     else {
        head = Frame->next = Frame;
        }
     currentFill += Frame->Count();
     Unlock();
     EnableGet();
     return true;
     }
  return false;
}

cFrame *cRingBufferFrame::Get(void)
{
  Lock();
  cFrame *p = head ? head->next : NULL;
  Unlock();
  return p;
}

void cRingBufferFrame::Delete(cFrame *Frame)
{
  currentFill -= Frame->Count();
  delete Frame;
}

void cRingBufferFrame::Drop(cFrame *Frame)
{
  Lock();
  if (head) {
     if (Frame == head->next) {
        if (head->next != head) {
           head->next = Frame->next;
           Delete(Frame);
           }
        else {
           Delete(head);
           head = NULL;
           }
        }
     else
        esyslog("ERROR: attempt to drop wrong frame from ring buffer!");
     }
  Unlock();
  EnablePut();
}

int cRingBufferFrame::Available(void)
{
  Lock();
  int av = currentFill;
  Unlock();
  return av;
}
