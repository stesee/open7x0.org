/*
 * dvbdevice.c: The DVB device interface
 *
 * See the main source file 'vdr.c' for copyright information and
 * how to reach the author.
 *
 * $Id: dvbdevice.c 1.159 2006/06/11 09:03:55 kls Exp $
 */

#include "dvbdevice.h"
#include <errno.h>
#include <limits.h>
#include <linux/videodev.h>
//M7X0 BEGIN AK
#include "m7x0_dvb/audio.h"
#include "m7x0_dvb/dmx.h"
#include "m7x0_dvb/dmx_ext.h"
#include "m7x0_dvb/frontend.h"
#include "m7x0_dvb/video.h"
//M7X0 END AK
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "channels.h"
#include "diseqc.h"
#include "dvbosd.h"
#include "eitscan.h"
#include "player.h"
#include "receiver.h"
#include "status.h"
#include "transfer.h"

#define DO_REC_AND_PLAY_ON_PRIMARY_DEVICE 1
#define DO_MULTIPLE_RECORDINGS 1
//#define DO_MULTIPLE_CA_CHANNELS

//M7X0 BEGIN AK
#define DEV_VIDEO         "/dev/video"
#define DEV_DVB_ADAPTER   "/dev/ost/"
#define DEV_DVB_OSD       "/dev/fb"
//M7X0 END AK
#define DEV_DVB_FRONTEND  "frontend"
#define DEV_DVB_DVR       "dvr"
#define DEV_DVB_DEMUX     "demux"
#define DEV_DVB_VIDEO     "video"
#define DEV_DVB_AUDIO     "audio"
#define DEV_DVB_CA        "ca"

//M7X0 BEGIN AK 
#define DVBS_TUNE_TIMEOUT  9000 //ms
#define DVBS_LOCK_TIMEOUT  3000 //ms
#define DVBC_TUNE_TIMEOUT  9000 //ms
#define DVBC_LOCK_TIMEOUT  3000 //ms
#define DVBT_TUNE_TIMEOUT  9000 //ms
#define DVBT_LOCK_TIMEOUT  3000 //ms


// Taken from gambler 
// For 16:9/4:3 switching
#define AVSWCMD_MODE_16_9  0x89
#define AVSWCMD_MODE_4_3   0x90
//M7X0 BEGIN AK
class cDvbName {
private:
  char buffer[PATH_MAX];
public:
  cDvbName(const char *Name, int n) {
//M7X0 BEGIN AK
    snprintf(buffer, sizeof(buffer), "%s%d", Name, n);
//M7X0 END AK
    }
  const char *operator*() { return buffer; }
  };

static int DvbOpen(const char *Name, int n, int Mode, bool ReportError = false)
{
  const char *FileName = *cDvbName(Name, n);
  int fd = open(FileName, Mode);
  if (fd < 0 && ReportError)
     LOG_ERROR_STR(FileName);
  return fd;
}

// --- cDvbTuner -------------------------------------------------------------

class cDvbTuner : public cThread {
private:
  enum eTunerStatus { tsIdle, tsSet, tsTuned, tsLocked };
  int fd_frontend;
  int cardIndex;
  int tuneTimeout;
  int lockTimeout;
  time_t lastTimeoutReport;
  fe_type_t frontendType;
  cCiHandler *ciHandler;
  cChannel channel;
  const char *diseqcCommands;
  eTunerStatus tunerStatus;
  cMutex mutex;
  cCondVar locked;
  cCondVar newSet;
  bool GetFrontendStatus(fe_status_t &Status, int TimeoutMs = 0);
  bool SetFrontend(void);
  virtual void Action(void);
public:
  cDvbTuner(int Fd_Frontend, int CardIndex, fe_type_t FrontendType, cCiHandler *CiHandler);
  virtual ~cDvbTuner();
  bool IsTunedTo(const cChannel *Channel) const;
  void Set(const cChannel *Channel, bool Tune);
  bool Locked(int TimeoutMs = 0);
  };

cDvbTuner::cDvbTuner(int Fd_Frontend, int CardIndex, fe_type_t FrontendType, cCiHandler *CiHandler)
{
  fd_frontend = Fd_Frontend;
  cardIndex = CardIndex;
  frontendType = FrontendType;
  ciHandler = CiHandler;
  tuneTimeout = 0;
  lockTimeout = 0;
  lastTimeoutReport = 0;
  diseqcCommands = NULL;
  tunerStatus = tsIdle;
  if (frontendType == FE_QPSK)
     CHECK(ioctl(fd_frontend, FE_SET_VOLTAGE, SEC_VOLTAGE_13)); // must explicitly turn on LNB power
  SetDescription("tuner on device %d", cardIndex + 1);
  Start();
}

cDvbTuner::~cDvbTuner()
{
  tunerStatus = tsIdle;
  newSet.Broadcast();
  locked.Broadcast();
  Cancel(3);
}

bool cDvbTuner::IsTunedTo(const cChannel *Channel) const
{
  return tunerStatus != tsIdle && channel.Source() == Channel->Source() && channel.Transponder() == Channel->Transponder();
}

void cDvbTuner::Set(const cChannel *Channel, bool Tune)
{
  cMutexLock MutexLock(&mutex);
  if (Tune)
     tunerStatus = tsSet;
  channel = *Channel;
  lastTimeoutReport = 0;
  newSet.Broadcast();
}

bool cDvbTuner::Locked(int TimeoutMs)
{
  bool isLocked = (tunerStatus >= tsLocked);
//M7X0 BEGIN AK
  if (TimeoutMs<0)
     TimeoutMs=(lockTimeout!=0)?lockTimeout:3000;
//M7X0 END
  if (isLocked || !TimeoutMs)
     return isLocked;

  cMutexLock MutexLock(&mutex);
  if (TimeoutMs && tunerStatus < tsLocked)
     locked.TimedWait(mutex, TimeoutMs);
  return tunerStatus >= tsLocked;
}
//M7X0 BEGIN AK
// m7x0 seems not to can handle FE_READ_STATUS very well.
// Seems to cause hangs while change switch and unneed(?) reinits
// and EBUSY Errors.
// Lets drop this call for while and see if problems are gone
bool cDvbTuner::GetFrontendStatus(fe_status_t &Status, int TimeoutMs)
{
  dvb_frontend_event Event;

  if (TimeoutMs) {
     cPoller Poller(fd_frontend);
     if (Poller.Poll(TimeoutMs))
        while (ioctl(fd_frontend, FE_GET_EVENT, &Event) == 0)
              ;
    }
    
  do {
     if (ioctl(fd_frontend, FE_READ_STATUS, &Status) == 0){
        return true;
        }
     if (errno == EINTR || errno == EBUSY) { // M7X0 returns in many cases EBUSY 
        cCondWait::SleepMs(3);
        continue;
        }
     } while (0); 

  return false;
#if 0 // Orginal code just in case ...
  if (TimeoutMs) {
     cPoller Poller(fd_frontend);
     if (Poller.Poll(TimeoutMs)) {
        dvb_frontend_event Event;
        while (ioctl(fd_frontend, FE_GET_EVENT, &Event) == 0)
              ; // just to clear the event queue - we'll read the actual status below
        }
     }

  do {
     int stat = ioctl(fd_frontend, FE_READ_STATUS, &Status);
     if (stat == 0)
        return true;
     if (stat < 0) {
        if (errno == EINTR)
           continue;
        }
     } while (0);
  return false;
#endif
}
//M7X0 END AK

static unsigned int FrequencyToHz(unsigned int f)
{
  while (f && f < 1000000)
        f *= 1000;
  return f;
}

bool cDvbTuner::SetFrontend(void)
{
  dvb_frontend_parameters Frontend;

  memset(&Frontend, 0, sizeof(Frontend));

  switch (frontendType) {
    case FE_QPSK: { // DVB-S

         unsigned int frequency = channel.Frequency();

         if (Setup.DiSEqC) {
            cDiseqc *diseqc = Diseqcs.Get(channel.Source(), channel.Frequency(), channel.Polarization());
            if (diseqc) {
               if (diseqc->Commands() && (!diseqcCommands || strcmp(diseqcCommands, diseqc->Commands()) != 0)) {
                  cDiseqc::eDiseqcActions da;
                  for (char *CurrentAction = NULL; (da = diseqc->Execute(&CurrentAction)) != cDiseqc::daNone; ) {
                      switch (da) {
                        case cDiseqc::daNone:      break;
                        case cDiseqc::daToneOff:   CHECK(ioctl(fd_frontend, FE_SET_TONE, SEC_TONE_OFF)); break;
                        case cDiseqc::daToneOn:    CHECK(ioctl(fd_frontend, FE_SET_TONE, SEC_TONE_ON)); break;
                        case cDiseqc::daVoltage13: CHECK(ioctl(fd_frontend, FE_SET_VOLTAGE, SEC_VOLTAGE_13)); break;
                        case cDiseqc::daVoltage18: CHECK(ioctl(fd_frontend, FE_SET_VOLTAGE, SEC_VOLTAGE_18)); break;
                        case cDiseqc::daMiniA:     CHECK(ioctl(fd_frontend, FE_DISEQC_SEND_BURST, SEC_MINI_A)); break;
                        case cDiseqc::daMiniB:     CHECK(ioctl(fd_frontend, FE_DISEQC_SEND_BURST, SEC_MINI_B)); break;
                        case cDiseqc::daCodes: {
                             int n = 0;
                             uchar *codes = diseqc->Codes(n);
                             if (codes) {
                                struct dvb_diseqc_master_cmd cmd;
                                memcpy(cmd.msg, codes, min(n, int(sizeof(cmd.msg))));
                                cmd.msg_len = n;
                                CHECK(ioctl(fd_frontend, FE_DISEQC_SEND_MASTER_CMD, &cmd));
                                }
                             }
                             break;
                        }
                      }
                  diseqcCommands = diseqc->Commands();
                  }
               frequency -= diseqc->Lof();
               }
            else {
               esyslog("ERROR: no DiSEqC parameters found for channel %d", channel.Number());
               return false;
               }
            }
         else {
            int tone = SEC_TONE_OFF;

            if (frequency < (unsigned int)Setup.LnbSLOF) {
               frequency -= Setup.LnbFrequLo;
               tone = SEC_TONE_OFF;
               }
            else {
               frequency -= Setup.LnbFrequHi;
               tone = SEC_TONE_ON;
               }
            int volt = (channel.Polarization() == 'v' || channel.Polarization() == 'V' || channel.Polarization() == 'r' || channel.Polarization() == 'R') ? SEC_VOLTAGE_13 : SEC_VOLTAGE_18;
            CHECK(ioctl(fd_frontend, FE_SET_VOLTAGE, volt));
            CHECK(ioctl(fd_frontend, FE_SET_TONE, tone));
            }

         frequency = abs(frequency); // Allow for C-band, where the frequency is less than the LOF
         Frontend.frequency = frequency * 1000UL;
         Frontend.inversion = fe_spectral_inversion_t(channel.Inversion());
         Frontend.u.qpsk.symbol_rate = channel.Srate() * 1000UL;
         Frontend.u.qpsk.fec_inner = fe_code_rate_t(channel.CoderateH());

         tuneTimeout = DVBS_TUNE_TIMEOUT;
         lockTimeout = DVBS_LOCK_TIMEOUT;
         }
         break;
    case FE_QAM: { // DVB-C

         // Frequency and symbol rate:

         Frontend.frequency = FrequencyToHz(channel.Frequency());
         Frontend.inversion = fe_spectral_inversion_t(channel.Inversion());
         Frontend.u.qam.symbol_rate = channel.Srate() * 1000UL;
         Frontend.u.qam.fec_inner = fe_code_rate_t(channel.CoderateH());
         Frontend.u.qam.modulation = fe_modulation_t(channel.Modulation());

         tuneTimeout = DVBC_TUNE_TIMEOUT;
         lockTimeout = DVBC_LOCK_TIMEOUT;
         }
         break;
    case FE_OFDM: { // DVB-T

         // Frequency and OFDM paramaters:

         Frontend.frequency = FrequencyToHz(channel.Frequency());
         Frontend.inversion = fe_spectral_inversion_t(channel.Inversion());
         Frontend.u.ofdm.bandwidth = fe_bandwidth_t(channel.Bandwidth());
         Frontend.u.ofdm.code_rate_HP = fe_code_rate_t(channel.CoderateH());
         Frontend.u.ofdm.code_rate_LP = fe_code_rate_t(channel.CoderateL());
         Frontend.u.ofdm.constellation = fe_modulation_t(channel.Modulation());
         Frontend.u.ofdm.transmission_mode = fe_transmit_mode_t(channel.Transmission());
         Frontend.u.ofdm.guard_interval = fe_guard_interval_t(channel.Guard());
         Frontend.u.ofdm.hierarchy_information = fe_hierarchy_t(channel.Hierarchy());

         tuneTimeout = DVBT_TUNE_TIMEOUT;
         lockTimeout = DVBT_LOCK_TIMEOUT;
         }
         break;
    default:
         esyslog("ERROR: attempt to set channel with unknown DVB frontend type");
         return false;
    }
  if (ioctl(fd_frontend, FE_SET_FRONTEND, &Frontend) < 0) {
//M7x0 BEGIN AK
     char __errorstr[256];
     strerror_r(errno,__errorstr,256); 
     __errorstr[255]=0;
     esyslog("ERROR: frontend %d: %s", cardIndex,__errorstr);
//M7x0 END AK
     return false;
     }
  return true;
}

void cDvbTuner::Action(void)
{
  cTimeMs Timer;
  bool LostLock = false;
  fe_status_t Status = (fe_status_t)0;
  while (Running()) {
        fe_status_t NewStatus;
        if (GetFrontendStatus(NewStatus, 10))
           Status = NewStatus;
        cMutexLock MutexLock(&mutex);
        switch (tunerStatus) {
          case tsIdle:
               break;
          case tsSet:
               tunerStatus = SetFrontend() ? tsTuned : tsIdle;
               Timer.Set(tuneTimeout);
               continue;
          case tsTuned:
               if (Timer.TimedOut()) {
                  tunerStatus = tsSet;
                  diseqcCommands = NULL;
                  if (time(NULL) - lastTimeoutReport > 60) { // let's not get too many of these
                     isyslog("frontend %d timed out while tuning to channel %d, tp %d", cardIndex, channel.Number(), channel.Transponder());
                     lastTimeoutReport = time(NULL);
                     }
                  continue;
                  }
          case tsLocked:
               if (Status & FE_REINIT) {
                  tunerStatus = tsSet;
                  diseqcCommands = NULL;
                  isyslog("frontend %d was reinitialized", cardIndex);
                  lastTimeoutReport = 0;
                  continue;
                  }
               else if (Status & FE_HAS_LOCK) {
                  if (LostLock) {
                     isyslog("frontend %d regained lock on channel %d, tp %d", cardIndex, channel.Number(), channel.Transponder());
                     LostLock = false;
                     }
                  tunerStatus = tsLocked;
                  locked.Broadcast();
                  lastTimeoutReport = 0;
                  }
               else if (tunerStatus == tsLocked) {
                  LostLock = true;
                  isyslog("frontend %d lost lock on channel %d, tp %d", cardIndex, channel.Number(), channel.Transponder());
                  tunerStatus = tsTuned;
                  Timer.Set(lockTimeout);
                  lastTimeoutReport = 0;
                  continue;
                  }
          }

        if (ciHandler)
           ciHandler->Process();
        if (tunerStatus != tsTuned)
           newSet.TimedWait(mutex, 1000);
        }
}
//M7X0 BEGIN AK
//
#define REPLAY_VPID 0x101
#define REPLAY_APID 0x102
#define INITIAL_TS_BUFFER_SIZE 2256
#define INITIAL_TRICK_SPEED_BUFFER_SIZE 524288
class c7x0Replayer {
private:
  cMutex playerMutex;
  cDvbDevice *dvbDevice;
  int fd_video;
  int fd_audio;
  
  int cardIndex;
  int fd_dvr;
  int fd_dmx_video;
  int fd_dmx_audio;
  
  int audioChannelId;
  
  // TS Buffers
  // For Normal Play: holds TS-Packets build of one PES-Packet
  uchar *tsBuffer;
  int tsBufferSize;
  int tsBufferLength;
  int tsBufferPayloadFree;

  // Trick Speed Buffer:
  uchar *trickSpeedBuffer;
  int trickSpeedBufferSize;
  int trickSpeedBufferLength;

  int trickSpeedBufferWriteLength;
  int trickSpeedWriteTimes;

  int videoPesHeaderLength;
  int videoStartcode;
  int mpeg;
  bool trickSpeedSynced;
  bool mpeg1HeaderDone;

  int vccount;
  int account;
  bool inFastplay;
  bool inSlowmotion;

  int pesLength;
  int pesExpectedLength;
  int pesStartcode;
  int skipPesBytes;
  int pesSyncSkiped;

  bool pre_1_3_19_PrivStr;
  uchar privateStreamPesHeader[255 + 3 + 1];
  int privateStreamPesHeaderLength;
  int privateStreamId;

//M7X0TODO
/*
This maybe need for Calling Audios.PlayAudio().
But m7x0 can't play external anyway so leave it
		uchar* pesAudioBuffer;
		int pesAudioBufferLength;
	   int pesAudioBufferSize;
*/

  bool ReallocTsBuffer (const int Size); 
  bool ReallocTrickSpeedBuffer (int Size); 

  void ResetAll();
  void Reset();

  bool SearchForStartcode(const uchar *&Data, int &Length);
  bool StartTsPacket(const int pid, int &ccounter);
  bool CopyPes2Ts(const uchar *&Data, int &Length, const int pid, int &ccounter);
  bool CheckPrivateStreamId(const uchar *&Data, int &Length);
  
  bool CopyVideo2TrickSpeed(const uchar *&Data, int &Length);
  void CheckAudioChannel();
  void DiscardLastInCache();

  int WriteData();

  void openDvr();
  void closeDvr();

  
//  FILE *test_writer;

public: 
  c7x0Replayer(cDvbDevice *dev, int fdVid,int fdAud, int cardI);
  ~c7x0Replayer();
  int PlayPes(const uchar *Data, int Length, const bool VideoOnly);
  void TrickSpeed(const int Speed,const bool UseFastForward);
  void Clear();
  void Play();
  void Freeze();
  bool Poll(cPoller &Poller,const int TimeoutMs);
  bool Flush(const int TimeoutMs);
  };

c7x0Replayer::c7x0Replayer(cDvbDevice *dev, int fdVid, int fdAud, int cardI)
{
  dvbDevice = dev;
  fd_video = fdVid;
  fd_audio = fdAud;
  cardIndex = cardI;
  ResetAll();

//  test_writer = fopen("/pc2/tests/video_es_test.m2v","w");

  tsBuffer = MALLOC(uchar, INITIAL_TS_BUFFER_SIZE);
  tsBufferSize = INITIAL_TS_BUFFER_SIZE;
  tsBufferLength = 0;
  tsBufferPayloadFree = 0;

  if (!tsBuffer) {
     tsBufferSize = 0;
     }

  trickSpeedBuffer = MALLOC(uchar, INITIAL_TRICK_SPEED_BUFFER_SIZE);
  trickSpeedBufferSize = INITIAL_TRICK_SPEED_BUFFER_SIZE;
  trickSpeedBufferLength = 0;

  if (!trickSpeedBuffer) {
     trickSpeedBufferSize = 0;
     }

  trickSpeedWriteTimes = 1;

  vccount = 0;
  account = 0;
  inFastplay = false;
  inSlowmotion = false;

  pre_1_3_19_PrivStr = false;
  fd_dvr = -1;
  audioChannelId=-1;
  openDvr();
}

c7x0Replayer::~c7x0Replayer()
{
 
//  fclose(test_writer);
  closeDvr();
  free(tsBuffer);
  free(trickSpeedBuffer);
}

void c7x0Replayer::openDvr(){
  if (fd_dvr >= 0)
     return;

  fd_dvr = DvbOpen(DEV_DVB_ADAPTER DEV_DVB_DVR, cardIndex, O_WRONLY, true);

  if (fd_dvr <0 )
     return;
  
  dmx_pes_filter_params pesFilterParams;
  memset(&pesFilterParams, 0, sizeof(pesFilterParams));
  fd_dmx_video = DvbOpen(DEV_DVB_ADAPTER DEV_DVB_DEMUX, cardIndex, O_RDWR | O_NONBLOCK, true); 

  if (fd_dmx_video < 0) {
     close(fd_dvr);
     fd_dvr = -1;
     return;
     }

  pesFilterParams.pid      = REPLAY_VPID;
  pesFilterParams.input    = DMX_IN_DVR;
  pesFilterParams.output   = DMX_OUT_DECODER;
  pesFilterParams.pes_type = DMX_PES_VIDEO;
  pesFilterParams.flags    = DMX_IMMEDIATE_START;

  int r, i = 0, errnoSave;

  // Is this loop really nessesary any more.
  // In earllier Versions the driver returns with EBUSY sometimes
  do {
     if ((r = ioctl(fd_dmx_video, DMX_SET_PES_FILTER, &pesFilterParams)) < 0) {
        errnoSave = errno;
        CHECK(r);
        cCondWait::SleepMs(3);
        } 
     else
        errnoSave = 0;
     i++;
     } while (errnoSave == EBUSY && i <= 100);

  if (errnoSave != 0) {
     close(fd_dmx_video);
     close(fd_dvr);
     fd_dvr = -1;
     return; 
     }
     
  memset(&pesFilterParams, 0, sizeof(pesFilterParams));
  fd_dmx_audio = DvbOpen(DEV_DVB_ADAPTER DEV_DVB_DEMUX, cardIndex, O_RDWR | O_NONBLOCK, true); 

  if (fd_dmx_audio < 0) {
     close(fd_dmx_video);
     close(fd_dvr);
     fd_dvr = -1;
     return;
     }
     
  // Uglly the driver needs setting exacty this Value
  // Yet another BUG in m7x0-drivers
  if ((r = ioctl(fd_dmx_audio, DMX_SET_BUFFER_SIZE,0x1e000)) < 0) { 
     CHECK(r);
     close(fd_dmx_audio);
     close(fd_dmx_video);
     close(fd_dvr);
     fd_dvr = -1;
     return; 
     }

  pesFilterParams.pid      = REPLAY_APID;
  pesFilterParams.input    = DMX_IN_DVR;
  pesFilterParams.output   = DMX_OUT_DECODER;
  pesFilterParams.pes_type = DMX_PES_AUDIO;
  pesFilterParams.flags    = DMX_IMMEDIATE_START;

  // Is this loop really nessesary any more.
  // In earllier Versions the driver returns with EBUSY sometimes
  do {
     if ((r = ioctl(fd_dmx_audio, DMX_SET_PES_FILTER, &pesFilterParams)) < 0) {
        errnoSave = errno;
        CHECK(r);
        cCondWait::SleepMs(3);
        } 
     else
        errnoSave = 0;
     i++;
     } while (errnoSave == EBUSY && i <= 100);

  if (errnoSave != 0) {
     close(fd_dmx_audio);
     close(fd_dmx_video);
     close(fd_dvr);
     fd_dvr = -1;
     }
}

void c7x0Replayer::closeDvr()
{
  if (fd_dvr < 0)
     return;
     
  CHECK(ioctl(fd_dmx_audio, DMX_STOP,1));
  close(fd_dmx_audio);
  CHECK(ioctl(fd_dmx_video, DMX_STOP,1));
  close(fd_dmx_video);
  close(fd_dvr);
  fd_dvr = -1;
}

bool c7x0Replayer::ReallocTsBuffer(const int Size)
{
  if (Size <= tsBufferSize) 
     return true;

  uchar *tmp = (uchar *) realloc(tsBuffer,Size);
  if (!tmp) {
     esyslog("m7x0 Replayer cannot alloc memory!");
     return false;
     }

  tsBufferSize=Size;
  tsBuffer=tmp;
  return true;
}

bool c7x0Replayer::ReallocTrickSpeedBuffer(const int Size)
{
  if (Size <= trickSpeedBufferSize) 
     return true;

  uchar *tmp = (uchar *) realloc(trickSpeedBuffer,Size);
  if (!tmp) {
     esyslog("m7x0 Replayer cannot alloc memory!");
     return false;
     }

  trickSpeedBufferSize=Size;
  trickSpeedBuffer=tmp;
  return true;
}

void c7x0Replayer::ResetAll()
{
  Reset();
  videoStartcode = 0xFFFFFFFF; 
  trickSpeedBufferLength = 0;
  trickSpeedSynced = false;
  trickSpeedBufferWriteLength = 0;
  skipPesBytes = 0;
  pesSyncSkiped = 0;
}

void c7x0Replayer::Reset()
{
  tsBufferLength = 0;
  tsBufferPayloadFree = 0;
  pesLength = 0;
  pesExpectedLength = 0;
  pesStartcode = 0xffffffff;
  privateStreamPesHeaderLength = 0;
  privateStreamId = -1;

}

bool c7x0Replayer::SearchForStartcode(const uchar *&Data, int &Length)
{
  if (pesLength >= 6)
     return true;
     
  
  for (; Length > 0 && ((pesStartcode&0xffffff00) != 0x100 || 
                        (pesStartcode&0xff) < 0xbb || 
                        (pesStartcode&0xff) > 0xef);
         Length--, Data++, pesSyncSkiped++) 
      pesStartcode = (pesStartcode<<8) | (*Data);

  if ((pesStartcode&0xffffff00) != 0x100 || 
      (pesStartcode&0xff) < 0xbb || (pesStartcode&0xff) > 0xef) {
     pesLength=0;
     return false;
     }
     
  if (pesSyncSkiped>4)
        esyslog("m7x0 Replayer: skiped %d Bytes to sync on PES-Packet",pesSyncSkiped-4);
        
  pesSyncSkiped = 0;
  if (pesLength==0)
     pesLength=4;

  for (; Length > 0 && pesLength < 6; pesLength++, Data++, Length--)
      pesExpectedLength= (pesExpectedLength<<8) | (*Data);

  return pesLength >= 6;
}

bool c7x0Replayer::StartTsPacket(const int pid, int &ccounter)
{
  if (tsBufferPayloadFree) // Payload free in Packet. 
     return true;

  if (!ReallocTsBuffer(tsBufferLength + 188)) 
     return false;
     
  
  *( (uint32_t*) (tsBuffer + tsBufferLength) ) = 0x47000010 | ((pid&0x1fff)<<8) | (ccounter&0xf);
  ccounter=(ccounter+1)&0xf;


  
  if (pesLength > 6) // PES Packet starts? 
     tsBufferPayloadFree = min(184, pesExpectedLength + 6 - pesLength);
  else {
     tsBufferPayloadFree = min(184, pesExpectedLength + 6);
     tsBuffer[tsBufferLength + 1] |= 0x40;
     }
     
  tsBufferLength += 4; // Skip TS-Packet Header
  
  if (tsBufferPayloadFree < 184) { // Does PES-Packet fill the whole TS-Pack?
     // Set adaption field present for padding
     tsBuffer[tsBufferLength-1] |= 0x20; 

     // Length of adaption field (0 is allowed for 1 Byte padding)
     tsBuffer[tsBufferLength] = 183 - tsBufferPayloadFree;  
  
     // No info in adaption field, cause we are padding only. 
     // Gets overwritten in case of 1 Byte padding.
     tsBuffer[tsBufferLength+1] = 0;
                                  
     // Do we need to pad more than 2 Bytes? In this case we have to fill 
     // the rest with 0xff
     if (tsBufferPayloadFree < 182)  
        memset(tsBuffer + tsBufferLength + 2, 0xff, 182 - tsBufferPayloadFree);

     tsBufferLength += 184 - tsBufferPayloadFree; // Skip Padding 
     }
  
  if (pesLength == 6) {
     *( (uint32_t*) (tsBuffer + tsBufferLength) ) = pesStartcode;
     *( (uint16_t*) (tsBuffer + tsBufferLength + 4) ) =  (uint16_t) pesExpectedLength;
        
     tsBufferLength += 6;
     tsBufferPayloadFree -= 6;
     }
     
  return true;
}

bool c7x0Replayer::CopyPes2Ts (const uchar *&Data, int &Length, const int pid, int &ccounter)
{
  int copyLength;
  
  while (Length > 0 && pesLength < pesExpectedLength + 6) {
        if (!StartTsPacket(pid, ccounter))
           return false;
        
        copyLength = min(tsBufferPayloadFree, Length);
        memcpy(tsBuffer + tsBufferLength, Data, copyLength);
        
        tsBufferLength += copyLength;
        tsBufferPayloadFree -= copyLength;
        pesLength += copyLength;
        Data += copyLength;
        Length -= copyLength;
        }
        
  return true;        
}

bool c7x0Replayer::CheckPrivateStreamId(const uchar *&Data, int &Length)
{
  if (pre_1_3_19_PrivStr){
     privateStreamPesHeaderLength = 0;
     privateStreamId = 0xbd;
     return true;
     }
     
  if (privateStreamId != -1)   
     return true;
  
  int headerBytesNeed;
  if (privateStreamPesHeaderLength < 3) {
     if (Length >= 3 - privateStreamPesHeaderLength) 
        headerBytesNeed = 3 - privateStreamPesHeaderLength +
                        Data[2-privateStreamPesHeaderLength] + 1;
     else
        headerBytesNeed = 3;
     }
  else {
     headerBytesNeed = 3 + privateStreamPesHeader[2] + 1;
     }
     
  int copyBytes = min(headerBytesNeed - privateStreamPesHeaderLength , Length);   
  
  memcpy(privateStreamPesHeader + privateStreamPesHeaderLength, Data, copyBytes);
  Data += copyBytes;
  Length -= copyBytes;
  privateStreamPesHeaderLength += copyBytes;
  
  if (privateStreamPesHeaderLength < headerBytesNeed) 
     return false;
  
  pesExpectedLength--;
  privateStreamPesHeaderLength--;
  privateStreamId = privateStreamPesHeader[privateStreamPesHeaderLength];
  
  switch (privateStreamId&0xF0) {
    case 0x20:
    case 0x30:
    case 0xA0:
         return true;
    case 0x80:
         dvbDevice->SetAvailableTrack(ttDolby, privateStreamId - 0x80, privateStreamId);
         return true;
    default:
         dsyslog("switching to pre 1.3.19 Dolby Digital compatibility mode");
         dvbDevice->ClrAvailableTracks();
         dvbDevice->SetAvailableTrack(ttDolby, 0, 0xbd);
         pesExpectedLength++;
         privateStreamPesHeaderLength++;
         pre_1_3_19_PrivStr = true;
         return true;
    }
  return true;
}


//M7X0TODO: Find out on which reason Slowmotion sometimes
// leads to broken frames and short pieces get replayed.
// Writting  to File leads to nothing: no broken frames in mplayer
// The errors occurs as bad if written direktly from a elementary stream
// to device. Setting broken link in GOP-Header leads to nothing.
// Maybe just another bug in driver or Stream has minimal errors,
// but mplayer should have shown this.
bool c7x0Replayer::CopyVideo2TrickSpeed(const uchar *&Data, int &Length)
{
  if (pesLength == 6) {
     if ((Data[0]&0xC0) == 0x80) {
        mpeg = 2;
        videoPesHeaderLength = 9;
        }
     else {
        mpeg = 1; 
        mpeg1HeaderDone=false;
        videoPesHeaderLength = 7;
        }
     }
     
  int skipBytes;
  // Skip Header Bytes;
  if (mpeg == 2) {
     if (pesLength < 9 && Length >= 9 - pesLength)
        videoPesHeaderLength += Data[8-pesLength];
        
     skipBytes = min(videoPesHeaderLength-pesLength, Length);
     Data += skipBytes;
     pesLength += skipBytes;
     Length -= skipBytes;
     }    
  else {
     while (Length > 0 && pesLength < videoPesHeaderLength  ) {
                     
           if (!mpeg1HeaderDone && pesLength + 1 == videoPesHeaderLength ) {
              if ((Data[0]&0xFF) == 0xFF) // Stuffing Bytes
                 videoPesHeaderLength++;
              else if ((Data[0]&0xC0) == 0x40) // STD buffer scale
                 videoPesHeaderLength += 2;
              else {
                 mpeg1HeaderDone = true;
                 if ((Data[0]&0xF0) == 0x20)  // PTS present
                    videoPesHeaderLength += 5;
                 else if ((Data[0]&0xF0) == 0x30) // PTS and DTS present
                    videoPesHeaderLength += 10;
                 }
              }
              
           skipBytes = min(videoPesHeaderLength - pesLength - 
                              (!mpeg1HeaderDone?1:0), Length);       
           Data += skipBytes;
           pesLength += skipBytes;
           Length -= skipBytes;
           }
     }
        
  if (Length == 0)
     return true;
     
  int copyBytes=0;
  
  if (trickSpeedSynced) {  
     copyBytes = min(pesExpectedLength + 6 - pesLength, Length);
     if (!ReallocTrickSpeedBuffer(trickSpeedBufferLength + copyBytes))
        return false;
        
     memcpy(trickSpeedBuffer + trickSpeedBufferLength, Data, copyBytes);
     trickSpeedBufferLength += copyBytes;
     
     }
  
  // Search for Sequence Header 
  while (Length > 0 && pesLength < pesExpectedLength + 6 && 
           videoStartcode != 0x1b3) {
        videoStartcode = (videoStartcode << 8) | Data[0];   
        pesLength++;
        Data++;
        Length--;
        }
        
  if (videoStartcode != 0x1b3)
     return true; 
     
  copyBytes = min(pesExpectedLength + 6 - pesLength, Length);
    
  if (trickSpeedSynced) {
     trickSpeedBufferWriteLength = trickSpeedBufferLength - copyBytes - 4;
     Data += copyBytes;
     pesLength += copyBytes;
     Length -= copyBytes;
     videoStartcode = 0xFFFFFFFF;
     return true;
     }
     
  trickSpeedSynced = true;
  
  if (!ReallocTrickSpeedBuffer(copyBytes + 4))
     return false;
     
  *((uint32_t *)trickSpeedBuffer) = videoStartcode;
  videoStartcode = 0xFFFFFFFF;
  memcpy(trickSpeedBuffer + 4, Data, copyBytes);
  
  trickSpeedBufferLength = copyBytes + 4;
  Data += copyBytes;
  pesLength += copyBytes;
  Length -= copyBytes;
  return true;
}


int c7x0Replayer::WriteData()
{
  
  if (!inFastplay && !inSlowmotion) {
     if (pesLength < pesExpectedLength + 6)
        return 0;
     int r = safe_write(fd_dvr, tsBuffer, tsBufferLength);
     Reset();
     return r;
     }

  if (pesLength == pesExpectedLength + 6)
     Reset();
  
  if (!trickSpeedBufferWriteLength)
     return 0;   
  
  int i;
  int r = 0;  
  for (i = 0; i < trickSpeedWriteTimes; i++)
      if ((r = safe_write(fd_video, trickSpeedBuffer, trickSpeedBufferWriteLength)) < 0)
         return r;
         
  trickSpeedBufferLength -= trickSpeedBufferWriteLength;
  memcpy(trickSpeedBuffer, trickSpeedBuffer + trickSpeedBufferWriteLength, trickSpeedBufferLength);

  trickSpeedBufferWriteLength=0;
  return r;
}

void c7x0Replayer::DiscardLastInCache(){
  if (inSlowmotion) {
     trickSpeedBufferLength = 0;
     if (!trickSpeedSynced)
        videoStartcode = 0xFFFFFFFF;
     }
     
  if (inFastplay && pesLength > videoPesHeaderLength && pesLength < pesExpectedLength +6) {
     int dis = min(pesLength - videoPesHeaderLength, trickSpeedBufferLength);
     trickSpeedBufferLength-=dis;
     if (trickSpeedBufferLength>=4)
        videoStartcode = *((uint32_t *) (trickSpeedBuffer + trickSpeedBufferLength - 4));
     else
        videoStartcode = 0xFFFFFFFF;
     }   
}

void c7x0Replayer::CheckAudioChannel()
{
  if (dvbDevice->GetCurrentAudioTrack() != ttNone) {
     int curAudioId=dvbDevice->GetTrack(dvbDevice->GetCurrentAudioTrack())->id;
           
     if (audioChannelId != -1 && curAudioId != audioChannelId) {
        CHECK(ioctl(fd_audio,AUDIO_STOP));
        CHECK(ioctl(fd_audio,AUDIO_PLAY));
        }
              
     audioChannelId = curAudioId;   
     }
}

int c7x0Replayer::PlayPes(const uchar *Data, int Length, const bool VideoOnly)
{
  cMutexLock MutexLock(&playerMutex);

  if (!Data) {
     dsyslog("m7x0 Replayer: Discarding last PES-Packet");
     DiscardLastInCache();
     skipPesBytes = 0;
     Reset();
     return 0;
     }

  int length = Length;
  int i;
  
  while (Length > 0) {
        if (skipPesBytes) {
           i = min(skipPesBytes, Length);
           Data += i;
           Length -= i;
           skipPesBytes -= i;
           continue;
           }

        if ((i = WriteData()) < 0)
           return i;
           
        if (!SearchForStartcode(Data,Length) || Length <= 0 ) 
           return length;

        switch (pesStartcode&0xff) {
          case 0xbd:
               if (!Setup.UseDolbyDigital) {
                  skipPesBytes = pesExpectedLength + 6 -pesLength;
                  Reset();
                  break;
                  }
                  
               if (!CheckPrivateStreamId(Data,Length))
                  return length;
                  
               CheckAudioChannel();
               
               if (VideoOnly || inFastplay || inSlowmotion ||                   
                                              audioChannelId != privateStreamId) { 
                  
                  skipPesBytes = pesExpectedLength + 6 - pesLength - 
                                 privateStreamPesHeaderLength;
                  Reset();
                  break;
                  }
                  
               if (privateStreamPesHeaderLength){
                  const uchar *tmp=privateStreamPesHeader;
                  if (!CopyPes2Ts(tmp, privateStreamPesHeaderLength, REPLAY_APID, account))
                     return -1;
                  }
                  
               if (!CopyPes2Ts(Data, Length, REPLAY_APID, account))
                  return -1;   
               break;
          case 0xc0 ... 0xdf:
               dvbDevice->SetAvailableTrack(ttAudio, 
                 (pesStartcode&0xff) - 0xC0, pesStartcode&0xff);
                 
               CheckAudioChannel();
               
               if (VideoOnly || inFastplay || inSlowmotion || 
                    audioChannelId != (pesStartcode&0xff)) {
                  skipPesBytes = pesExpectedLength + 6 - pesLength;
                  Reset();
                  break;
                  }
                  
               if (!CopyPes2Ts(Data, Length, REPLAY_APID, account))
                  return -1;

               break;
          case 0xbe: // For MPEG-1 lets see if this work
               if (inFastplay || inSlowmotion) {
                  skipPesBytes = pesExpectedLength + 6 - pesLength;
                  Reset();
                  break;
                  }
          case 0xe0 ... 0xef:
               if (!inFastplay && !inSlowmotion) {
                  if (!CopyPes2Ts(Data, Length, REPLAY_VPID, vccount))
                     return -1;
                  break;
                  }

               if (!CopyVideo2TrickSpeed(Data,Length))
                  return -1;
               break;
          default:
               skipPesBytes = pesExpectedLength + 6 - pesLength;
               Reset();
          }
        }
  return length;
}

void c7x0Replayer::TrickSpeed(const int Speed,const bool UseFastForward)
{
  dsyslog("TrickSpeed called Speed %d %d!",Speed, UseFastForward);
  cMutexLock MutexLock(&playerMutex);
  closeDvr();
  if (!inFastplay && !inSlowmotion || inFastplay != UseFastForward) {
     if (pesLength > 6) {
        skipPesBytes = pesExpectedLength + 6 - pesLength;
        Reset();
        }   
     trickSpeedSynced = false;
     trickSpeedBufferLength = 0;
     videoStartcode = 0xFFFFFFFF;
     trickSpeedBufferWriteLength=0;
     }
   
  if (UseFastForward) {
     trickSpeedWriteTimes = Speed;
     CHECK(ioctl(fd_video, VIDEO_FAST_FORWARD, 1));
     } 
  else {
     trickSpeedWriteTimes=1;
     CHECK(ioctl(fd_video, VIDEO_SLOWMOTION, Speed));
     }
 
  inFastplay=UseFastForward;
  inSlowmotion=!inFastplay;
}

void c7x0Replayer::Play(void)
{
  dsyslog("Play called!");
  cMutexLock MutexLock(&playerMutex);

  if (inSlowmotion || inFastplay){
     if (pesLength >= 6) {
        skipPesBytes = pesExpectedLength + 6 - pesLength;
        Reset();
        }
     inSlowmotion = false;
     inFastplay = false;
  }

  openDvr();
}

void c7x0Replayer::Clear(void)
{
  dsyslog("Clear called!");
  cMutexLock MutexLock(&playerMutex);
  ResetAll();
}

void c7x0Replayer::Freeze(void)
{
  dsyslog("Freeze called!");
  cMutexLock MutexLock(&playerMutex);
  closeDvr();
}

bool c7x0Replayer::Poll(cPoller &Poller,const int TimeoutMs)
{
  cMutexLock MutexLock(&playerMutex);
  return true;
}

bool c7x0Replayer::Flush(const int TimeoutMs)
{
  dsyslog("Flush called!");
  cMutexLock MutexLock(&playerMutex);

  if (WriteData()<0)
     return false;
  return true;
}

// --- c7x0TSBuffer ----------------------------------------------------------
// This class is need cause m7x0 does not support poll()/select() calls on
// dvr-devices. The calls just returns immediatelly with POLLIN set.
// This is a ugly workaround, but I can't do anything about this.
// If someone who has access to the driver-source read this:
// FIX THIS HORRIBLE BROKEN DRIVERS !

#define DVR_RING_BUFFER_SIZE (1536*1024)
#define DVR_READ_TIMEOUT 400 // ms
#define DVR_READ_RETRY 200 // ms

class c7x0TSBuffer {
private:
  int maxFill;
  int cardIndex;
  int f;
  uchar *dvrRingBuffer;
  uchar ringBufferLeft[TS_SIZE];
  int ringBufferLeftLength;
  dvr_ring_buffer_state bufferState;
  void updateStats() { maxFill = max (maxFill, bufferState.fill); }
public:
  c7x0TSBuffer(int File, int CardIndex);
  ~c7x0TSBuffer();
  uchar *Get(int &Length, int &Pid);
  };


c7x0TSBuffer::c7x0TSBuffer(int File, int CardIndex)
{
  maxFill=0;
  f = File;
  cardIndex = CardIndex;

  ringBufferLeftLength=0;
  memset(&bufferState,0,sizeof(struct dvr_ring_buffer_state));
  
  if ((dvrRingBuffer = (uchar *) mmap(NULL, DVR_RING_BUFFER_SIZE, PROT_READ, MAP_SHARED, f, 0)) == MAP_FAILED) {
     LOG_ERROR;
     }
   
}

c7x0TSBuffer::~c7x0TSBuffer()
{
  int lostbytes=0;
  CHECK(ioctl(f,DVR_GET_BYTESLOST,&lostbytes));
  munmap(dvrRingBuffer,DVR_RING_BUFFER_SIZE);
  isyslog("M7X0 TS-Buffer on device %d has lost %d Bytes during Recording. Buffer Stats %d Bytes (%d%%)", cardIndex, lostbytes,maxFill,(maxFill*100)/DVR_RING_BUFFER_SIZE);
  
}

uchar *c7x0TSBuffer::Get(int &Length,int &Pid)
{
    
  if (!bufferState.readable) {
     int TimeoutMs=0; 
     do {
        if ( ioctl(f, DVR_GET_RING_BUFFER_STATE, &bufferState) < 0 ) {
           cCondWait::SleepMs(DVR_READ_RETRY);
           TimeoutMs += DVR_READ_RETRY;
           }
        else
           updateStats();
        } while ( !bufferState.readable && TimeoutMs < DVR_READ_TIMEOUT );
 
     if ( TimeoutMs >= DVR_READ_TIMEOUT ) 
        return NULL;
     }
     
  if (ringBufferLeftLength) {
     int copy = min(TS_SIZE - ringBufferLeftLength, bufferState.readable);
     memcpy(ringBufferLeft+ringBufferLeftLength, dvrRingBuffer + bufferState.offset,copy);
     
     ringBufferLeftLength += copy;
     bufferState.offset += copy;
     bufferState.readable -= copy;
     
     if (ringBufferLeftLength == TS_SIZE) {
        ringBufferLeftLength=0;
        Length = TS_SIZE;
        Pid = (( *( (int *) ringBufferLeft)) >> 8) & 0x1fff;
        return ringBufferLeft;
        }
     
     return NULL;
     }
     
  if (dvrRingBuffer[bufferState.offset] != TS_SYNC_BYTE){
     int i;
     for (i = 1; bufferState.readable && 
                 dvrRingBuffer[bufferState.offset] != TS_SYNC_BYTE ; 
                    i++, bufferState.readable--, bufferState.offset++)
         ; 
   
     esyslog("ERROR: skipped %d bytes to sync on TS packet on device %d", i, cardIndex);
     }
              
  if (bufferState.readable < TS_SIZE) {
  
     if (bufferState.readable) {
        memcpy(ringBufferLeft, dvrRingBuffer + bufferState.offset, bufferState.readable);
        ringBufferLeftLength=bufferState.readable;
        bufferState.readable=0;
        }
        
     return NULL;
     }
  
  // Checks for same PID and TS_SYNC_BYTE  
  uchar *p =dvrRingBuffer + bufferState.offset;
  Pid = (*((int *) p ))&0xff1fff00;
   
  for (Length = TS_SIZE ; Length <= bufferState.readable - TS_SIZE &&
        ((*((int*) (p+Length)))&0xff1fff00) == Pid ; Length+=TS_SIZE)
      ;

  bufferState.offset+=Length;
  bufferState.readable-=Length;
  Pid=(Pid>>8)&0x1fff;

  return p;
}


//M7X0 END AK

// --- cDvbDevice ------------------------------------------------------------

int cDvbDevice::devVideoOffset = -1;
//M7X0 BEGIN AK
int cDvbDevice::setTransferModeForDolbyDigital = 3;
//M7X0 END AK
cDvbDevice::cDvbDevice(int n)
{
//M7X0 BEGIN AK
  audioChannel = 0;
  replayer = NULL;
//M7X0 END AK
  dvbTuner = NULL;
  frontendType = fe_type_t(-1); // don't know how else to initialize this - there is no FE_UNKNOWN
  spuDecoder = NULL;
  digitalAudio = false;
  playMode = pmNone;

  // Devices that are present on all card types:

  int fd_frontend = DvbOpen(DEV_DVB_ADAPTER DEV_DVB_FRONTEND, n, O_RDWR | O_NONBLOCK, true);

  // Devices that are only present on cards with decoders:
//M7X0 BEGIN AK
  fd_osd       = DvbOpen(DEV_DVB_OSD, 0, O_RDWR, true);
  fd_video     = DvbOpen(DEV_DVB_ADAPTER DEV_DVB_VIDEO, n, O_RDWR | O_NONBLOCK, true);
  fd_video_v4l = DvbOpen(DEV_VIDEO, n, O_RDWR, true);
  fd_audio     = DvbOpen(DEV_DVB_ADAPTER DEV_DVB_AUDIO, n, O_RDWR | O_NONBLOCK, true);
  // fd_stc      = DvbOpen(DEV_DVB_DEMUX,  n, O_RDWR); 
  fd_stc       = -1; // STC doesn't work so don't open a device
//M7X0 END AK
  // The DVR device (will be opened and closed as needed):

  fd_dvr = -1;

  // The offset of the /dev/video devices:

  if (devVideoOffset < 0) { // the first one checks this
     FILE *f = NULL;
     char buffer[PATH_MAX];
     for (int ofs = 0; ofs < 100; ofs++) {
         snprintf(buffer, sizeof(buffer), "/proc/video/dev/video%d", ofs);
         if ((f = fopen(buffer, "r")) != NULL) {
            if (fgets(buffer, sizeof(buffer), f)) {
               if (strstr(buffer, "DVB Board")) { // found the _first_ DVB card
                  devVideoOffset = ofs;
                  dsyslog("video device offset is %d", devVideoOffset);
                  break;
                  }
               }
            else
               break;
            fclose(f);
            }
         else
            break;
         }
     if (devVideoOffset < 0)
        devVideoOffset = 0;
     if (f)
        fclose(f);
     }
  devVideoIndex = (devVideoOffset >= 0 && HasDecoder()) ? devVideoOffset++ : -1;

  // Video format:

  SetVideoFormat(Setup.VideoFormat);

  // We only check the devices that must be present - the others will be checked before accessing them://XXX

  if (fd_frontend >= 0) {
     dvb_frontend_info feinfo;
     if (ioctl(fd_frontend, FE_GET_INFO, &feinfo) >= 0) {
        frontendType = feinfo.type;
        ciHandler = cCiHandler::CreateCiHandler(*cDvbName(DEV_DVB_ADAPTER DEV_DVB_CA, n));
        dvbTuner = new cDvbTuner(fd_frontend, CardIndex(), frontendType, ciHandler);
        }
     else
        LOG_ERROR;
     }
  else
     esyslog("ERROR: can't open DVB device %d", n);

  StartSectionHandler();
}

cDvbDevice::~cDvbDevice()
{
//M7X0 BEGIN AK
  if (replayer != NULL)
     delete replayer;
//M7X0 END AK
  delete spuDecoder;
  delete dvbTuner;
  // We're not explicitly closing any device files here, since this sometimes
  // caused segfaults. Besides, the program is about to terminate anyway...
}

bool cDvbDevice::Probe(const char *FileName)
{
  if (access(FileName, F_OK) == 0) {
     dsyslog("probing %s", FileName);
     int f = open(FileName, O_RDONLY);
     if (f >= 0) {
        close(f);
        return true;
        }
     else if (errno != ENODEV && errno != EINVAL)
        LOG_ERROR_STR(FileName);
     }
  else if (errno != ENOENT)
     LOG_ERROR_STR(FileName);
  return false;
}

bool cDvbDevice::Initialize(void)
{
  int found = 0;
  int i;
  for (i = 0; i < MAXDVBDEVICES; i++) {
      if (UseDevice(NextCardIndex())) {
         if (Probe(*cDvbName(DEV_DVB_ADAPTER DEV_DVB_FRONTEND, i))) {
            new cDvbDevice(i);
            found++;
            }
         else
            break;
         }
      else
         NextCardIndex(1); // skips this one
      }
  NextCardIndex(MAXDVBDEVICES - i); // skips the rest
  if (found > 0)
     isyslog("found %d video device%s", found, found > 1 ? "s" : "");
  else
     isyslog("no DVB device found");
  return found > 0;
}
//M7X0 BEGIN AK
void cDvbDevice::MakePrimaryDevice(bool On)
{
  if (HasDecoder()&&On&&!cOsdProvider::Available())
     new cDvbOsdProvider(fd_osd);
  
  if (!On) {
     TurnOffLiveMode(true);
     // Urgly workaround. But seems to be need in case of Primary-Switch  
     // Maybe we find a better way. 
     close(fd_video);
     close(fd_audio);
     fd_video    = DvbOpen(DEV_DVB_ADAPTER DEV_DVB_VIDEO,  CardIndex(), O_RDWR | O_NONBLOCK);
     fd_audio    = DvbOpen(DEV_DVB_ADAPTER DEV_DVB_AUDIO,  CardIndex(), O_RDWR | O_NONBLOCK);
     }
}
//M7X0 END AK

bool cDvbDevice::HasDecoder(void) const
{
  return fd_video >= 0 && fd_audio >= 0;
}

bool cDvbDevice::Ready(void)
{
  if (ciHandler) {
     ciHandler->Process();
     return ciHandler->Ready();
     }
  return true;
}

int cDvbDevice::ProvidesCa(const cChannel *Channel) const
{
  int NumCams = 0;
  if (ciHandler) {
     NumCams = ciHandler->NumCams();
     if (Channel->Ca() >= CA_ENCRYPTED_MIN) {
        unsigned short ids[MAXCAIDS + 1];
        for (int i = 0; i <= MAXCAIDS; i++) // '<=' copies the terminating 0!
            ids[i] = Channel->Ca(i);
        if (ciHandler->ProvidesCa(ids))
           return NumCams + 1;
        }
     }
  int result = cDevice::ProvidesCa(Channel);
  if (result > 0)
     result += NumCams;
  return result;
}

cSpuDecoder *cDvbDevice::GetSpuDecoder(void)
{
  if (!spuDecoder && IsPrimaryDevice())
     spuDecoder = new cDvbSpuDecoder();
  return spuDecoder;
}

//M7X0 BEGIN AK
#ifdef WITH_LIBJPEG
uchar *cDvbDevice::GrabImage(int &Size, bool Jpeg, int Quality, int SizeX, int SizeY)
{
  if (devVideoIndex < 0)
     return NULL;
  char buffer[PATH_MAX];
  snprintf(buffer, sizeof(buffer), "%s%d", DEV_VIDEO, devVideoIndex);
  int videoDev = open(buffer, O_RDWR);
  if (videoDev >= 0) {
     uchar *result = NULL;
     struct video_mbuf mbuf;
     if (ioctl(videoDev, VIDIOCGMBUF, &mbuf) == 0) {
        int msize = mbuf.size;
        unsigned char *mem = (unsigned char *)mmap(0, msize, PROT_READ | PROT_WRITE, MAP_SHARED, videoDev, 0);
        if (mem && mem != (unsigned char *)-1) {
           // set up the size and RGB
           struct video_capability vc;
           if (ioctl(videoDev, VIDIOCGCAP, &vc) == 0) {
              struct video_mmap vm;
              vm.frame = 0;
              if ((SizeX > 0) && (SizeX <= vc.maxwidth) &&
                  (SizeY > 0) && (SizeY <= vc.maxheight)) {
                 vm.width = SizeX;
                 vm.height = SizeY;
                 }
              else {
                 vm.width = vc.maxwidth;
                 vm.height = vc.maxheight;
                 }
              vm.format = VIDEO_PALETTE_RGB24;
              if (ioctl(videoDev, VIDIOCMCAPTURE, &vm) == 0 && ioctl(videoDev, VIDIOCSYNC, &vm.frame) == 0) {
                 // make RGB out of BGR:
                 int memsize = vm.width * vm.height;
                 unsigned char *mem1 = mem;
                 for (int i = 0; i < memsize; i++) {
                     unsigned char tmp = mem1[2];
                     mem1[2] = mem1[0];
                     mem1[0] = tmp;
                     mem1 += 3;
                     }

                 if (Quality < 0)
                    Quality = 100;

                 dsyslog("grabbing to %s %d %d %d", Jpeg ? "JPEG" : "PNM", Quality, vm.width, vm.height);
                 if (Jpeg) {
                    // convert to JPEG:
                    result = RgbToJpeg(mem, vm.width, vm.height, Size, Quality);
                    if (!result)
                       esyslog("ERROR: failed to convert image to JPEG");
                    }
                 else {
                    // convert to PNM:
                    char buf[32];
                    snprintf(buf, sizeof(buf), "P6\n%d\n%d\n255\n", vm.width, vm.height);
                    int l = strlen(buf);
                    int bytes = memsize * 3;
                    Size = l + bytes;
                    result = MALLOC(uchar, Size);
                    if (result) {
                       memcpy(result, buf, l);
                       memcpy(result + l, mem, bytes);
                       }
                    else
                       esyslog("ERROR: failed to convert image to PNM");
                    }
                 }
              }
           munmap(mem, msize);
           }
        else
           esyslog("ERROR: failed to memmap video device");
        }
     close(videoDev);
     return result;
     }
  else
     LOG_ERROR_STR(buffer);
  return NULL;
}
#endif

// Taken from gambler 
/*
 * M7X0
 * wow realy wired we have to set the ioctl on /dev/videoX
 * and this takes effect on /dev/ost/videoX
 *
 *
 */
void cDvbDevice::SetVideoDisplayFormat(eVideoDisplayFormat VideoDisplayFormat)
{

   if (HasDecoder()) {
      if (Setup.VideoFormat) {	
         CHECK(ioctl(fd_video_v4l, M7X0_SET_TV_ASPECT_MODE, M7X0_VIDEO_LETTER_BOX));
        //CHECK(ioctl(fd_video_set, M7X0_VIDEO_SET_DISPLAY_FORMAT, M7X0_VIDEO_PAN_SCAN));   
         dsyslog("DEBUG set setup mode: letterbox");
      }
      else {
         switch (VideoDisplayFormat) {
            case vdfPanAndScan:
               CHECK(ioctl(fd_video_v4l, M7X0_SET_TV_ASPECT_MODE, M7X0_VIDEO_PAN_SCAN));               
               dsyslog("DEBUG set mode: pan scan");
               break;
            case vdfLetterBox:
               CHECK(ioctl(fd_video_v4l, M7X0_SET_TV_ASPECT_MODE, M7X0_VIDEO_LETTER_BOX));
               dsyslog("DEBUG set mode: letterbox");
               break;
            case vdfCenterCutOut:
	       //some test stuff
               //CHECK(ioctl(fd_video, M7X0_SET_TV_ASPECT_MODE, VIDEO_CENTER_CUT_OUT));
	       //  struct video_window vid_win = {
               //          50,50,640,480
               //  };
	       //struct video_window *vid_win; 
               //if(ioctl(fd_video_set, VIDIOCSWIN, &vid_win) < 0){
               //        perror("ioctl (VIDIOCSWIN)");
               //      }
               dsyslog("DEBUG set mode: cut out, not working yet");
               break;
         }
      }
   }


#if 0 // Original Code just in case ...
  cDevice::SetVideoDisplayFormat(VideoDisplayFormat);
  if (HasDecoder()) {
     if (Setup.VideoFormat) {
        CHECK(ioctl(fd_video, VIDEO_SET_DISPLAY_FORMAT, VIDEO_LETTER_BOX));
        }
     else {
        switch (VideoDisplayFormat) {
          case vdfPanAndScan:
               CHECK(ioctl(fd_video, VIDEO_SET_DISPLAY_FORMAT, VIDEO_PAN_SCAN));
               break;
          case vdfLetterBox:
               CHECK(ioctl(fd_video, VIDEO_SET_DISPLAY_FORMAT, VIDEO_LETTER_BOX));
               break;
          case vdfCenterCutOut:
               CHECK(ioctl(fd_video, VIDEO_SET_DISPLAY_FORMAT, VIDEO_CENTER_CUT_OUT));
               break;
          }
        }
     }
#endif
}
// Taken from gambler 
void cDvbDevice::SetVideoFormat(bool VideoFormat16_9)
{
/* 
 *
 * NOTE: m7x0 use avswitch for 16/9 4/3
 * but pan scan, letterbox stuff is done 
 * on /dev/videoX
 *
 * thx to anonymous
 *
 */
   int avs = open("/dev/avswitch", O_WRONLY);
   if(avs== -1) {
      esyslog("m7x0 can not open /dev/avswitch");
   }

   if (HasDecoder()) {
      if(VideoFormat16_9){
         dsyslog("DEBUG: set 16/9");
         CHECK(ioctl(fd_video_v4l, M7X0_SET_TV_ASPECT_RATIO, M7X0_VIDEO_FORMAT_16_9));
         CHECK(ioctl(avs, AVSWCMD_MODE_16_9, 0));
      }else{
         dsyslog("DEBUG: set 4/3");
         CHECK(ioctl(fd_video_v4l, M7X0_SET_TV_ASPECT_RATIO, M7X0_VIDEO_FORMAT_4_3));
         CHECK(ioctl(avs, AVSWCMD_MODE_4_3, 0));
      }
      SetVideoDisplayFormat(eVideoDisplayFormat(Setup.VideoDisplayFormat));
   }

   close(avs);
#if 0 // Original code just in case ....
  if (HasDecoder()) {
     CHECK(ioctl(fd_video, VIDEO_SET_FORMAT, VideoFormat16_9 ? VIDEO_FORMAT_16_9 : VIDEO_FORMAT_4_3));
     SetVideoDisplayFormat(eVideoDisplayFormat(Setup.VideoDisplayFormat));
     }
#endif
//M7X0 END AK
}

eVideoSystem cDvbDevice::GetVideoSystem(void)
{
  eVideoSystem VideoSystem = vsPAL;
//M7X0 BEGIN AK
// Nothing in here works yet
//M7X0TODO: fix this
#if 0
  video_size_t vs;
  if (ioctl(fd_video, VIDEO_GET_SIZE, &vs) == 0) {
     if (vs.h == 480 || vs.h == 240)
        VideoSystem = vsNTSC;
     }
  else
     LOG_ERROR;
#endif
//M7X0 END AK
  return VideoSystem;
}

//M7X0 BEGIN AK
// If anybody who has access to the driver source reads this.
// FIX YOUR HORRIBLE BUGGY DRIVERS !!!  AC3 is a only bug
// Do not work really in any case. 
// (not even in wavebox workarounds, which even not work in all cases,
//  are bugs no fixes).
// AC3 Test Cases: (X := do nothing)
// Case | ENABLE_ANALOG | ENABLE_AC3
//------+---------------+------------
//  1   |      X        |    X       only works after boot 
//  2   |    false      |   false    (-)
//  3   |    false      |   true     (-)
//  4   |    true       |   false    (-)
//  5   |    true       |   true     (-)


bool cDvbDevice::SetAudioBypass(bool On)
{
  if (setTransferModeForDolbyDigital != 1 && setTransferModeForDolbyDigital != 3)
    return false;
 // dsyslog("cDvbDevice: SetAudioBypass %d", On);
   
 // CHECK(ioctl(fd_audio,AUDIO_ENABLE_ANALOG,true));
  //CHECK(ioctl(fd_audio,AUDIO_ENABLE_ANALOG,!On));
  //CHECK(ioctl(fd_audio,AUDIO_ENABLE_AC3,On))
#if 0
  if (!setTransferModeForDolbyDigital)
     return false;
  return ioctl(fd_audio, AUDIO_SET_BYPASS_MODE, On) == 0;
#endif
  return true;
//M7XO END AK
}

//                            ptAudio        ptVideo        ptPcr        ptTeletext        ptDolby        ptOther
//M7X0 BEGIN AK
dmx_pes_type_t PesTypes[] = { DMX_PES_AUDIO, DMX_PES_VIDEO, DMX_PES_PCR, DMX_PES_TELETEXT, DMX_PES_AUDIO, DMX_PES_OTHER };
//M7X0TODO: Get Dolby working
bool cDvbDevice::SetPid(cPidHandle *Handle, int Type, bool On)
{
  if (Handle->pid) {
     dmx_pes_filter_params pesFilterParams;
     memset(&pesFilterParams, 0, sizeof(pesFilterParams));
     if (On) {

// For debuging only
        dsyslog("DEBUG: Demux Setting PID: %u Type: %d", Handle->pid, Type);

        if (Handle->handle < 0) {
           Handle->handle = DvbOpen(DEV_DVB_ADAPTER DEV_DVB_DEMUX, CardIndex(), O_RDWR | O_NONBLOCK, true);
           if (Handle->handle < 0) {
              LOG_ERROR;
              return false;
              }
           }

        int r,errnoSave, i = 0;
        if (Type == ptAudio || Type == ptDolby){
           // Uglly the driver needs setting exacty this Value
           // Yet another BUG in m7x0-drivers
           if ((r = ioctl(Handle->handle, DMX_SET_BUFFER_SIZE,0x1e000)) < 0) { 
              CHECK(r);
              close(Handle->handle);
              Handle->handle = -1;
              return false;
              }
           } 

        pesFilterParams.pid     = Handle->pid;
        pesFilterParams.input   = DMX_IN_FRONTEND;
        pesFilterParams.output  = (Type < ptOther) ? DMX_OUT_DECODER : DMX_OUT_TS_TAP;
        pesFilterParams.pes_type= PesTypes[Type < ptOther ? Type : ptOther];
        pesFilterParams.flags   = DMX_IMMEDIATE_START;

        // Is this loop really nessesary any more.
        // In earllier Versions the driver returns with EBUSY sometimes
        do {
           if ((r = ioctl(Handle->handle, DMX_SET_PES_FILTER, &pesFilterParams)) < 0) {
              errnoSave=errno;
              CHECK(r);
              cCondWait::SleepMs(10);
              }
           else
              errnoSave = 0;
           i++;
           } while (errnoSave==EBUSY && i<=100);

        if (errnoSave != 0){
           return false; 
           close(Handle->handle);
           Handle->handle = -1;
           }
        }
     else if (!Handle->used) {
        // For debuging only
        dsyslog("DEBUG: Demux Stopping PID: %u Type: %d", Handle->pid, Type);
        CHECK(ioctl(Handle->handle, DMX_STOP));
        if (PesTypes[Type] == DMX_PES_VIDEO) // let's only do this once
           SetPlayMode(pmNone); // necessary to switch a PID from DMX_PES_VIDEO/AUDIO to DMX_PES_OTHER
        close(Handle->handle);
        Handle->handle = -1;
        }
     }
  return true;
}
//M7X0 END AK

int cDvbDevice::OpenFilter(u_short Pid, u_char Tid, u_char Mask)
{
   const char *FileName = *cDvbName(DEV_DVB_ADAPTER DEV_DVB_DEMUX, CardIndex());
  int f = open(FileName, O_RDWR | O_NONBLOCK);
  if (f >= 0) {
     dmx_sct_filter_params sctFilterParams;
     memset(&sctFilterParams, 0, sizeof(sctFilterParams));
     sctFilterParams.pid = Pid;
     sctFilterParams.timeout = 0;
//M7X0 BEGIN AK
// CRC checking won't work, driver delivers CRC-Invalid sections
// What the heck is this flag for as additional checking is need with or without it.
     sctFilterParams.flags = DMX_IMMEDIATE_START; //| DMX_CHECK_CRC;
//M7X0 END AK
     sctFilterParams.filter.filter[0] = Tid;
     sctFilterParams.filter.mask[0] = Mask;
     if (ioctl(f, DMX_SET_FILTER, &sctFilterParams) >= 0)
        return f;
     else {
//M7x0 BEGIN AK
        char __errorstr[256];
        strerror_r(errno,__errorstr,256); 
        __errorstr[255]=0;
        esyslog("ERROR: can't set filter (pid=%d, tid=%02X, mask=%02X): %s", Pid, Tid, Mask,__errorstr);
//M7x0 END AK
        close(f);
        }
     }
  else
     esyslog("ERROR: can't open filter handle on '%s'", FileName);
  return -1;
}

void cDvbDevice::TurnOffLiveMode(bool LiveView)
{
  if (LiveView) {
     // Avoid noise while switching:
//M7X0 BEGIN AK
     CHECK(ioctl(fd_audio, AUDIO_STOP, 0));
     CHECK(ioctl(fd_video, VIDEO_STOP, 0));

     }

  // Turn off live PIDs:
// Live Pids get Special PID-slots so don't detach any receivers
#if 0 
  DetachAll(pidHandles[ptAudio].pid);
  DetachAll(pidHandles[ptVideo].pid);
  DetachAll(pidHandles[ptPcr].pid);
  DetachAll(pidHandles[ptTeletext].pid);
#endif
  DelPid(pidHandles[ptAudio].pid, ptAudio);
  DelPid(pidHandles[ptVideo].pid, ptVideo);
  DelPid(pidHandles[ptPcr].pid, ptPcr);
  DelPid(pidHandles[ptTeletext].pid, ptTeletext);
  DelPid(pidHandles[ptDolby].pid, ptDolby);
//M7X0 END AK
}

bool cDvbDevice::ProvidesSource(int Source) const
{
  int type = Source & cSource::st_Mask;
  return type == cSource::stNone
      || type == cSource::stCable && frontendType == FE_QAM
      || type == cSource::stSat   && frontendType == FE_QPSK
      || type == cSource::stTerr  && frontendType == FE_OFDM;
}

bool cDvbDevice::ProvidesTransponder(const cChannel *Channel) const
{
  return ProvidesSource(Channel->Source()) && (!cSource::IsSat(Channel->Source()) || !Setup.DiSEqC || Diseqcs.Get(Channel->Source(), Channel->Frequency(), Channel->Polarization()));
}

bool cDvbDevice::ProvidesChannel(const cChannel *Channel, int Priority, bool *NeedsDetachReceivers) const
{
  bool result = false;
  bool hasPriority = Priority < 0 || Priority > this->Priority();
  bool needsDetachReceivers = false;

  if (ProvidesSource(Channel->Source()) && ProvidesCa(Channel)) {
     result = hasPriority;
     if (Priority >= 0 && Receiving(true)) {
        if (dvbTuner->IsTunedTo(Channel)) {
           if (Channel->Vpid() && !HasPid(Channel->Vpid()) || Channel->Apid(0) && !HasPid(Channel->Apid(0))) {
#ifdef DO_MULTIPLE_RECORDINGS
#ifndef DO_MULTIPLE_CA_CHANNELS
              if (Ca() >= CA_ENCRYPTED_MIN || Channel->Ca() >= CA_ENCRYPTED_MIN)
                 needsDetachReceivers = Ca() != Channel->Ca();
              else
#endif
              if (!IsPrimaryDevice())
                 result = true;
#ifdef DO_REC_AND_PLAY_ON_PRIMARY_DEVICE
              else
                 result = Priority >= Setup.PrimaryLimit;
#endif
#endif
              }
           else
              result = !IsPrimaryDevice() || Priority >= Setup.PrimaryLimit;
           }
        else
           needsDetachReceivers = true;
        }
     }
  if (NeedsDetachReceivers)
     *NeedsDetachReceivers = needsDetachReceivers;
  return result;
}

bool cDvbDevice::IsTunedToTransponder(const cChannel *Channel)
{
  return dvbTuner->IsTunedTo(Channel);
}

//M7X0 BEGIN AK
// Wie do not need a 
bool cDvbDevice::SetChannelDevice(const cChannel *Channel, bool LiveView)
{
  bool DoTune = !dvbTuner->IsTunedTo(Channel);

  bool TurnOffLivePIDs = HasDecoder()
                         && (DoTune
                            || !IsPrimaryDevice()
                            || LiveView // for a new live view the old PIDs need to be turned off
                            );

  
  bool TurnOnLivePIDs = HasDecoder() && LiveView;

#ifndef DO_MULTIPLE_RECORDINGS
  TurnOffLivePIDs = TurnOnLivePIDs = true;
  StartTransferMode = false;
#endif

  // Turn off live PIDs if necessary:

  if (TurnOffLivePIDs)
     TurnOffLiveMode(LiveView);

  // Set the tuner:

  dvbTuner->Set(Channel, DoTune);

  // If this channel switch was requested by the EITScanner we don't wait for
  // a lock and don't set any live PIDs (the EITScanner will wait for the lock
  // by itself before setting any filters):

  if (EITScanner.UsesDevice(this)) //XXX
     return true;

  // PID settings:

  //Wait for tuner lock: M740AV needs this befor setting PIDS 
  if (!HasLock(-1))
     esyslog("ERROR: Cannot get Tuner-Lock!");

  
  if (TurnOnLivePIDs) {

     SetAudioBypass(false);
     if (!(AddPid(Channel->Ppid(), ptPcr) && AddPid(Channel->Vpid(), ptVideo) && AddPid(Channel->Apid(0), ptAudio))) {
        esyslog("ERROR: failed to set PIDs for channel %d on device %d", Channel->Number(), CardIndex() + 1);
        return false;
        }

     //XXX quick workaround for additional live audio PIDs:
     if (ciHandler) {
        ciHandler->SetPid(Channel->Apid(1), true);
        ciHandler->SetPid(Channel->Dpid(0), true);
        }
 
     if (IsPrimaryDevice())
        AddPid(Channel->Tpid(), ptTeletext);
     
     CHECK(ioctl(fd_audio, AUDIO_SET_AV_SYNC, Channel->Vpid()>0));
     CHECK(ioctl(fd_audio, AUDIO_PLAY,0));
     CHECK(ioctl(fd_video, VIDEO_PLAY,0));
     }
 
  return true;

}
//M7X0 END AK

bool cDvbDevice::HasLock(int TimeoutMs)
{
  return dvbTuner ? dvbTuner->Locked(TimeoutMs) : false;
}
//M7X0 BEGIN AK
int cDvbDevice::GetAudioChannelDevice(void)
{
  return audioChannel;
}

void cDvbDevice::SetAudioChannelDevice(int AudioChannel)
{
  CHECK(ioctl(fd_audio, AUDIO_STOP));
  audioChannel=AudioChannel;
  CHECK(ioctl(fd_audio,AUDIO_STEERING, AudioChannel));
  CHECK(ioctl(fd_audio, AUDIO_PLAY));
}

void cDvbDevice::SetVolumeDevice(int Volume)
{
  if (HasDecoder()&&!digitalAudio) {
     audio_mixer_t am;
     // conversion for linear volume response:
     am.volume_left = am.volume_right = 2 * Volume - Volume * Volume / 255;
     CHECK(ioctl(fd_audio, AUDIO_SET_MIXER, &am));
     }
}

void cDvbDevice::SetDigitalAudioDevice(bool On)
{
  //dsyslog("cDvbDevice: Set Digital Audio %d", On);
  if (digitalAudio != On) {
        
     digitalAudio = On;
    
     // CHECK(ioctl(fd_audio,AUDIO_ENABLE_AC3,On))
 /*    if (On)
        //Is this right? Does this call select the decoder for AC3?
        CHECK(ioctl(fd_audio,AUDIO_ENABLE_AC3,true))
     else
        // Maybe need for normal Analog mode (?)
        CHECK(ioctl(fd_audio,AUDIO_ENABLE_AC3,false))*/
  /*   if (On)
        //Is this right? Does this call select the decoder for AC3?
        CHECK(ioctl(fd_audio,AUDIO_ENABLE_AC3,CardIndex()))
     else
        // Maybe need for normal Analog mode (?)
        CHECK(ioctl(fd_audio,AUDIO_ENABLE_AC3,0))*/
              
     SetVolumeDevice(IsMute() ? 0 : CurrentVolume());
     }
}

void cDvbDevice::SetTransferModeForDolbyDigital(int Mode)
{
  dsyslog("cDvbDevice: Set Transfermode Digital Audio %d", Mode);
  setTransferModeForDolbyDigital = Mode;
}


void cDvbDevice::SetAudioTrackDevice(eTrackType Type)
{
  const tTrackId *TrackId = GetTrack(Type);
  if (TrackId && TrackId->id) {
     SetAudioBypass(false);
     if (IS_AUDIO_TRACK(Type) || (IS_DOLBY_TRACK(Type) && SetAudioBypass(true))) {
        if (pidHandles[ptAudio].pid && pidHandles[ptAudio].pid != TrackId->id) {
           CHECK(ioctl(fd_audio,AUDIO_STOP));
                      
           pidHandles[ptAudio].pid = TrackId->id;
           SetPid(&pidHandles[ptAudio], ptAudio, true);
           CHECK(ioctl(fd_audio,AUDIO_PLAY));
           
           if (IS_DOLBY_TRACK(Type)){
              int diff; int stat;
              if (pidHandles[ptPcr].handle >= 0) { 
                 stat = ioctl(pidHandles[ptPcr].handle,DMX_GET_AUDIO_SYNC_DIFF,&diff);
                 dsyslog("cDvbDevice DEBUG: AC3 Audio Snyc Diff ioctl-Status %d Difference %d", stat, diff);
                 }
              }
           }
        }
     else if (IS_DOLBY_TRACK(Type)) {
        if (setTransferModeForDolbyDigital == 0 || setTransferModeForDolbyDigital == 3)
           return;
        // Currently this works only in Transfer Mode
        ForceTransferMode();
        }
     }
}
//M7X0 END AK

bool cDvbDevice::CanReplay(void) const
{
#ifndef DO_REC_AND_PLAY_ON_PRIMARY_DEVICE
  if (Receiving())
     return false;
#endif
  return cDevice::CanReplay();
}

//M7X0 BEGIN AK
//M7X0TODO: Get other playmodes working 
bool cDvbDevice::SetPlayMode(ePlayMode PlayMode)
{
  if (PlayMode != pmExtern_THIS_SHOULD_BE_AVOIDED && fd_video < 0 && fd_audio < 0) {
     // reopen the devices
     fd_video = DvbOpen(DEV_DVB_ADAPTER DEV_DVB_VIDEO,  CardIndex(), O_RDWR | O_NONBLOCK);
     fd_audio = DvbOpen(DEV_DVB_ADAPTER DEV_DVB_AUDIO,  CardIndex(), O_RDWR | O_NONBLOCK);
     SetVideoFormat(Setup.VideoFormat);
     }

  switch (PlayMode) {
    case pmNone:
         CHECK(ioctl(fd_audio, AUDIO_STOP, 0));
         CHECK(ioctl(fd_video, VIDEO_STOP, 0));
         if (replayer != NULL) {
            delete replayer;
            replayer=NULL;
         }
         break;
    case pmAudioVideo:

         if (playMode == pmNone)
            TurnOffLiveMode(true);
         CHECK(ioctl(fd_audio, AUDIO_STOP,0));
         CHECK(ioctl(fd_video, VIDEO_STOP, 1));
			if(replayer == NULL){
				replayer = new c7x0Replayer(this,fd_video,fd_audio,CardIndex());
			}
			CHECK(ioctl(fd_audio, AUDIO_SET_AV_SYNC,true));
			CHECK(ioctl(fd_audio, AUDIO_PLAY,0));
			CHECK(ioctl(fd_video, VIDEO_PLAY,0));
         break;
	 case pmAudioOnlyBlack:
    case pmAudioOnly:
         CHECK(ioctl(fd_video, VIDEO_SET_BLANK, true));
         CHECK(ioctl(fd_audio, AUDIO_STOP, true));
         CHECK(ioctl(fd_audio, AUDIO_SET_AV_SYNC, false));
         CHECK(ioctl(fd_audio, AUDIO_PLAY));
         CHECK(ioctl(fd_video, VIDEO_SET_BLANK, false));
         break;
    case pmVideoOnly:
         CHECK(ioctl(fd_video, VIDEO_SET_BLANK, true));
         CHECK(ioctl(fd_video, VIDEO_STOP, true));
         CHECK(ioctl(fd_audio, AUDIO_SET_AV_SYNC, false));
         CHECK(ioctl(fd_audio, AUDIO_PLAY));
         CHECK(ioctl(fd_video, VIDEO_PLAY));
         break;
    case pmExtern_THIS_SHOULD_BE_AVOIDED:
         return false; 
         close(fd_video);
         close(fd_audio);
         fd_video = fd_audio = -1;
         break;
    }
  playMode = PlayMode;
  return true;
}


int64_t cDvbDevice::GetSTC(void)
{
#if 0
  if (fd_stc >= 0) {
     struct dmx_stc stc;
     stc.num = 0;
     if (ioctl(fd_stc, DMX_GET_STC, &stc) == -1) {
        esyslog("ERROR: stc %d: %m", CardIndex() + 1);
        return -1;
        }
     return stc.stc / stc.base;
     }
#endif
  return -1;
}

void cDvbDevice::TrickSpeed(int Speed, bool UseFastForward)
{
  if (replayer != NULL)
     replayer->TrickSpeed(Speed,UseFastForward);
}

void cDvbDevice::Clear(void)
{
  if (replayer != NULL)
     replayer->Clear();
  cDevice::Clear();
}

void cDvbDevice::Play(void)
{
  CHECK(ioctl(fd_video, VIDEO_STOP, 0));
  CHECK(ioctl(fd_audio, AUDIO_STOP, 0));
  CHECK(ioctl(fd_audio, AUDIO_SET_MUTE, false));
  if (replayer != NULL)
     replayer->Play();
  if (playMode == pmAudioOnly || playMode == pmAudioOnlyBlack) {
     if (fd_audio >= 0)
        CHECK(ioctl(fd_audio, AUDIO_CONTINUE));
     }
  else {
     if (fd_audio >= 0) {
        CHECK(ioctl(fd_audio, AUDIO_SET_AV_SYNC, true));
        CHECK(ioctl(fd_audio, AUDIO_PLAY,0)); 
        }
     if (fd_video >= 0)
        CHECK(ioctl(fd_video, VIDEO_PLAY,0));
     }
  
  cDevice::Play();
}

void cDvbDevice::Freeze(void)
{
  if (playMode == pmAudioOnly || playMode == pmAudioOnlyBlack) {
     if (fd_audio >= 0)
        CHECK(ioctl(fd_audio, AUDIO_PAUSE));
     }

  if (replayer != NULL)
     replayer->Freeze();
#ifndef REPLAY_BUFFER_RUNOUT_ON_FREEZE
  CHECK(ioctl(fd_audio, AUDIO_SET_MUTE, true));
  if (fd_audio >= 0){
     CHECK(ioctl(fd_audio, AUDIO_STOP,0));
     }
  if (fd_video >= 0) {
     CHECK(ioctl(fd_video, VIDEO_STOP,0));
     }
#endif
  cDevice::Freeze();
}

void cDvbDevice::Mute(void)
{
  if (fd_audio >= 0) {
     CHECK(ioctl(fd_audio, AUDIO_SET_AV_SYNC, false));
     CHECK(ioctl(fd_audio, AUDIO_SET_MUTE, true));
     }
  cDevice::Mute();
}

void cDvbDevice::StillPicture(const uchar *Data, int Length)
{
  if (Data[0] == 0x00 && Data[1] == 0x00 && Data[2] == 0x01 && (Data[3] & 0xF0) == 0xE0) {
     // PES data
     char *buf = MALLOC(char, Length);
     if (!buf)
        return;
     int i = 0;
     int blen = 0;
     while (i < Length - 6) {
           if (Data[i] == 0x00 && Data[i + 1] == 0x00 && Data[i + 2] == 0x01) {
              int len = Data[i + 4] * 256 + Data[i + 5];
              if ((Data[i + 3] & 0xF0) == 0xE0) { // video packet
                 // skip PES header
                 int offs = i + 6;
                 // skip header extension
                 if ((Data[i + 6] & 0xC0) == 0x80) {
                    // MPEG-2 PES header
                    if (Data[i + 8] >= Length)
                       break;
                    offs += 3;
                    offs += Data[i + 8];
                    len -= 3;
                    len -= Data[i + 8];
                    if (len < 0 || offs + len > Length)
                       break;
                    }
                 else {
                    // MPEG-1 PES header
                    while (offs < Length && len > 0 && Data[offs] == 0xFF) {
                          offs++;
                          len--;
                          }
                    if (offs <= Length - 2 && len >= 2 && (Data[offs] & 0xC0) == 0x40) {
                       offs += 2;
                       len -= 2;
                       }
                    if (offs <= Length - 5 && len >= 5 && (Data[offs] & 0xF0) == 0x20) {
                       offs += 5;
                       len -= 5;
                       }
                    else if (offs <= Length - 10 && len >= 10 && (Data[offs] & 0xF0) == 0x30) {
                       offs += 10;
                       len -= 10;
                       }
                    else if (offs < Length && len > 0) {
                       offs++;
                       len--;
                       }
                    }
                 if (blen + len > Length) // invalid PES length field
                    break;
                 memcpy(&buf[blen], &Data[offs], len);
                 i = offs + len;
                 blen += len;
                 }
              else if (Data[i + 3] >= 0xBD && Data[i + 3] <= 0xDF) // other PES packets
                 i += len + 6;
              else
                 i++;
              }
           else
              i++;
           }
     CHECK(ioctl(fd_video, VIDEO_FAST_FORWARD, 1));
     WriteAllOrNothing(fd_video,(const uchar*) buf,blen, 1000, 10);;
     free(buf);
     }
  else {
     // non-PES data
     CHECK(ioctl(fd_video, VIDEO_FAST_FORWARD, 1));
     WriteAllOrNothing(fd_video, Data,Length, 1000, 10);
     }
}

bool cDvbDevice::Poll(cPoller &Poller, int TimeoutMs)
{
  if (replayer!=NULL)
     return replayer->Poll(Poller, TimeoutMs);
  return false;
}

bool cDvbDevice::Flush(int TimeoutMs)
{
  //TODO actually this function should wait until all buffered data has been processed by the card, but how?
  if (replayer!=NULL)
     return replayer->Flush(TimeoutMs);
  return true;
}

int cDvbDevice::PlayPes(const uchar *Data, int Length, bool VideoOnly)
{
	cMutexLock MutexLock(&mutexCurrentAudioTrack);
	 if (replayer!=NULL)
     return replayer->PlayPes(Data,Length,VideoOnly);
	 
	esyslog("PlayPes called without replayer");
	return -1;
}

int cDvbDevice::PlayVideo(const uchar *Data, int Length)
{
	esyslog("PlayVideo should not be called any more");
 	return -1;
}

int cDvbDevice::PlayAudio(const uchar *Data, int Length, uchar Id)
{
  esyslog("PlayAudio should not be called any more");
 return -1;
}


// In orginal 2 MBs are used as buffer. 
// Due to restrictions of mmaping on the dvr-device
// Ring-Buffer-Size is know fixed to 1536 KBs.
bool cDvbDevice::OpenDvr(void)
{
  CloseDvr();
  fd_dvr = DvbOpen(DEV_DVB_ADAPTER DEV_DVB_DVR, CardIndex(), O_RDONLY | O_NONBLOCK, true);
  if (fd_dvr >= 0)
     tsBuffer = new c7x0TSBuffer(fd_dvr, CardIndex() + 1);
  return fd_dvr >= 0;
}
//M7X0 END AK

void cDvbDevice::CloseDvr(void)
{
  if (fd_dvr >= 0) {
     delete tsBuffer;
     tsBuffer = NULL;
     close(fd_dvr);
     fd_dvr = -1;
     }
}

//M7X0 BEGIN AK
// Seems to perform much better, if more than one ts packet (with the same PID)
// is deliviered in one call
bool cDvbDevice::GetTSPackets(uchar *&Data, int &Length, int &Pid)
{
  if (tsBuffer) {
     Data = tsBuffer->Get(Length, Pid);
     return true;
     }
  return false;
}
//M7X0 END AK
