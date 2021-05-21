/*
**             Copyright 2017 by Kvaser AB, Molndal, Sweden
**                         http://www.kvaser.com
**
** License: BSD-new
** ==============================================================================
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the <organization> nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
** ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
** LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
** CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
** SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
** BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
** IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
** ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
** POSSIBILITY OF SUCH DAMAGE.
** IMPORTANT NOTICE:
**
** ==============================================================================
** This source code is made available for free, as an open license, by Kvaser AB,
** for use with its applications. Kvaser AB does not accept any liability
** whatsoever for any third party patent or other immaterial property rights
** violations that may result from any usage of this source code, regardless of
** the combination of source code and various applications that it can be used
** in, or with.
**
** -----------------------------------------------------------------------------
*/

/*
 * Kvaser Linux Canlib
 * Read CAN FD messages and print out their contents
 */

#if 0
/*
Sending from one host, and receiving on vicanlog device

# ip link set can6 down
# ip link set can6 type can bitrate 500000 listen-only off dbitrate 2000000 fd on restart-ms 100
# ip link set can6 up
# cansequence8 -p --loop=100000000 can6
    

Receive can-fd frames on vicanlog, and verify the content.
There are some missing frames when I read from > 2 interfaces. Sometimes the timestamp is completely wrong.
Might be some performance issue... but also some mutex problem with my tests.

vicanlog@vicanlog-light:~/zcqcomlib/build$ ./examples/canfdmonitor 0 1 2 -t 10 -r -s

Expecting can0 frame 99, got 98 (missing 1) (time_diff=230 68659769 68659539)
Expecting can0 frame 99, got 9A (missing 1) (time_diff=230 68659999 68659769)
Expecting can0 frame C5, got C4 (missing 1) (time_diff=230 68729120 68728890)
Expecting can0 frame C5, got C6 (missing 1) (time_diff=229 68729349 68729120)
Expecting can0 frame 01, got 00 (missing 1) (time_diff=242 68743137 68742895)
Expecting can0 frame 01, got 02 (missing 1) (time_diff=240 68743377 68743137)
Expecting can0 frame 35, got 34 (missing 1) (time_diff=230 69051486 69051256)
Expecting can0 frame 35, got 36 (missing 1) (time_diff=230 69051716 69051486)
Expecting can0 frame 31, got 30 (missing 1) (time_diff=230 69346785 69346555)
Expecting can0 frame 31, got 32 (missing 1) (time_diff=230 69347015 69346785)
Expecting can0 frame 59, got 58 (missing 1) (time_diff=230 69415246 69415016)
Expecting can0 frame 59, got 5A (missing 1) (time_diff=234 69415480 69415246)
Expecting can0 frame 55, got 54 (missing 1) (time_diff=-69473347 0 69473347)
Expecting can0 frame 55, got 56 (missing 1) (time_diff=69473805 69473805 0)
Expecting can0 frame 25, got 24 (missing 1) (time_diff=230 70173448 70173218)
Expecting can0 frame 25, got 26 (missing 1) (time_diff=230 70173678 70173448)
Expecting can0 frame 11, got 10 (missing 1) (time_diff=232 70642753 70642521)
Expecting can0 frame 11, got 12 (missing 1) (time_diff=232 70642985 70642753)
Expecting can0 frame 75, got 74 (missing 1) (time_diff=230 70665784 70665554)
Expecting can0 frame 75, got 76 (missing 1) (time_diff=230 70666014 70665784)
Expecting can0 frame 24, got 23 (missing 1) (time_diff=-71357884 0 71357884)
Expecting can0 frame 24, got 25 (missing 1) (time_diff=71358346 71358346 0)
Expecting can0 frame 39, got 38 (missing 1) (time_diff=234 71481460 71481226)
Expecting can0 frame 39, got 3A (missing 1) (time_diff=236 71481696 71481460)
Expecting can0 frame 41, got 40 (missing 1) (time_diff=228 71483344 71483116)
Expecting can0 frame 41, got 42 (missing 1) (time_diff=228 71483572 71483344)
*/
#endif

#include <canlib.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include "vcanevt.h"

#ifndef _WIN32
#include <unistd.h>
#endif
#include <sys/types.h>
#include <sys/stat.h>
#include <getopt.h>
#include <pthread.h>
#include <string.h> // for memset

#define ALARM_INTERVAL_IN_S   (1)
#define READ_WAIT_INFINITE    (unsigned long)(-1)

static unsigned int msgCounter = 0;
extern int optind, opterr, optopt;
static int verbose = 1;
static int silent = 0;
long timeout = READ_WAIT_INFINITE;
long max_idle_time = 0;
long loops = 0;
#define MAX_CHANNELS 6
int channel[MAX_CHANNELS] = { 0 };
int error_cnt[MAX_CHANNELS] = { 0 };
int error_frame_cnt[MAX_CHANNELS] = { 0 };
int check_cansequence = 0;

static void check(char* id, canStatus stat)
{
  if (stat != canOK) {
    char buf[50];
    buf[0] = '\0';
    canGetErrorText(stat, buf, sizeof(buf));
    printf("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
  }
}

static void printUsageAndExit(char *prgName)
{
  printf("Usage: '%s <channel> [<channel2>...] [-b 500000] [-B 2000000] [-s] [-v] [-t 10] [-l 100]'\n", prgName);
  printf("\t-b\tBitrate\n");
  printf("\t-B\tHigher data-bitrate on CAN-FD\n");
  printf("\t-t\tExit after x seconds idle time\n");
  printf("\t-l\tRead x frames before exit\n");
  printf("\t-s\tSilent mode without much output\n");
  printf("\t-r\tRead and compare frames with cansequence output\n");
  printf("\t-v\tVerbose output\n");
  printf("\t-V\tVersion\n");
  exit(1);
}

#if 0
static void sighand(int sig)
{
  static unsigned int last;

  switch (sig) {
  case SIGINT:
    break;
  case SIGALRM:
    if (msgCounter != last) {
      printf("rx : %u total: %u\n", msgCounter - last, msgCounter);
    }
    last = msgCounter;
    alarm(ALARM_INTERVAL_IN_S);
    break;
  }
}
#endif



static char* busStatToStr(const unsigned long flag) {
    char* tempStr = NULL;
    #define MACRO2STR(x) case x: tempStr = #x; break
    switch (flag) {
        MACRO2STR( CHIPSTAT_BUSOFF        );
        MACRO2STR( CHIPSTAT_ERROR_PASSIVE );
        MACRO2STR( CHIPSTAT_ERROR_WARNING );
        MACRO2STR( CHIPSTAT_ERROR_ACTIVE  );
        default: tempStr = ""; break;
    }
    #undef MACRO2STR
    return tempStr;
}

void notifyCallback(canNotifyData *data) {
    if(silent) return;
  switch (data->eventType) {
  case canEVENT_STATUS:
    printf("CAN Status Event: %s\n", busStatToStr(data->info.status.busStatus));
    break;
  case canEVENT_ERROR:
    printf("CAN Error Event\n");
    break;
  case canEVENT_TX:
    printf("CAN Tx Event\n");
    break;
  case canEVENT_RX:
    printf("CAN Rx Event\n");
    break;
  }
  return;
}

int to_bitrate(int x)
{
    switch(x) {
    case 250000:
        return canBITRATE_250K;
    case 500000:
        return canBITRATE_500K;
    case 1000000:
        return canBITRATE_1M;
    }
    return canBITRATE_500K;
}

int to_bitrate_fd(int x)
{
    switch(x) {
    case 500000:
        return canFD_BITRATE_500K_80P;
    case 1000000:
        return canFD_BITRATE_1M_80P;
    case 2000000:
        return canFD_BITRATE_2M_80P;
    case 4000000:
        return canFD_BITRATE_4M_80P;
    }
    return canFD_BITRATE_500K_80P;
}

void *read_thread(void *v)
{
    canHandle *hnd = (canHandle *)v;
    canStatus stat;
    int idle_time = 0;
    int msgCounter = 0;
    int nr_bytes = 0;
    unsigned int next_expected_id = 0;

    printf("can%d thread started\n", *hnd);

    do {
        long id;
        unsigned char msg[64];
        unsigned int dlc;
        unsigned int flag;
        unsigned long time;
        unsigned long last_time;

        stat = canReadWait(*hnd, &id, &msg, &dlc, &flag, &time, timeout);
        if(stat == canERR_TIMEOUT) {
            idle_time++;
            printf("can%d Idle waiting %d / %ld\n", *hnd, idle_time, max_idle_time);
            if(idle_time >= max_idle_time) {
                printf("can%d Exit since idle %ld seconds\n", *hnd, max_idle_time);
                break;
            }
            stat = canOK;
        } else if (stat == canOK) {
            char *can_std;
            unsigned int i;
            msgCounter++;
            if (flag & canMSG_ERROR_FRAME) {
                printf("(%u) ERROR FRAME flags:0x%x time:%llu\n", msgCounter, flag, (unsigned long long)time);
                error_frame_cnt[*hnd]++;
                continue;
            }

            idle_time = 0;
            nr_bytes += dlc;

            if(check_cansequence) {
                if(msg[0] != next_expected_id) {
                    // Verify frame-id with generated cansequence frames
                    unsigned int missing;
                    if(msg[0] && (msg[0] == next_expected_id)) {
                        printf("Duplicate can%d frame? %02X (time=%ld last_time=%ld)\n", *hnd, msg[0], time, last_time);
                        error_cnt[*hnd]++;
                    } else {
                        if(msg[0] < next_expected_id) {
                            missing = next_expected_id - msg[0];
                        } else {
                            missing = msg[0] - next_expected_id;
                        }
                        printf("Expecting can%d frame %02X, got %02X (missing %d) (time_diff=%ld %ld %ld)\n", *hnd, next_expected_id, msg[0], missing, time-last_time, time, last_time);
                        error_cnt[*hnd]++;
                    }
                }
                if(next_expected_id == 0xFF) {
                    next_expected_id = 0;
                } else {
                    next_expected_id = msg[0] + 1;
                }
                last_time = time;
            }
            
            if(!silent) {
                // Flags set is: CanFDFrame CanFDBitrateSwitch TxMsgAcknowledge Extended Standard ErrorFrame
                // but they are not available here yet!!! canfd flags are not matching the defines called canFDMSG*
                if (flag & canFDMSG_FDF) {
                    if (flag & canFDMSG_BRS) {
                        can_std = "FD+";
                    }
                    else {
                        can_std = "FD ";
                    }
                }
                else {
                    can_std = "STD";
                }

                printf("CH:%2d %s:%s:%2u:%08lx", channel[i],
                       can_std,
                       (flag & canMSG_EXT) ? "X" : " ",
                       dlc,id);
#if 0
                if (flag & canFDMSG_ESI) {
                    printf("ESI "); // Sender of the message is in error passive mode
                }
#endif
                printf(" flags:0x%x time:%llu", flag, (unsigned long long)time);

                for (i = 0; i < dlc; i++) {
                    unsigned char byte = msg[i];
#if 0
                    if ((i % 16) == 0) {
                        printf("\n    ");
                    }
#endif
                    printf(" %02x ", byte);
                }
                printf("\n");
            }
        }
        else {
            if (errno == 0) {
                check("\ncanReadWait", stat);
            }
            else {
                perror("\ncanReadWait error");
            }
            printf("got not canok2\n");
        }
        if(loops && (msgCounter > loops)) break;
    } while (stat == canOK);

#if 0
    sighand(SIGALRM);
#endif

    printf("can%d received %lu frames (%lu bytes) (error_cnt=%d %d)\n", *hnd, (unsigned long)msgCounter, (unsigned long)nr_bytes, error_cnt[*hnd], error_frame_cnt[*hnd]);
}

int main(int argc, char *argv[])
{
  canHandle hnd[MAX_CHANNELS];
  canStatus stat;
  int opt;
  int i;
  int nr_channels = 0;
  int use_event = 0;
  int bitrate = canBITRATE_500K;
  int bitrate_fd = canFD_BITRATE_2M_80P;

  struct option long_options[] = {
      { "help",   no_argument,      0, 'h' },
      { "loop",   required_argument,   0, 'l' },
      { 0,      0,         0, 0},
   };

   if (argc < 2) {
       printUsageAndExit(argv[0]);
   }
   while ((opt = getopt_long(argc, argv, "hVvl:t:b:B:sre", long_options, NULL)) != -1) {
       switch (opt) {
       case 'h':
           printUsageAndExit(argv[0]);
           break;

       case 'r':
           check_cansequence = 1;
           break;

       case 'e':
           use_event = 1;
           break;

       case 'v':
           verbose++;
           break;
            
       case 's':
           silent = 1;
           break;
            
       case 't':
           if (optarg) {
               max_idle_time = strtoul(optarg, NULL, 0);
               if(max_idle_time <= 0) {
                   max_idle_time = 0;
               } else {
                   timeout = 1000;
               }
           }
           break;
            
       case 'b':
           if (optarg) {
               bitrate = to_bitrate(strtoul(optarg, NULL, 0));
           }
           break;
            
       case 'B':
           if (optarg) {
               bitrate_fd = to_bitrate_fd(strtoul(optarg, NULL, 0));
           }
           break;
            
       case 'l':
           if (optarg) {
               loops = strtoul(optarg, NULL, 0);
           }
           break;
            
       case 'V':
           printf("Version 0.1\n");
           break;
       default:
           printUsageAndExit(argv[0]);
           break;
       }
   }
   while(optind < argc) {
       if (argv[optind]) {
           channel[nr_channels] = strtol(argv[optind], NULL, 10);
           if(channel[nr_channels] < MAX_CHANNELS) {
               printf("use channel %d\n", channel[nr_channels]);
               nr_channels++;
           }
       }
       optind++;
   }
   if (!nr_channels) {
       printUsageAndExit(argv[0]);
   }

   printf("Reading messages on %d channels\n", nr_channels);

#if 0
  /* Use sighand as our signal handler */
  signal(SIGALRM, sighand);
  signal(SIGINT, sighand);

  /* Allow signals to interrupt syscalls */
  siginterrupt(SIGINT, 1);
#endif

  canInitializeLibrary();

  for(i=0; i<nr_channels; i++) {
      /* Open channel, set parameters and go on bus */
      hnd[i] = canOpenChannel(channel[i], canOPEN_CAN_FD);
      if (hnd[i] < 0) {
          printf("canOpenChannel %d", channel[i]);
          check("", hnd[i]);
          return -1;
      }

      unsigned long events = canNOTIFY_STATUS | canNOTIFY_ENVVAR;
      if(use_event) {
          printf("setNotify on all event\n");
          // skip notify if running silient... just want to test read-performacne.
          events |= canNOTIFY_RX | canNOTIFY_TX | canNOTIFY_ERROR;
      }
      stat = canSetNotify(hnd[i], notifyCallback, events, (char*)0);
      check("canSetNotify", stat);

      stat = canSetBusParams(hnd[i], bitrate, 0, 0, 0, 0, 0);
      check("canSetBusParams", stat);
      if (stat != canOK) {
          goto ErrorExit;
      }
      stat = canSetBusParamsFd(hnd[i], bitrate_fd, 0, 0, 0);
      check("canSetBusParamsFd", stat);
      if (stat != canOK) {
          goto ErrorExit;
      }
      stat = canBusOn(hnd[i]);
      check("canBusOn", stat);
      if (stat != canOK) {
          goto ErrorExit;
      }
  }

  // alarm(ALARM_INTERVAL_IN_S);
  pthread_attr_t attr;
  int rc;
  memset(&attr, 0, sizeof(pthread_attr_t));
  if ((rc = pthread_attr_init(&attr)) != 0) {
      printf("Failed to initialize attribute %d", rc);
      return -1;
  }
#if 0  
  if (pthread_attr_setstacksize(&attr, MAX(stackSize, PTHREAD_STACK_MIN))) {
      printf("Unable to set stack size");
      return -1;
  }
#endif
  pthread_t thread[MAX_CHANNELS];
  for(i=0; i<nr_channels; i++) {
      pthread_create(&thread[i], &attr, read_thread, &hnd[i]);
  }
  for(i=0; i<nr_channels; i++) {
      void *retval = NULL;
      if(pthread_join(thread[i], &retval)) {
          printf("Failed to stop and join thread");
      }
  }
  
ErrorExit:
  // alarm(0);
  for(i=0; i<nr_channels; i++) {
      stat = canBusOff(hnd[i]);
      check("canBusOff", stat);
      stat = canClose(hnd[i]);
      check("canClose", stat);
  }
  stat = canUnloadLibrary();
  check("canUnloadLibrary", stat);

  return 0;
}
