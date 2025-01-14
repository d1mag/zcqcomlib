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
**
** IMPORTANT NOTICE:
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
 * Read CAN messages and print out their contents
 */

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
#include <pthread.h>

#define READ_WAIT_INFINITE    (unsigned long)(-1)

extern int optind, opterr, optopt;
static int verbose = 1;
static int silent = 0;
long timeout = READ_WAIT_INFINITE;
long max_idle_time = 0;
long loops = 0;
#define MAX_CHANNELS 6
canHandle hnd[MAX_CHANNELS];
int can_ndx[MAX_CHANNELS] = { 0 };
int channel[MAX_CHANNELS] = { 0 };
int error_cnt[MAX_CHANNELS] = { 0 };
int error_frame_cnt[MAX_CHANNELS] = { 0 };
int check_cansequence = 0;
pthread_mutex_t output_mutex;

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
  printf("Usage: '%s <channel> [<channel2>...] [-b 500000] [-s] [-v] [-t 10] [-l 100]'\n", prgName);
  printf("\t-b\tBitrate\n");
  printf("\t-B\tHigher data-bitrate on CAN-FD\n");
  printf("\t-t\tExit after x seconds idle time\n");
  printf("\t-l\tRead x frames before exit\n");
  printf("\t-r\tRead and compare frames with cansequence output\n");
  printf("\t-s\tSilent mode without much output\n");
  printf("\t-v\tVerbose output\n");
  printf("\t-V\tVersion\n");
  exit(1);
}

#if 0
static void sighand(int sig)
{
  (void)sig;
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
  case canEVENT_ENVVAR:
      printf("CAN EnvVar Event\n");
      break;
  case canEVENT_BUSONOFF:
      printf("CAN BusOff Event\n");
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

void *read_thread(void *v)
{
    int can_ndx = *((int *)v);
    int can_port = channel[can_ndx];
    canStatus stat;
    int idle_time = 0;
    int nr_bytes = 0;
    unsigned int msgCounter = 0;
    unsigned int next_expected_id = 0;

    printf("can_ndx=%d can_port=%d can%d thread started\n", can_ndx, can_port, channel[can_ndx]);

    do {
        long id;
        unsigned char msg[8];
        unsigned int dlc;
        unsigned int flag;
        unsigned long time;
        unsigned long last_time;

        stat = canReadWait(hnd[can_ndx], &id, &msg, &dlc, &flag, &time, timeout);
        if(stat == canERR_TIMEOUT) {
            idle_time++;
            printf("can%d Idle waiting %d / %ld\n", channel[can_ndx], idle_time, max_idle_time);
            if(idle_time >= max_idle_time) {
                printf("can%d Exit since idle %ld seconds\n", channel[can_ndx], max_idle_time);
                break;
            }
            stat = canOK;
        } else if (stat == canOK) {
            msgCounter++;
            if (flag & canSTAT_HW_OVERRUN /* 0x200 ErrorHWOverrun */) { 
                pthread_mutex_lock(&output_mutex);
                printf("can%d HW_OVERRUN flags:0x%x time:%llu\n", channel[can_ndx], flag, (unsigned long long)time);
                pthread_mutex_unlock(&output_mutex);
                error_frame_cnt[can_ndx]++;
                //continue;
            }

            if (flag & canMSG_ERROR_FRAME) {
                pthread_mutex_lock(&output_mutex);
                printf("can%d ERROR FRAME flags:0x%x time:%llu\n", channel[can_ndx], flag, (unsigned long long)time);
                pthread_mutex_unlock(&output_mutex);
                error_frame_cnt[can_ndx]++;
                continue;
            }

            idle_time = 0;
            nr_bytes += dlc;

            if(check_cansequence) {
                if(msg[0] != next_expected_id) {
                    // Verify frame-id with generated cansequence frames
                    unsigned int missing;
                    if(msg[0] && (msg[0] == next_expected_id)) {
                        pthread_mutex_lock(&output_mutex);
                        printf("Duplicate can%d frame? %02X (time=%ld last_time=%ld)\n", channel[can_ndx], msg[0], time, last_time);
                        pthread_mutex_unlock(&output_mutex);
                        error_cnt[can_ndx]++;
                    } else {
                        if(msg[0] < next_expected_id) {
                            missing = next_expected_id - msg[0];
                        } else {
                            missing = msg[0] - next_expected_id;
                        }
                        pthread_mutex_lock(&output_mutex);
                        printf("Expecting can%d frame %02X, got %02X (missing %d) (time_diff=%ld %ld %ld) flag=0x%X\n", channel[can_ndx], next_expected_id, msg[0], missing, time-last_time, time, last_time, flag);
                        pthread_mutex_unlock(&output_mutex);
                        error_cnt[can_ndx]++;
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
                pthread_mutex_lock(&output_mutex);
                if (flag & canMSG_ERROR_FRAME) {
                    printf("(%u) ERROR FRAME flags:0x%x time:%llu\n", msgCounter, flag, (unsigned long long)time);
                }
                else {
                    unsigned j;

                    printf("can%d id:%lx dlc:%u data: ", channel[can_ndx], id, dlc);
                    if (dlc > 8) {
                        dlc = 8;
                    }
                    for (j = 0; j < dlc; j++) {
                        printf("%2.2x ", msg[j]);
                    }
                    printf(" flags:0x%x time:%llu\n", flag, (unsigned long long)time);
                }
                pthread_mutex_unlock(&output_mutex);
            }
        }
        else {
            if (errno == 0) {
                check("\ncanReadWait", stat);
            }
            else {
                perror("\ncanReadWait error");
            }
        }

        if(loops && (msgCounter > loops)) break;
    } while (stat == canOK);

    printf("can%d received %lu frames (%lu bytes) (error_cnt=%d %d)\n", channel[can_ndx], (unsigned long)msgCounter, (unsigned long)nr_bytes, error_cnt[can_ndx], error_frame_cnt[can_ndx]);
}


int main(int argc, char *argv[])
{
  canStatus stat;
  int opt;
  int i;
  long loops = 0;
  int nr_channels = 0;
  int use_event = 0;
  int bitrate = canBITRATE_500K;
  
   struct option long_options[] = {
      { "help",   no_argument,      0, 'h' },
      { "loop",   required_argument,   0, 'l' },
      { 0,      0,         0, 0},
   };

   if (argc < 2) {
       printUsageAndExit(argv[0]);
   }
   while ((opt = getopt_long(argc, argv, "hVvl:t:b:sre", long_options, NULL)) != -1) {
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
  /* Allow signals to interrupt syscalls */
  signal(SIGINT, sighand);
  siginterrupt(SIGINT, 1);
#endif

  canInitializeLibrary();

  for(i=0; i<nr_channels; i++) {
      /* Open channel, set parameters and go on bus */
      hnd[i] = canOpenChannel(channel[i], canOPEN_EXCLUSIVE | canOPEN_REQUIRE_EXTENDED | canOPEN_ACCEPT_VIRTUAL);
      if (hnd[i] < 0) {
          printf("canOpenChannel %d", channel[i]);
          check("", hnd[i]);
          return -1;
      }

      unsigned long events = canNOTIFY_STATUS | canNOTIFY_ENVVAR | canNOTIFY_ERROR;
      if(use_event) {
          printf("setNotify on all event\n");
          // skip notify if running silient... just want to test read-performacne.
          events |= canNOTIFY_RX | canNOTIFY_TX;
      }
      stat = canSetNotify(hnd[i], notifyCallback, events, (char*)0);
      check("canSetNotify", stat);
      
      stat = canSetBusParams(hnd[i], bitrate, 0, 0, 0, 0, 0);
      check("canSetBusParams", stat);
      if (stat != canOK) {
          goto ErrorExit;
      }
      stat = canBusOn(hnd[i]);
      check("canBusOn", stat);
      if (stat != canOK) {
          goto ErrorExit;
      }
  }


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

  for(i=0; i<nr_channels; i++) {
      stat = canBusOff(hnd[i]);
      check("canBusOff", stat);
      // usleep(50*1000); // Sleep just to get the last notification.
      stat = canClose(hnd[i]);
      check("canClose", stat);
  }
  stat = canUnloadLibrary();
  check("canUnloadLibrary", stat);

  return 0;
}
