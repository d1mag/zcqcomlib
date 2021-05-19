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

#include <canlib.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <getopt.h>

#define ALARM_INTERVAL_IN_S   (1)
#define READ_WAIT_INFINITE    (unsigned long)(-1)

static unsigned int msgCounter = 0;
extern int optind, opterr, optopt;

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
  printf("Usage: '%s <channel>'\n", prgName);
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

int main(int argc, char *argv[])
{
  canHandle hnd;
  canStatus stat;
  int channel;
  int verbose = 1;
  int silent = 0;
  int opt;
  long timeout = READ_WAIT_INFINITE;
  long loops = 0;
  int got_channel = 0;
  
   struct option long_options[] = {
      { "help",   no_argument,      0, 'h' },
      { "loop",   required_argument,   0, 'l' },
      { 0,      0,         0, 0},
   };

   if (argc < 2) {
       printUsageAndExit(argv[0]);
   }
   while ((opt = getopt_long(argc, argv, "hVvl:t:s", long_options, NULL)) != -1) {
       switch (opt) {
       case 'h':
           printUsageAndExit(argv[0]);
           break;

       case 'v':
           verbose++;
           break;
            
       case 's':
           silent = 1;
           break;
            
       case 't':
           if (optarg) {
               timeout = strtoul(optarg, NULL, 0);
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
   if (argv[optind]) {
       channel = strtol(argv[optind], NULL, 10);
       got_channel = 1;
   }

   if (!got_channel) {
       printUsageAndExit(argv[0]);
   }

  printf("Reading messages on channel %d\n", channel);

#if 0
  /* Use sighand as our signal handler */
  signal(SIGALRM, sighand);
  signal(SIGINT, sighand);

  /* Allow signals to interrupt syscalls */
  siginterrupt(SIGINT, 1);
#endif

  canInitializeLibrary();

  /* Open channel, set parameters and go on bus */
  hnd = canOpenChannel(channel, canOPEN_CAN_FD);
  if (hnd < 0) {
    printf("canOpenChannel %d", channel);
    check("", hnd);
    return -1;
  }

  stat = canSetBusParams(hnd, canBITRATE_500K, 0, 0, 0, 0, 0);
  check("canSetBusParams", stat);
  if (stat != canOK) {
    goto ErrorExit;
  }
  stat = canSetBusParamsFd(hnd, canFD_BITRATE_2M_80P, 0, 0, 0);
  check("canSetBusParamsFd", stat);
  if (stat != canOK) {
    goto ErrorExit;
  }
  stat = canBusOn(hnd);
  check("canBusOn", stat);
  if (stat != canOK) {
    goto ErrorExit;
  }

  // alarm(ALARM_INTERVAL_IN_S);

  int nr = 0;
  int nr_received = 0;
  int nr_bytes = 0;
  do {
    long id;
    unsigned char msg[64];
    unsigned int dlc;
    unsigned int flag;
    unsigned long time;

    stat = canReadWait(hnd, &id, &msg, &dlc, &flag, &time, timeout);
    if(stat == canERR_TIMEOUT) {
        nr++;
        printf("Idle waiting %d / 10\n", nr);
        if(nr >= 10) {
            printf("Exit since idle 10 seconds\n");
            break;
        }
        stat = canOK;
    } else if (stat == canOK) {
      char *can_std;
      unsigned int i;

      msgCounter++;
      if (flag & canMSG_ERROR_FRAME) {
          /*
            Error frames have completely different timestamps.
            Error frames and can frames have 64 bits. CAN-FD frames have 32 bits.
                        
            (9358) ERROR FRAME flags:0x10020 time:950667868
            (9359) ERROR FRAME flags:0x10020 time:950743906
            CH: 0 FD+: : 8:00000101 flags:0x30002 time:74939612
            20  00  00  00  00  00  00  00
            CH: 0 FD+: : 8:00000506 flags:0x30002 time:74939743
            00  00  00  00  00  00  00  00
            CH: 0 FD+: : 8:00000101 flags:0x30002 time:76638295
            40  00  00  00  00  00  00  00
            CH: 0 FD+: : 8:00000506 flags:0x30002 time:76638410
            00  00  00  00  00  00  00  00
            CH: 0 FD+: :64:00000021 flags:0x30002 time:76638797
            00  00  00  00  00  00  00  00  00  00  00  00  00  00  00  00
            00  00  00  00  00  00  00  00  00  00  00  00  00  00  00  00
            00  00  00  00  00  00  00  00  00  00  00  00  00  00  00  00
            00  00  00  00  00  00  00  00  00  00  00  00  00  00  00  00
          */
          
          printf("(%u) ERROR FRAME flags:0x%x time:%llu\n", msgCounter, flag, (unsigned long long)time);
        continue;
      }

      nr_received++;
      nr_bytes += dlc;
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

          printf("CH:%2d %s:%s:%2u:%08lx", channel,
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
    if(loops && (nr_received > loops)) break;
  } while (stat == canOK);

#if 0
  sighand(SIGALRM);
#endif

  printf("received %lu frames (%lu bytes)\n", (unsigned long)nr_received, (unsigned long)nr_bytes);
  
ErrorExit:
  // alarm(0);
  stat = canBusOff(hnd);
  check("canBusOff", stat);
  stat = canClose(hnd);
  check("canClose", stat);
  stat = canUnloadLibrary();
  check("canUnloadLibrary", stat);

  return 0;
}
