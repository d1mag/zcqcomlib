#define _GNU_SOURCE
#include <errno.h>
#include <getopt.h>
#include <libgen.h>
#include <limits.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#include <net/if.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <sys/stat.h>

#include <linux/can.h>
#include <linux/can/raw.h>

extern int optind, opterr, optopt;

static int s = -1;
static int running = 1;

#ifndef VERSION
#define VERSION "5.0"
#endif
#ifndef PF_CAN
#define PF_CAN 29
#endif

enum {
   VERSION_OPTION = CHAR_MAX + 1,
};

#define CAN_ID_DEFAULT   (2)
#define CAN_DLC_DEFAULT   (8)
#define MAX_CAN_DLC      (64)
#define DEBUG_PRINT_FILE "/tmp/debug_print_packets"

void print_usage(char *prg)
{
   fprintf(stderr, "Usage: %s [<can-interface>|any] [Options]\n"
      "\n"
      "cansequence sends CAN messages with a rising sequence number as payload.\n"
      "When the -r option is given, cansequence expects to receive these messages\n"
      "and prints an error message if a wrong sequence number is encountered.\n"
      "The main purpose of this program is to test the reliability of CAN links.\n"
      "\n"
      "Options:\n"
      " -e  --extended      send extended frame\n"
      " -d, --dlc=1-%d      CAN data length (default = %u)\n"
      " -i, --identifier=ID CAN Identifier (default = %u)\n"
      " -r, --receive       work as receiver\n"
      "     --loop=COUNT    send message COUNT times\n"
      " -p  --poll          use poll(2) to wait for buffer space while sending\n"
      " -q  --quit          quit if a wrong sequence is encountered\n"
      " -v, --verbose       be verbose (twice to be even more verbose\n"
      " -s, --sleep=us      sleep x us between each write()\n"
      " -L, --len=1         read length. If >1, then use recmmsg()\n"
      " -x, --verify        verify byte 2-8 in frame\n"
      " -h  --help          this help\n"
      "     --version       print version information and exit\n",
      prg, MAX_CAN_DLC, CAN_DLC_DEFAULT, CAN_ID_DEFAULT);
}

void sigterm(int signo)
{
   running = 0;
}

#define MAXIFNAMES 6

static char devname[MAXIFNAMES][IFNAMSIZ+1];
static int  dindex[MAXIFNAMES];
static int  max_devname_len; /* to prevent frazzled device name output */

int idx2dindex(int ifidx, int socket)
{
   
   int i;
   struct ifreq ifr;
   
   for (i=0; i < MAXIFNAMES; i++) {
      if (dindex[i] == ifidx)
         return i;
   }
   
   /* create new interface index cache entry */
      
   /* remove index cache zombies first */
   for (i=0; i < MAXIFNAMES; i++) {
      if (dindex[i]) {
         ifr.ifr_ifindex = dindex[i];
         if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
            dindex[i] = 0;
      }
   }
   
   for (i=0; i < MAXIFNAMES; i++)
      if (!dindex[i]) /* free entry */
         break;
   
   if (i == MAXIFNAMES) {
      fprintf(stderr, "Interface index cache only supports %d interfaces.\n",
         MAXIFNAMES);
      exit(1);
   }
   
   dindex[i] = ifidx;
   
   ifr.ifr_ifindex = ifidx;
   if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
      perror("SIOCGIFNAME");
   
   if (max_devname_len < strlen(ifr.ifr_name))
      max_devname_len = strlen(ifr.ifr_name);
   
   strcpy(devname[i], ifr.ifr_name);
   
#ifdef DEBUG
   printf("new index %d (%s)\n", i, devname[i]);
#endif
      
   return i;
}



int main(int argc, char **argv)
{
   struct ifreq ifr;
   struct sockaddr_can addr;
   struct canfd_frame frame = {
      .len = CAN_DLC_DEFAULT,
   };
   struct can_filter filter[] = {
      {
         .can_id = CAN_ID_DEFAULT,
      },
   };
   char *interface = "can0";
   unsigned char sequence = 0;
   int seq_wrap = 0;
   int family = PF_CAN, type = SOCK_RAW, proto = CAN_RAW;
   int loopcount = 1, infinite = 1;
   int use_poll = 0;
   int extended = 0;
   int nbytes;
   int opt;
   int i;
   int receive = 0;
   int sequence_init = 1;
   int verbose = 0, quit = 0;
   int exit_value = EXIT_SUCCESS;
   unsigned int sleep_delay = 0;
   unsigned int VLEN = 1;
   int verify_all_data = 0;
   
   signal(SIGTERM, sigterm);
   signal(SIGHUP, sigterm);

   struct option long_options[] = {
      { "extended",   no_argument,      0, 'e' },
      { "help",   no_argument,      0, 'h' },
      { "poll",   no_argument,      0, 'p' },
      { "quit",   no_argument,      0, 'q' },
      { "receive",   no_argument,      0, 'r' },
      { "verbose",   no_argument,      0, 'v' },
      { "version",   no_argument,      0, VERSION_OPTION},
      { "dlc",   required_argument,   0, 'd' },
      { "sleep", required_argument,   0, 's' },
      { "len", required_argument,   0, 'L' },
      { "verify",   no_argument,      0, 'x' },
      { "identifier",   required_argument,   0, 'i' },
      { "loop",   required_argument,   0, 'l' },
      { 0,      0,         0, 0},
   };

   while ((opt = getopt_long(argc, argv, "ehpqrvxL:s:d:i:l:", long_options, NULL)) != -1) {
      switch (opt) {
         case 'e':
            extended = 1;
            break;

         case 'h':
            print_usage(basename(argv[0]));
            exit(EXIT_SUCCESS);
            break;

         case 'p':
            use_poll = 1;
            break;

         case 'q':
            quit = 1;
            break;

         case 'r':
            receive = 1;
            break;

         case 'v':
            verbose++;
            break;

         case VERSION_OPTION:
            printf("cansequence %s\n", VERSION);
            exit(EXIT_SUCCESS);
            break;

         case 'l':
            if (optarg) {
               loopcount = strtoul(optarg, NULL, 0);
               infinite = 0;
            } else
               infinite = 1;
            break;

         case 'i':
            filter->can_id = strtoul(optarg, NULL, 0);
            break;


         case 'd':
            frame.len = strtoul(optarg, NULL, 0);
            if((frame.len < 1) || (frame.len > MAX_CAN_DLC)) {
               printf("bad value can_dlc=%d\n", frame.len);
               exit(EXIT_FAILURE);
            }
            break;

         case 's':
            sleep_delay = strtoul(optarg, NULL, 0);
            if(sleep_delay > 10000000) {
               sleep_delay = 10000000;
            }
            break;

         case 'L':
            VLEN = strtoul(optarg, NULL, 0);
            if(VLEN == 0) {
               VLEN = 1;
            }
            if(VLEN > 8192) {
               VLEN = 8192;
            }
            break;

         case 'x':
            verify_all_data = 1;
            break;


         default:
            fprintf(stderr, "Unknown option %c\n", opt);
            break;
      }
   }

   if (argv[optind] != NULL)
      interface = argv[optind];

   if (extended) {
      filter->can_mask = CAN_EFF_MASK;
      filter->can_id  &= CAN_EFF_MASK;
      filter->can_id  |= CAN_EFF_FLAG;
   } else {
      filter->can_mask = CAN_SFF_MASK;
      filter->can_id  &= CAN_SFF_MASK;
   }
   frame.can_id = filter->can_id;

   printf("interface = %s, family = %d, type = %d, proto = %d\n",
      interface, family, type, proto);

   s = socket(family, type, proto);
   if (s < 0) {
      perror("socket");
      return 1;
   }

   addr.can_family = family;
   if(strcmp(interface, "any")) {
      strncpy(ifr.ifr_name, interface, sizeof(ifr.ifr_name));
      if (ioctl(s, SIOCGIFINDEX, &ifr)) {
         perror("ioctl");
         return 1;
      }
      addr.can_ifindex = ifr.ifr_ifindex;
   } else {
      addr.can_ifindex = 0; // any interface
   }
   
   /* first don't recv. any msgs */
   if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0)) {
      perror("setsockopt");
      exit(EXIT_FAILURE);
   }

#define ENABLE_RECMMSG
#ifdef ENABLE_RECMMSG
#define CL_CFSZ 512
   /* we only support one socket */
   static __u32 dropcnt;
   static __u32 last_dropcnt;
   struct sockaddr_can addrs[VLEN];

   char ctrlmsgs[VLEN][CMSG_SPACE(sizeof(struct timeval)) + CMSG_SPACE(sizeof(__u32))];
   struct cmsghdr *cmsg;
   
   //struct canfd_frame frames[VLEN];
   struct can_frame frames[VLEN];
   struct iovec iovecs[VLEN];
   struct mmsghdr mmsghdrs[VLEN];
   
   char buf[CL_CFSZ]; /* max length */
   int nframes, maxdlen, idx;
   struct timeval tv = { 0, 0 };
   struct timeval last_tv = {0};
#endif
   
   int enable_sockopt = 1;
   if (setsockopt(s, SOL_SOCKET, SO_TIMESTAMP, &enable_sockopt, sizeof(enable_sockopt)) < 0) {
      perror("setsockopt SO_TIMESTAMP");
   }
   
   if (setsockopt(s, SOL_SOCKET, SO_RXQ_OVFL, &enable_sockopt, sizeof(enable_sockopt)) < 0) {
      perror("setsockopt SO_RXQ_OVFL not supported by your Linux Kernel");
      /* continue without dropmonitor */
   }
   
   if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      perror("bind");
      return 1;
   }
   
   memset(frame.data, 0, sizeof(frame.data));
   
   size_t clen;
   clen = sizeof(struct can_frame);
   if(frame.len > 8) {
      unsigned int canfd_on = 1;
      if(setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on)) < 0) {
         printf("canfd_on failed with %d bytes. Send %d bytes instead.\n", frame.len, CAN_DLC_DEFAULT);
         frame.len = CAN_DLC_DEFAULT;
      } else {
         clen = sizeof(struct canfd_frame);
      }
   }

   unsigned char last_sequence = 0;
   unsigned int verbose_prints = 0;
   
   if (receive) {
      /* enable recv. now */
      if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, filter, sizeof(filter))) {
         perror("setsockopt");
         exit(EXIT_FAILURE);
      }

#ifdef ENABLE_RECMMSG
      /* these settings are static and can be held out of the hot path */
      memset(frames, 0, sizeof(frames));
      memset(addrs, 0, sizeof(addrs));
      memset(iovecs, 0, sizeof(iovecs));
      memset(mmsghdrs, 0, sizeof(mmsghdrs));
      for (i = 0; i < VLEN; i++) {
         iovecs[i].iov_base = &frames[i];
         //             iovecs[i].iov_len = BUFSIZE;
         mmsghdrs[i].msg_hdr.msg_name= &addrs[i];
         mmsghdrs[i].msg_hdr.msg_iov = &iovecs[i];
         mmsghdrs[i].msg_hdr.msg_iovlen = 1;
         mmsghdrs[i].msg_hdr.msg_control = &ctrlmsgs[i];
      }
      
      //     iov.iov_base = &frame;
      //     msg.msg_name = &addr;
      //     msg.msg_iov = &iov;
      //     msg.msg_iovlen = 1;
      //     msg.msg_control = &ctrlmsg;

      if(VLEN > 1) {
         struct timeval timeout;
         timeout.tv_sec = 0;
         timeout.tv_usec = 100000ULL;
         if(setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (void *) &timeout, sizeof(timeout)) < 0) {
            perror("setsockopt(SO_RCVTIMEO) failed");
         }
      }
      
#endif
      nframes = VLEN;
      
      while ((infinite || loopcount--) && running) {   

         if(VLEN > 1) {
            /* these settings may be modified by recvmmsg() */
            for (i = 0; i < nframes; i++) {
               iovecs[i].iov_len = sizeof(frames[0]);
               mmsghdrs[i].msg_hdr.msg_namelen = sizeof(addrs[0]);
               mmsghdrs[i].msg_hdr.msg_controllen = sizeof(ctrlmsgs[0]);
               mmsghdrs[i].msg_hdr.msg_flags = 0;
            }
         }
         
         if(VLEN == 1) {
            nbytes = read(s, &frame, clen);
            if (nbytes < 0) {
               perror("read");
               return 1;
            }
            nframes = 1;
         } else {
            struct timespec timeout = { 0, 100000000ULL }; //100ms
            nframes = recvmmsg(s, mmsghdrs, VLEN, MSG_WAITFORONE, &timeout);
            if (nframes <= 0) {
               // recvmmsg return -1 if timeout
               // errno == EAGAIN
               if(errno == EINTR) {
                  // Ignore this and read again.
                  if (verbose > 1) {
                     printf("recvmmsg() return EINTR\n");
                  }
               } else if(errno != EAGAIN) {
                  perror("recvmmsg()");
                  return 1;
               }
               continue;
            }
            if (verbose > 1) {
               struct timeval tnow;
               gettimeofday(&tnow, NULL);
               printf("recvmmsg() return %d frames %ld.%06ld\n", nframes, tnow.tv_sec, tnow.tv_usec);
            }
         }
         for (i = 0; i < nframes; i++) {
            if(VLEN > 1) {
               if ((size_t)mmsghdrs[i].msg_len == CAN_MTU)
                  maxdlen = CAN_MAX_DLEN;
               else if ((size_t)mmsghdrs[i].msg_len == CANFD_MTU)
                  maxdlen = CANFD_MAX_DLEN;
               else {
                  fprintf(stderr, "read: incomplete CAN frame\n");
                  return 1;
               }
            
               for (cmsg = CMSG_FIRSTHDR(&mmsghdrs[i].msg_hdr);
                    cmsg && (cmsg->cmsg_level == SOL_SOCKET);
                    cmsg = CMSG_NXTHDR(&mmsghdrs[i].msg_hdr,cmsg)) {
                  if (cmsg->cmsg_type == SO_TIMESTAMP)
                     tv = *(struct timeval *)CMSG_DATA(cmsg);
                  else if (cmsg->cmsg_type == SO_RXQ_OVFL)
                     dropcnt = *(__u32 *)CMSG_DATA(cmsg);
               }
            
               /* check for (unlikely) dropped frames on this specific socket */
               if (dropcnt != last_dropcnt) {
               
                  __u32 drops;
               
                  if (dropcnt > last_dropcnt)
                     drops = dropcnt - last_dropcnt;
                  else
                     drops = UINT32_MAX - last_dropcnt + dropcnt;
               
                  printf("DROPCOUNT: dropped %d CAN frame%s (total drops %d)\n",
                     drops, (drops > 1)?"s":"", dropcnt);
               
                  last_dropcnt = dropcnt;
               }
               frame.len = frames[i].can_dlc;
               memcpy(frame.data, frames[i].data, frame.len);


               
#if 1
               if(verbose_prints > 0) {
                  verbose_prints--;
                  struct timeval diff;
                  if (last_tv.tv_sec == 0)   /* first init */
                     last_tv = tv;
                  diff.tv_sec  = tv.tv_sec  - last_tv.tv_sec;
                  diff.tv_usec = tv.tv_usec - last_tv.tv_usec;
                  if (diff.tv_usec < 0)
                     diff.tv_sec--, diff.tv_usec += 1000000;
                  if (diff.tv_sec < 0)
                     diff.tv_sec = diff.tv_usec = 0;

                  int x, pos = 0;
                  for(x=0; x<frame.len; x++) {
                     pos += sprintf(&buf[pos], "%02X ", frames[i].data[x]);
                  }
                  
                  idx = idx2dindex(addrs[i].can_ifindex, s);
                  printf("[%010ld.%06ld](%010ld.%06ld) %*s %s\n",
                     tv.tv_sec, tv.tv_usec,
                     diff.tv_sec, diff.tv_usec,
                     max_devname_len, devname[idx], buf);
                  /*
                  printf("[%010ld.%06ld](%010ld.%06ld) idx=%d %s\n",
                     tv.tv_sec, tv.tv_usec,
                     diff.tv_sec, diff.tv_usec,
                     addrs[i].can_ifindex, buf);
                  */
               }
#endif
            }

         
            if (sequence_init) {
               sequence_init = 0;
               sequence = frame.data[0];
               last_sequence = sequence;
            }

            if (verbose > 2)
               printf("received frame. sequence number: %d\n", frame.data[0]);
         
            if(addr.can_ifindex == 0) {
               if(frame.data[0] == 0) {
                  //sequence = 255;
               }
               if(frame.data[0] == (unsigned char)(sequence+1)) {
                  // Allow data to be both sequence and sequence+1, but not sequence-1
                  sequence++;
                  if (verbose && !sequence) {
                     printf("sequence wrap around (%d)\n", seq_wrap++);
                     struct stat st;
                     if(stat(DEBUG_PRINT_FILE, &st) == 0) {
                        unlink(DEBUG_PRINT_FILE);
                        verbose_prints = 50;
                     }
                  }
               }
            }
            if (frame.data[0] != sequence) {
               printf("received wrong sequence count. expected: %02X, got: %02X %010ld.%06ld (last=%02X %010ld.%06ld)\n",
                  sequence,
                  frame.data[0], tv.tv_sec, tv.tv_usec,
                  last_sequence, last_tv.tv_sec, last_tv.tv_usec);
               if (quit) {
                  exit_value = EXIT_FAILURE;
                  break;
               }
               sequence = frame.data[0];
            }
            last_sequence = frame.data[0];

            if(verify_all_data) {
               int err = 0;
               int x;
               for(x=1; x<frame.len; x++) {
                  if(frame.data[x] && (frame.data[x] != ((unsigned char)(sequence+x)))) {
                     printf("received wrong sequence count. expected: %02X, got: %02X (x=%d)\n",
                        (sequence+x), frame.data[x], x);
                     err = 1;
                     break;
                  }
               }
               if(err) {
                  if (quit) {
                     exit_value = EXIT_FAILURE;
                     break;
                  }
               }
            }
            if(addr.can_ifindex > 0) {
               sequence++;
               if (verbose && !sequence) {
                  printf("sequence wrap around (%d)\n", seq_wrap++);
                  struct stat st;
                  if(stat(DEBUG_PRINT_FILE, &st) == 0) {
                     unlink(DEBUG_PRINT_FILE);
                     verbose_prints = 50;
                  }
               }
            }
            last_tv = tv;
         }
      }
   } else {
      struct canfd_frame framefd;
      memset(&framefd, 0, sizeof(struct canfd_frame));
      framefd.can_id = frame.can_id;
      framefd.len = frame.len;
      for(i=0; i<frame.len; i++) {
         framefd.data[i] = i;
      }
      //memcpy(framefd.data, frame.data, 8);

      unsigned char nr = 0;
      while ((infinite || loopcount--) && running) {
         ssize_t len;

         if (verbose > 1)
            printf("sending frame. sequence number: %d\n", sequence);

         if(sleep_delay > 0) {
            usleep(sleep_delay);
         }
   
      again:
         len = write(s, &framefd, clen);
         if (len == -1) {
            switch (errno) {
               case ENOBUFS: {
                  int err;
                  struct pollfd fds[] = {
                     {
                        .fd   = s,
                        .events   = POLLOUT,
                     },
                  };

                  if (!use_poll) {
                     perror("write");
                     exit(EXIT_FAILURE);
                  }

                  err = poll(fds, 1, 1000);
                  if (err == -1 && errno != -EINTR) {
                     perror("poll()");
                     exit(EXIT_FAILURE);
                  }
               }
               case EINTR:   /* fallthrough */
                  goto again;
               default:
                  perror("write");
                  exit(EXIT_FAILURE);
            }
         }

         nr++;
         for(i=0; i<frame.len; i++) {
            framefd.data[i] = (nr+i);
         }
         sequence++;

         if (verbose && !sequence)
            printf("sequence wrap around (%d)\n", seq_wrap++);
      }
   }

   exit(exit_value);
}
