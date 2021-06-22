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

#include <thread>
#include <iostream>
#include <canlib.h>
#include <mutex>

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

static std::mutex printf_mutex;
static std::chrono::microseconds t0;

static void check_canlib_error(const char* id, canStatus stat)
{
  if (stat == canOK) return;

  char buf[64];
  buf[0] = '\0';
  canGetErrorText(stat, buf, sizeof(buf));
  printf("%s: failed, stat=%d (%s)\n", id, int(stat), buf);
}

void rx_worker(int channel)
{
    std::cout << "RX thread started on CAN channel:" << channel << std::endl;

    canHandle hnd;
    canStatus stat;

    /* Open channel, set parameters and go on bus */
    hnd = canOpenChannel(channel, canOPEN_EXCLUSIVE | canOPEN_REQUIRE_EXTENDED | canOPEN_ACCEPT_VIRTUAL);
    if (hnd < 0) {
        printf("ERROR: failed to open channel %d\n", channel);
        check_canlib_error("canOpenChannel", canStatus(hnd));
        return;
    }

    stat = canSetBusParams(hnd, canBITRATE_500K, 0, 0, 0, 0, 0);
    check_canlib_error("canSetBusParams", stat);
    if (stat != canOK) {
        canClose(hnd);
        return;
    }

    stat = canBusOn(hnd);
    check_canlib_error("canBusOn", stat);
    if (stat != canOK) {
        canClose(hnd);
        return;
    }

    unsigned int msg_counter = 0;
    do {
      long id;
      unsigned char msg[8];
      unsigned int dlc;
      unsigned int flag;
      unsigned long time;

      stat = canReadWait(hnd, &id, &msg, &dlc, &flag, &time, unsigned(-1));
      time -= static_cast<unsigned long>(t0.count());

      if (stat != canOK) {
          check_canlib_error("\ncanReadWait", stat);
          continue;
      }

      msg_counter++;
      std::lock_guard<std::mutex> lock(printf_mutex);
      if (flag & canMSG_ERROR_FRAME)
          printf("(%u) ERROR FRAME", msg_counter);
      else {
          unsigned j;

          printf("Ch%d (%u) id:%ld dlc:%u data: ", channel, msg_counter, id, dlc);
          if (dlc > 8) {
              dlc = 8;
          }
          for (j = 0; j < dlc; j++) {
              printf("%2.2x ", msg[j]);
          }
      }

      printf("flags:0x%x time:%lu\n", flag, time);
    } while (stat == canOK);

    stat = canBusOff(hnd);
    check_canlib_error("canBusOff", stat);

    stat = canClose(hnd);
    check_canlib_error("canClose", stat);
}

int main(int argc, char **argv)
{
    canInitializeLibrary();

    t0 = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch();

    std::thread rx_thread1(rx_worker, 0);
    std::thread rx_thread2(rx_worker, 1);
    std::thread rx_thread3(rx_worker, 2);
    std::thread rx_thread4(rx_worker, 3);
    std::thread rx_thread5(rx_worker, 4);
    std::thread rx_thread6(rx_worker, 5);

    rx_thread1.join();
    rx_thread2.join();
    rx_thread3.join();
    rx_thread4.join();
    rx_thread5.join();
    rx_thread6.join();

    canStatus stat = canUnloadLibrary();
    check_canlib_error("canUnloadLibrary", stat);

    return 0;
}
