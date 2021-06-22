/*
 *             Copyright 2020 by Morgan
 *
 * This software BSD-new. See the included COPYING file for details.
 *
 * License: BSD-new
 * ==============================================================================
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the \<organization\> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "linlib.h"

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

void linInitializeLibrary()
{
    
}

void linUnloadLibrary()
{
    
}

LinStatus linGetTransceiverData(int channel,
                                unsigned char eanNo[],
                                unsigned char serNo[],
                                int* ttype)
{
    return linERR_NOT_IMPLEMENTED;
}

LinHandle linOpenChannel(int channel, int flags)
{
    return linERR_NOT_IMPLEMENTED;
}

LinStatus linClose(LinHandle h)
{
    return linERR_NOT_IMPLEMENTED;
}

LinStatus linGetVersion(int* major, int* minor, int* build)
{
    return linERR_NOT_IMPLEMENTED;
}

LinStatus linGetFirmwareVersion(LinHandle h, unsigned char* bootVerMajor, unsigned char* bootVerMinor, unsigned char* bootVerBuild, unsigned char* appVerMajor, unsigned char* appVerMinor, unsigned char* appVerBuild)
{
    return linERR_NOT_IMPLEMENTED;
}

LinStatus linGetChannelData(int channel, int item, void* buffer, size_t bufsize)
{
    return linERR_NOT_IMPLEMENTED;
}

LinStatus linSetBitrate(LinHandle h, unsigned int bps)
{
    return linERR_NOT_IMPLEMENTED;
}

LinStatus linBusOn(LinHandle h)
{
    return linERR_NOT_IMPLEMENTED;
}

LinStatus linBusOff(LinHandle h)
{
    return linERR_NOT_IMPLEMENTED;
}

unsigned long linReadTimer(LinHandle h)
{
    return static_cast<unsigned long>(linERR_NOT_IMPLEMENTED);
}

LinStatus linWriteMessage(LinHandle h,
                          unsigned int id,
                          const void* msg,
                          unsigned int dlc)
{
    return linERR_NOT_IMPLEMENTED;
}

LinStatus linRequestMessage(LinHandle h, unsigned int id)
{
    return linERR_NOT_IMPLEMENTED;
}

LinStatus linReadMessage(LinHandle h,
                         unsigned int* id,
                         void* msg,
                         unsigned int* dlc,
                         unsigned int* flags,
                         LinMessageInfo* msgInfo)
{
    return linERR_NOT_IMPLEMENTED;
}

LinStatus linReadMessageWait(LinHandle h,
                             unsigned int* id,
                             void* msg,
                             unsigned int* dlc,
                             unsigned int* flags,
                             LinMessageInfo* msgInfo,
                             unsigned long timeout)
{
    return linERR_NOT_IMPLEMENTED;
}

LinStatus linUpdateMessage(LinHandle h, unsigned int id, const void* msg, unsigned int dlc)
{
    return linERR_NOT_IMPLEMENTED;
}

LinStatus linSetupIllegalMessage(LinHandle h, unsigned int id, unsigned int cFlags, unsigned int delay)
{
    return linERR_NOT_IMPLEMENTED;
}

LinStatus linSetupLIN(LinHandle h, unsigned int lFlags, unsigned int bps)
{
    return linERR_NOT_IMPLEMENTED;
}

LinStatus linWriteWakeup(LinHandle h, unsigned int count, unsigned int interval)
{
    return linERR_NOT_IMPLEMENTED;
}
