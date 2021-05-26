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

#include "zglobal.h"
#ifdef Z_OS_WINDOWS
#define CANLIBAPI ZDECL_EXPORT
#endif

#include "canlib.h"
#include "zcqcore.h"
#include "zcanchannel.h"
#include "zdebug.h"
#include <string.h>
#include <mutex>

/*** ---------------------------==*+*+*==---------------------------------- ***/
#define CANLIB_PRODUCT_MAJOR_VERSION (8 - 3)
#define CANLIB_MAJOR_VERSION 8
#define CANLIB_MINOR_VERSION 26
#define CANLIB_BUILD_VERSION 214

/*** ---------------------------==*+*+*==---------------------------------- ***/
static const char *canlib_error_text_list[] = {
  "No error",                        // canOK
  "Error in parameter",              // canERR_PARAM
  "No messages available",           // canERR_NOMSG
  "Specified device not found",      // canERR_NOTFOUND
  "Out of memory",                   // canERR_NOMEM
  "No channels available",           // canERR_NOCHANNELS
  "Interrupted by signal",           // canERR_INTERRUPTED
  "Timeout occurred",                // canERR_TIMEOUT
  "Library not initialized",         // canERR_NOTINITIALIZED
  "No more handles",                 // canERR_NOHANDLES
  "Handle is invalid",               // canERR_INVHANDLE
  "Unknown error (-11)",             // canERR_INIFILE
  "CAN driver type not supported",   // canERR_DRIVER
  "Transmit buffer overflow",        // canERR_TXBUFOFL
  "Unknown error (-14)",             // canERR_RESERVED_1
  "A hardware error was detected",   // canERR_HARDWARE
  "Can not find requested DLL",      // canERR_DYNALOAD
  "DLL seems to be wrong version",   // canERR_DYNALIB
  "Error initializing DLL or driver", // canERR_DYNAINIT
  "Operation not supported by hardware or firmware", // canERR_NOT_SUPPORTED
  "Unknown error (-20)",             // canERR_RESERVED_5
  "Unknown error (-21)",             // canERR_RESERVED_6
  "Unknown error (-22)",             // canERR_RESERVED_2
  "Can not load or open the device driver", // canERR_DRIVERLOAD
  "The I/O request failed, probably due to resource shortage", //canERR_DRIVERFAILED
  "Unknown error (-25)",             // canERR_NOCONFIGMGR
  "Card not found",                  // canERR_NOCARD
  "Unknown error (-27)",             // canERR_RESERVED_7
  "Config not found",                // canERR_REGISTRY
  "The license is not valid",        // canERR_LICENSE
  "Internal error in the driver",    // canERR_INTERNAL
  "Access denied",                   // canERR_NO_ACCESS
  "Not implemented",                 // canERR_NOT_IMPLEMENTED
  "Device File error",               // canERR_DEVICE_FILE
  "Host File error",                 // canERR_HOST_FILE
  "Disk error",                      // canERR_DISK
  "CRC error",                       // canERR_CRC
  "Config error",                    // canERR_CONFIG
  "Memo failure",                    // canERR_MEMO_FAIL
  "Script error",                    // canERR_SCRIPT_FAIL
  "Script version mismatch",         // canERR_SCRIPT_WRONG_VERSION
  "Script container version mismatch",  // canERR_SCRIPT_TXE_CONTAINER_VERSION
  "Script container format error",   // canERR_SCRIPT_TXE_CONTAINER_FORMAT
  "Buffer provided too small to hold data",           // canERR_BUFFER_TOO_SMALL
  "I/O pin doesn't exist or I/O pin type mismatch",   // canERR_IO_WRONG_PIN_TYPE
  "I/O pin configuration is not confirmed",           // canERR_IO_NOT_CONFIRMED,
  "Configuration changed after last call to kvIoConfirmConfig",  // canERR_IO_CONFIG_CHANGED
  "The previous I/O pin value has not yet changed the output and is still pending", //canERR_IO_PENDING
};
/*** ---------------------------==*+*+*==---------------------------------- ***/

#define MAX_CANLIB_HANDLES 128
static ZRef<ZCANChannel> handle_map_list[MAX_CANLIB_HANDLES];
static std::mutex open_close_mutex;

static inline ZCANChannel* getChannel(CanHandle handle)
{
    if ( handle < 0 || handle >= MAX_CANLIB_HANDLES ) return nullptr;
    return handle_map_list[handle];
}

/*** ---------------------------==*+*+*==---------------------------------- ***/
void CANLIBAPI canInitializeLibrary (void)
{
    initializeZCQCommLibrary();
}

canStatus CANLIBAPI canClose (const CanHandle handle)
{
    std::lock_guard<std::mutex> lock(open_close_mutex);
    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    can_channel->close();
    handle_map_list[handle] = nullptr;

    return canOK;
}

canStatus CANLIBAPI canBusOn (const CanHandle handle)
{
    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    bool r = can_channel->busOn();
    if (!r) return canERR_INTERNAL;

    return canOK;
}

canStatus CANLIBAPI canBusOff (const CanHandle handle)
{
    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    bool r = can_channel->busOff();
    if (!r) return canERR_INTERNAL;

    return canOK;
}

canStatus CANLIBAPI canSetBusParams (const CanHandle handle,
                                     long freq,
                                     unsigned int tseg1,
                                     unsigned int tseg2,
                                     unsigned int sjw,
                                     unsigned int no_samp,
                                     unsigned int syncmode)
{
    ZUNUSED(tseg1)
    ZUNUSED(tseg2)
    ZUNUSED(no_samp)
    ZUNUSED(syncmode)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    int bitrate;
    switch(freq) {
    case canBITRATE_1M:
        bitrate = 1000000;
        break;
    case canBITRATE_500K:
        bitrate = 500000;
        break;
    case canBITRATE_250K:
        bitrate = 250000;
        break;
    case canBITRATE_125K:
        bitrate = 125000;
        break;
    case canBITRATE_100K:
        bitrate = 100000;
        break;
    case canBITRATE_62K:
        bitrate = 62000;
        break;
    case canBITRATE_50K:
        bitrate = 50000;
        break;
    case canBITRATE_83K:
        bitrate = 83000;
        break;
    case canBITRATE_10K:
        bitrate = 10000;
        break;
    default:
        return canERR_PARAM;
    }
    bool r = can_channel->setBusParameters(bitrate,70,int(sjw));
    if (!r) return canERR_INTERNAL;

    return canOK;
}

canStatus CANLIBAPI canSetBusParamsFd(const CanHandle handle,
                                      long freq_brs,
                                      unsigned int tseg1_brs,
                                      unsigned int tseg2_brs,
                                      unsigned int sjw_brs)
{
    ZUNUSED(freq_brs)
    ZUNUSED(tseg1_brs)
    ZUNUSED(tseg2_brs)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    int bitrate;
    switch(freq_brs) {
    case canFD_BITRATE_500K_80P:
        bitrate = 500000;
        break;
    case canFD_BITRATE_1M_80P:
        bitrate = 1000000;
        break;
    case canFD_BITRATE_2M_80P:
        bitrate = 2000000;
        break;
    case canFD_BITRATE_4M_80P:
        bitrate = 4000000;
        break;
    case canFD_BITRATE_8M_60P:
        bitrate = 8000000;
        break;
    default:
        return canERR_PARAM;
    }
    bool r = can_channel->setBusParametersFd(bitrate,70,int(sjw_brs));
    if (!r) return canERR_INTERNAL;

    return canOK;
}

canStatus CANLIBAPI canGetBusParams (const CanHandle handle,
                                     long  *freq,
                                     unsigned int *tseg1,
                                     unsigned int *tseg2,
                                     unsigned int *sjw,
                                     unsigned int *noSamp,
                                     unsigned int *syncmode)
{
    ZUNUSED(freq)
    ZUNUSED(tseg1)
    ZUNUSED(tseg2)
    ZUNUSED(sjw)
    ZUNUSED(noSamp)
    ZUNUSED(syncmode)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canGetBusParamsFd(const CanHandle handle,
                                      long  *freq_brs,
                                      unsigned int *tseg1_brs,
                                      unsigned int *tseg2_brs,
                                      unsigned int *sjw_brs)
{
    ZUNUSED(freq_brs)
    ZUNUSED(tseg1_brs)
    ZUNUSED(tseg2_brs)
    ZUNUSED(sjw_brs)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canSetBusOutputControl (const CanHandle handle,
                                            const unsigned int drivertype)
{
    ZUNUSED(drivertype)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}


canStatus CANLIBAPI canGetBusOutputControl (const CanHandle handle,
                                            unsigned int *drivertype)
{
    ZUNUSED(drivertype)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canAccept (const CanHandle handle,
                               const long envelope,
                               const unsigned int flags)
{
    ZUNUSED(envelope)
    ZUNUSED(flags)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canReadStatus (const CanHandle handle,
                                   unsigned long *const flags)
{
    ZUNUSED(flags)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canReadErrorCounters (const CanHandle handle,
                                          unsigned int *tx_err,
                                          unsigned int *rx_err,
                                          unsigned int *ov_err)
{
    ZUNUSED(tx_err)
    ZUNUSED(rx_err)
    ZUNUSED(ov_err)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canWrite (const CanHandle handle,
                              long id,
                              void *msg,
                              unsigned int dlc,
                              unsigned int flags)
{
    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    ZCANChannel::SendResult r;
    r = can_channel->send(static_cast<const uint32_t>(id),
                          static_cast<uint8_t*>(msg),
                          static_cast<uint8_t>(dlc),
                          flags, 0);

    canStatus status;
    switch(r) {
    case ZCANChannel::SendStatusOK:
        status = canOK;
        break;
    case ZCANChannel::SendTimeout:
        status = canERR_TIMEOUT;
        break;
    case ZCANChannel::TransmitBufferOveflow:
        status = canERR_TXBUFOFL;
        break;
    case ZCANChannel::SendInvalidParam:
        status = canERR_PARAM;
        break;
    case ZCANChannel::SendError:
        status = canERR_INTERNAL;
        break;
    }

    return status;
}

canStatus CANLIBAPI canWriteSync (const CanHandle handle,
                                  unsigned long timeout)
{
    ZUNUSED(timeout)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canRead (const CanHandle handle,
                             long *id,
                             void *msg,
                             unsigned int *dlc,
                             unsigned int *flags,
                             unsigned long *time)
{
    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    uint64_t __time;
    ZCANChannel::ReadResult r;
    r = can_channel->readWait(reinterpret_cast<uint32_t&>(*id),
                              reinterpret_cast<uint8_t*>(msg),
                              reinterpret_cast<uint8_t&>(*dlc),
                              *flags, __time, 0);
    if ( r != ZCANChannel::ReadStatusOK ) {
        if ( r == ZCANChannel::ReadTimeout ) return canERR_NOMSG;
        else return canERR_INTERNAL;
    }

    *time = static_cast<unsigned long>(__time);

    return canOK;
}

canStatus CANLIBAPI canReadWait (const CanHandle handle,
                                 long *id,
                                 void *msg,
                                 unsigned int  *dlc,
                                 unsigned int  *flags,
                                 unsigned long *time,
                                 unsigned long timeout)
{
    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    ZCANChannel::ReadResult r;
    uint64_t __time;
    uint8_t __dlc;
    *id = 0;
    r = can_channel->readWait(reinterpret_cast<uint32_t&>(*id),
                              reinterpret_cast<uint8_t*>(msg),
                              __dlc, *flags, __time, int(timeout));
    if ( r != ZCANChannel::ReadStatusOK ) {
        if ( r == ZCANChannel::ReadTimeout ) return canERR_TIMEOUT;
        else return canERR_INTERNAL;
    }

    *dlc = __dlc;
    *time = static_cast<unsigned long>(__time);

    return canOK;
}

canStatus CANLIBAPI canReadSpecific (const CanHandle handle, long id, void * msg,
                                     unsigned int * dlc, unsigned int * flag,
                                     unsigned long * time)
{
    ZUNUSED(id)
    ZUNUSED(msg)
    ZUNUSED(dlc)
    ZUNUSED(flag)
    ZUNUSED(time)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canReadSync (const CanHandle handle,
                                 unsigned long timeout)
{
    ZUNUSED(timeout)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canReadSyncSpecific (const CanHandle handle,
                                         long id,
                                         unsigned long timeout)
{
    ZUNUSED(id)
    ZUNUSED(timeout)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canReadSpecificSkip (const CanHandle handle,
                                         long id,
                                         void * msg,
                                         unsigned int * dlc,
                                         unsigned int * flag,
                                         unsigned long * time)
{
    ZUNUSED(id)
    ZUNUSED(msg)
    ZUNUSED(dlc)
    ZUNUSED(flag)
    ZUNUSED(time)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canSetNotify (const CanHandle handle,
                                  void (*callback)(canNotifyData *),
                                  unsigned int notifyFlags,
                                  void *tag)
{
    ZUNUSED(notifyFlags)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    if ( callback == nullptr ) {
        can_channel->setEventCallback(notifyFlags, std::function<void(const ZCANChannel::EventData&)>());
        return canOK;
    }

    can_channel->setEventCallback(notifyFlags, [=](const ZCANChannel::EventData& data) {
        canNotifyData can_notify_data;
        can_notify_data.tag = tag;

        switch(data.event_type) {
        case ZCANChannel::RX:
            can_notify_data.eventType = canEVENT_RX;
            can_notify_data.info.rx.id = long(data.d.msg.id);
            can_notify_data.info.rx.time = static_cast<unsigned long>(data.timetstamp);
            break;

        case ZCANChannel::TX:
            can_notify_data.eventType = canEVENT_TX;
            can_notify_data.info.tx.id = long(data.d.msg.id);
            can_notify_data.info.tx.time = static_cast<unsigned long>(data.timetstamp);
            break;

        case ZCANChannel::Error:
            can_notify_data.eventType = canEVENT_ERROR;
            can_notify_data.info.busErr.time = static_cast<unsigned long>(data.timetstamp);
            break;

        case ZCANChannel::Status:
            can_notify_data.eventType = canEVENT_STATUS;
            can_notify_data.info.status.time = static_cast<unsigned long>(data.timetstamp);
            can_notify_data.info.status.busStatus = data.d.status.bus_status;
            can_notify_data.info.status.rxErrorCounter = static_cast<unsigned char>(data.d.status.rx_error_count);
            can_notify_data.info.status.txErrorCounter = static_cast<unsigned char>(data.d.status.tx_error_count);
            break;

        default:
            return;
        }

        callback(&can_notify_data);
    });

    return canOK;
}

canStatus CANLIBAPI canGetRawHandle (const CanHandle handle, void *raw_handle_ptr)
{
    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    raw_handle_ptr = reinterpret_cast<void*>(can_channel);

    return canOK;
}

canStatus CANLIBAPI canTranslateBaud (long *const freq,
                                      unsigned int *const tseg1,
                                      unsigned int *const tseg2,
                                      unsigned int *const sjw,
                                      unsigned int *const nosamp,
                                      unsigned int *const sync_mode)
{
    if (!freq || !tseg1 || !tseg2 || !sjw  || !nosamp) return canERR_PARAM;
    if (sync_mode != nullptr) *sync_mode = 0;
    *nosamp = 1;

    switch (*freq) {
    case 1000000:
        *tseg1 = 30;
        *tseg2 = 7;
        *sjw   = 7;
        break;

    case 500000:
        *tseg1 = 62;
        *tseg2 = 15;
        *sjw   = 15;
        break;

    case 250000:
        *tseg1 = 126;
        *tseg2 = 31;
        *sjw   = 31;
        break;

    case 125000:
        *tseg1 = 254;
        *tseg2 = 63;
        *sjw   = 63;
        break;

    case 100000:
        *tseg1 = 16;
        *tseg2 = 7;
        *sjw   = 0;
        break;

    case 83333:
    case 83000:
        *tseg1 = 16;
        *tseg2 = 6;
        *sjw   = 0;
        break;

    case 62000:
        *tseg1 = 15;
        *tseg2 = 6;
        *sjw   = 3;
        break;

    case 50000:
        *tseg1 = 16;
        *tseg2 = 7;
        *sjw   = 0;
        break;

    case 33333:
        *tseg1 = 16;
        *tseg2 = 7;
        *sjw   = 0;
        break;

    case 10000:
        *tseg1 = 16;
        *tseg2 = 7;
        *sjw   = 0;
        break;

    default:
        return canERR_PARAM;
    }

    return canOK;
}

canStatus CANLIBAPI canGetErrorText (canStatus error_code, char *buffer,
                                     unsigned int buffer_size)
{
    unsigned int error_code_index;
    if (!buffer || buffer_size == 0) return canERR_PARAM;

    error_code_index = static_cast<unsigned int>(-error_code);
    if (error_code_index >= sizeof(canlib_error_text_list) / sizeof(const char *)) return canERR_PARAM;

    strncpy(buffer, canlib_error_text_list[error_code_index], buffer_size);

    return canOK;
}

unsigned short CANLIBAPI canGetVersion (void)
{
    return (CANLIB_MAJOR_VERSION << 8) + CANLIB_MINOR_VERSION;
}

canStatus CANLIBAPI canIoCtl (const CanHandle handle,
                              unsigned int func,
                              void *buf,
                              unsigned int buflen)
{
    ZUNUSED(func)
    ZUNUSED(buf)
    ZUNUSED(buflen)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canReadTimer (const CanHandle handle, unsigned long *time)
{
    ZUNUSED(time)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

CanHandle CANLIBAPI canOpenChannel (int channel, int flags)
{
    std::lock_guard<std::mutex> lock(open_close_mutex);

    ZRef<ZCANChannel> can_channel = getCANChannel(channel);
    if ( can_channel == nullptr ) return canERR_NOTFOUND;

    int handle = canINVALID_HANDLE;
    for(int i = 0; i < MAX_CANLIB_HANDLES; ++i) {
        if ( handle_map_list[i] == nullptr ) {
            handle = i;
            break;
        }
    }
    if ( handle == canERR_INVHANDLE ) {
        return canERR_NOHANDLES;
    }

    uint32_t capabilities = can_channel->getCapabilites();

    if ( (flags & canOPEN_REQUIRE_EXTENDED) &&
        !(capabilities & ZCANChannel::ExtendedCAN) ) {
        return canERR_NO_ACCESS;
    }

    int _flags = 0;

    if ( flags & canOPEN_CAN_FD ) {
        _flags |= ZCANChannel::CanFD;
    }

    if ( flags & canOPEN_CAN_FD_NONISO ) {
        _flags |= ZCANChannel::CanFD | ZCANChannel::CanFDNonISO;
    }

    if (!can_channel->open(_flags)) {
        return canERR_INTERNAL;
    }

    handle_map_list[handle] = can_channel;

    return handle;
}

canStatus CANLIBAPI canGetNumberOfChannels (int *channel_count)
{
    if ( channel_count == nullptr ) return canERR_PARAM;
    *channel_count = getNumberOfZCQCANChannels();

    return canOK;
}


canStatus CANLIBAPI canGetChannelData (int channel,
                                       int item,
                                       void *buffer,
                                       size_t buffer_size)
{
    if ( buffer == nullptr || buffer_size == 0 ) return canERR_PARAM;
    if  ( channel < 0 || channel >= getNumberOfZCQCANChannels() ) return canERR_PARAM;
    switch(item) {
    case canCHANNELDATA_CHANNEL_NAME: {
        std::string channel_text;
        if (getCANDeviceLocalChannelName(channel,channel_text) < 0)
            return canERR_INTERNAL;

        strncpy(static_cast<char*>(buffer), channel_text.c_str(), buffer_size - 1);
        return canOK;
    }

    case canCHANNELDATA_CHAN_NO_ON_CARD: {
        if (buffer_size < sizeof(int)) {
            return canERR_PARAM;
        }
        int channel_nr = getCANDeviceLocalChannelNr(channel);
        if ( channel_nr < 0 ) return canERR_INTERNAL;

        memcpy(buffer, &channel_nr, sizeof(int));
        return canOK;
    }

    case canCHANNELDATA_DEVDESCR_ASCII: {
        std::string device_description;
        if (getCANDeviceDescription(channel,device_description) < 0)
            return canERR_INTERNAL;

        strncpy(static_cast<char*>(buffer), device_description.c_str(), buffer_size - 1);
        return canOK;
    }
    case canCHANNELDATA_CARD_UPC_NO: {
        uint64_t *p = static_cast<uint64_t*>(buffer);
        if (buffer_size < 8) return canERR_PARAM;
        if (getCANDeviceProductCode(channel,*p) < 0)
            return canERR_INTERNAL;

        return canOK;
    }
    case canCHANNELDATA_CARD_FIRMWARE_REV: {
        uint32_t *p = static_cast<uint32_t*>(buffer);
        if (buffer_size < 8) return canERR_PARAM;

        uint32_t fw_version;
        if (getCANDeviceFWVersion(channel,fw_version) < 0)
            return canERR_INTERNAL;

        p[0] = (fw_version & 0xff);
        p[1] = ((fw_version >>  8) & 0xff) |
               (((fw_version >> 16) & 0xff) << 16);

        return canOK;
    }
    case canCHANNELDATA_CARD_SERIAL_NO: {
        uint64_t *p = static_cast<uint64_t*>(buffer);
        if (buffer_size < 8) return canERR_PARAM;
        if (getCANDeviceSerialNumber(channel,*p) < 0)
            return canERR_INTERNAL;

        return canOK;
    }
    case canCHANNELDATA_DRIVER_NAME:
        strncpy(static_cast<char*>(buffer), "zcqdrv", buffer_size - 1);
        return canOK;
    case canCHANNELDATA_MFGNAME_ASCII:
        strncpy(static_cast<char*>(buffer), "Zuragon LTD", buffer_size - 1);
        return canOK;

    case canCHANNELDATA_DLL_FILE_VERSION: {
        uint16_t *p = static_cast<uint16_t*>(buffer);
        if (buffer_size < 8) return canERR_PARAM;
        *p++ = 0;
        *p++ = CANLIB_BUILD_VERSION;
        *p++ = CANLIB_MINOR_VERSION;
        *p++ = CANLIB_MAJOR_VERSION;
        return canOK;
    }

    case canCHANNELDATA_DLL_PRODUCT_VERSION: {
        unsigned short *p = static_cast<unsigned short *>(buffer);
        if (buffer_size < 8) return canERR_PARAM;
        *p++ = 0;
        *p++ = 0;
        *p++ = CANLIB_MINOR_VERSION;
        *p++ = CANLIB_PRODUCT_MAJOR_VERSION;
        return canOK;
    }

    case canCHANNELDATA_TRANS_CAP:
    case canCHANNELDATA_TRANS_SERIAL_NO:
    case canCHANNELDATA_TRANS_UPC_NO:
    case canCHANNELDATA_DLL_FILETYPE:
    case canCHANNELDATA_DEVICE_PHYSICAL_POSITION:
    case canCHANNELDATA_UI_NUMBER:
    case canCHANNELDATA_TIMESYNC_ENABLED:
    case canCHANNELDATA_MFGNAME_UNICODE:
    case canCHANNELDATA_DEVDESCR_UNICODE:
    case canCHANNELDATA_CHANNEL_QUALITY:
    case canCHANNELDATA_ROUNDTRIP_TIME:
    case canCHANNELDATA_BUS_TYPE:
    case canCHANNELDATA_TIME_SINCE_LAST_SEEN:
    case canCHANNELDATA_DEVNAME_ASCII:
    case canCHANNELDATA_REMOTE_OPERATIONAL_MODE:
    case canCHANNELDATA_REMOTE_PROFILE_NAME:
    case canCHANNELDATA_REMOTE_HOST_NAME:
    case canCHANNELDATA_REMOTE_MAC:
      return canERR_NOT_IMPLEMENTED;
    }

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canSetBusParamsC200 (const CanHandle handle,
                                         unsigned char btr0,
                                         unsigned char btr1)
{
    ZUNUSED(btr0)
    ZUNUSED(btr1)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canSetDriverMode (const CanHandle handle,
                                      int line_mode, int res_net)
{
    ZUNUSED(line_mode)
    ZUNUSED(res_net)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canGetDriverMode (const CanHandle handle,
                                      int *line_mode, int *res_net)
{
    ZUNUSED(line_mode)
    ZUNUSED(res_net)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

unsigned int CANLIBAPI canGetVersionEx (unsigned int item_code)
{
    switch (item_code) {
    case canVERSION_CANLIB32_VERSION:
        return (CANLIB_MAJOR_VERSION << 8) + CANLIB_MINOR_VERSION;

    case canVERSION_CANLIB32_PRODVER:
        return (CANLIB_PRODUCT_MAJOR_VERSION << 8) + CANLIB_MINOR_VERSION;

    case canVERSION_CANLIB32_PRODVER32:
        return (CANLIB_MAJOR_VERSION << 16) +
               (CANLIB_MINOR_VERSION << 8);

    case canVERSION_CANLIB32_BETA:
        return 0;

    // default:
    //    return 0;
    }

    return canOK;
}

canStatus CANLIBAPI canObjBufFreeAll (const CanHandle handle)
{
    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufAllocate (const CanHandle handle, int type)
{
    ZUNUSED(type)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufFree (const CanHandle handle, int idx)
{
    ZUNUSED(idx)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufWrite (const CanHandle handle,
                                    int idx,
                                    int id,
                                    void* msg,
                                    unsigned int dlc,
                                    unsigned int flags)
{
    ZUNUSED(idx)
    ZUNUSED(id)
    ZUNUSED(msg)
    ZUNUSED(dlc)
    ZUNUSED(flags)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufSetFilter (const CanHandle handle,
                                        int idx,
                                        unsigned int code,
                                        unsigned int mask)
{
    ZUNUSED(idx)
    ZUNUSED(code)
    ZUNUSED(mask)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufSetFlags (const CanHandle handle,
                                       int idx,
                                       unsigned int flags)
{
    ZUNUSED(idx)
    ZUNUSED(flags)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufSetPeriod (const CanHandle handle,
                                        int idx,
                                        unsigned int period)
{
    ZUNUSED(idx)
    ZUNUSED(period)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufSetMsgCount (const CanHandle handle,
                                          int idx,
                                          unsigned int count)
{
    ZUNUSED(idx)
    ZUNUSED(count)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufEnable (const CanHandle handle, int idx)
{
    ZUNUSED(idx)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufDisable (const CanHandle handle, int idx)
{
    ZUNUSED(idx)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canObjBufSendBurst (const CanHandle handle,
                                        int idx,
                                        unsigned int burstlen)
{
    ZUNUSED(idx)
    ZUNUSED(burstlen)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canResetBus (const CanHandle handle)
{
    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canWriteWait (const CanHandle handle,
                                  long id,
                                  void *msg,
                                  unsigned int dlc,
                                  unsigned int flags,
                                  unsigned long timeout)
{
    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    ZCANChannel::SendResult r;
    r = can_channel->send(static_cast<const uint32_t>(id),
                          static_cast<uint8_t*>(msg),
                          static_cast<uint8_t>(dlc),
                          flags, int(timeout));

    canStatus status;
    switch(r) {
    case ZCANChannel::SendStatusOK:
        status = canOK;
        break;
    case ZCANChannel::SendTimeout:
        status = canERR_TIMEOUT;
        break;
    case ZCANChannel::TransmitBufferOveflow:
        status = canERR_TXBUFOFL;
        break;
    case ZCANChannel::SendInvalidParam:
        status = canERR_PARAM;
        break;
    case ZCANChannel::SendError:
        status = canERR_INTERNAL;
        break;
    }

    return status;
}

canStatus CANLIBAPI canUnloadLibrary (void)
{
    uninitializeZCQCommLibrary();
    return canOK;
}

canStatus CANLIBAPI canSetAcceptanceFilter (const CanHandle handle,
                                            unsigned int code,
                                            unsigned int mask,
                                            int is_extended)
{
    ZUNUSED(code)
    ZUNUSED(mask)
    ZUNUSED(is_extended)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canFlushReceiveQueue (const CanHandle handle)
{
    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canFlushTransmitQueue (const CanHandle handle)
{
    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvFlashLeds (const CanHandle handle, int action, int timeout)
{
    ZUNUSED(action)
    ZUNUSED(timeout)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canRequestChipStatus (const CanHandle handle)
{
    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canRequestBusStatistics (const CanHandle handle)
{
    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canGetBusStatistics (const CanHandle handle,
                                         canBusStatistics *stat,
                                         size_t bufsiz)
{
    ZUNUSED(stat)
    ZUNUSED(bufsiz)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI canGetHandleData (const CanHandle handle,
                                      int item,
                                      void *buffer,
                                      size_t bufsize)
{
    ZUNUSED(item)
    ZUNUSED(buffer)
    ZUNUSED(bufsize)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvTimeDomainCreate (kvTimeDomain *domain)
{
    ZUNUSED(domain)

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvTimeDomainDelete (kvTimeDomain domain)
{
    ZUNUSED(domain)

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvTimeDomainResetTime (kvTimeDomain domain)
{
    ZUNUSED(domain)

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvTimeDomainGetData (kvTimeDomain domain,
                                        kvTimeDomainData *data,
                                        size_t bufsiz)
{
    ZUNUSED(domain)
    ZUNUSED(data)
    ZUNUSED(bufsiz)

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvTimeDomainAddHandle(kvTimeDomain domain,
                                         const CanHandle handle)
{
    ZUNUSED(domain)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvTimeDomainRemoveHandle (kvTimeDomain domain,
                                             const CanHandle handle)
{
    ZUNUSED(domain)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvSetNotifyCallback (const CanHandle handle,
                                        kvCallback_t callback,
                                        void* context,
                                        unsigned int notifyFlags)
{
    ZUNUSED(callback)
    ZUNUSED(context)
    ZUNUSED(notifyFlags)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvGetSupportedInterfaceInfo (int index,
                                                char *hwName,
                                                size_t nameLen,
                                                int *hwType,
                                                int *hwBusType)
{
    ZUNUSED(index)
    ZUNUSED(hwName)
    ZUNUSED(nameLen)
    ZUNUSED(hwType)
    ZUNUSED(hwBusType)

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvReadDeviceCustomerData (const CanHandle handle,
                                             int userNumber,
                                             int itemNumber,
                                             void *data,
                                             size_t bufsiz)
{
    ZUNUSED(userNumber)
    ZUNUSED(itemNumber)
    ZUNUSED(data)
    ZUNUSED(bufsiz)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptStart (const CanHandle handle, int slotNo)
{
    ZUNUSED(slotNo)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptStop (const CanHandle handle, int slotNo, int mode)
{
    ZUNUSED(slotNo)
    ZUNUSED(mode)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptUnload (const CanHandle handle, int slotNo)
{
    ZUNUSED(slotNo)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptSendEvent (const CanHandle handle,
                                      int slotNo,
                                      int eventType,
                                      int eventNo,
                                      unsigned int data)
{
    ZUNUSED(slotNo)
    ZUNUSED(eventType)
    ZUNUSED(eventNo)
    ZUNUSED(data)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

kvEnvHandle CANLIBAPI kvScriptEnvvarOpen (const CanHandle handle,
                                          char* envvarName,
                                          int *envvarType,
                                          int *envvarSize)
{
    ZUNUSED(envvarName)
    ZUNUSED(envvarType)
    ZUNUSED(envvarSize)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return -1;
}

kvStatus CANLIBAPI kvScriptEnvvarClose (kvEnvHandle e_handle)
{
    ZUNUSED(e_handle)

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptEnvvarSetInt (kvEnvHandle e_handle, int val)
{
    ZUNUSED(e_handle)
    ZUNUSED(val)

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptEnvvarGetInt (kvEnvHandle e_handle, int *val)
{
    ZUNUSED(e_handle)
    ZUNUSED(val)

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptEnvvarSetFloat (kvEnvHandle e_handle, float val)
{
    ZUNUSED(e_handle)
    ZUNUSED(val)

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptEnvvarGetFloat (kvEnvHandle e_handle, float *val)
{
    ZUNUSED(e_handle)
    ZUNUSED(val)

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptEnvvarSetData (kvEnvHandle e_handle,
                                          void *buf,
                                          int start_index,
                                          int data_len)
{
    ZUNUSED(e_handle)
    ZUNUSED(buf)
    ZUNUSED(start_index)
    ZUNUSED(data_len)

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptEnvvarGetData (kvEnvHandle e_handle,
                                          void *buf,
                                          int start_index,
                                          int data_len)
{
    ZUNUSED(e_handle)
    ZUNUSED(buf)
    ZUNUSED(start_index)
    ZUNUSED(data_len)

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptLoadFileOnDevice (const CanHandle handle,
                                             int slotNo,
                                             char *localFile);

kvStatus CANLIBAPI kvScriptLoadFile (const CanHandle handle,
                                     int slotNo,
                                     char *filePathOnPC)
{
    ZUNUSED(slotNo)
    ZUNUSED(filePathOnPC)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptStatus(const CanHandle handle,
                                  int  slot,
                                  unsigned int *status)
{
    ZUNUSED(slot)
    ZUNUSED(status)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptGetMaxEnvvarSize(int hnd, int *envvarSize)
{
    ZUNUSED(hnd)
    ZUNUSED(envvarSize)

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvScriptTxeGetData(const char *filePathOnPC,
                                      int item,
                                      void *buffer,
                                      unsigned int *bufsize)
{
    ZUNUSED(filePathOnPC)
    ZUNUSED(item)
    ZUNUSED(buffer)
    ZUNUSED(bufsize)

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvFileCopyToDevice (const CanHandle handle,
                                       char *hostFileName,
                                       char *deviceFileName)
{
    ZUNUSED(hostFileName)
    ZUNUSED(deviceFileName)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvFileCopyFromDevice (const CanHandle handle,
                                         char *deviceFileName,
                                         char *hostFileName)
{
    ZUNUSED(deviceFileName)
    ZUNUSED(hostFileName)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvFileDelete (const CanHandle handle, char *deviceFileName)
{
    ZUNUSED(deviceFileName)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvFileGetName (const CanHandle handle,
                                  int fileNo,
                                  char *name,
                                  int namelen)
{
    ZUNUSED(fileNo)
    ZUNUSED(name)
    ZUNUSED(namelen)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvFileGetCount (const CanHandle handle, int *count)
{
    ZUNUSED(count)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvFileGetSystemData (const CanHandle handle,
                                        int itemCode,
                                        int *result)
{
    ZUNUSED(itemCode)
    ZUNUSED(result)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvDeviceSetMode (const CanHandle handle, int mode)
{
    ZUNUSED(mode)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvDeviceGetMode (const CanHandle handle, int *result)
{
    ZUNUSED(result)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvReadTimer (const CanHandle handle, unsigned int *time)
{
    ZUNUSED(time)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

kvStatus CANLIBAPI kvReadTimer64 (const CanHandle handle, uint64_t *time)
{
    ZUNUSED(time)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoGetNumberOfPins (const CanHandle handle, unsigned int *pinCount)
{
    ZUNUSED(pinCount)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoPinGetInfo (const CanHandle handle, unsigned int pin, int item, void *buffer, const unsigned int bufsize)
{
    ZUNUSED(pin)
    ZUNUSED(item)
    ZUNUSED(buffer)
    ZUNUSED(bufsize)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoPinSetInfo (const CanHandle handle, unsigned int pin, int item, const void *buffer, const unsigned int bufsize)
{
    ZUNUSED(pin)
    ZUNUSED(item)
    ZUNUSED(buffer)
    ZUNUSED(bufsize)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoPinSetDigital (const CanHandle handle, unsigned int pin, unsigned int value)
{
    ZUNUSED(pin)
    ZUNUSED(value)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoPinGetDigital (const CanHandle handle, unsigned int pin, unsigned int *value)
{
    ZUNUSED(pin)
    ZUNUSED(value)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoPinGetOutputDigital (const CanHandle handle, unsigned int pin, unsigned int *value)
{
    ZUNUSED(pin)
    ZUNUSED(value)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoPinSetRelay (const CanHandle handle, unsigned int pin, unsigned int value)
{
    ZUNUSED(pin)
    ZUNUSED(value)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoPinGetOutputRelay (const CanHandle handle, unsigned int pin, unsigned int *value)
{
    ZUNUSED(pin)
    ZUNUSED(value)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoPinSetAnalog (const CanHandle handle, unsigned int pin, float value)
{
    ZUNUSED(pin)
    ZUNUSED(value)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoPinGetAnalog (const CanHandle handle, unsigned int pin, float* value)
{
    ZUNUSED(pin)
    ZUNUSED(value)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}

canStatus CANLIBAPI kvIoPinGetOutputAnalog (const CanHandle handle, unsigned int pin, float* value)
{
    ZUNUSED(pin)
    ZUNUSED(value)

    auto can_channel = getChannel(handle);
    if ( can_channel == nullptr ) return canERR_INVHANDLE;

    return canERR_NOT_IMPLEMENTED;
}
