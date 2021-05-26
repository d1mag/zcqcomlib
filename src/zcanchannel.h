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

#ifndef ZCANCHANNEL_H_
#define ZCANCHANNEL_H_

#include "zcanflags.h"
#include "zrefcountingobjbase.h"
#include <stdint.h>
#include <functional>
#include <string>

class ZCANDriver;
class ZCANChannel : public ZRefCountingObjBase,
                    public ZCANFlags
{
protected:
    ZCANChannel() { }

public:
    virtual const std::string getObjectText() const = 0;
    virtual const std::string getDevicetText() const = 0;
    virtual const std::string getLastErrorText() = 0;
    virtual int getChannelNr() = 0;
    virtual bool open(int open_flags) = 0;
    virtual bool close() = 0;
    virtual uint32_t getCapabilites() = 0;
    virtual bool busOn() = 0;
    virtual bool busOff() = 0;
    virtual bool setBusParameters(int bitrate, int sample_point, int sjw) = 0;
    virtual bool setBusParametersFd(int bitrate, int sample_point, int sjw) {
        /* Optionally implemented */
        ZUNUSED(bitrate)
        ZUNUSED(sample_point)
        ZUNUSED(sjw)

        return false;
    }
    virtual bool setDriverMode(DriverMode driver_mode) = 0;
    virtual ReadResult readWait(uint32_t& id, uint8_t *msg,
                                uint8_t& dlc, uint32_t& flags,
                                uint64_t& driver_timestmap_in_us,
                                int timeout_in_ms) = 0;

    virtual SendResult send(const uint32_t id, const uint8_t *msg,
                            const uint8_t dlc, const uint32_t flag,
                            int timeout_in_ms) = 0;

    enum EventTypeID {
        RX,
        TX,
        Error,
        Status,
    };

    struct EventData {
        EventTypeID event_type;
        uint64_t timetstamp;
        union {
            struct {
                uint32_t id;
                uint8_t msg[64];
                uint8_t dlc;
                uint32_t flags;
            } msg;
            struct {
                uint8_t bus_status;
                uint32_t tx_error_count;
                uint32_t rx_error_count;
            } status;
        } d;
    };

    virtual void setEventCallback(unsigned int notifyFlags, std::function<void(const EventData&)> callback) = 0;

    virtual uint64_t getDeviceClock() = 0;

    virtual uint64_t getSerialNumber() {
        /* Optionally implemented */
        return uint64_t(-1);
    }

    virtual uint32_t getFirmwareVersion() {
        /* Optionally implemented */
        return uint32_t(-1);
    }

    virtual uint64_t getProductCode() {
        /* Optionally implemented */
        return uint64_t(-1);
    }

    virtual int getBusLoad() = 0;

    virtual ZCANDriver* getCANDriver() const = 0;
protected:

};

#endif /* VXCANCHANNEL_H_ */
