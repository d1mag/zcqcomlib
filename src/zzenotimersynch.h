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

#ifndef ZZENOTIMERSYNCH_H
#define ZZENOTIMERSYNCH_H

#include "zglobal.h"
#include <chrono>
#include <stdint.h>

class ZZenoTimerSynch {
public:
    typedef std::chrono::microseconds ZTimeVal;

    ZZenoTimerSynch(uint64_t _TIMER_wrap_around_mask =  0xffffffff,
                    uint64_t _TIMER_wrap_around_step = 0x100000000);
    virtual ~ZZenoTimerSynch();

    bool adjustInitialDeviceTimeDrift();

    void ajdustDeviceTimeDrift(ZTimeVal device_time_in_us, ZTimeVal new_drift_time_in_us, ZTimeVal max_adjust);

    virtual bool getDeviceTimeInUs(int64_t& timestamp_in_us) = 0;

    ZTimeVal getTimeDriftInUs() const {
        return time_drift_in_us;
    }

    ZTimeVal getLastDriverTimeStampInUs() const {
        return last_driver_timestamp_in_us;
    }    

    int64_t caluclateTimeStamp(const int64_t driver_timestamp_in_us);
    void onReadTimeoutCheck();

    void synchToTimerOffset(ZTimeVal t);

protected:
    void adjustDeviceTimerWrapAround(int64_t& timer_timestamp_in_us);
    void adjustEventTimestampWrapAround(int64_t& event_timestamp_in_us);
    ZTimeVal systemTimeNow() const;

    /* Timer drift */
    ZTimeVal time_drift_in_us;
    ZTimeVal last_driver_timestamp_in_us;
    ZTimeVal last_t0_timestamp_in_us;
    ZTimeVal last_device_timestamp_in_us;
    ZTimeVal last_event_timestamp_in_us;
    uint64_t timer_msb_timestamp_part;
    uint64_t event_msb_timestamp_part;
    double   drift_factor;
    ZTimeVal synch_offset;

    ZTimeVal average_round_trip_in_us;
    ZTimeVal last_appl_timestamp_in_us;

    // VxReference<VxTimerBase> drift_timer;

    /* Wrap around masks */
    const uint64_t TIMER_wrap_around_mask;
    const uint64_t TIMER_wrap_around_step;
};

#endif /* ZZENOTIMERSYNCH_H */
