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

#include "zzenotimersynch.h"
#include "zdebug.h"
#include <algorithm>
#include <cmath>

ZZenoTimerSynch::ZZenoTimerSynch(uint64_t _TIMER_wrap_around_mask,
                                 uint64_t _TIMER_wrap_around_step)
    : event_msb_timestamp_part(0),
      TIMER_wrap_around_mask(_TIMER_wrap_around_mask),
      TIMER_wrap_around_step(_TIMER_wrap_around_step)
{
    /* no op */
}

ZZenoTimerSynch::~ZZenoTimerSynch()
{
    /*  no op */
}

void ZZenoTimerSynch::initializeDeviceTimeDrift()
{
    last_driver_timestamp_in_us = ZTimeVal();
    last_event_timestamp_in_us = ZTimeVal();
    last_appl_timestamp_in_us = ZTimeVal();
    synch_offset = ZTimeVal();
    timer_msb_timestamp_part = 0;
    event_msb_timestamp_part = 0;
}

int64_t ZZenoTimerSynch::caluclateTimeStamp(const int64_t driver_timestamp_in_us, const double drift_factor)
{
    int64_t timestamp_in_us = driver_timestamp_in_us;
    adjustEventTimestampWrapAround(timestamp_in_us);

    timestamp_in_us += synch_offset.count();
    double drift = double(timestamp_in_us) * drift_factor;
    timestamp_in_us -= int64_t(drift);

    // zDebug("Drift %Lf - factor %Lf synch_offset %lld", drift, drift_factor, synch_offset.count());

    // zDebug("last_appl_timestamp_in_us %lld %lld %lld",
    //        last_appl_timestamp_in_us.count(),
    //        timestamp_in_us,
    //        driver_timestamp_in_us);

    if (last_appl_timestamp_in_us.count() > timestamp_in_us) {
        last_appl_timestamp_in_us ++; /* Advance 1us */
        timestamp_in_us = last_appl_timestamp_in_us.count();
    }
    else last_appl_timestamp_in_us = ZTimeVal(timestamp_in_us);

    return timestamp_in_us;
}


void ZZenoTimerSynch::onReadTimeoutCheck()
{
    /* NOTE: this is very unlikely to happen, it would only happen
     * if no event, that means, no FlexRay event and no CAN event
     * in around 40 min, and the 32bit timer in micro-seconds wraps around,
     * then this could happen.
     * In that rare case, we need to prevent that the timestamps
     * on events has it's MSB bits incorrect */
    if (timer_msb_timestamp_part > event_msb_timestamp_part) {
        event_msb_timestamp_part = timer_msb_timestamp_part;
        last_event_timestamp_in_us = last_driver_timestamp_in_us;
    }
}

void ZZenoTimerSynch::synchToTimerOffset(ZZenoTimerSynch::ZTimeVal t)
{
    zDebug("Synch to %ld", t.count());
    synch_offset = t;
}

void ZZenoTimerSynch::adjustDeviceTimerWrapAround(int64_t& timer_timestamp_in_us)
{
    /* Check if timestamp has wrapped around */
    timer_timestamp_in_us &= TIMER_wrap_around_mask; /* Truncate to 32 bits */
    auto diff = timer_timestamp_in_us - last_driver_timestamp_in_us.count();

    if (diff < 0) {
        zDebug("Device timer has wrapped around: %Lx - %Lx - %Lx",
               last_driver_timestamp_in_us.count(),
               timer_timestamp_in_us,
               last_driver_timestamp_in_us.count());

        timer_msb_timestamp_part += TIMER_wrap_around_step;
    }

    last_driver_timestamp_in_us = ZTimeVal(timer_timestamp_in_us);

    timer_timestamp_in_us += timer_msb_timestamp_part;
}

void ZZenoTimerSynch::adjustEventTimestampWrapAround(int64_t& event_timestamp_in_us)
{
    /* Check if timestamp has wrapped around */
    event_timestamp_in_us &= TIMER_wrap_around_mask; /* Truncate to 32 bits */
    auto diff = event_timestamp_in_us - last_event_timestamp_in_us.count();

    if (diff < 0) {
        /* Check sanity of timestamp before wrap around */
        int64_t time_in_us;
        if (!getDeviceTimeInUs(time_in_us)) {
            zError("FATAL: could not get time from device, everything is screwed up");
            return;
        }

        int64_t t = event_timestamp_in_us + int64_t(event_msb_timestamp_part + TIMER_wrap_around_step);
        // t -= initial_time_adjust_in_us.count();

        /* Timestamp should not differ too much from device-timestamp -
         * or something is really screwed up, let's test with 5 seconds, just to be safe */
        if ( std::abs(time_in_us-t) > 5000000) {
            /* Just make the timestamp go forward */
            event_timestamp_in_us = last_event_timestamp_in_us.count()+1;
            last_event_timestamp_in_us = ZTimeVal(event_timestamp_in_us & int64_t(TIMER_wrap_around_mask));

            event_timestamp_in_us += event_msb_timestamp_part;
            // event_timestamp_in_us -= initial_time_adjust_in_us.count();
            return;
        }

        zDebug("Event timestamp has wrapped around");
        event_msb_timestamp_part += TIMER_wrap_around_step;
    }

    last_event_timestamp_in_us = ZTimeVal(event_timestamp_in_us);

    event_timestamp_in_us += event_msb_timestamp_part;
    // event_timestamp_in_us -= initial_time_adjust_in_us.count();
}

ZZenoTimerSynch::ZTimeVal ZZenoTimerSynch::systemTimeNow() const
{
    return std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch();
}
