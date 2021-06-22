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

#include "zcqcore.h"
#include "zzenocandriver.h"
#include "zzenolindriver.h"
#include <cassert>
#include <vector>

/*** ---------------------------==*+*+*==---------------------------------- ***/
class ZCQCore {
public:
  ZCQCore();

  std::vector<ZRef<ZCANDriver> > can_driver_list;
  std::vector<ZRef<ZCANChannel> > can_channel_list;
  std::vector<ZRef<ZLINChannel> > lin_channel_list;
};

static std::mutex init_mutex;
static ZCQCore* __instance = nullptr;

/*** ---------------------------==*+*+*==---------------------------------- ***/
ZCQCore::ZCQCore()
{
    ZRef<ZZenoCANDriver> zeno_can_driver = new ZZenoCANDriver();
    can_driver_list.push_back(zeno_can_driver.cast<ZCANDriver>());

    int number_of_channels = zeno_can_driver->getNumberOfChannels();
    for(int i = 0; i < number_of_channels; ++i) {
        ZRef<ZCANChannel> can_channel = zeno_can_driver->getChannel(i);
        can_channel_list.push_back(can_channel);
    }
    can_channel_list.shrink_to_fit();

    ZRef<ZZenoLINDriver> zeno_lin_driver = zeno_can_driver->getZenoLINDriver();
    if ( zeno_lin_driver != nullptr ) {
        number_of_channels = zeno_lin_driver->getNumberOfChannels();
        for(int i = 0; i < number_of_channels; ++i) {
            ZRef<ZLINChannel> lin_channel = zeno_lin_driver->getChannel(i);
            lin_channel_list.push_back(lin_channel);
        }
        lin_channel_list.shrink_to_fit();
    }
}

/*** ---------------------------==*+*+*==---------------------------------- ***/
void initializeZCQCommLibrary()
{
    std::lock_guard<std::mutex> lock(init_mutex);
    if ( __instance == nullptr ) {
        __instance = new ZCQCore();
    }
}

void uninitializeZCQCommLibrary()
{
    std::lock_guard<std::mutex> lock(init_mutex);
    if ( __instance != nullptr ) {
        delete __instance;
        __instance = nullptr;
    }
}

ZCANChannel* getCANChannel(unsigned can_channel_index)
{
    if ( __instance == nullptr ) return nullptr;
    if (can_channel_index >= __instance->can_channel_list.size()) return nullptr;

    return __instance->can_channel_list[can_channel_index].get();
}

unsigned getNumberOfZCQCANChannels()
{
    if ( __instance == nullptr ) return 0;
    return unsigned(__instance->can_channel_list.size());
}

unsigned getNumberOfZCQLINChannels()
{
    if ( __instance == nullptr ) return 0;
    return unsigned(__instance->lin_channel_list.size());
}

int getCANDeviceLocalChannelNr(int can_channel_index)
{
    if ( __instance == nullptr ) return -1;
    assert(can_channel_index >=0);
    assert(can_channel_index < int(__instance->can_channel_list.size()));

    return __instance->can_channel_list[unsigned(can_channel_index)]->getChannelNr();
}

int getCANDeviceLocalChannelName(int can_channel_index, std::string& channel_name)
{
    if ( __instance == nullptr ) return -1;
    assert(can_channel_index >=0);
    assert(can_channel_index < int(__instance->can_channel_list.size()));

    channel_name = __instance->can_channel_list[unsigned(can_channel_index)]->getObjectText();
    return 0;
}

int getCANDeviceDescription(int can_channel_index, std::string& device_description)
{
    if ( __instance == nullptr ) return -1;
    assert(can_channel_index >=0);
    assert(can_channel_index < int(__instance->can_channel_list.size()));

    device_description = __instance->can_channel_list[unsigned(can_channel_index)]->getDevicetText();

    return 0;
}

int getCANDeviceFWVersion(int can_channel_index, uint32_t& fw_version)
{
    if ( __instance == nullptr ) return -1;
    assert(can_channel_index >=0);
    assert(can_channel_index < int(__instance->can_channel_list.size()));

    fw_version = __instance->can_channel_list[unsigned(can_channel_index)]->getFirmwareVersion();

    return 0;
}

int getCANDeviceProductCode(int can_channel_index, uint64_t& product_code)
{
    if ( __instance == nullptr ) return -1;
    assert(can_channel_index >=0);
    assert(can_channel_index < int(__instance->can_channel_list.size()));

    product_code = __instance->can_channel_list[unsigned(can_channel_index)]->getProductCode();

    return 0;
}

int getCANDeviceSerialNumber(int can_channel_index, uint64_t& serial_number)
{
    if ( __instance == nullptr ) return -1;
    assert(can_channel_index >=0);
    assert(can_channel_index < int(__instance->can_channel_list.size()));

    serial_number = __instance->can_channel_list[unsigned(can_channel_index)]->getSerialNumber();

    return 0;
}
