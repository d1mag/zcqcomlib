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

#ifndef ZZENOCANCHANNEL_H_
#define ZZENOCANCHANNEL_H_

#include "zcanchannel.h"
#include "zthreadlocalstring.h"
#include "zzenotimersynch.h"
#include "zenocan.h"
#include "zring.h"

#include <condition_variable>
#include <atomic>
#include <mutex>

class ZZenoUSBDevice;
class ZZenoCANChannel : public ZCANChannel, public ZZenoTimerSynch {
public:
    ZZenoCANChannel(int _channel_index, ZZenoUSBDevice* _usb_can_device);
    ~ZZenoCANChannel() override;

    const std::string getObjectText() const override;
    const std::string getDevicetText() const override;
    const std::string getLastErrorText() override;
    int getChannelNr() override;
    bool open(int open_flags) override;
    bool close() override;
    uint32_t getCapabilites() override;
    bool busOn() override;
    bool busOff() override;
    bool setBusParameters(int bitrate, int sample_point, int sjw) override;
    bool setBusParametersFd(int bitrate, int sample_point, int sjw) override;
    bool setDriverMode(DriverMode driver_mode) override;
    ReadResult readWait(uint32_t& id, uint8_t *msg,
                        uint8_t& dlc, uint32_t& flags,
                        uint64_t& driver_timestmap_in_us,
                        int timeout_in_ms) override;
    SendResult send(const uint32_t id, const uint8_t *msg,
                    const uint8_t dlc, const uint32_t flags,
                    int timeout_in_ms) override;

    void setEventCallback(unsigned int notifyFlags, std::function<void(const EventData&)> callback) override;

    uint64_t getSerialNumber() override;
    uint32_t getFirmwareVersion() override;
    uint64_t getProductCode() override;

    int getBusLoad() override;

    void queueMessage(ZenoCAN20Message& message);
    void queueMessageCANFDP1(ZenoCANFDMessageP1& message_p1);
    void queueMessageCANFDP2(ZenoCANFDMessageP2& message_p2);
    void queueMessageCANFDP3(ZenoCANFDMessageP3& message_p3);
    void txAck(ZenoTxCANRequestAck& tx_ack);

    bool getDeviceTimeInUs(int64_t& timestamp_in_us) override;

    uint64_t getDeviceClock() override;

    ZCANDriver* getCANDriver() const override;

private:
    void flushRxFifo();
    void flushTxFifo();
    bool checkOpen();
    bool waitForSpaceInTxFifo(std::unique_lock<std::mutex>& lock, int& timeout_in_ms);
    bool getZenoDeviceTimeInUs(int64_t &timestamp_in_us);
    SendResult sendFD(const uint32_t id, const uint8_t *msg,
                      const uint8_t dlc, const uint32_t flags,
                      int timeout_in_ms);

    int channel_index;
    std::atomic<int> is_open;

    bool is_canfd_mode;
    ZZenoUSBDevice* usb_can_device;

    ZThreadLocalString last_error_text;

    /* RX logic */
    std::mutex rx_message_fifo_mutex;
    std::condition_variable rx_message_fifo_cond;

    /* TX logic */
    std::mutex tx_message_fifo_mutex;
    std::condition_variable tx_message_fifo_cond;
    int tx_request_count;
    int tx_next_trans_id;
    // int tx_request_received;
    unsigned int max_outstanding_tx_requests;

    struct FifoRxCANMessage {
        uint64_t timestamp;
        uint32_t id;
        uint32_t flags;
        uint8_t dlc;
        uint8_t data[64];
    };
    bool readFromRXFifo(FifoRxCANMessage& rx, int timeout_in_ms);

    struct FifoTxCANMessage {
        uint32_t id;
        uint32_t flags;
        uint8_t transaction_id;
        uint8_t dlc;
        uint8_t data[64];
    };

    void dispatchRXEvent(FifoRxCANMessage* rx_message);
    void dispatchTXEvent(FifoRxCANMessage* rx_message);

    ZenoCANFDMessageP1 canfd_msg_p1;
    ZenoCANFDMessageP2 canfd_msg_p2;

    ZRing<FifoRxCANMessage> rx_message_fifo;
    ZRing<FifoTxCANMessage> tx_message_fifo;

    /* Calculate bus load */
    int64_t bus_active_bit_count;
    int64_t last_measure_time_in_us;
    int current_bitrate;

    // int tx_id;

    uint64_t open_start_ref_timestamp_in_us;
    std::string usb_display_name;
    uint32_t serial_number;

    // std::mutex timer_adjust_mutex;
    unsigned int base_clock_divisor;

    std::function<void(const EventData&)> event_callback;
    unsigned int notify_flags;

    friend class ZZenoUSBDevice;
};

#endif /* ZZENOCANCHANNEL_H_ */
