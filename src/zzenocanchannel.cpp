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

#include "zzenocanchannel.h"
#include "zzenousbdevice.h"
#include "zzenocandriver.h"
#include "zdebug.h"
#include <string.h>
#include <algorithm>

ZZenoCANChannel::ZZenoCANChannel(int _channel_index,
                                 ZZenoUSBDevice* _usb_can_device)
    : channel_index(_channel_index),
      is_open(false),is_canfd_mode(false),
      initial_timer_adjustment_done(false),
      usb_can_device(_usb_can_device),
      tx_request_count(0), tx_next_trans_id(0),
      max_outstanding_tx_requests(31),
      rx_message_fifo(2048),
      tx_message_fifo(1024),
      bus_active_bit_count(0),
      last_measure_time_in_us(0),
      current_bitrate(0),
      // tx_id(0),
      open_start_ref_timestamp_in_us(0),
      usb_display_name(usb_can_device->getObjectText()),
      serial_number(usb_can_device->getSerialNumber()),
      base_clock_divisor(1)
{
    // connect(usb_can_device, &VxZenoUSBDevice::destroyed, [this]() {
    //     usb_can_device = nullptr;
    // });

    memset(&canfd_msg_p1, 0 , sizeof(canfd_msg_p1));
    memset(&canfd_msg_p2, 0 , sizeof(canfd_msg_p2));
}

ZZenoCANChannel::~ZZenoCANChannel()
{
    if (is_open) close();
}

const std::string ZZenoCANChannel::getObjectText() const
{
    std::string channel_name = usb_display_name + ' ';

    channel_name += "Ch" + std::to_string(channel_index+1) + ' ';
    channel_name += '(' + std::to_string(serial_number) + ')';

    return channel_name;
}

const std::string ZZenoCANChannel::getDevicetText() const
{
    return usb_can_device->getObjectText();
}

const std::string ZZenoCANChannel::getLastErrorText()
{
    return last_error_text;
}

int ZZenoCANChannel::getChannelNr()
{
    return channel_index;
}

bool ZZenoCANChannel::open(int open_flags)
{
    is_open++;

    if (is_open.load() > 1) {
        is_open--;
        last_error_text = "CAN Channel " + std::to_string(channel_index+1) + " is already open";
        return false;
    }

    if ( open_flags & ZCANFlags::SharedMode ) {
        last_error_text = "Shared mode not supported on this CAN channel";
        return false;
    }

    flushRxFifo();
    flushTxFifo();

    if (!usb_can_device->open()) {
        is_open--;
        return false;
    }


    ZenoOpen cmd;
    ZenoOpenResponse reply;

    memset(&cmd,0,sizeof(ZenoOpen));
    cmd.h.cmd_id = ZENO_CMD_OPEN;
    cmd.channel = uint8_t(channel_index);
    cmd.base_clock_divisor = uint8_t(usb_can_device->getClockResolution() / 1000);

    if ( open_flags & ZCANFlags::CanFD ) {
        cmd.can_fd_mode = 1;
        is_canfd_mode = true;
    }
    else is_canfd_mode = false;

    if ( open_flags & ZCANFlags::CanFDNonISO ) {
        cmd.can_fd_non_iso = 1;
    }

    if (!usb_can_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = usb_can_device->getLastErrorText();
        zCritical("(ZenoUSB) Ch%d failed to open Zeno CAN channel: %s", channel_index+1, last_error_text.c_str());
        close();
        return false;
    }

    // last_device_timestamp_in_us = 0;
    max_outstanding_tx_requests = reply.max_pending_tx_msgs;
    open_start_ref_timestamp_in_us = uint64_t(reply.clock_start_ref);
    initial_timer_adjustment_done = 0;
    base_clock_divisor = std::min(uint(reply.base_clock_divisor),1u);

    zDebug("Zeno - max outstanding TX: %d Base clock divisor: %d", reply.max_pending_tx_msgs, base_clock_divisor);

    return true;
}

bool ZZenoCANChannel::close()
{
    busOff();

    ZenoClose cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoClose));
    cmd.h.cmd_id = ZENO_CMD_CLOSE;
    cmd.channel = uint8_t(channel_index);

    if (!usb_can_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = usb_can_device->getLastErrorText();
        zCritical("(ZenoUSB) Ch%d failed to close Zeno CAN channel: %s", channel_index+1, last_error_text.c_str());
    }

    is_canfd_mode = false;
    current_bitrate = 0;
    usb_can_device->close();
    is_open--;

    return true;
}

uint32_t ZZenoCANChannel::getCapabilites()
{
    uint32_t capabilities =
             GenerateError |
             ErrorCounters |
             BusStatistics |
             ErrorCounters |
             ExtendedCAN   |
             TxRequest     |
             TxAcknowledge;

    if ( channel_index < 4) {
        capabilities |= CanFD | CanFDNonISO;
    }

    return capabilities;
}

bool ZZenoCANChannel::busOn()
{
    zDebug("ZenoCAN Ch%d Bus On", channel_index+1);
    if (!checkOpen()) return false;

    ZenoBusOn cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoBusOn));
    cmd.h.cmd_id = ZENO_CMD_BUS_ON;
    cmd.channel = uint8_t(channel_index);

    if (!usb_can_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = usb_can_device->getLastErrorText();
        zCritical("(ZenoUSB) Ch%d failed to go bus on: %s", channel_index+1, last_error_text.c_str());
        return false;
    }

    /* Bus load statistic */
    auto t_now = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch();
    last_measure_time_in_us = t_now.count();
    bus_active_bit_count = 0;

    return true;
}

bool ZZenoCANChannel::busOff()
{
    zDebug("ZenoCAN Ch%d Bus Off", channel_index+1);
    if (!checkOpen()) return false;

    ZenoBusOff cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoBitTiming));
    cmd.h.cmd_id = ZENO_CMD_BUS_OFF;
    cmd.channel = uint8_t(channel_index);

    if (!usb_can_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = usb_can_device->getLastErrorText();
        zCritical("(ZenoUSB) Ch%d failed to go bus off: %s", channel_index+1, last_error_text.c_str());
        return false;
    }

    return true;
}

bool ZZenoCANChannel::setBusParameters(int bitrate, int sample_point, int sjw)
{
    ZUNUSED(sample_point)
    ZUNUSED(sjw)

    if (!checkOpen()) return false;

    ZenoBitTiming cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoBitTiming));
    cmd.h.cmd_id = ZENO_CMD_SET_BIT_TIMING;
    cmd.channel = uint8_t(channel_index);

    switch (bitrate) {
    case 1000000:
        cmd.brp   = 0;
        cmd.tseg1 = 30;
        cmd.tseg2 = 7;
        cmd.sjw   = 7;

        if ( channel_index >= 4) {
            /* NOTE: 1mbit is not supported on the Zeno CANquatro 42x
             * due to the internal clock of 70Mhz, there no possible way
             * to configure the ECAN to 1mbit */
            last_error_text =  "1Mbit bitrate is not supported on this channel";
            return false;
        }

        /* ECAN1 and ECAN2 */
        cmd.cancks = 1;
        cmd.cicfg1 = 0x0002;
        cmd.cicfg2 = 0x02A0;
        break;

    case 500000:
        cmd.brp   = 0;
        cmd.tseg1 = 62;
        cmd.tseg2 = 15;
        cmd.sjw   = 15;

        /* ECAN1 and ECAN2 */
        cmd.cancks = 1;
        cmd.cicfg1 = 0x0006;
        cmd.cicfg2 = 0x01A1;
        break;

    case 250000:
        cmd.brp   = 0;
        cmd.tseg1 = 126;
        cmd.tseg2 = 31;
        cmd.sjw   = 31;

        /* ECAN1 and ECAN2 */
        cmd.cancks = 1;
        cmd.cicfg1 = 0x000D;
        cmd.cicfg2 = 0x01A1;
        break;

    case 125000:
        cmd.brp   = 0;
        cmd.tseg1 = 254;
        cmd.tseg2 = 63;
        cmd.sjw   = 63;

        /* ECAN1 and ECAN2 */
        cmd.cancks = 1;
        cmd.cicfg1 = 0x001B;
        cmd.cicfg2 = 0x01A1;
        break;

    case 100000:
        cmd.brp   = 14;
        cmd.tseg1 = 16;
        cmd.tseg2 = 7;
        cmd.sjw   = 0;

        /* ECAN1 and ECAN2 */
        cmd.cancks = 1;
        cmd.cicfg1 = 0x0022;
        cmd.cicfg2 = 0x01A1;
        break;

    case 83333:
    case 83000:
        cmd.brp   = 18;
        cmd.tseg1 = 16;
        cmd.tseg2 = 6;
        cmd.sjw   = 0;

        /* ECAN1 and ECAN2 */
        cmd.cancks = 1;
        cmd.cicfg1 = 0x0029;
        cmd.cicfg2 = 0x01A1;
        break;

    case 62000:
        cmd.brp   = 26;
        cmd.tseg1 = 15;
        cmd.tseg2 = 6;
        cmd.sjw   = 3;

        /* ECAN1 and ECAN2 */
        cmd.cancks = 1;
        cmd.cicfg1 = 0x002E;
        cmd.cicfg2 = 0x02A9;
        break;

    case 50000:
        cmd.brp   = 30;
        cmd.tseg1 = 16;
        cmd.tseg2 = 7;
        cmd.sjw   = 0;

        /* ECAN1 and ECAN2 */
        cmd.cancks = 1;
        cmd.cicfg1 = 0x0031;
        cmd.cicfg2 = 0x01B3;
        break;

    case 33333:
        cmd.brp   = 46;
        cmd.tseg1 = 16;
        cmd.tseg2 = 7;
        cmd.sjw   = 0;

        /* ECAN1 and ECAN2 */
        cmd.cancks = 1;
        cmd.cicfg1 = 0x003D;
        cmd.cicfg2 = 0x04BA;
        break;

    case 10000:
        cmd.brp   = 156;
        cmd.tseg1 = 16;
        cmd.tseg2 = 7;
        cmd.sjw   = 0;

        /* ECAN1 and ECAN2 */
        cmd.cancks = 0;
        cmd.cicfg1 = 0x00FF;
        cmd.cicfg2 = 0x07BF;
        break;

    default:
        last_error_text = "Unsupported bitrate: " + std::to_string(bitrate);
        return false;
    }

    zDebug("ZenoCAN Ch%d Bus params BRP:%d TSEG1:%d TSEG2:%d SJW:%d", channel_index+1, cmd.brp, cmd.tseg1, cmd.tseg2, cmd.sjw);

    if (!usb_can_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = usb_can_device->getLastErrorText();
        zCritical("(ZenoUSB) Ch%d failed to set bit timings: %s", channel_index+1, last_error_text.c_str());
        return false;
    }

    current_bitrate = bitrate;

    return true;
}

bool ZZenoCANChannel::setBusParametersFd(int bitrate, int sample_point, int sjw)
{
    ZUNUSED(sample_point)
    ZUNUSED(sjw)

    if (!checkOpen()) return false;

    ZenoBitTiming cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoBitTiming));
    cmd.h.cmd_id = ZENO_CMD_SET_DATA_BIT_TIMING;
    cmd.channel = uint8_t(channel_index);

    switch (bitrate) {
    case 1000000:
        cmd.brp = 0;
        cmd.tseg1 = 30;
        cmd.tseg2 = 7;
        cmd.sjw = 7;
        // SSP
        cmd.tdc_offset = 31;
        cmd.tdc_value = 0;
        break;
    case 2000000:
        // Data BR
        cmd.brp = 0;
        cmd.tseg1 = 14;
        cmd.tseg2 = 3;
        cmd.sjw = 3;
        // SSP
        cmd.tdc_offset = 15;
        cmd.tdc_value = 0;
        break;
    case 3000000:
        // Data BR
        cmd.brp = 0;
        cmd.tseg1 = 8;
        cmd.tseg2 = 2;
        cmd.sjw = 2;
        // SSP
        cmd.tdc_offset = 9;
        cmd.tdc_value = 0;
        break;
    case 4000000:
        // Data BR
        cmd.brp = 0;
        cmd.tseg1 = 6;
        cmd.tseg2 = 1;
        cmd.sjw = 1;
        // SSP
        cmd.tdc_offset = 7;
        cmd.tdc_value = 0;
        break;
    case 5000000:
        // Data BR
        cmd.brp = 0;
        cmd.tseg1 = 4;
        cmd.tseg2 = 1;
        cmd.sjw = 1;
        // SSP
        cmd.tdc_offset = 5;
        cmd.tdc_value = 0;
        break;
    case 6700000:
        // Data BR
        cmd.brp = 0;
        cmd.tseg1 = 3;
        cmd.tseg2 = 0;
        cmd.sjw = 0;
        // SSP
        cmd.tdc_offset = 4;
        cmd.tdc_value = 0;
        break;
    case 8000000:
        // Data BR
        cmd.brp = 0;
        cmd.tseg1 = 2;
        cmd.tseg2 = 0;
        cmd.sjw = 0;
        // SSP
        cmd.tdc_offset = 3;
        cmd.tdc_value = 1;
        break;
    case 10000000:
        // Data BR
        cmd.brp = 0;
        cmd.tseg1 = 1;
        cmd.tseg2 = 0;
        cmd.sjw = 0;
        // SSP
        cmd.tdc_offset = 2;
        cmd.tdc_value = 0;
        break;

    case 500000:
        cmd.brp = 1;
        cmd.tseg1 = 30;
        cmd.tseg2 = 7;
        cmd.sjw = 7;
        // SSP
        cmd.tdc_offset = 31;
        cmd.tdc_value = 0;
        cmd.tdc_ssp_mode_off = 1;
        break;
    case 833000:
        cmd.brp = 1;
        cmd.tseg1 = 17;
        cmd.tseg2 = 4;
        cmd.sjw = 4;
        // SSP
        cmd.tdc_offset = 18;
        cmd.tdc_value = 0;
        cmd.tdc_ssp_mode_off = 1;
        break;
    case 1500000:
        cmd.brp = 0;
        cmd.tseg1 = 18;
        cmd.tseg2 = 5;
        cmd.sjw = 5;
        // SSP
        cmd.tdc_offset = 19;
        cmd.tdc_value = 0;
        break;
    default:
        last_error_text = "Unsupported data-bitrate: " + std::to_string(bitrate);
        return false;
    }

    zDebug("ZenoCAN Ch%d data timing params BRP:%d TSEG1:%d TSEG2:%d SJW:%d", channel_index+1, cmd.brp, cmd.tseg1, cmd.tseg2, cmd.sjw);

    if (!usb_can_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = usb_can_device->getLastErrorText();
        zCritical("(ZenoUSB) Ch%d failed to set data bit timings: %s", channel_index+1, last_error_text.c_str());
        return false;
    }

    return true;
}

bool ZZenoCANChannel::setDriverMode(ZCANFlags::DriverMode driver_mode)
{
    ZUNUSED(driver_mode)

    /* TODO: implement me .... */
    return true;
}

bool ZZenoCANChannel::readFromRXFifo(ZZenoCANChannel::FifoRxCANMessage& rx,
                                     int timeout_in_ms)
{
    std::unique_lock<std::mutex> lock(rx_message_fifo_mutex);

    if ( rx_message_fifo.isEmpty()) {
        std::chrono::milliseconds timeout(timeout_in_ms);

        /* Wait for RX FIFO */
        rx_message_fifo_cond.wait_for(lock,timeout);

        /* If RX FIFO is still empty */
        if ( rx_message_fifo.isEmpty() ) {
            rx_message_fifo_mutex.unlock();
            return false;
        }
    }

    rx = rx_message_fifo.read();
    return false;
}


ZCANFlags::ReadResult ZZenoCANChannel::readWait(uint32_t& id, uint8_t *msg,
                                                uint8_t& dlc, uint32_t& flags,
                                                uint64_t& driver_timestmap_in_us,
                                                int timeout_in_ms)
{
    FifoRxCANMessage rx;
    if (!readFromRXFifo(rx, timeout_in_ms)) return ReadTimeout;

    flags = 0;
    id = long(rx.id);
    int msg_bit_count = 0;
    if ( rx.flags & ZenoCANFlagExtended ) {
        flags |= Extended;
        msg_bit_count = 38 + 25;
    } else if ( rx.flags & ZenoCANFlagStandard ) {
        flags |= Standard;
        msg_bit_count = 19 + 25;
    }

    if (rx.flags & ZenoCANErrorFrame ) {
        flags |= ErrorFrame | InternalFrame;
        flags |= (rx.flags & ZenoCANErrorMask);
    }
    if (rx.flags & ZenoCANErrorHWOverrun)
        flags |= ErrorHWOverrun;

    if (rx.flags & ZenoCANFlagTxAck)
        flags |= TxMsgAcknowledge;

    if ( is_canfd_mode ) {
        if (rx.flags & ZenoCANFlagFD)
            flags |= CanFDFrame;

        if (rx.flags & ZenoCANFlagFDBRS)
            flags |= CanFDBitrateSwitch;
        //TODO: Check flag CanFDESI
    }

    dlc = std::min(uint(rx.dlc),uint(is_canfd_mode ? 64 : 8));
    msg_bit_count += dlc * 8;
    bus_active_bit_count += msg_bit_count;

    driver_timestmap_in_us = uint64_t(rx.timestamp / base_clock_divisor);

    memcpy(msg, rx.data, rx.dlc);

    return ReadStatusOK;
}

ZCANFlags::SendResult ZZenoCANChannel::send(const uint32_t id,
                                            const uint8_t *msg,
                                            const uint8_t dlc,
                                            const uint32_t flags,
                                            int timeout_in_ms)
{

    if (!checkOpen()) return SendError;
    if (is_canfd_mode && (flags &  ZCANChannel::CanFDFrame))
        return sendFD(id,msg,dlc,flags,timeout_in_ms);
    // qDebug() << "ZENO_CMD_CAN20_TX_REQUEST" << hex << id;;

    ZenoTxCAN20Request request;
    request.h.cmd_id = ZENO_CMD_CAN20_TX_REQUEST;
    request.channel = uint8_t(channel_index);
    request.h.transaction_id = tx_next_trans_id & 0x7f;
    tx_next_trans_id ++;

    request.id = uint32_t(id);
    request.dlc = dlc & 0x0F;
    request.flags = 0;

    if (flags & Extended) {
        request.flags |= ZenoCANFlagExtended;
    } else if (flags & Standard ){
        request.flags |= ZenoCANFlagStandard;
    } else {
        return SendInvalidParam;
    }

    memcpy(request.data, msg, 8);

    std::unique_lock<std::mutex> tx_lock;
    tx_request_count ++;
    if ( tx_request_count >= int(max_outstanding_tx_requests) ) {
        if (timeout_in_ms > 0) {
            if ( !waitForSpaceInTxFifo(tx_lock, timeout_in_ms) ) {
                last_error_text = "Timeout request waiting for space in transmit buffer";
                tx_request_count --;

                return SendTimeout;
            }
        } else {
            last_error_text ="Transmit request buffer overflow";
            tx_request_count --;

            return TransmitBufferOveflow;
        }
    }

    if (! usb_can_device->queueTxRequest(zenoRequest(request), timeout_in_ms)) {
        last_error_text = usb_can_device->getLastErrorText();
        tx_request_count --;

        return SendError;
    }

    // qDebug() << " enqueue" << (tx_next_trans_id & 0x7f) << tx_request_count;
    // qDebug() << " enqueue " << request.cmdIOPSeq.transId;

    FifoTxCANMessage* tx_request = tx_message_fifo.writePtr();
    tx_request->id = uint32_t(id);
    tx_request->flags = request.flags;
    tx_request->transaction_id = request.h.transaction_id;
    tx_request->dlc = request.dlc;
    memcpy(tx_request->data, msg, 8);

    tx_message_fifo.write();

    return SendStatusOK;
}

ZCANFlags::SendResult ZZenoCANChannel::sendFD(const uint32_t id,
                                               const uint8_t *msg,
                                               const uint8_t dlc,
                                               const uint32_t flags,
                                               int timeout_in_ms)
{
    bool dlc_valid = false;
    if ( dlc <= 8 ) {
        dlc_valid = true;
    } else {
        switch(dlc) {
        case 12:
        case 16:
        case 20:
        case 24:
        case 32:
        case 48:
        case 64:
            dlc_valid = true;
            break;
        default:
            dlc_valid = false;
        }
    }

    if (!dlc_valid) {
        last_error_text = "Invalid parameter, allowed data length can be one of the following 0-8, 12, 16, 20, 24, 32, 48, 64.";
        return SendInvalidParam;
    }

    ZenoTxCANFDRequestP1 p1_request;
    p1_request.h.cmd_id = ZENO_CMD_CANFD_P1_TX_REQUEST;
    p1_request.channel = uint8_t(channel_index);
    p1_request.h.transaction_id = tx_next_trans_id & 0x7f;
    tx_next_trans_id ++;

    p1_request.id = uint32_t(id);
    p1_request.dlc = uint8_t(dlc);
    p1_request.flags = 0;

    if (flags & Extended) {
        p1_request.flags |= ZenoCANFlagExtended;
    } else if (flags & Standard ){
        p1_request.flags |= ZenoCANFlagStandard;
    } else {
        return SendInvalidParam;
    }
    if ( flags & CanFDBitrateSwitch ) {
        p1_request.flags |= ZenoCANFlagFDBRS;
    }

    memcpy(p1_request.data, msg, std::min(dlc, uint8_t(20)));

    std::unique_lock<std::mutex> tx_lock(tx_message_fifo_mutex);

    tx_request_count ++;
    if ( tx_request_count >= int(max_outstanding_tx_requests) ) {
        if (timeout_in_ms > 0) {
            if ( !waitForSpaceInTxFifo(tx_lock, timeout_in_ms) ) {
                last_error_text = "Timeout request waiting for space in transmit buffer";
                tx_request_count --;

                return SendTimeout;
            }
        } else {
            last_error_text ="Transmit request buffer overflow";
            tx_request_count --;

            return TransmitBufferOveflow;
        }
    }

    if (! usb_can_device->queueTxRequest(zenoRequest(p1_request), timeout_in_ms)) {
        last_error_text = usb_can_device->getLastErrorText();
        tx_request_count --;

        return SendError;
    }

    if ( dlc > 20 ) {
        ZenoTxCANFDRequestP2 p2_request;
        p2_request.h.cmd_id = ZENO_CMD_CANFD_P2_TX_REQUEST;
        p2_request.channel = uint8_t(channel_index);
        p2_request.h.transaction_id = p1_request.h.transaction_id;
        p2_request.dlc = p1_request.dlc;

        memcpy(p2_request.data, msg + 20, std::min(dlc-20, 28));

        if (! usb_can_device->queueTxRequest(zenoRequest(p2_request), timeout_in_ms)) {
            last_error_text = usb_can_device->getLastErrorText();
            tx_request_count --;

            return SendError;
        }

        if ( dlc > 48 ) {
            ZenoTxCANFDRequestP3 p3_request;
            p3_request.h.cmd_id = ZENO_CMD_CANFD_P3_TX_REQUEST;
            p3_request.channel = uint8_t(channel_index);
            p3_request.h.transaction_id = p1_request.h.transaction_id;
            p3_request.dlc = p1_request.dlc;

            memcpy(p3_request.data, msg + 48, std::min(dlc-48, 16));

            if (! usb_can_device->queueTxRequest(zenoRequest(p3_request), timeout_in_ms)) {
                last_error_text = usb_can_device->getLastErrorText();
                tx_request_count --;

                return SendError;
            }
        }
    }

    // qDebug() << " enqueue" << (tx_next_trans_id & 0x7f) << tx_request_count;
    // qDebug() << " enqueue " << request.cmdIOPSeq.transId;
    // tx_message_fifo.write(request);

    FifoTxCANMessage* tx_request = tx_message_fifo.writePtr();
    tx_request->id = uint32_t(id);
    tx_request->flags = p1_request.flags;
    tx_request->transaction_id = p1_request.h.transaction_id;
    tx_request->dlc = p1_request.dlc;
    memcpy(tx_request->data, msg, std::min(dlc, uint8_t(64)));

    tx_message_fifo.write();

    return SendStatusOK;
}


uint64_t ZZenoCANChannel::getSerialNumber()
{
    uint64_t unique_device_nr = serial_number;
    unique_device_nr <<= 8;
    unique_device_nr |= (channel_index & 0xff);

    return serial_number;
}

uint32_t ZZenoCANChannel::getFirmwareVersion()
{
    return usb_can_device->getFWVersion();
}

uint64_t ZZenoCANChannel::getProductCode()
{
    uint64_t product_code = 0x4ced7a332a86e499UL;
    return product_code;
}

int ZZenoCANChannel::getBusLoad()
{
    auto t_now = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch();
    uint64_t t0 = t_now.count();
    uint64_t delta_time_in_us = t0 - last_measure_time_in_us;
    delta_time_in_us = std::max(delta_time_in_us, uint64_t(1));

    // qDebug() << " busload delta time " << delta_time_in_ms << " active " << bus_active_bit_count << " rate " << bitrate;

    uint64_t busload = (bus_active_bit_count * 100000000) / (current_bitrate * delta_time_in_us);
    last_measure_time_in_us = t0;
    bus_active_bit_count = 0;
    if ( busload > 100 ) busload = 100;

    return int(busload);
}

void ZZenoCANChannel::queueMessage(ZenoCAN20Message& message)
{
    std::lock_guard<std::mutex> lock(rx_message_fifo_mutex);

    if ( rx_message_fifo.available() == 0 ) {
        zDebug("ZenoCAN Ch%d Zeno: RX buffer overflow: %d - %d", channel_index+1,rx_message_fifo.count(), rx_message_fifo.isEmpty());
        return;
    }

    rx_message_fifo_cond.notify_one();

    FifoRxCANMessage* rx_message = rx_message_fifo.writePtr();
    rx_message->timestamp = message.timestamp | (uint64_t(message.timestamp_msb) << 32);
    rx_message->id        = message.id;
    rx_message->flags     = message.flags;
    rx_message->dlc       = message.dlc;
    memcpy(rx_message->data, message.data,8);

    rx_message_fifo.write();
}

void ZZenoCANChannel::queueMessageCANFDP1(ZenoCANFDMessageP1 &message_p1)
{
    if ( message_p1.dlc <= 18 ) {
        std::lock_guard<std::mutex> lock(rx_message_fifo_mutex);

        if ( rx_message_fifo.available() == 0 ) {
            zDebug("ZenoCAN Ch%d P1 -Zeno: RX buffer overflow: %d - %d", channel_index+1,rx_message_fifo.count(), rx_message_fifo.isEmpty());
            return;
        }

        rx_message_fifo_cond.notify_one();

        FifoRxCANMessage* rx_message = rx_message_fifo.writePtr();
        rx_message->timestamp = message_p1.timestamp;
        rx_message->id        = message_p1.id;
        rx_message->flags     = message_p1.flags;
        rx_message->dlc       = message_p1.dlc;
        memcpy(rx_message->data, message_p1.data, message_p1.dlc);

        rx_message_fifo.write();
    }
    else {
        canfd_msg_p1 = message_p1;
    }
}

void ZZenoCANChannel::queueMessageCANFDP2(ZenoCANFDMessageP2 &message_p2)
{
    if ( canfd_msg_p1.dlc <= 46 ) {
        std::lock_guard<std::mutex> lock(rx_message_fifo_mutex);

        if ( rx_message_fifo.available() == 0 ) {
            zDebug("ZenoCAN Ch%d P2 -Zeno: RX buffer overflow: %d - %d", channel_index+1,rx_message_fifo.count(), rx_message_fifo.isEmpty());
            return;
        }

        rx_message_fifo_cond.notify_one();

        FifoRxCANMessage* rx_message = rx_message_fifo.writePtr();
        rx_message->timestamp = canfd_msg_p1.timestamp;
        rx_message->id        = canfd_msg_p1.id;
        rx_message->flags     = canfd_msg_p1.flags;
        rx_message->dlc       = canfd_msg_p1.dlc;

        memcpy(rx_message->data,      canfd_msg_p1.data, 18);
        memcpy(rx_message->data + 18, message_p2.data,   canfd_msg_p1.dlc - 18);
        rx_message_fifo.write();
    }
    else {
        canfd_msg_p2 = message_p2;
    }
}

void ZZenoCANChannel::queueMessageCANFDP3(ZenoCANFDMessageP3 &message_p3)
{
    // QByteArray b = QByteArray::fromRawData(reinterpret_cast<char*>(message_p3.data), int(18));
    // qDebug() << "P3 data" << b.toHex('-');

    if ( canfd_msg_p1.dlc < 46 ) {
        memset(&canfd_msg_p1, 0 , sizeof(canfd_msg_p1));
        memset(&canfd_msg_p2, 0 , sizeof(canfd_msg_p2));
        return;
    }

    std::lock_guard<std::mutex> lock(rx_message_fifo_mutex);

    if ( rx_message_fifo.available() == 0 ) {
        zDebug("ZenoCAN Ch%d P3 -Zeno: RX buffer overflow: %d - %d", channel_index+1,rx_message_fifo.count(), rx_message_fifo.isEmpty());
        return;
    }

    rx_message_fifo_cond.notify_one();

    FifoRxCANMessage* rx_message = rx_message_fifo.writePtr();
    rx_message->timestamp = canfd_msg_p1.timestamp;
    rx_message->id        = canfd_msg_p1.id;
    rx_message->flags     = canfd_msg_p1.flags;
    rx_message->dlc       = canfd_msg_p1.dlc;

    memcpy(rx_message->data,      canfd_msg_p1.data, 18);
    memcpy(rx_message->data + 18, canfd_msg_p2.data, 28);
    memcpy(rx_message->data + 46, message_p3.data,   canfd_msg_p1.dlc - 46);

    rx_message_fifo.write();

    memset(&canfd_msg_p1, 0 , sizeof(canfd_msg_p1));
    memset(&canfd_msg_p2, 0 , sizeof(canfd_msg_p2));
}

void ZZenoCANChannel::txAck(ZenoTxCANRequestAck& tx_ack)
{
    tx_message_fifo_mutex.lock();
    tx_message_fifo_cond.notify_one();

    // static int count = 0;

    int tx_count = int(tx_message_fifo.count());

    for (int i = 0; i < tx_count ; ++i ) {
        FifoTxCANMessage m = tx_message_fifo.read();

        if ( m.transaction_id == tx_ack.trans_id )  {
            FifoRxCANMessage message;

            message.id = m.id;
            message.dlc = m.dlc;
            message.flags = uint8_t(m.flags | ZenoCANFlagTxAck);
            // message.dlc = tx_ack.dlc;
            message.timestamp = tx_ack.timestamp | (uint64_t(tx_ack.timestamp_msb) << 32);
            memcpy(message.data, m.data, m.dlc);

            tx_request_count --;
            assert(tx_request_count >= 0);

            tx_message_fifo_mutex.unlock();

            if ( tx_ack.flags & ZenoCANErrorFrame ) {
                zDebug("ZenoCAN Ch%d TX failed, remove pending TX", channel_index+1);;
                flushTxFifo();
                return;
            }

            rx_message_fifo_mutex.lock();

            if ( rx_message_fifo.available() == 0 ) {
                zDebug("Zeno: RX buffer overflow (TxACK) fifo-count: %d-%d", rx_message_fifo.count(), rx_message_fifo.isEmpty());
                rx_message_fifo_mutex.unlock();
                return;
            }
            rx_message_fifo.write(message);
            rx_message_fifo_cond.notify_one();

            rx_message_fifo_mutex.unlock();

            return;
        }

        zDebug("ZenoCAN Ch%d *** Writing back to TX fifo TID%d-%d-%x", channel_index+1, m.transaction_id, tx_ack.trans_id, m.id);
        tx_message_fifo.write(m);
    }

    tx_message_fifo_mutex.unlock();
    zDebug("ZenoCAN Ch%d -- TX ack with transId: %d not queued", channel_index+1, tx_ack.trans_id);
}

bool ZZenoCANChannel::getZenoDeviceTimeInUs(uint64_t& timestamp_in_us)
{
    if (!checkOpen()) return false;

    ZenoReadClock cmd;
    ZenoReadClockResponse reply;

    memset(&cmd,0,sizeof(ZenoBusOn));
    cmd.h.cmd_id = ZENO_CMD_READ_CLOCK;
    cmd.channel = channel_index;

    if (!usb_can_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = usb_can_device->getLastErrorText();
        zError("(ZenoUSB) Ch%d failed to read Zeno device clock: %s", channel_index+1, last_error_text.c_str());
        return false;
    }

    timestamp_in_us = reply.clock_value;
    timestamp_in_us /= reply.divisor;

    return true;
}


uint64_t ZZenoCANChannel::getDeviceClock()
{
    if (!checkOpen()) return false;

    uint64_t t;
    /* Read the clock from the Zeno device */
    if (!getZenoDeviceTimeInUs(t)) return false;

    return t;
}

ZCANDriver* ZZenoCANChannel::getCANDriver() const
{
    return usb_can_device->getZenoDriver();
}

void ZZenoCANChannel::flushRxFifo()
{
    std::lock_guard<std::mutex> lock(tx_message_fifo_mutex);
    rx_message_fifo.clear();
}

void ZZenoCANChannel::flushTxFifo()
{
    std::lock_guard<std::mutex> lock(tx_message_fifo_mutex);
    tx_message_fifo.clear();
    tx_request_count = 0;
    tx_next_trans_id = 0;

    /* TODO: send flush CMD to Zeno */
}

bool ZZenoCANChannel::checkOpen()
{
    if (is_open.load() == 0) {
        last_error_text = "CAN Channel " + std::to_string(channel_index+1) + " is not open";
        return false;
    }

    return true;
}

bool ZZenoCANChannel::waitForSpaceInTxFifo(std::unique_lock<std::mutex>& lock, int& timeout_in_ms)
{
    auto t_start = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()).time_since_epoch();
    std::chrono::milliseconds timeout(timeout_in_ms);

    while ( timeout.count() > 0 &&
            tx_message_fifo.count() >=  max_outstanding_tx_requests) {
        std::cv_status wait_result;

        wait_result = tx_message_fifo_cond.wait_for(lock, timeout);
        // qDebug()  << "ZenoCAN-Ch" << channel_index  << "wait for TX space" << tx_message_fifo.count() << tx_ack_received << wait_success;

        auto t_now = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()).time_since_epoch();
        timeout -= (t_now - t_start);

        if (wait_result == std::cv_status::no_timeout) break;
    }

    timeout_in_ms = timeout.count();
    return tx_message_fifo.count() < max_outstanding_tx_requests;
}
