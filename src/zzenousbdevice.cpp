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

#include "zzenousbdevice.h"
#include "zzenocandriver.h"
#include "zusbcontext.h"
#include "zdebug.h"

#include <string.h>
#include <assert.h>
#include <algorithm>
#include <chrono>
#include <cmath>

#ifdef _WIN32
  /* some bloody #define of min conflicts with C++ std::min and std::max */
  #ifdef min
    #undef min
  #endif
  #ifdef max
    #undef max
  #endif
#endif

ZZenoUSBDevice::ZZenoUSBDevice(ZZenoCANDriver* _driver,
                               int _device_no, libusb_device* _device,
                               const std::string& _display_name)
: driver(_driver), usb_context(new ZUSBContext(driver->getUSBContext())),
  device_gone_or_disconnected(false),
  device_no(_device_no), open_ref_count(0),
  device(_device), handle(nullptr),
  in_end_point_address(0), in_end_point_interrupt_address(0), in_max_packet_size(0),
  in_bulk_transfer_complete(0), in_interrupt_transfer_complete(0),
  in_bulk_transfer(nullptr), in_buffer(new uint8_t[ZENO_USB_MAX_PACKET_IN]),
  out_end_point_address(0), display_name(_display_name),
  reply_timeout_in_ms(1000), reply_cmd_id(0), reply_received(0), reply_command(nullptr),
  next_transaction_id(0),
  zeno_clock_resolution(1),
  serial_number(0),
  fw_version(0),
  t2_clock_start_ref_in_us(0),
  t2_e_clock_start_ref_in_us(0),
  t2_e_clock_start_diff_utc_us(0),
  init_calibrate_count(0),
  drift_time_in_us(0),
  time_drift_in_us(0),
  drift_factor(0)
{
    libusb_ref_device(device);
    int res;
    libusb_config_descriptor* config = nullptr;

    res = libusb_get_config_descriptor(device,0,&config);
    if ( res ) {
        last_error_text = ZUSBContext::translateLibUSBErrorCode(res);
        device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
        zCritical("(ZenoUSB) get active configuration failed: %s", last_error_text.c_str());
        return;
    }

    if ( config->bNumInterfaces < 2 ) {
        zCritical("(ZenoUSB): bNumInterfaces %d", config->bNumInterfaces);
        libusb_free_config_descriptor(config);
        return;
    }

    if ( config->interface[0].num_altsetting < 1 ) {
        zCritical("(ZenoUSB): interface 0 -> num_altsetting %d",config->interface[0].num_altsetting);
        libusb_free_config_descriptor(config);
        return;
    }

    const libusb_interface_descriptor* interface = config->interface[0].altsetting;

    /* Interface 0 */
    for ( int i = 0; i < interface->bNumEndpoints; ++i ) {
        const libusb_endpoint_descriptor* end_point = interface->endpoint + i;
        zDebug("end_point->bEndpointAddress: %x", end_point->bEndpointAddress);
        zDebug("end_point->bmAttributes: %x", end_point->bmAttributes);

        if ( !in_end_point_address &&
             (end_point->bEndpointAddress & LIBUSB_ENDPOINT_IN) &&
             (end_point->bmAttributes & 0x3) == LIBUSB_TRANSFER_TYPE_BULK) {
            in_end_point_address = end_point->bEndpointAddress;
            in_max_packet_size = end_point->wMaxPacketSize;
        }

        if ( !out_end_point_address &&
             !(end_point->bEndpointAddress & LIBUSB_ENDPOINT_IN) &&
             (end_point->bmAttributes & 0x3) == LIBUSB_TRANSFER_TYPE_BULK) {
            out_end_point_address = end_point->bEndpointAddress;
            out_max_packet_size = end_point->wMaxPacketSize;
        }
    }

    zDebug(" -- in address: %d -- out address: %d", in_end_point_address, out_end_point_address);
    zDebug(" -- Max in: %d Max out: %d", in_max_packet_size, out_max_packet_size);

    /* Interface 1 */
    interface = config->interface[1].altsetting;
    zDebug("Interface 1 bNumEndpoints: %d", interface->bNumEndpoints);
    if ( interface->bNumEndpoints > 0 ) {
        /* Assume only 1 IN interrupt end-point */
        in_end_point_interrupt_address = interface->endpoint->bEndpointAddress;
        zDebug("Intterrupt end-point address: %d", interface->endpoint->bEndpointAddress);
    }

    libusb_free_config_descriptor(config);

    out_buffer[0] = new uint8_t[ZENO_USB_MAX_PACKET_OUT];
    out_buffer[1] = new uint8_t[ZENO_USB_MAX_PACKET_OUT];
    out_bulk_transfer[0] = nullptr;
    out_bulk_transfer[1] = nullptr;
    bulk_out_transfer_completed = 0;
    next_transfer_index = -1; /* No pending TX bulk transfer */

    retrieveDeviceInfo();
}

ZZenoUSBDevice::~ZZenoUSBDevice()
{
    if ( handle != nullptr ) close();
    libusb_unref_device(device);
    delete[] in_buffer;
    delete[] out_buffer[0];
    delete[] out_buffer[1];
}

void ZZenoUSBDevice::retrieveDeviceInfo()
{
    if (!open()) return;

    ZenoCmd cmd;
    ZenoResponse reply;
    memset(&cmd, 0, sizeof(cmd));
    cmd.h.cmd_id = ZENO_CMD_RESET;

    zDebug("Sending reset");
    if  ( !sendAndWhaitReply(&cmd, &reply) ) {
        zCritical("(ZenoUSB): failed to reset device: %s",last_error_text.c_str());
        close();
        return;
    }

    cmd.h.cmd_id = ZENO_CMD_INFO;
    if  ( !sendAndWhaitReply(&cmd, &reply) ) {
        zError("(ZenoUSB): failed to retrieve device info: %s", last_error_text.c_str());
        close();
        return;
    }

    ZenoInfoResponse* info_response = reinterpret_cast<ZenoInfoResponse*>(&reply);
    zDebug("Zeno Capabilities: %x", info_response->capabilities);
    zDebug("       fw_version: %x", info_response->fw_version);
    zDebug("        serial-nr: %x", info_response->serial_number);
    zDebug("          clock @: %fMhz", double(info_response->clock_resolution) / 1000.0);
    zDebug("CAN channel count: %d", info_response->can_channel_count);
    zDebug("LIN channel count: %d", info_response->lin_channel_count);

    zeno_clock_resolution = info_response->clock_resolution;
    serial_number = info_response->serial_number;
    fw_version = info_response->fw_version;

    can_channel_list.clear();
    for( int i = 0; i < info_response->can_channel_count; ++i ) {
        can_channel_list.push_back(new ZZenoCANChannel(i, this));
    }

    lin_channel_list.clear();
    int display_index = 1;
    for( int i = info_response->lin_channel_count-1; i >=0 ; --i ) {
        lin_channel_list.push_back(new ZZenoLINChannel(i, display_index++, this));
    }

    close();
}

const std::string ZZenoUSBDevice::getLastErrorText()
{
    return last_error_text;
}

const std::string ZZenoUSBDevice::getObjectText() const
{
    return display_name;
}

bool ZZenoUSBDevice::open()
{
    std::lock_guard<std::mutex> lock(device_mutex);
    if ( open_ref_count > 0 ) {
        open_ref_count ++;
        return true;
    }

    assert(open_ref_count == 0);
    assert(handle == NULL);
    assert(in_bulk_transfer == NULL);

    int res = libusb_open(device, &handle);
    if ( res ) {
        last_error_text = ZUSBContext::translateLibUSBErrorCode(res);
        device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
        zError("(ZenoUSB) open failed: %s", last_error_text.c_str());
        return false;
    }

    res = libusb_claim_interface(handle,0);
    if ( res ) {
        last_error_text = ZUSBContext::translateLibUSBErrorCode(res);
        device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
        zError("(ZenoUSB) claim interface[0] failed: %s", last_error_text.c_str());

        libusb_close(handle);
        handle = nullptr;

        return false;
    }


    res = libusb_claim_interface(handle,1);
    if ( res ) {
        last_error_text = ZUSBContext::translateLibUSBErrorCode(res);
        device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
        zError("(ZenoUSB) claim interface[1] failed: %s", last_error_text.c_str());

        libusb_release_interface(handle, 0);
        libusb_close(handle);
        handle = nullptr;

        return false;
    }

    in_bulk_transfer = libusb_alloc_transfer(0);
    in_interrupt_transfer = libusb_alloc_transfer(0);
    out_bulk_transfer[0] = libusb_alloc_transfer(0);
    out_bulk_transfer[1] = libusb_alloc_transfer(0);
    if ( in_bulk_transfer == nullptr ||
         in_interrupt_transfer == nullptr ||
         out_bulk_transfer[0] == nullptr ||
         out_bulk_transfer[1] == nullptr) {
        last_error_text = ZUSBContext::translateLibUSBErrorCode(LIBUSB_ERROR_NO_MEM);
        device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
        zError("(ZenoUSB) failed to allocate bulk in/out transfers");

        freeTransfers();
        libusb_release_interface(handle, 1);
        libusb_release_interface(handle, 0);
        libusb_close(handle);
        handle = nullptr;

        return false;
    }

    libusb_fill_bulk_transfer(in_bulk_transfer, handle, in_end_point_address,
                              in_buffer, ZENO_USB_MAX_PACKET_IN,
                              &__inBulkTransferCallback, this, ZENO_USB_BULK_TRANSFER_TIMEOUT);

    in_bulk_transfer_complete = 0;


    libusb_fill_interrupt_transfer(in_interrupt_transfer, handle, in_end_point_interrupt_address,
                              in_interrupt_buffer, sizeof(in_interrupt_buffer),
                              &__inInterruptTransferCallback, this, ZENO_USB_INTERRUPT_TRANSFER_TIMEOUT);

    in_interrupt_transfer_complete = 0;

    res = libusb_submit_transfer(in_bulk_transfer);
    if ( res ) {
        last_error_text = ZUSBContext::translateLibUSBErrorCode(res);
        device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
        zError("(ZenoUSB) failed to submit bulk transfer: %s", last_error_text.c_str());

        freeTransfers();

        libusb_release_interface(handle, 1);
        libusb_release_interface(handle, 0);
        libusb_close(handle);
        handle = nullptr;

        return false;
    }


    res = libusb_submit_transfer(in_interrupt_transfer);
    if ( res ) {
        last_error_text = ZUSBContext::translateLibUSBErrorCode(res);
        device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
        zError("ERROR(ZenoUSB) failed to submit interrupt transfer: %s", last_error_text.c_str());

        freeTransfers();

        libusb_release_interface(handle, 1);
        libusb_release_interface(handle, 0);
        libusb_close(handle);
        handle = nullptr;

        return false;
    }

    open_ref_count ++;
    usb_context->startRef();

    startClockInt();

    return true;
}

void ZZenoUSBDevice::freeTransfers()
{
    if ( in_bulk_transfer != nullptr ) libusb_free_transfer(in_bulk_transfer);
    if ( in_interrupt_transfer != nullptr ) libusb_free_transfer(in_interrupt_transfer);
    if ( out_bulk_transfer[0] != nullptr ) libusb_free_transfer(out_bulk_transfer[0]);
    if ( out_bulk_transfer[1] != nullptr ) libusb_free_transfer(out_bulk_transfer[1]);

    in_bulk_transfer = nullptr;
    in_interrupt_transfer = nullptr;
    out_bulk_transfer[0] = nullptr;
    out_bulk_transfer[1] = nullptr;
}

int ZZenoUSBDevice::getNextTransferIndex()
{
    int buffer_index;
    if ( next_transfer_index == -1 ) {
        libusb_fill_bulk_transfer(out_bulk_transfer[0], handle, out_end_point_address,
                                  out_buffer[0], 0,
                                  &__outBulkTransferCallback, this, ZENO_USB_BULK_TRANSFER_TIMEOUT);

        libusb_fill_bulk_transfer(out_bulk_transfer[1], handle, out_end_point_address,
                                  out_buffer[1], 0,
                                  &__outBulkTransferCallback, this, ZENO_USB_BULK_TRANSFER_TIMEOUT);

        buffer_index = 0;
    } else {
        buffer_index = next_transfer_index;
    }

    return buffer_index;
}

bool ZZenoUSBDevice::close()
{
    std::lock_guard<std::mutex> lock(device_mutex);
    assert(handle != nullptr);

    if ( open_ref_count == 1 ) stopClockInt();

    open_ref_count--;
    assert(open_ref_count >= 0);

    if ( open_ref_count == 0 ) {
        libusb_cancel_transfer(in_bulk_transfer);
        libusb_cancel_transfer(in_interrupt_transfer);

        if ( out_bulk_transfer[0]->length > 0 ) libusb_cancel_transfer(out_bulk_transfer[0]);
        if ( out_bulk_transfer[1]->length > 0 ) libusb_cancel_transfer(out_bulk_transfer[1]);

        while (!in_bulk_transfer_complete || next_transfer_index != -1) {
            if (!usb_context->handleEvents(in_bulk_transfer_complete))
                break;
        }

        while (!in_interrupt_transfer_complete) {
            if (!usb_context->handleEvents(in_interrupt_transfer_complete))
                break;
        }

        freeTransfers();

        // int res = libusb_reset_device(handle);
        // if ( res ) {
        //   translateLibUSBErrorCode(res);
        //   qDebug() << "ERROR(ZenoUSB) reset device failed: " << last_error_text;
        // }
        libusb_release_interface(handle, 0);
        libusb_release_interface(handle, 1);
        libusb_close(handle);
        handle = nullptr;

        usb_context->stopUnRef();
    }

    return true;
}

bool ZZenoUSBDevice::isOpen() const
{
    std::lock_guard<std::mutex> lock(device_mutex);
    return open_ref_count > 0;
}

uint32_t ZZenoUSBDevice::getCANChannelCount() const
{
    return uint32_t(can_channel_list.size());
}

ZRef<ZCANChannel> ZZenoUSBDevice::getCANChannel(unsigned int channel_index)
{
    if ( channel_index >= can_channel_list.size()) return nullptr;
    return can_channel_list[channel_index].cast<ZCANChannel>();
}

uint32_t ZZenoUSBDevice::getLINChannelCount() const
{
    return uint32_t(lin_channel_list.size());
}

ZRef<ZLINChannel> ZZenoUSBDevice::getLINChannel(unsigned int channel_index)
{
    if (channel_index >= lin_channel_list.size()) return nullptr;
    return lin_channel_list[channel_index].cast<ZLINChannel>();
}

uint32_t ZZenoUSBDevice::getSerialNumber() const
{
    return serial_number;
}

uint32_t ZZenoUSBDevice::getFWVersion() const
{
    return fw_version;
}

bool ZZenoUSBDevice::sendAndWhaitReply(ZenoCmd* request, ZenoResponse* reply)
{
    assert(handle != nullptr);
    std::lock_guard<std::mutex> lock(send_reply_mutex);

    if (!queueRequest(request)) return false;

    std::unique_lock<std::mutex> command_lock(command_mutex);
    reply_received = 0;
    reply_command = reply;
    reply_cmd_id = request->h.cmd_id;

    // qDebug() << "Queue reqeust";    
    while (!reply_received) {
        std::chrono::milliseconds reply_timeout(reply_timeout_in_ms);
        if (command_cond.wait_for(command_lock, reply_timeout) == std::cv_status::timeout) {
            last_error_text = "Timeout waiting for reply";
            break;
        }
    }

    reply_command = nullptr;
    // qDebug() << "last_error_text" << last_error_text << reply_received;

    return (reply_received != 0);
}

bool ZZenoUSBDevice::___queueRequestUnlocked(ZenoCmd* request, std::unique_lock<std::mutex>& lock, int timeout_in_ms) {
    int buffer_index = getNextTransferIndex();
    if ( buffer_index == -1) return false;

    int offset = out_bulk_transfer[buffer_index]->length;
    // if ( (offset & -out_max_packet_size) != ((offset+HYDRA_CMD_SIZE) & -out_max_packet_size) ) {
    //    out_bulk_transfer[buffer_index]->buffer[offset] = 0;
    //    offset += out_max_packet_size;
    //    offset &= - out_max_packet_size;
    // }

    if ( (offset + ZENO_CMD_SIZE) >= ZENO_USB_MAX_PACKET_OUT ) {
        if ( next_transfer_index == -1 ) {
            int res = libusb_submit_transfer(out_bulk_transfer[buffer_index]);
            if ( res ) {
                last_error_text = ZUSBContext::translateLibUSBErrorCode(res);
                device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
                zError("(ZenoUSB) failed to submit bulk transfer: %s", last_error_text.c_str());
                return false;
            }
        }

        zDebug("ZenoUSB: Wait for next bulk transfer");
        next_transfer_index = (buffer_index + 1) % 2;
        if (!waitForBulkTransfer(lock, timeout_in_ms)) return false;
        buffer_index = next_transfer_index;

        offset = 0;
    }

    memcpy(out_bulk_transfer[buffer_index]->buffer + offset, request, ZENO_CMD_SIZE);
    out_bulk_transfer[buffer_index]->length += ZENO_CMD_SIZE;

    if ( next_transfer_index == -1 ) {
        // qDebug() << "-- submit buffer_index: " << buffer_index;
        int res = libusb_submit_transfer(out_bulk_transfer[buffer_index]);
        if ( res ) {
            last_error_text = ZUSBContext::translateLibUSBErrorCode(res);
            device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
            zError("(ZenoUSB) failed to submit bulk transfer: %s", last_error_text.c_str());
            return false;
        }
        next_transfer_index = (buffer_index + 1) % 2;
    }

    // qDebug() << "-- nt: " << next_transfer_index;
    // qDebug() << "b0 length: " << out_bulk_transfer[0]->length;
    // qDebug() << "b1 length: " << out_bulk_transfer[1]->length;

    return true;
}

bool ZZenoUSBDevice::queueRequest(ZenoCmd* request, int timeout_in_ms)
{
    std::unique_lock<std::mutex> lock(out_transfer_mutex);
    request->h.transaction_id = uint8_t(next_transaction_id ++);

    return ___queueRequestUnlocked(request, lock, timeout_in_ms);
}

bool ZZenoUSBDevice::queueTxRequest(ZenoCmd* request, int timeout_in_ms)
{
    std::unique_lock<std::mutex> lock(out_transfer_mutex);
    return ___queueRequestUnlocked(request, lock, timeout_in_ms);
}

ZZenoLINDriver* ZZenoUSBDevice::getZenoLINDriver() const
{
    ZZenoLINDriver* lin_driver = driver->getZenoLINDriver();
    assert(lin_driver != nullptr);
    return lin_driver;
}

bool ZZenoUSBDevice::waitForBulkTransfer(std::unique_lock<std::mutex>& lock, int timeout_in_ms)
{
    while (next_transfer_index != -1 &&
           out_bulk_transfer[next_transfer_index]->length > 0 ) {
//        qDebug() << "nt: " << next_transfer_index;
//        qDebug() << "length: " << out_bulk_transfer[next_transfer_index]->length;

        bulk_out_transfer_completed = 0;
        while (!bulk_out_transfer_completed) {
            std::chrono::milliseconds timeout(timeout_in_ms);
            if (out_transfer_cond.wait_for(lock, timeout) == std::cv_status::timeout) {
                last_error_text = "Timeout, TX buffer overflow";
                return false;
            }
        }

        /* Device closed or handleEvents() failed */
        if ( open_ref_count == 0 ) return false;
    }

    return true;
}

void ZZenoUSBDevice::handleIncomingData(int bytes_transferred)
{
    int offset = 0;
    ZenoCmd* zeno_cmd;


    // if (bytes_transferred > 32) qDebug() << " R bytes_transfered " << bytes_transferred;
    while ( offset < bytes_transferred ) {
        zeno_cmd = reinterpret_cast<ZenoCmd*>(in_buffer + offset);

        // qDebug() << " R bytes_transfered " << bytes_transferred << zeno_cmd->h.cmd_id;
        // if ( bytes_transferred != 32 ) qDebug() << " R cmdNo: " << command_ptr->cmdNo << " trans id " << command_ptr->cmdIOPSeq.transId << " srcHE " << command_ptr->cmdIOPSeq.srcHE << " dst " << command_ptr->cmdIOP.dstAddr << "srcChannel" <<  command_ptr->cmdIOP.srcChannel;

        offset += ZENO_CMD_SIZE;
        handleCommand(zeno_cmd);

        if ( zeno_cmd->h.cmd_id == ZENO_CMD_RESPONSE) {
            std::lock_guard<std::mutex> lock(command_mutex);
            if ( reply_command != nullptr ) {

                ZenoResponse* response = reinterpret_cast<ZenoResponse*>(zeno_cmd);
                if ( response->response_cmd_id == reply_cmd_id ) {
                    *reply_command = *response;
                    reply_received = 1;
                    command_cond.notify_one();
                }
            }
        }
    }
}

void ZZenoUSBDevice::handleInterruptData()
{
    ZenoIntCmd* zeno_int_cmd = reinterpret_cast<ZenoIntCmd*>(in_interrupt_buffer);
    switch(zeno_int_cmd->h.cmd_id) {
    case ZENO_CLOCK_INFO_INT:  {
        auto t_now = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch();
        auto host_t0 = t_now - std::chrono::microseconds(t2_clock_start_ref_in_us);
        ZenoIntClockInfoCmd* clock_info = reinterpret_cast<ZenoIntClockInfoCmd*>(zeno_int_cmd);

        ZZenoTimerSynch::ZTimeVal zeno_clock_in_us = ZZenoTimerSynch::ZTimeVal(clock_info->clock_value_t1 / clock_info->clock_divisor);
        auto drift_in_us = std::chrono::microseconds(zeno_clock_in_us) - host_t0;

        // zDebug("Zeno clock: %ld - %lld - %lld t_now %lld host_t0 %lld",zeno_clock_in_us.count(), drift_in_us.count(), drift_time_in_us, t_now.count(), host_t0.count());
        ZZenoTimerSynch::ZTimeVal max_adjust;
        if ( init_calibrate_count > 0 ) {
            init_calibrate_count--;
            drift_time_in_us += drift_in_us.count();
            if ( init_calibrate_count == 0 ) {
                drift_time_in_us /= 5;
                zDebug("Avg drift: %ld", drift_time_in_us);
                t2_clock_start_ref_in_us -= drift_time_in_us;
            }
            max_adjust = ZZenoTimerSynch::ZTimeVal(12);
        }
        else {
            max_adjust = ZZenoTimerSynch::ZTimeVal(120);
            int64_t e_diff = t2_clock_start_ref_in_us - t2_e_clock_start_ref_in_us;
            if (e_diff != 0) {
                e_diff = std::min(e_diff, int64_t(32));
                t2_e_clock_start_ref_in_us += e_diff;
            }
        }

        ajdustDeviceTimeDrift(ZZenoTimerSynch::ZTimeVal(zeno_clock_in_us),
                              ZZenoTimerSynch::ZTimeVal(drift_in_us),
                              max_adjust);
        break;
    }
    default:
        break;
    }
}

#include <iostream>
void ZZenoUSBDevice::hex_dump(const void *buffer, std::size_t bufsize, bool showPrintableChars)
{
    std::ostream& os = std::cout;
    if (buffer == nullptr) {
        return; // os;
    }
    auto oldFormat = os.flags();
    auto oldFillChar = os.fill();
    constexpr std::size_t maxline{8};
    // create a place to store text version of string
    char renderString[maxline+1];
    char *rsptr{renderString};
    // convenience cast
    const unsigned char *buf{reinterpret_cast<const unsigned char *>(buffer)};

    for (std::size_t linecount=maxline; bufsize; --bufsize, ++buf) {
        //os << std::setw(2) << std::setfill('0') << std::hex 
        os << std::hex 
           << static_cast<unsigned>(*buf) << ' ';
        *rsptr++ = std::isprint(*buf) ? *buf : '.';
        if (--linecount == 0) {
            *rsptr++ = '\0';  // terminate string
            if (showPrintableChars) {
                os << " | " << renderString;
            } 
            os << '\n';
            rsptr = renderString;
            linecount = std::min(maxline, bufsize);
        }
    }
    // emit newline if we haven't already
    if (rsptr != renderString) {
        if (showPrintableChars) {
            for (*rsptr++ = '\0'; rsptr != &renderString[maxline+1]; ++rsptr) {
                 os << "   ";
            }
            os << " | " << renderString;
        }
        os << '\n';
    }

    os.fill(oldFillChar);
    os.flags(oldFormat);
    //return os;
}

void ZZenoUSBDevice::handleCommand(ZenoCmd* zeno_cmd)
{
    // qDebug() << zeno_cmd->h.cmd_id;
    switch(zeno_cmd->h.cmd_id) {
    case ZENO_CMD_CAN_TX_ACK: {
        ZenoTxCANRequestAck* zeno_tx_ack = reinterpret_cast<ZenoTxCANRequestAck*>(zeno_cmd);
        // qDebug() << "TX ack " << zeno_tx_ack->trans_id << zeno_tx_ack->dlc << zeno_tx_ack->id << zeno_tx_ack->timestamp;

        if ( zeno_tx_ack->channel < can_channel_list.size()) {
            static_cast<ZZenoCANChannel*>(can_channel_list[zeno_tx_ack->channel].get())->txAck(*zeno_tx_ack);
        }
        break;
    }
    case ZENO_CMD_CAN_RX: {
        ZenoCAN20Message* zeno_can_msg = reinterpret_cast<ZenoCAN20Message*>(zeno_cmd);
        // qDebug() << "RX ch:" << zeno_can_msg->channel << zeno_can_msg->dlc << zeno_can_msg->id << zeno_can_msg->timestamp;

        if ( zeno_can_msg->channel < can_channel_list.size()) {
            can_channel_list[zeno_can_msg->channel]->queueMessage(*zeno_can_msg);
        }

        break;
    }
    case ZENO_CMD_CANFD_P1_RX: {
        // Ouch... ZenoCANFDMessageP1, ZENO_CMD_CANFD_P2_RX and ZENO_CMD_CANFD_P3_RX are missing timestamp high-bits? have only 32 bits available
        ZenoCANFDMessageP1* zeno_canfd_msg_p1 = reinterpret_cast<ZenoCANFDMessageP1*>(zeno_cmd);  // Is this message only contain a 32-bit timestamp?!?
        // qDebug() << "RX CANFD P1 ch:" << zeno_canfd_msg_p1->channel << zeno_canfd_msg_p1->dlc << zeno_canfd_msg_p1->id << zeno_canfd_msg_p1->timestamp;
        //printf("ZENO_CMD_CANFD_P1_RX: ts=%lu\n", (unsigned long)zeno_canfd_msg_p1->timestamp);
        //hex_dump(zeno_canfd_msg_p1, sizeof(ZenoCANFDMessageP1));
        
        if ( zeno_canfd_msg_p1->channel < can_channel_list.size()) {
            can_channel_list[zeno_canfd_msg_p1->channel]->queueMessageCANFDP1(*zeno_canfd_msg_p1);
        }

        break;
    }
    case ZENO_CMD_CANFD_P2_RX: {
        ZenoCANFDMessageP2* zeno_canfd_msg_p2 = reinterpret_cast<ZenoCANFDMessageP2*>(zeno_cmd);
        // qDebug() << "RX CANFD P2 ch:" << zeno_canfd_msg_p2->channel;
        //hex_dump(zeno_canfd_msg_p2, sizeof(ZenoCANFDMessageP2));

        if ( zeno_canfd_msg_p2->channel < can_channel_list.size()) {
            can_channel_list[zeno_canfd_msg_p2->channel]->queueMessageCANFDP2(*zeno_canfd_msg_p2);
        }

        break;
    }
    case ZENO_CMD_CANFD_P3_RX: {
        ZenoCANFDMessageP3* zeno_canfd_msg_p3 = reinterpret_cast<ZenoCANFDMessageP3*>(zeno_cmd);
        // qDebug() << "RX CANFD P3 ch:" << zeno_canfd_msg_p3->channel;
        //hex_dump(zeno_canfd_msg_p3, sizeof(ZenoCANFDMessageP3));

        if ( zeno_canfd_msg_p3->channel < can_channel_list.size()) {
            can_channel_list[zeno_canfd_msg_p3->channel]->queueMessageCANFDP3(*zeno_canfd_msg_p3);
        }

        break;
    }
    case ZENO_CMD_LIN_RX_MESSAGE: {
        ZenoLINMessage* zeno_lin_msg = reinterpret_cast<ZenoLINMessage*>(zeno_cmd);
        // qDebug() << "ZenoLIN-RX" << zeno_lin_msg->channel;

        if ( lin_channel_list.size() == 2) {
            switch(zeno_lin_msg->channel) {
            case 0:
                lin_channel_list[1]->queueMessage(*zeno_lin_msg);
                break;
            case 1:
                lin_channel_list[0]->queueMessage(*zeno_lin_msg);
                break;
            }
        }

        break;
    }
    case ZENO_CMD_LIN_TX_ACK: {
        ZenoTxLINRequestAck* zeno_lin_tx_ack = reinterpret_cast<ZenoTxLINRequestAck*>(zeno_cmd);

        /* Assume Zeno CANquatro - with 2 LIN channels */
        if ( lin_channel_list.size() == 2) {
            switch(zeno_lin_tx_ack->channel) {
            case 0:
                lin_channel_list[1]->txAck(*zeno_lin_tx_ack);
                break;
            case 1:
                lin_channel_list[0]->txAck(*zeno_lin_tx_ack);
                break;
            }
        }

        break;
    }
    case ZENO_DEBUG: {
        ZenoDebug* zeno_debug = reinterpret_cast<ZenoDebug*>(zeno_cmd);
        unsigned debug_msg_len = 30;
        for ( unsigned i = 0; i < 30; ++i ) {
            if ( zeno_debug->debug_msg[i] == 0 ) {
                debug_msg_len = i;
                break;
            }
        }
        std::string debug_msg(reinterpret_cast<const char*>(zeno_debug->debug_msg), debug_msg_len);
        zDebug("ZenoDBG: %s", debug_msg.c_str());
        break;
    }
    default:
        break;
    }
}

void ZZenoUSBDevice::startClockInt()
{
    ZenoCmd cmd;
    ZenoResponse reply;
    memset(&cmd, 0, sizeof(cmd));
    cmd.h.cmd_id = ZEMO_CMD_START_CLOCK_INT;


    zDebug("Sending Start-Clock INT cmd");
    auto t0 = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch();

    if  ( !sendAndWhaitReply(&cmd, &reply) ) {
        zCritical("(ZenoUSB): failed to start clock INT: %s", last_error_text.c_str());
        return;
    }

    auto t_now = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch();
    auto t_diff = t_now - t0;

    /* Estimate around 256us round-trip */
    t2_clock_start_ref_in_us = t_now.count() + t_diff.count();
    t2_e_clock_start_ref_in_us = t2_clock_start_ref_in_us;

    unsigned long t_now_utc = std::chrono::system_clock::now().time_since_epoch() / std::chrono::microseconds(1);
    t2_e_clock_start_diff_utc_us = t2_e_clock_start_ref_in_us + t_now_utc - t_now.count();
    zDebug("start_diff_utc_us = %ld\n", t2_e_clock_start_diff_utc_us);
    
    init_calibrate_count = 5;
    drift_time_in_us = 0;
    time_drift_in_us = ZZenoTimerSynch::ZTimeVal();
    drift_factor = 0;
}

void ZZenoUSBDevice::stopClockInt()
{
    ZenoCmd cmd;
    ZenoResponse reply;
    memset(&cmd, 0, sizeof(cmd));
    cmd.h.cmd_id = ZEMO_CMD_STOP_CLOCK_INT;

    zDebug("Sending Stop-Clock INT cmd");
    if  ( !sendAndWhaitReply(&cmd, &reply) ) {
        zCritical("(ZenoUSB): failed to stop clock INT: %s", last_error_text.c_str());
        return;
    }
}

void ZZenoUSBDevice::ajdustDeviceTimeDrift(ZZenoTimerSynch::ZTimeVal device_time_in_us,
                                           ZZenoTimerSynch::ZTimeVal new_drift_time_in_us,
                                           ZZenoTimerSynch::ZTimeVal max_adjust)
{
    ZZenoTimerSynch::ZTimeVal diff = time_drift_in_us - new_drift_time_in_us;

    ZZenoTimerSynch::ZTimeVal adjust = std::min(ZZenoTimerSynch::ZTimeVal(std::abs(diff.count())), max_adjust);

    //zDebug("adjust time with diff=%ld adjust=%ld\n", diff.count(), adjust.count());
    
    if ( diff.count() > 0 )
        time_drift_in_us -= adjust;
    else
        time_drift_in_us += adjust;

    if ( device_time_in_us != ZZenoTimerSynch::ZTimeVal::zero() )
        drift_factor = double(time_drift_in_us.count()) / double(device_time_in_us.count());
}

void ZZenoUSBDevice::__inBulkTransferCallback(libusb_transfer* in_bulk_transfer)
{
    ZZenoUSBDevice* _this = static_cast<ZZenoUSBDevice*>(in_bulk_transfer->user_data);

    // qDebug() << " bulk complete status: " << in_bulk_transfer->status << " transferrred: " << in_bulk_transfer->actual_length;

    if ( _this->open_ref_count == 0) {
        /* Device closed do not re-submit transfer */
        _this->in_bulk_transfer_complete = 1;
        return;
    }

    if ( in_bulk_transfer->status != LIBUSB_TRANSFER_TIMED_OUT &&
         in_bulk_transfer->status != LIBUSB_TRANSFER_CANCELLED &&
         in_bulk_transfer->status != LIBUSB_TRANSFER_COMPLETED ) {
        zError("(ZenoUSB) unexcepted bulk transfer status: %d", in_bulk_transfer->status);
        _this->in_bulk_transfer_complete = 1;
        return;
    }

    if ( in_bulk_transfer->status == LIBUSB_TRANSFER_CANCELLED) {
        zDebug(" --- in bulk transfer was canceled");
    }

    if ( in_bulk_transfer->status == LIBUSB_TRANSFER_COMPLETED) {
        _this->handleIncomingData(in_bulk_transfer->actual_length);
    }

    /* Re-submit bulk transfer */
    int res;
    assert(_this->in_bulk_transfer_complete == 0);
    res = libusb_submit_transfer(_this->in_bulk_transfer);
    if ( res ) {
        _this->last_error_text = ZUSBContext::translateLibUSBErrorCode(res);
        _this->device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
        zError("(ZenoUSB) failed to submit bulk transfer: %s", _this->last_error_text.c_str());
        _this->in_bulk_transfer_complete = 1;
    }
}

void ZZenoUSBDevice::__inInterruptTransferCallback(libusb_transfer *in_interrupt_transfer)
{
    ZZenoUSBDevice* _this = static_cast<ZZenoUSBDevice*>(in_interrupt_transfer->user_data);

    // qDebug() << __PRETTY_FUNCTION__ << "in" << in_interrupt_transfer->status <<  _this->open_ref_count;

    if ( _this->open_ref_count == 0) {
        /* Device closed do not re-submit transfer */
        _this->in_interrupt_transfer_complete = 1;
        return;
    }

    if ( in_interrupt_transfer->status != LIBUSB_TRANSFER_TIMED_OUT &&
         in_interrupt_transfer->status != LIBUSB_TRANSFER_CANCELLED &&
         in_interrupt_transfer->status != LIBUSB_TRANSFER_COMPLETED ) {
        zCritical("(ZenoUSB) unexcepted interrupt transfer status: %d", in_interrupt_transfer->status);
        _this->in_interrupt_transfer_complete = 1;
        return;
    }


    if ( in_interrupt_transfer->status == LIBUSB_TRANSFER_CANCELLED) {
        zDebug(" --- in interrupt transfer was canceled");
    }

    if ( in_interrupt_transfer->status == LIBUSB_TRANSFER_COMPLETED) {
        // qDebug() << "Interrupt data ready:" << in_interrupt_transfer->length;
        _this->handleInterruptData();
    }

    /* Re-submit bulk transfer */
    int res;
    assert(_this->in_interrupt_transfer_complete == 0);
    res = libusb_submit_transfer(_this->in_interrupt_transfer);
    if ( res ) {
        _this->last_error_text = ZUSBContext::translateLibUSBErrorCode(res);
        _this->device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
        zError("(ZenoUSB) failed to submit bulk transfer: %s", _this->last_error_text.c_str());
        _this->in_interrupt_transfer_complete = 1;
    }
}

void ZZenoUSBDevice::__outBulkTransferCallback(libusb_transfer* out_bulk_transfer)
{
    ZZenoUSBDevice* _this = static_cast<ZZenoUSBDevice*>(out_bulk_transfer->user_data);
    std::lock_guard<std::mutex> lock(_this->out_transfer_mutex);

    // qDebug() << " TX bulk complete status: " << out_bulk_transfer->status << " transferrred: " << out_bulk_transfer->actual_length;
    _this->bulk_out_transfer_completed = 1;
    _this->out_transfer_cond.notify_one();

    if ( _this->open_ref_count == 0) {
        /* Device closed do not re-submit transfer */
        _this->next_transfer_index = -1;
        return;
    }

    if ( out_bulk_transfer->status != LIBUSB_TRANSFER_TIMED_OUT &&
         out_bulk_transfer->status != LIBUSB_TRANSFER_CANCELLED &&
         out_bulk_transfer->status != LIBUSB_TRANSFER_COMPLETED ) {
        zCritical("(ZenoUSB) unexcepted transfer status: %d", out_bulk_transfer->status);
        _this->next_transfer_index = -1;
        return;
    }

    assert(_this->next_transfer_index != -1);
    int res;
    out_bulk_transfer->length = 0;

    libusb_transfer* next_transfer;
    if ( out_bulk_transfer == _this->out_bulk_transfer[0] ) {
        _this->next_transfer_index = 0;
        next_transfer = _this->out_bulk_transfer[1];
    } else {
        assert(out_bulk_transfer == _this->out_bulk_transfer[1]);
        _this->next_transfer_index = 1;
        next_transfer = _this->out_bulk_transfer[0];
    }

    if ( next_transfer->length > 0 ) {
        res = libusb_submit_transfer(next_transfer);
        if ( res ) {
            _this->last_error_text = ZUSBContext::translateLibUSBErrorCode(res);
            _this->device_gone_or_disconnected = (res == LIBUSB_ERROR_NO_DEVICE);
            zError("(ZenoUSB) failed to submit bulk transfer: %s", _this->last_error_text.c_str());
            _this->next_transfer_index = -1;
        }
        // _this->next_transfer_index = (_this->next_transfer_index + 1) % 2;
    } else {
        _this->next_transfer_index = -1;
    }
}
