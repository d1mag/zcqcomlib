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

#ifndef ZZENOUSBDEVICE_H_
#define ZZENOUSBDEVICE_H_

#include "zzenocanchannel.h"
#include "zzenolinchannel.h"
#include "zenocan.h"
#include <libusb.h>

#include <vector>
#include <condition_variable>
#include <mutex>

#define ZENO_USB_BULK_TRANSFER_TIMEOUT      60000 /* 60 seconds in ms */
#define ZENO_USB_INTERRUPT_TRANSFER_TIMEOUT 60000 /* 60 seconds in ms */
#define ZENO_USB_TX_TIMEOUT                  5000  /* 5 seconds in ms */
#define ZENO_USB_MAX_PACKET_IN               3072
#define ZENO_USB_MAX_PACKET_OUT              3072
// #define ZENO_MAX_OUTSTANDING_TX_REQUEST        31

class ZUSBContext;
class ZZenoCANDriver;
class ZZenoLINDriver;
class ZZenoUSBDevice : public ZRefCountingObjBase {
public:
    ZZenoUSBDevice(ZZenoCANDriver* _driver,
                   int _device_no, libusb_device* _device,
                   const std::string& _display_name);
    ~ZZenoUSBDevice();

    void retrieveDeviceInfo();

    const std::string getObjectText() const;
    const std::string getLastErrorText();

    bool open();
    bool close();
    bool isOpen() const;

    int getCANChannelCount() const;
    ZRef<ZCANChannel> getCANChannel(unsigned int channel_index);

    int getLINChannelCount() const;
    ZRef<ZLINChannel> getLINChannel(unsigned int channel_index);

    uint32_t getSerialNumber() const;

    uint32_t getFWVersion() const;

    bool sendAndWhaitReply(ZenoCmd* request, ZenoResponse* reply);
    bool queueRequest(ZenoCmd* request, int timeout_in_ms = ZENO_USB_TX_TIMEOUT);
    bool queueTxRequest(ZenoCmd* request, int timeout_in_ms = ZENO_USB_TX_TIMEOUT);

    int getNextTransactionID() const {
        return next_transaction_id;
    }

    int getClockResolution() const {
        return zeno_clock_resolution;
    }

    ZZenoCANDriver* getZenoDriver() const {
        return driver;
    }

    ZZenoLINDriver* getZenoLINDriver() const;

    bool isDeviceGoneOrDisconnected() const {
        return device_gone_or_disconnected;
    }

    bool isInitClockCalibrationDone() const {
        return init_calibrate_count == 0;
    }
    void hex_dump(const void *buffer, std::size_t bufsize, bool showPrintableChars = true);

protected:
    void freeTransfers();
    int getNextTransferIndex();
    bool waitForBulkTransfer(std::unique_lock<std::mutex>& lock, int timeout_in_ms);
    void handleIncomingData(int bytes_transferred);
    void handleInterruptData();
    void handleCommand(ZenoCmd* zeno_cmd);
    void startClockInt();
    void stopClockInt();

    ZZenoCANDriver* driver;
    ZUSBContext* usb_context;
    bool device_gone_or_disconnected;

    mutable std::mutex device_mutex;
    int device_no;
    int open_ref_count;

    libusb_device* device;
    libusb_device_handle* handle;

    /* USB incoming data from device */
    uint8_t in_end_point_address;
    uint8_t in_end_point_interrupt_address;
    int in_max_packet_size;
    int in_bulk_transfer_complete;
    int in_interrupt_transfer_complete;
    libusb_transfer* in_bulk_transfer;
    libusb_transfer* in_interrupt_transfer;
    uint8_t* in_buffer;
    uint8_t in_interrupt_buffer[16];

    /* USB outgoing data from device */
    uint8_t out_end_point_address;
    int out_max_packet_size;
    uint8_t* out_buffer[2];
    libusb_transfer* out_bulk_transfer[2];

    int next_transfer_index;
    int bulk_out_transfer_completed;
    std::mutex out_transfer_mutex;
    std::condition_variable out_transfer_cond;

    std::vector<ZRef<ZZenoCANChannel> > can_channel_list;
    std::vector<ZRef<ZZenoLINChannel> > lin_channel_list;
    std::string display_name;

    mutable std::string last_error_text;

    /* Reply state */
    int reply_timeout_in_ms;
    uint8_t reply_cmd_id;
    int reply_received;
    ZenoResponse* reply_command;
    std::mutex send_reply_mutex;
    std::mutex command_mutex;
    std::condition_variable command_cond;

    /* Card info */
    uint8_t next_transaction_id;
    int zeno_clock_resolution;
    uint32_t serial_number;
    uint32_t fw_version;

    /* Clock info */
    uint64_t t2_clock_start_ref_in_us;
    int init_calibrate_count;
    int drift_time_in_us;

private:
    bool ___queueRequestUnlocked(ZenoCmd* request, std::unique_lock<std::mutex>& lock, int timeout_in_ms);
    static void __inBulkTransferCallback(libusb_transfer* in_bulk_transfer);
    static void __inInterruptTransferCallback(libusb_transfer* in_interrupt_transfer);
    static void __outBulkTransferCallback(libusb_transfer* out_bulk_transfer);
};

#endif /* ZZENOUSBDEVICE_H_ */
