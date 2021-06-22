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

#ifndef ZENOCAN_H
#define ZENOCAN_H

#include "zglobal.h"
#include <stdint.h>

#define ZENO_CMD_SIZE 32
#define ZENO_EXT_CMD_SIZE 64

#define zenoRequest(c) reinterpret_cast<ZenoCmd*>(&c)
#define zenoReply(c)   reinterpret_cast<ZenoResponse*>(&c)

enum ZenoCmdID {
    ZENO_CMD_RESET,
    ZENO_CMD_INFO,
    ZENO_CMD_OPEN,
    ZENO_CMD_CLOSE,
    ZENO_CMD_RESPONSE,
    ZENO_CMD_BUS_ON,
    ZENO_CMD_BUS_OFF,
    ZENO_CMD_SET_BIT_TIMING,

    ZENO_CMD_CAN20_TX_REQUEST,
    ZENO_CMD_CANFD_TX_REQUEST,

    ZENO_CMD_READ_CLOCK,
    ZENO_DEBUG,
    
    ZENO_CMD_CAN_TX_ACK,
    ZENO_CMD_CAN_RX,
    
    ZEMO_CMD_START_CLOCK_INT,
    ZEMO_CMD_STOP_CLOCK_INT,
    ZENO_CLOCK_INFO_INT,
    
    ZENO_CMD_CANFD_P1_RX,
    ZENO_CMD_CANFD_P2_RX,
    ZENO_CMD_CANFD_P3_RX,

    ZENO_CMD_CANFD_P1_TX_REQUEST,
    ZENO_CMD_CANFD_P2_TX_REQUEST,
    ZENO_CMD_CANFD_P3_TX_REQUEST,
    
    ZENO_CMD_READ_CAN_CHIP_CLOCK,
    
    /* LIN commands */
    ZENO_CMD_LIN_OPEN,
    ZENO_CMD_LIN_CLOSE,
    ZEMO_CMD_LIN_SET_BITRATE,
    ZENO_CMD_LIN_BUS_ON,
    ZENO_CMD_LIN_BUS_OFF,
    ZENO_CMD_LIN_UPDATE_MESSAGE,
    ZENO_CMD_LIN_CLEAR_MESSAGE,
    ZENO_CMD_LIN_CLEAR_ALL_MESSAGES,
    ZENO_CMD_LIN_TX_MESSAGE,
    ZENO_CMD_LIN_TX_ACK,
    ZENO_CMD_LIN_RX_MESSAGE,
    
    /* CAN FD */
    ZENO_CMD_SET_DATA_BIT_TIMING,

    /* LIN command */
    ZENO_CMD_LIN_RESET_AUTO_BAUD,

    /* CAN operation mode */
    ZENO_CMD_SET_OP_MODE,
};

enum ZenoOpModeID {
    ZENO_NORMAL_CAN20_ONLY_MODE,
    ZENO_NORMAL_CANFD_MODE,
    ZENO_SILENT_MODE,
};

#define ZenoCAPExtendedCAN      0x00000001
#define ZenoCAPBusStatistics    0x00000002
#define ZenoCAPErrorCounters    0x00000004
#define ZenoCAPCanDiagnostics   0x00000008
#define ZenoCAPGenerateError    0x00000010
#define ZenoCAPGenerateOverload 0x00000020
#define ZenoCAPTxRequest        0x00000040
#define ZenoCAPTxAcknowledge    0x00000080
#define ZenoCAPRemote           0x00040000
#define ZenoCAPCanFD            0x00080000
#define ZenoCAPCanFDNonISO      0x00100000

#define ZenoCANFlagRTR          0x00001
#define ZenoCANFlagStandard     0x00002
#define ZenoCANFlagExtended     0x00004
#define ZenoCANFlagFD           0x00008
#define ZenoCANFlagFDBRS        0x00010
#define ZenoCANFlagFDNext       0x00010
#define ZenoCANErrorFrame       0x00020
#define ZenoCANFlagTxAck        0x00040
#define ZenoCANErrorHWOverrun   0x00080
#define ZenoCANErrorMask        0x0ff00
#define ZenoCANErrorStuff       0x00800
#define ZenoCANErrorForm        0x01000
#define ZenoCANErrorCRC         0x02000
#define ZenoCANErrorBIT0        0x04000
#define ZenoCANErrorBIT1        0x08000

/* LIN flags */
#define ZenoLINParityError       0x0001  // Rx: parity error (the identifier)
#define ZenoLINChecksumError     0x0002  // Rx: checksum error
#define ZenoLINNoData            0x0004  // Rx: header only
#define ZenoLINBitError          0x0008  // Tx: transmitted 1, got 0 or vice versa
#define ZenoLINTxMsgAcknowledge  0x0040

/* LIN TX flags */
#define ZenoLINEnhancedCRC       0x0080

#define ZenoCANStatusOpNormal    0x01
#define ZenoCANStatusOpConf      0x02
#define ZenoCANStatusTXPassive   0x03
#define ZenoCANStatusRXPassive   0x04
#define ZenoCANStatusTXWarning   0x05
#define ZenoCANStatusRXWarning   0x06
#define ZenoCANStatusErrorBusOff 0x07
#define ZenoCANInvalidMessage    0x08


#pragma pack(push, 1)

typedef struct {
    uint8_t cmd_id;
    uint8_t transaction_id;
} ZenoHeader;

typedef struct {
    ZenoHeader h;

    /* Up to 30 bytes payload */
    uint8_t cmd_payload[30];
} ZenoCmd;

typedef struct {
    ZenoHeader h;

    /* Up to 14 bytes payload */
    uint8_t cmd_payload[14];
} ZenoIntCmd;

typedef struct {
    ZenoHeader h;
    
    uint32_t clock_value_t0;
    uint64_t clock_value_t1;
    uint8_t clock_divisor;
    uint8_t usb_overflow_count;
} ZenoIntClockInfoCmd;

typedef struct {
    ZenoHeader h;
    uint8_t response_cmd_id;
    uint8_t cmd_result_code;

    uint8_t response_payload[28];
} ZenoResponse;

typedef struct {
    ZenoHeader h;
    uint8_t response_cmd_id;
    uint8_t cmd_result_code;

    uint32_t capabilities;

    uint32_t fw_version;
    uint32_t serial_number;

    uint8_t can_channel_count;
    uint8_t lin_channel_count;

    uint8_t hw_revision;
    uint8_t type_code;

    uint16_t clock_resolution; /* In Khz */
    
    /* Fill up up to 32 bytes */
    uint8_t unused[10];
} ZenoInfoResponse;

typedef struct {
    ZenoHeader h;
    uint8_t channel;
    
    /* Fill up up to 32 bytes */
    uint8_t unused[29];
} ZenoReadClock;

typedef struct {
    ZenoHeader h;
    uint8_t response_cmd_id;
    uint8_t cmd_result_code;
    
    uint64_t clock_value;
    uint32_t clock_wrap_around_count;
    uint8_t read_count;
    uint8_t divisor;
    
    uint8_t unused[14];
} ZenoReadClockResponse;

typedef struct {
    ZenoHeader h;
    uint8_t channel;
    uint8_t base_clock_divisor;
    uint8_t can_fd_mode;
    uint8_t can_fd_non_iso;

    /* Fill up up to 32 bytes */
    uint8_t unused[28];
} ZenoOpen;

typedef struct {
    ZenoHeader h;
    uint8_t channel;
    uint8_t base_clock_divisor;
    uint8_t is_master;
    
    /* Fill up up to 32 bytes */
    uint8_t unused[27];
} ZenoLinOpen;

typedef struct {
    ZenoHeader h;
    uint8_t response_cmd_id;
    uint8_t cmd_result_code;
    
    uint64_t clock_start_ref;
    uint16_t max_pending_tx_msgs;

    uint8_t base_clock_divisor;

    /* Fill up up to 32 bytes */
    uint8_t unused[17];
} ZenoOpenResponse;

typedef struct {
    ZenoHeader h;
    uint8_t channel;

    /* Fill up up to 32 bytes */
    uint8_t unused[29];
} ZenoClose;

typedef struct {
    ZenoHeader h;
    uint8_t channel;

    /* Fill up up to 32 bytes */
    uint8_t unused[29];
} ZenoBusOn;

typedef struct {
    ZenoHeader h;
    uint8_t channel;

    /* Fill up up to 32 bytes */
    uint8_t unused[29];
} ZenoBusOff;

typedef struct {
    ZenoHeader h;
    uint8_t channel;

    uint8_t brp;
    uint8_t tseg1;
    uint8_t tseg2;
    uint8_t sjw;

    /* For ECAN1 and ECAN2 */
    uint8_t  cancks; /* 1: FCAN = 2 * Fp - 0: FCAN = Fp */
    uint16_t cicfg1;
    uint16_t cicfg2;
    
    /* CAN FD - Transmitter delay compensation */
    uint8_t tdc_offset;
    uint8_t tdc_value;
    uint8_t tdc_ssp_mode_off;

    /* Fill up up to 32 bytes */
    uint8_t unused[17];
} ZenoBitTiming;

typedef struct {
    ZenoHeader h;
    uint8_t channel;
    uint8_t unused1;
    
    uint16_t bitrate;

    /* Fill up up to 32 bytes */
    uint8_t unused2[26];
} ZenoLinBitrate;

typedef struct {
    ZenoHeader h;
    uint8_t channel;
    uint8_t unused1;

    /* Fill up up to 32 bytes */
    uint8_t unused2[28];
} ZenoLinResetAutoBaud;

typedef struct {
    ZenoHeader h;
    uint8_t channel;
    uint8_t op_mode;

    /* Fill up up to 32 bytes */
    uint8_t unused[29];
} ZenoOpMode;

typedef struct {
    ZenoHeader h;
    
    uint8_t trans_id;
    uint8_t flags;
    
    uint32_t id;
    uint32_t timestamp;
    
    uint8_t data[8];

    uint8_t dlc;
    uint8_t channel;

    uint32_t timestamp_msb;

    /* Fill up up to 32 bytes */
    uint8_t unused2[6];
} ZenoCAN20Message;

typedef struct {
    ZenoHeader h;

    uint8_t trans_id;
    uint8_t flags;

    uint8_t pid;
    uint8_t checksum;
    
    uint64_t timestamp_start;
    uint64_t timestamp_end;
    
    uint8_t data[8];

    uint8_t dlc;
    uint8_t channel;
 } ZenoLINMessage;

typedef struct {
    ZenoHeader h;

    uint8_t trans_id;
    uint8_t flags;

    uint32_t id;
    uint32_t timestamp; // Should be 64 bits.
    //uint32_t timestamp_msb; // timestamp_msb is really missing... Probably need to rearrange the USB-messages
    // total data: 18+28+18 == 64

    uint8_t data[18];

    uint8_t dlc;
    uint8_t channel;
} ZenoCANFDMessageP1;

typedef struct {
    ZenoHeader h;
    
    uint8_t trans_id;
    uint8_t channel;

    uint8_t data[28];
} ZenoCANFDMessageP2;

typedef struct {
    ZenoHeader h;

    uint8_t trans_id;
    uint8_t channel;

    uint8_t data[18];

    /* Fill up up to 32 bytes */
    uint8_t unused[10]; // perhaps add timestamp_msg here?
} ZenoCANFDMessageP3;

typedef struct {
    ZenoHeader h;

    uint32_t id;
    uint32_t flags;

    uint8_t dlc;
    uint8_t channel;

    uint8_t data[8];

    /* Fill up up to 32 bytes */
    uint8_t unused2[12];
} ZenoTxCAN20Request;

typedef struct {
    ZenoHeader h;

    uint8_t pid;
    uint8_t is_master_request;
    uint32_t flags;

    uint8_t dlc;
    uint8_t channel;

    uint8_t data[8];

    /* Fill up up to 32 bytes */
    uint8_t unused2[14];
} ZenoTxLINRequest;

typedef struct {
    ZenoHeader h;

    uint32_t id;
    uint32_t flags;

    uint8_t dlc;
    uint8_t channel;

    uint8_t data[20];
} ZenoTxCANFDRequestP1;

typedef struct {
    ZenoHeader h;

    uint8_t dlc;
    uint8_t channel;

    uint8_t data[28];
} ZenoTxCANFDRequestP2;

typedef struct {
    ZenoHeader h;

    uint8_t dlc;
    uint8_t channel;

    uint8_t data[16];
} ZenoTxCANFDRequestP3;

typedef struct {
    ZenoHeader h;
    uint8_t trans_id;
    uint8_t flags;
    
    uint32_t id;
    uint32_t timestamp;

    uint8_t dlc;
    uint8_t channel;
    
    uint32_t timestamp_msb;

    /* Fill up up to 32 bytes */    
    uint8_t unused2[10];
} ZenoTxCANRequestAck;

typedef struct {
    ZenoHeader h;
    uint8_t trans_id;
    uint8_t flags;
    
    uint64_t timestamp_start;
    uint64_t timestamp_end;
    uint8_t channel;
    
    /* Fill up up to 32 bytes */    
    uint8_t unused2[11];
} ZenoTxLINRequestAck;

typedef struct {
    ZenoHeader h;

} ZenoTxAck;

typedef struct {
    ZenoHeader h;

    /*  Debug msg */
    uint8_t debug_msg[30];
} ZenoDebug;

typedef struct {
    ZenoHeader h;

    /* Fill up up to 32 bytes */
    uint8_t unused[30];
} ZenoStartStopClockInt;

typedef struct {
    ZenoHeader h;
    uint8_t channel;
    uint8_t pid;
    
    /* Fill up up to 32 bytes */
    uint8_t unused[28];
} ZenoLinClearMessage;

typedef struct {
    ZenoHeader h;
    uint8_t channel;
    
    /* Fill up up to 32 bytes */
    uint8_t unused[29];
} ZenoLinClearAllMessages;

#pragma pack(pop)

#endif /* ZENOCAN_H */
