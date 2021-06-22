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

#include "PCANBasic.h"
#include "zglobal.h"

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

DWORD CAN_Initialize(WORD channel_handle,
                     WORD baud_rate,
                     BYTE can_hw_type,
                     DWORD io_port,
                     WORD interrupt)
{
    return PCAN_ERROR_UNKNOWN;
}

DWORD CAN_InitializeFD(WORD channel_handle, LPSTR baud_rate)
{
    return PCAN_ERROR_UNKNOWN;
}

DWORD CAN_Uninitialize(WORD channel_handle)
{
    return PCAN_ERROR_UNKNOWN;
}

DWORD CAN_Reset(WORD channel_handle)
{
    return PCAN_ERROR_UNKNOWN;
}

DWORD CAN_GetStatus(WORD channel_handle)
{
    return PCAN_ERROR_UNKNOWN;
}

DWORD CAN_Read(WORD channel_handle,
               TPCANMsg* can_msg,
               TPCANTimestamp* timestamp)
{
    return PCAN_ERROR_UNKNOWN;
}

DWORD CAN_ReadFD(WORD channel_handle,
                 TPCANMsgFD* can_msg,
                 UINT64* timestamp)
{
    return PCAN_ERROR_UNKNOWN;
}

DWORD CAN_Write(WORD channel_handle, TPCANMsg* can_msg)
{
    return PCAN_ERROR_UNKNOWN;
}

DWORD CAN_WriteFD(WORD channel_handle, TPCANMsgFD* can_msg)
{
    return PCAN_ERROR_UNKNOWN;
}

DWORD CAN_FilterMessages(WORD channel_handle,
                         DWORD from_can_id,
                         DWORD to_can_id,
                         BYTE can_mode)
{
    return PCAN_ERROR_UNKNOWN;
}

DWORD CAN_GetValue(WORD channel_handle,
                   BYTE can_parameter,
                   void* buffer,
                   DWORD buffer_length)
{
    return PCAN_ERROR_UNKNOWN;
}

DWORD CAN_SetValue(WORD channel_handle,
                   BYTE Parameter,
                   void* Buffer,
                   DWORD BufferLength)
{
    return PCAN_ERROR_UNKNOWN;
}

DWORD CAN_GetErrorText(DWORD Error, WORD Language, LPSTR Buffer)
{
    return PCAN_ERROR_UNKNOWN;
}
