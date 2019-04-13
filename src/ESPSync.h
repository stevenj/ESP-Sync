/**
 *  ESP Sync protocol handler
 * 
 * Copyright (c) 2019 Sakura Industries Limited.
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ESPSYNC_H_
#define __ESPSYNC_H_

#include "Arduino.h"

class ESPSync
{
    public:
        ESPSync(void);

        void setSerial(HardwareSerial *streamObject);
        bool protocol_active(void);
        bool ProcessByte(uint8_t byte);

    private:
        HardwareSerial *_streamRef;
        uint8_t   _rxstate;
        uint16_t  _csum_hi;
        uint16_t  _csum_lo;

        uint8_t   _prev_cmn;
        uint8_t   _prev_fun;
        uint32_t  _prev_size;

        uint8_t   _this_cmn;
        uint8_t   _this_fun;
        uint32_t  _this_size;
        uint8_t   _data_size;

        uint8_t  _chk_mode;

        uint8_t  *_dbuf;

        void TX_Header(uint8_t func, uint32_t size_opt);
        void TX_NAK(uint8_t code);
        void TX_ACK(uint32_t timeout);

        void TX_DataChunk(uint32_t *chk, uint8_t size);

        void TX_DataBuf(uint8_t func, uint8_t size);

        void PROCESS_SetTime(void);
        void PROCESS_Format(void);
        void PROCESS_Listing(void);
        void PROCESS_Remove(void);
        void PROCESS_Rename(void);
        void PROCESS_FileRX(void);

        void MSG_Complete(void);

};

#endif