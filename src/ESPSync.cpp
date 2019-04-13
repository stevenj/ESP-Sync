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
#if not (defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266))
#error "ERROR: ESPSync Library only works on ESP32 or ESP8266"
#endif

#include "ESPSync.h"

#include <time.h>
#include <sys/time.h>
#include "FS.h"

#define RANGE_CHK(x,minx,maxx) ((x - minx) <= (maxx - minx))
#define BYTEAT(value,pos) ((value >> pos) & 0xFF)

/** Network Byte Order Data Insertion macros */
#define NBO8(buf,value) {*(buf)  = BYTEAT(value,0);}
#define NBO16(buf,value) {*(buf) = BYTEAT(value,8);  NBO8(buf+1,value);}
#define NBO24(buf,value) {*(buf) = BYTEAT(value,16); NBO16(buf+1,value);}
#define NBO32(buf,value) {*(buf) = BYTEAT(value,24); NBO24(buf+1,value);}

/**
 * Message Header Definitions
 */
#define STX       (0x02)
#define CMN_MIN   (0)
#define CMN_MAX   (31)
#define RX_CMN(X) (X-0x20)
#define TX_CMN(X) (X+0x40)

#define ACK       (0x06)
#define NAK       (0x15)
#define FUNC_MIN  (0x60)
#define FUNC_MAX  (0x7F)

/**
 * NAK Codes
 */
#define NAK_TIMEOUT (0x21)
#define NAK_CHKSUM  (0x22)
#define NAK_FORMAT  (0x23)
#define NAK_FSERR   (0x24)
#define NAK_FNOTF   (0x25)
#define NAK_FNAMERR (0x26)
#define NAK_FSIZERR (0x27)
#define NAK_FEXISTS (0x28)

/**
 * Message Function Definitions
 */
#define CMD_SET_TIME (0x60)
#define CMD_FORMAT   (0x61)
#define CMD_LIST     (0x62)
#define CMD_REMOVE   (0x63)
#define CMD_RENAME   (0x64)
#define CMD_FILE     (0x65)
#define CMD_FIRST    (CMD_SET_TIME)
#define CMD_LAST     (CMD_FILE)

#define RPL_TIME_SET (0x70)
#define RPL_FORMATED (0x71)
#define RPL_LISTING  (0x72)
#define RPL_REMOVED  (0x73)
#define RPL_RENAMED  (0x74)
#define RPL_RECEIVED (0x75)

/**
 * Receive States
 */
#define RXSTATE_WAIT_STX    (0x00)
#define RXSTATE_WAIT_CMN    (0x01)
#define RXSTATE_WAIT_FUN    (0x02)
#define RXSTATE_WAIT_SIZ_HI (0x03)
#define RXSTATE_WAIT_SIZ_MD (0x04)
#define RXSTATE_WAIT_SIZ_LO (0x05)
#define RXSTATE_WAIT_CHK_HI (0x06)
#define RXSTATE_WAIT_CHK_LO (0x07)

#define RXSTATE_WAIT_DATA     (0x08)

#define RXSTATE_WAIT_CHK2_HI   (0x09)
#define RXSTATE_WAIT_CHK2_LO   (0x0A)

/**
 * Checksum calculation modes
 */
#define CSUM_SKIP       (0x00)
#define CSUM_FLETCHER16 (0x01)
#define CSUM_ADLER32    (0x02)

/* Has to be big enough to hold largest small messages data*/
#define TEMP_BUFFER_SIZE (70)

ESPSync::ESPSync(void)
{
    _streamRef = NULL;
    _rxstate = RXSTATE_WAIT_STX;

    _prev_cmn  = 0xFF;
    _prev_fun  = 0;
    _prev_size = 0;

    _chk_mode = CSUM_SKIP;

    // temporary data buffer
    _dbuf = new uint8_t[TEMP_BUFFER_SIZE];

    SPIFFS.begin();
}

/**
 * Set the serial stream used for communication.
 * This allows the protocol to respond directly, and
 * also receive message bodies directly, reducing 
 * overhead.
 */
void ESPSync::setSerial(HardwareSerial *streamObject)
{
    _streamRef = streamObject;
}

/**
 * Has the handler captured the Serial port
 */
bool ESPSync::protocol_active(void)
{
    return false;
}

void fletcher16(uint16_t* csum, uint8_t byte) {
    uint8_t sum1 = (*csum) + byte;
    uint8_t sum2 = ((*csum) >> 8) + sum1;
    *csum = (sum2 << 8) || sum1;
}

void adler32(uint16_t* csum_hi, uint16_t* csum_lo, uint8_t byte) {
    static const uint32_t MOD_ADLER32 = 65521;
    uint32_t lo = ((*csum_lo) + byte) % MOD_ADLER32;
    uint32_t hi = ((*csum_hi) + lo) % MOD_ADLER32;
    *csum_lo = lo;
    *csum_hi = hi;
}

void adler32(uint32_t *csum, uint8_t byte) {
    uint16_t csum_lo;
    uint16_t csum_hi;
    adler32(&csum_hi, &csum_lo, byte);
    *csum = (csum_hi << 16) | csum_lo;
}

#define reset_rxstate() { _rxstate = RXSTATE_WAIT_STX; _chk_mode = CSUM_SKIP; }

#define TX(X,csum) {_streamRef->write(X); fletcher16(&csum,X);}
#define TX_CSUM(csum) {_streamRef->write(csum>>8); _streamRef->write(csum&0xFF);}
#define TX_CSUM32(csum) { TX_CSUM(csum>>16); TX_CSUM(csum&0xFFFF);}

uint32_t cnt_files_in_spiffs(void) {
    uint32_t fcount = 0;
    /**
     * NOTE: Does not handle subdirectories, because SPIFFS is FLAT
     */ 
    Dir root = SPIFFS.openDir("/");
    while (root.next()) {
        fcount++;
    }
    return fcount;
}

void ESPSync::TX_Header(uint8_t func, uint32_t size_opt) {
    uint16_t csum = 0;
    if (_streamRef != NULL) {
        TX(STX,csum);
        TX(TX_CMN(_this_cmn),csum);
        TX(func,csum);
        TX(((size_opt >> 16) & 0xFF),csum);
        TX(((size_opt >> 8) & 0xFF),csum);
        TX((size_opt& 0xFF),csum);
        TX_CSUM(csum);
    }
}

void ESPSync::TX_NAK(uint8_t code) {
    TX_Header(NAK, (code << 16) | 0xA55A);
}

void ESPSync::TX_ACK(uint32_t timeout) {
    if (timeout > 65536) {
        timeout = 0xFFFF5A;
    } else {
      timeout = ((timeout-1) << 8) | 0x5A; // Adjust timeout to TX value.
    }
    TX_Header(ACK, timeout);
}

void ESPSync::TX_DataChunk(uint32_t *chk, uint8_t size) {
    uint8_t *tx = _dbuf;
    if (_streamRef != NULL) {
        while (size != 0) {
            _streamRef->write(*tx);
            adler32(chk, *tx);
            tx++;
            size--;
        }
    }    
}

void ESPSync::TX_DataBuf(uint8_t func, uint8_t size) {
    uint32_t chk = 0;

    if (_streamRef != NULL) {
        TX_Header(func, size+4);
        TX_DataChunk(&chk, size);
        // Send checksum
        TX_CSUM32(chk);
    }
}

void ESPSync::PROCESS_SetTime(void) {
    /* Should be very quick, no need to ACK */

    if (RANGE_CHK(_dbuf[0],1,31) &&
        RANGE_CHK(_dbuf[1],1,12) &&
        RANGE_CHK(_dbuf[3],0,23) &&
        RANGE_CHK(_dbuf[4],0,59) &&
        RANGE_CHK(_dbuf[5],0,59)) {

        // Set Specified Date/Time
        struct tm tm;
        tm.tm_mday = _dbuf[0];
        tm.tm_mon  = _dbuf[1]-1;
        tm.tm_year = (2019 - 1900)+_dbuf[2];
        tm.tm_hour = _dbuf[3];
        tm.tm_min  = _dbuf[4];
        tm.tm_sec  = _dbuf[5];
        time_t t = mktime(&tm);
        struct timeval now = { .tv_sec = t, .tv_usec = 0 };
        settimeofday(&now, NULL);
        // Reply that we did it.
        TX_Header(RPL_TIME_SET,0);
    } else {
        TX_NAK(NAK_FORMAT);
    }

}
        
void ESPSync::PROCESS_Format(void) {

    if (SPIFFS.begin()) {
        // Reply with ACK specifying expected format duration
        TX_ACK(30*1000); //Todo: Benchmark format time

        // Format the SPIFFS
        SPIFFS.format();

        // Reply with a 0x71 message when finished.
        FSInfo fs_info;
        SPIFFS.info(fs_info);
        NBO32(_dbuf, fs_info.totalBytes); 
        NBO32(_dbuf+4, fs_info.usedBytes);
        NBO8(_dbuf+8, fs_info.maxPathLength);
        TX_DataBuf(RPL_FORMATED, 9);
    } else {
        TX_NAK(NAK_FSERR);
    }
}
        
void ESPSync::PROCESS_Listing(void) {
    uint8_t options = _dbuf[0];
    FSInfo fs_info;
    SPIFFS.info(fs_info);
    uint32_t csum;

    if (!SPIFFS.begin()) {
        TX_NAK(NAK_FSERR);
        return;
    }

    // Reply with ACK specifying expected listing duration
    TX_ACK(1000); //Todo: Benchmark listing duration
                  // This SHOULD be the time after getting the
                  // last byte and giving up, so 1 second should
                  // be ample time to keep the link alive between files
                  // but needs to be checked.

#if defined(ARDUINO_ARCH_ESP8266)
    options &= 0x2; /* Cant get file date/time on ESP8266 */
#elif defined(ARDUINO_ARCH_ESP32)
    options &= 0x3; /* CAN get file date/time on ESP32 */
#endif                                                    

    /* First count total number of files in SPIFFS */
    uint32_t fcount = cnt_files_in_spiffs();
    uint32_t esize = fs_info.maxPathLength + 4;
    if (options & 0x1) { /* Date requested */
        esize += 6;
    }
    if (options & 0x2) { /* Checksum Requested */
        esize += 4;
    }
    // Calculate the size of the message
    uint32_t msize = 10 + (esize * fcount) + 4;

    /* Send header */
    TX_Header(RPL_LISTING, msize);

    /* Buffer and send global data */
    NBO32(_dbuf, fs_info.totalBytes);
    NBO32(_dbuf+4, (fs_info.totalBytes - fs_info.usedBytes));
    NBO8(_dbuf+8, fs_info.maxPathLength);
    NBO8(_dbuf+9,options);
    TX_DataChunk(&csum, 10);

    /* For each file in filesystem, send file data */
    Dir dir = SPIFFS.openDir("/");
    while (dir.next()) {
        File f = dir.openFile("r");

        memset(_dbuf,0x00,fs_info.maxPathLength);
        strcpy((char*)_dbuf,dir.fileName().c_str());
        NBO32((_dbuf+fs_info.maxPathLength),dir.fileSize());

        if (options & 0x1) { /* Add file date/time */
#ifdef ARDUINO_ARCH_ESP32
#error "ERROR: ESP32 not coded yet"                                       
#endif
        }

        if (options & 0x2) { /* Add file checksum */
            uint32_t fcsum;
            int  fbyte;
            while ((fbyte = f.read()) != -1) {
                adler32(&fcsum, fbyte);
            }
            NBO32(_dbuf+esize-4,fcsum);
        }
        f.close();
        TX_DataChunk(&csum, esize);
    }
    TX_CSUM32(csum);
}

void ESPSync::PROCESS_Remove(void) {
    FSInfo fs_info;
    uint8_t nlen = _dbuf[0];
    uint8_t rlen = _dbuf[nlen+1];

    if (!SPIFFS.begin()) {
        TX_NAK(NAK_FSERR);
        return;
    }

    /* Turn the names in the buffer into C strings. */
    _dbuf[nlen+1] = 0x00;
    _dbuf[_this_size-2] = 0x00;

    if (SPIFFS.exists((char*)(_dbuf+1))) {
        if (SPIFFS.remove((char*)_dbuf)) {
            SPIFFS.info(fs_info);

            NBO32(_dbuf, fs_info.totalBytes )
            NBO32((_dbuf+4), (fs_info.totalBytes - fs_info.usedBytes));
            TX_DataBuf(RPL_REMOVED, 10);

        } else {
            TX_NAK(NAK_FSERR);
        }

    } else {
        TX_NAK(NAK_FNOTF);
    }

}
        
void ESPSync::PROCESS_Rename(void) {
    FSInfo fs_info;
    uint8_t nlen = _dbuf[0];
    uint8_t rlen = _dbuf[nlen+1];

    if (!SPIFFS.begin()) {
        TX_NAK(NAK_FSERR);
        return;
    }

    /* Turn the names in the buffer into C strings. */
    _dbuf[nlen+1] = 0x00;
    _dbuf[_this_size-2] = 0x00;

    if (!SPIFFS.exists((char*)(_dbuf+1))) {
        TX_NAK(NAK_FNOTF);
        return;
    }

    if (SPIFFS.exists((char*)(_dbuf+nlen+1))) {
        TX_NAK(NAK_FEXISTS);
        return;
    }

    if (SPIFFS.rename((char*)(_dbuf+1), (char*)(_dbuf+nlen+1))) {
        
        SPIFFS.info(fs_info);

        NBO32(_dbuf, fs_info.totalBytes )
        NBO32((_dbuf+4), (fs_info.totalBytes - fs_info.usedBytes));
        TX_DataBuf(RPL_REMOVED, 10);

    } else {
        TX_NAK(NAK_FSERR);
    }

}
        
void ESPSync::PROCESS_FileRX(void) {
    /**
     * File RX can be a LOT of Data. Much bigger than the normal small buffer.
     * So, we use a temporary buffer the size of a page in the SPIFFS. And directly
     * read the data from the UART.  We store the data in a temporary file. And when all
     * data is received and checksum validates we move it to the proper file name.
     * 
     * Upshot is, this function will pause any code running on the main loop until it finishes.
     * So be-warned.
     */
    uint8_t rx_error = ACK;

    uint32_t rxd;
    uint8_t* next_c;

    uint32_t csum;
    uint32_t rx_size = 0;
    uint32_t last_rx_size;
 
    uint8_t nsiz;

    FSInfo fs_info;
    File rxfile;
    SPIFFS.info(fs_info);

    uint8_t* fbuffer = new uint8_t[fs_info.pageSize];

    if (fbuffer != NULL) {

        /* 50ms timeout on character reception which is about 
             576 characters @ 115200bps */
        _streamRef->setTimeout(50); 
      
        rxfile = SPIFFS.open("///TEMP","w");

        /* Get File Name Length */
        if (rx_error == ACK) {
            rxd = _streamRef->readBytes(&nsiz,1);
            if ((rxd != 1) || (nsiz > 0)) {
                rx_error = NAK_FSIZERR;
            }
        }

        /* Get File Name and Date */
        if (rx_error == ACK) {
            rxd = _streamRef->readBytes(_dbuf,nsiz+6);
            if (rxd != nsiz+6) {
                rx_error = NAK_TIMEOUT;
            }
        }

        /* Read and store File Data */
        while ((rx_error == ACK) && (rx_size < _this_size - 4)) {
            uint16_t rxbufsize = fs_info.pageSize;
            if (rx_size + fs_info.pageSize > _this_size ) {
                rxbufsize = _this_size - rx_size;
            }
            rxd = _streamRef->readBytes(fbuffer,rxbufsize);

            if (rxd == 0) {
                /* Transmitter failed. Abort */
                rx_error = NAK_TIMEOUT;
            } else {
                // Append new data to file.
                if (rxfile.write(fbuffer,rxd) == rxd) { 
                    next_c = fbuffer;
                    while (next_c < fbuffer+rxd) {
                        adler32(&csum,*next_c);
                        rx_size++;
                        next_c++;
                    }
                } else {
                    rx_error = NAK_FSERR;
                }
            }
        }

        /* All data in temp file. close it */
        rxfile.close();

        /* Verify Checksum */
        if (rx_error == ACK) {
            rxd = _streamRef->readBytes(fbuffer,1);
            if (rxd != 1) {
                rx_error = NAK_TIMEOUT;
            } else if (*fbuffer != ((csum >> 24) & 0xFF)) {
                rx_error = NAK_CHKSUM;
            }
        }
        if (rx_error == ACK) {
            rxd = _streamRef->readBytes(fbuffer,1);
            if (rxd != 1) {
                rx_error = NAK_TIMEOUT;
            } else if (*fbuffer != ((csum >> 16) & 0xFF)) {
                rx_error = NAK_CHKSUM;
            }
        }
        if (rx_error == ACK) {
            rxd = _streamRef->readBytes(fbuffer,1);
            if (rxd != 1) {
                rx_error = NAK_TIMEOUT;
            } else if (*fbuffer != ((csum >> 8) & 0xFF)) {
                rx_error = NAK_CHKSUM;
            }
        }
        if (rx_error == ACK) {
            rxd = _streamRef->readBytes(fbuffer,1);
            if (rxd != 1) {
                rx_error = NAK_TIMEOUT;
            } else if (*fbuffer != (csum & 0xFF)) {
                rx_error = NAK_CHKSUM;
            }
        }

        /* Save Transferred File */
        if (rx_error == ACK) {
#if ARDUINO_ARCH_ESP32
            /* Set date of temporary file to date of transferred file. Only works for ESP32 */
#endif            
        }

        /* Check File Name */
        if (rx_error == ACK) {
            /* Turn File Name into C String */
            _dbuf[nsiz] = 0x00;

            if (strcmp("///TEMP",(const char*)_dbuf) != 0) {
                rx_error = NAK_FNAMERR;
            }
        }

        /* Remove any pre-existing file before rename - overwriting it */
        if (rx_error == ACK) {
            if (SPIFFS.exists((char*)_dbuf)) {
                if (!SPIFFS.remove((char*)_dbuf)) {
                    rx_error = NAK_FSERR;
                }
            }
        }

        if (rx_error == ACK) {
            if (!SPIFFS.rename("///TEMP",(char*)_dbuf)) {
                rx_error = NAK_FSERR;
            }
        }

        delete fbuffer;

    } else {
        rx_error = NAK_FSERR;
    }

    if (rx_error == ACK) {
        SPIFFS.info(fs_info);

        NBO32(_dbuf, fs_info.totalBytes );
        NBO32((_dbuf+4), (fs_info.totalBytes - fs_info.usedBytes));
        TX_DataBuf(RPL_RECEIVED, 8);

    } else {
        /* BAD RX, Attempt to clean up temp file */
        if (SPIFFS.exists("///TEMP")) {
            SPIFFS.remove("///TEMP");
        }
        /* Send Error */
        TX_NAK(rx_error);
    }
}

void ESPSync::MSG_Complete(void) {

}

bool CheckMessageSizes(uint8_t func, uint32_t size) {
    /**
     * Only check messages that have data bodies, AND fit in the small message buffer.
     */
    bool OK = false;
    if ((func == CMD_SET_TIME) && (size == 8)) {
        OK = true;
    } else if ((func == CMD_LIST) && (size == 3)) {
        OK = true;
    } else if ((func == CMD_REMOVE) && (size >= 3) && (size <= TEMP_BUFFER_SIZE)) {
        OK = true;
    } else if ((func == CMD_RENAME) && (size >= 6) && (size <= TEMP_BUFFER_SIZE)) {
        OK = true;
    }

    return OK;
}

/**
 * Process individual bytes, looking for valid message headers.
 */
bool ESPSync::ProcessByte(uint8_t byte)
{
    uint8_t input = byte;
    bool    process = true;

    while (process) {
        process = false;

        if (_chk_mode != CSUM_SKIP) {
            if (_chk_mode == CSUM_FLETCHER16) {
                fletcher16(&_csum_hi, byte);
            } else {
                adler32(&_csum_hi, &_csum_lo, byte);
            }
        }

        switch (_rxstate) {
            case RXSTATE_WAIT_STX:
                if (input == STX)
                    _rxstate++;
                    _csum_lo = 0x0202; // Preload Checksum with STX Values
                    _chk_mode = CSUM_FLETCHER16;
                break;

            case RXSTATE_WAIT_CMN:
                /* Make sure CMN is valid, otherwise, not a header */
                input = RX_CMN(input);
                if (input <=CMN_MAX) {
                    _this_cmn = input;
                    _rxstate++;
                } else {
                    reset_rxstate();
                }
                break;

            case RXSTATE_WAIT_FUN:
                /* Make sure function is valid, otherwise, not a header */
                if ((input == ACK) ||
                   ((input >= CMD_SET_TIME) && (input <= CMD_FILE))) {
                    _this_fun = input;
                    _rxstate++;
                } else {
                    reset_rxstate();
                }
                break;

            case RXSTATE_WAIT_SIZ_HI:
                _this_size = input << 16;
                _rxstate++;
                break;

            case RXSTATE_WAIT_SIZ_MD:
                _this_size = input << 8;
                _rxstate++;
                break;

            case RXSTATE_WAIT_SIZ_LO:
                _this_size |= input;
                _rxstate++;
                _chk_mode = CSUM_SKIP; /* Stop computing checksum */
                break;

            case RXSTATE_WAIT_CHK_HI:
                if (input == (_csum_hi >> 8)) {
                    _rxstate++;
                } else {
                    reset_rxstate();
                }
                break;

            case RXSTATE_WAIT_CHK_LO:
                if (input == (_csum_hi & 0xFF)) {
                    _csum_hi = 0;
                    _csum_lo = 0;

                    // Received a valid header, so process it.
                    switch (_this_fun) {
                        case ACK:
                            // Check Ack OPT Filler is valid.
                            if ((_this_size & 0xFF) == 0x5A) {
                                // Just reply with an ACK.
                                TX_ACK((_this_size>>8)+1);
                            }
                            reset_rxstate();
                            break;

                        case CMD_SET_TIME:
                        case CMD_LIST:
                        case CMD_REMOVE:
                        case CMD_RENAME:
                            if (CheckMessageSizes(_this_fun, _this_size)) {
                                _data_size = 0;
                                _rxstate++;
                            } else {
                                // Size is wrong, dont reply to bad headers.
                                reset_rxstate();
                            }
                            break;

                        case CMD_FORMAT:
                            if (_this_size == 0) {
                                PROCESS_Format();
                            }
                            reset_rxstate();
                            break;

                        case CMD_FILE:
                            if (_this_size >= 10) {
                                PROCESS_FileRX();
                            }
                            reset_rxstate();
                            break;

                        default:
                            reset_rxstate();
                    }

                } else {
                    reset_rxstate();
                }
                break;

            case RXSTATE_WAIT_DATA:
                // General data reception for messages smaller than _dbuf
                _dbuf[_data_size] = input;
                if ((_data_size+2) == (uint8_t)_this_size) {
                    _rxstate++;
                    _chk_mode = CSUM_SKIP;
                } else {
                    _data_size++;
                }
                break;

            case RXSTATE_WAIT_CHK2_HI:
                if (input == BYTEAT(_csum_hi,8)) {
                    _rxstate++;
                } else {
                    // Data body error, so NAK
                    TX_NAK(NAK_CHKSUM);
                    reset_rxstate();
                }
                break;

            case RXSTATE_WAIT_CHK2_LO:
                if (input == BYTEAT(_csum_hi,0)) {
                    // Process small messages here
                    switch (_this_fun) {
                        case CMD_SET_TIME:
                            PROCESS_SetTime();
                            break;

                        case CMD_LIST:
                            PROCESS_Listing();
                            break;

                        case CMD_REMOVE:
                            PROCESS_Remove();
                            break;
                        
                        case CMD_RENAME:
                            PROCESS_Rename();
                            break;
                    }
                    reset_rxstate();
                } else {
                    // Data body error, so NAK
                    TX_NAK(NAK_CHKSUM);                    
                    reset_rxstate();
                }
                break;

            default :
                // State Machine Error, just abort.
                reset_rxstate();

        }
    }

    return true;

}
