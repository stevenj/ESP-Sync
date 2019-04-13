<!-- markdownlint-disable MD033 -->
# ESP-Sync

ESP32/8266 Serial File Transfer and Sync

This is a single file library and companion tool written in Python to allow easy and efficient syncing of the contents of a native file system with the SPIFFS on an ESP8266 or ESP32.

The library works by inserting itself into the serial stream, if it detects sync data, it will automatically start syncing the file system contents.

There are tools already to do this, but they create a whole binary SPIFFS image and then flash that which is SLOW.  Especially for modules with large flashes.

Eg, To update a file that is 1K, at 115200 bps, on a 14MB SPIFFS, uploading a image will take more than 21 minutes.  Even at a theoretical maximum data rate of 3Mbps (Max for USB to UART Interfaces) the filesystem sync will take ~49 seconds.  Unbearably slow when one is developing and needing to iterate web page designs.  

Whereas if the transfer was efficient, the 1K file would take ~0.08 seconds to transfer at 115200.

I explored using off the shelf protocols, the most applicable seemed to be ZMODEM, but all existing code is horrible, probably because the specification is horrible.

So I rolled my own, simple transparent file transfer protocol for serial lines.

## Packet Format

The packet is **BASED** on DDCMP principles, but is not a DDCMP packet.  The advantage of a DDCMP type packet is its data transparent, there is no need to escape bytes, like Async HDLC, etc, does.

A Packet has an 8 byte Header, and an optional Data section (Up to 16MB in size)  This means there is fixed overhead, which is large for small files, but insignificant for large files.

| Code | Value | Description |
| ---- | ----- | ----------- |
| STX  | 0x02  | Start Message Sentinel |
| CMN  | 0x20-0x3F<br>0x40-0x5F | Request Cyclic Message Number <br> Reply Cyclic Message Number |
| FUN  | 0x06[ACK],0x15[NAK],0x60-0x7E | Function Code |
| SIZ<br>OPT  | 0x000000-0xFFFFFF<br>Option Data | Data Payload Size <br>Function Option Data|
| CHK  | 0x0000-0xFFFF | FLETCHER-16 checksum of all data from STX to the end of SIZ/OPT |
| DATA | 0x00-0xFF x N | Variable Length data body. |
| CHK2 | 0x00000000-0xFFFFFFFF | ADLER-32 for Data Payloads |

ALL Multi-byte fields are sent MOST SIGNIFICANT BYTE FIRST (Network Byte Order).

### STX - Start Message Sentinel

This is a traditional start of message character for asynchronous protocols, and is defined by ASCII as 0x02.

### CMN - Cyclic Message Number

This message number starts at 0x20 and for each unique message sent increments to 0x3F where it wraps back to 0x20.  The reply will echo this CMN but will add 0x20 to it before replying.  The CMN is used to detect duplicate transmissions, in a re-transmission scenario.

### FUN - Function Code

The Function of the message. Defines how the Size/Option bytes will be interpreted.

| CODE | FUNCTION | Description |
| ---- | -------- | ----------- |
| 0x06 | ACK      | Optional, sent immediately after a valid header if processing may be slow. [OPT] |
| 0x15 | NAK      | Sent IF there is some problem with the data payload only.  Header errors are ignored. [OPT] |
| 0x60 | Set Time | Set RTC Time [SIZE] |
| 0x61 | Format   | Format the SPIFFS [SIZE] |
| 0x62 | List     | Get complete file listing of the SPIFFS [SIZE] |
| 0x63 | Remove   | Remove the named file [SIZE] |
| 0x64 | Rename   | Rename a file [SIZE] |
| 0x65 | File     | Send a File [SIZE] |
| 0x70 | Time Set   | Response to the set time command [SIZE] |
| 0x71 | Formated   | Response to the reply command [SIZE] |
| 0x72 | Listing    | Response to the list command [SIZE] |
| 0x73 | Removed    | Response to the Remove command [SIZE] |
| 0x74 | Renamed    | Response to the Rename command [SIZE] |
| 0x75 | Received   | Response to the File command [SIZE] |

### SIZ / OPT - Data Size or Function option

For most messages, this is the size of the data payload following the header.  For ACK and NAK there are no data payloads and these three bytes are options associated with the ACK or NAK reply.

#### SIZ - Data Payload Size

A Value of 0x000000 to 0xFFFFFF sent Most Significant Byte first.  The size is this value allowing from 0 bytes thru to 16,777,215 bytes (16MB-1) of data to be sent following the header.  This limits a single file to a size of 16MB-1, but given the largest ESP32/8266 Flash is only 16MB this is not a practical limitation.  If the payload size is 0, no data payload follows the header.

#### OPT - Function Option

The control messages ACK and NAK have no data body.  Instead this is an option value for the function.

In the case of ACK, which is only sent in the case of a long running function, the first 2 bytes of the OPT value specifies the number of milliseconds the sender should wait to receive a reply, before concluding the message failed.  The last byte is unused and MUST be 0x5A.  This response is optional, short run functions can immediately reply with their response.  The ACK can be sent before all Data in the Data payload has been transmitted, provided the header is valid.  The timeout is +1 this value giving timeouts of 1ms to ~1 Minute in length.  The timeout is sent Most Significant Byte first.

In the case of NAK, the first byte is an error code, the remaining 16 bits may specify additional data related to the error, and IF they are not used are set to 0xA55A.

### CHK - Header Checksum

This is the Fletcher-16 Checksum of all bytes from STX through to the end of SIZ/OPT.  This checksum was chosen because it is almost as good as a CRC-16 but uses far less program and data space, and is very fast.

### DATA - Data Payload

If the message data size is not zero (True for every message other than ACK/NAK) then immediately following the header is the RAW Binary data being transmitted, there is no character escaping.  The number of bytes are declared by the header, and the size excludes the checksum that follows the data.

### CHK2 - Checksum of the Data Payload

The 32 bit data payload checksum is an ADLER-32 checksum of all bytes in the Data payload ONLY.

The checksum value is appended to the data stream Most significant byte first.  

If the checksum fails to evaluate correctly, the Slave will send a NAK indicating a checksum error, and no action is taken on the data.  In the case of a master receiving a data payload checksum error, the master can resend the original request, to trigger a re-transmission of the reply, OR can abort the transfer.

## Basic Protocol Interaction

The Master sends a message, and the receiver either processes it and replies, OR ignores it if there is an error in the formatting of the header.  The only Negative replies are sent in response to a data payload error.

This allows the protocol to transparently co-exist with other serial communications on the same line, IF and only IF a properly formatted header is received will the receiver process the messages.

## Messages

### 0x06, ACK - Acknowledgement

If a Slave receives a command from the Master that may take some long time to execute, it can send the Master an ACK reply, which signals that the message is being processed, but that it could take "up to" the number of milliseconds specified to complete, so the Master can wait the appropriate time before retrying.  It is optional.

As a special case, A Master may send an ACK message to a slave.  The slave does nothing, except reply with an ACK message.

### 0x15, NAK - Negative Acknowledgement

A Master never sends NAK, and a Slave ignores it if received.

IF and ONLY IF there is a data payload error, the Slave will send an NAK signaling the error.  Error codes are:

| Code | Error | Description |
| ---- | ----- | ----------- |
| 0x21 | TIMEOUT | Data payload not received in time |
| 0x22 | CHKSUM  | Data payload checksum in error |
| 0x23 | FORMAT  | Data payload formatting error detected |
| 0x24 | FSERR   | File System Error occurred |
| 0x25 | FNOTF   | File Not Found |
| 0x26 | FNAMERR | File Name too long or contains invalid characters |
| 0x27 | FSIZERR | File Too Big |
| 0x28 | FEXISTS | File already exists |

### 0x60, Set Time - Set RTC Time

This message allows the Master to set the time of the Slave, so that file operations can be properly recorded with the time.

The Data is the Time:
| Field | Size | Description |
| ----- | ---- | ----------- |
| DAY   | 1    | Day (1-31) |
| MONTH | 1    | Month (1-12) |
| YEAR  | 1    | Year (0-255) Where the Year is encoded as years since 2019. |
| HOUR  | 1    | Hour (0-23) |
| MINUTE | 1   | Minute (0-59) |
| SECOND | 1   | Second (0-59) |
| CHK2   | 4   | Checksum of DAY thru SECOND |

IF the data is valid, then the time is set as requested, and an 0x70 Time Set message is returned as a reply.

Otherwise the time is not changed, a NAK is returned, and the Error code is 0x23 FORMAT.

### 0x70, Time Set - The Time has been set

This message is sent in reply to 0x60 and contains no data.  Data size is 0.

### 0x61, Format - Format the SPIFFS

This message instructs the Slave to format its SPIFFS.  Formatting can take a long time, the Slave should reply with an ACK, indicating the maximum duration the Master should wait for the Format to complete, followed by a 0x71, Formatted reply when the format operation is complete.

If the Master does not see the ACK or 0x71 reply, it may re-transmit the format message.  If the CMN remains unchanged, the Slave will simply reply with the status of the operation.  Otherwise if CMN is changed, a new format is started.  

It is valid for the Slave to ignore further messages received during the format.  The Master should not send further messages until a 0x71 is received, or a timeout occurs, triggering a retransmit or a failed communications session.

If the format fails due to an error in the SPIFFS code, a NAK with a 0x24 FSERR code will be sent, aborting the session.

### 0x71, Formatted - The SPIFFS has been formatted

Indicates the 0x61 command completed successfully.  The reply contains the available SPIFFS size.

The Data is the SPIFFS Size:
| Field | Size | Description |
| ----- | ---- | ----------- |
| SIZE  | 4    | Size of the SPIFFS |
| USED  | 4    | Number of bytes used by the SPIFFS |
| NSIZ  | 1    | Maximum Name Length |
| CHK2  | 4    | Checksum of SIZE thru NSIZ |

### 0x62, List - Get complete file listing of the SPIFFS

This instructs the Slave to return a list of ALL files on the SPIFFS.  The slave replies with a 0x72, Listing reply.  There is a single byte of data, so the data size is 1.

The Data is:
| Field | Size | Description |
| ----- | ---- | ----------- |
| OPT | 1   | Bit 0 = 0 -> No Time Requested. <br>Bit 0 = 1 -> File Time Requested <br> Bit 1 = 0 -> Adler-32 Checksum NOT Requested. <br>Bit 1 = 1 -> Adler-32 Checksum of each file Requested.<br>Bit 2-7 Ignored |
| CHK2  | 4    | Checksum of OPT |

### 0x72, Listing - Response to the list command

This reply contains a full list of all files in the SPIFFS and relevant information about them.  The first fields specify global data about the SPIFFS, the following fields is a list of file entries.

The Data is:
| Field | Size | Description |
| ----- | ---- | ----------- |
| SIZE  | 4    | Size of the SPIFFS |
| FREE  | 4    | Free space in the SPIFFS |
| NSIZ  | 1    | The MAX SIZE of a file name |
| OPT | 1   | Bit 0 = 0 -> File Time Not Present. <br>Bit 0 = 1 -> File Time Present <br> Bit 1 = 0 -> Adler-32 Checksum NOT Present. <br>Bit 1 = 1 -> Adler-32 Checksum of each file Present.<br>Bit 2-7 Ignored |
| FILE 1 | X   | File 1 fields |
| ... | ... | ... |
| FILE N | X | File N fields. |
| CHK2  | 4    | Checksum of all Data |

The File fields are:
| Field | Size | Description |
| ----- | ---- | ----------- |
| NAME | X | The Files Name, this is fixed length and always = NSIZ, if the filename is shorter, it is padded with NULL 0x00 bytes |
| FSIZ | 4 | The Size of the File |
| DATE | 6 | Modified Time and Date of the File (ONLY PRESENT IF OPT SPECIFIES IT, Otherwise Field not present) |
| FCHK | 4 | File Adler-32 Checksum (ONLY PRESENT IF OPT SPECIFIES IT, Otherwise Field not present) |

Each File entry is fixed size, and so is the number of files in the file system, which allows the Slave to quickly calculate the size of the reply, for inclusion in the header, without pre-buffering the entire reply.  The File Entry fields can be returned progressively, and as the Checksum may take some time to complete, there may be a pause between individual file entries.  The Master is to buffer the reply, and it is ONLY valid if the checksum matches a the end.

The DATE field has the exact same format as the fields in the Set Time message.

### 0x63, Remove - Remove the named file

Causes the file named in the data to be deleted.  The Slave will reply with a ACK, if the delete operation will take a significant amount of time, and will indicate how long the Master should wait before retrying or giving up.  Once the delete operation is complete, the Slave will reply with 0x73, Removed.  The Data is simply the file name to delete.  If the file does not exist, the Slave will reply with a NAK, and a FNOTF Error code.

The Data is:
| Field | Size | Description |
| ----- | ---- | ----------- |
| NAME  | X    | The Name of the File to delete, not padded |
| CHK2  | 4    | Checksum of NAME |

### 0x73, Removed - File was removed

Reply to the Remove File command.  Data indicates the maximum and remaining space available if the SPIFFS.

The Data is:
| Field | Size | Description |
| ----- | ---- | ----------- |
| SIZE  | 4    | Size of the SPIFFS |
| FREE  | 4    | Free space in the SPIFFS |
| CHK2  | 4    | Checksum of SIZE & FREE |

### 0x64 - Rename - Rename a File

Causes the file named in the data to be renamed to the second name in the data.  The Slave will reply with a ACK, if the rename operation will take a significant amount of time, and will indicate how long the Master should wait before retrying or giving up.  Once the rename operation is complete, the Slave will reply with 0x74, Renamed.  The Data is simply the file name to rename, and the new name.  If the file does not exist, the Slave will reply with a NAK, and a FNOTF Error code.  If the file to rename to already exists, it will not be over written and a NAK will reply with FEXISTS Error Code.

The Data is:
| Field | Size | Description |
| ----- | ---- | ----------- |
| NLEN  | 1    | The length of the File Name |
| NAME  | X    | The Name of the File to rename, not padded, no zero termination |
| RLEN  | 1    | The length of the Renamed File name |
| RNAME | X    | The new Name of the File to rename, not padded, no zero termination |
| CHK2  | 4    | Checksum of NLEN thru to end of RNAME |

### 0x74, Renamed - The File has been renamed

This message is sent in reply to 0x64 and contains no data.  Data size is 0.

### 0x65 - File - Send a File to the Slave

This message sends a file to the Slave, the Slave DOES NOT verify if this is the same as a previous file of the same name, and over-writes it if it exists,  It is the responsibility of the Master to ensure only necessary files are sent to be stored.  The data is the files name, its file time, and its data.

The Data is:
| Field | Size | Description |
| ----- | ---- | ----------- |
| NSIZ  | 1    | The Size of the File Name 1-255 |
| NAME  | X    | The Name of the File, NSIZ Bytes long, not Padded |
| DATE  | 6    | The Date and Time of the file |
| FDAT  | X    | File Data, may be zero bytes long |
| CHK2  | 4    | Adler-32 Checksum of all Data |

If the Name Size is 0 or too large for the SPIFFS to store or the NAME field has any other problems, NAK is replied, with a FNAMERR code.  If the Date is not properly formatted, NAK is replied with a FORMAT error code. The Date Field has the same format as the Set Time message. If there is not enough space to store the file NAK will be replied with FSIZERR.  If any filesystem errors occur, NAK will be replied with FSERR code.

To save buffering in RAM, the Slave will immediately start writing the file to a temporary file name.  When the Checksum is received, IF and ONLY IF it is valid, the temporary file is renamed to the destination file name.  The Temporary file name is "///TEMP" and the Slave will refuse to receive a file of this name, it will also delete any file of this name on start up.

### 0x75, Received - File was received OK

Reply to the File command.  Data indicates the maximum and remaining space available if the SPIFFS.

The Data is:
| Field | Size | Description |
| ----- | ---- | ----------- |
| SIZE  | 4    | Size of the SPIFFS |
| FREE  | 4    | Free space in the SPIFFS |
| CHK2  | 4    | Checksum of SIZE & FREE |
