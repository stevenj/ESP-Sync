#!/usr/bin/env python3
"""espsync - file synchronizer for ESP8266 and ESP32.

Usage:
  espsync.py synch  <port> <path> [-b BAUD] [-crXZfv] [--debug]
  espsync.py watch  <port> <path> [-b BAUD] [-crXZfv] [--debug]
  espsync.py list   <port> <path> [-b BAUD] [-XZv] [--debug]
  espsync.py rm     <port> <file> [-b BAUD] [-XZv] [--debug]
  espsync.py mv     <port> <file> <dst> [-b BAUD] [-XZv] [--debug]
  espsync.py format <port> [-b BAUD] [-XZv] [--debug]
  espsync.py ping   <port> [-b BAUD] [-t TRIES] [-XZv] [--debug]
  espsync.py (-h | --help)
  espsync.py --version

The commands are:
   synch      Synch the file or directory at path with the ESP8266/32
   watch      Like Synch, except it watches for changes and
              resynchs automatically.
   list       Just list the files on the ESP8266/32
   rm         Remove (Delete) the file on the ESP8266/32
   mv         Rename the file on the ESP8266/32
   format     Format the SPIFFS only.
   ping       Check comms to ESP8266/32 only.

Arguments:
  <port>      serial port to communicate to board on.
  <path>      directory or file to sync.
  <file>      file name on the ESP8266/32
  <dst>       destination file name on the ESP8266/32

Options:
  -h --help       Show this screen.
  --version       Show version.
  -b BAUD         Set Data Rate [default: 115200]
  -t TRIES        Number of times to ping [default: 10]
  -c --clean      Clean synch, causes files on ESP8266/32 not in synch path to
                  be removed from ESP8266/32.
  -r --recursive  Synch recursively.
  -X --reset      Reset board BEFORE attempting to synch.
  -Z --after      Reset board AFTER attempting to synch.
  -f --format     Format the SPIFFS before synch.
  -v --verbose    Verbose output.
  --debug         Output debug information.

"""
from docopt import docopt
import serial
from zlib import adler32  # We use the zlib adler32 implementation
import time


def group(a, *ns):
    for n in ns:
        a = [a[i:i+n] for i in range(0, len(a), n)]
    return a


def join(a, *cs):
    return [cs[0].join(join(t, *cs[1:])) for t in a] if cs else a


def hexdump(data):
    def toHex(c): return '{:02X}'.format(c)
    def toChr(c): return chr(c) if 32 <= c < 127 else '.'
    def make(f, *cs): return join(group(list(map(f, data)), 8, 2), *cs)
    hs = make(toHex, '  ', ' ')
    cs = make(toChr, ' ', '')
    for i, (h, c) in enumerate(zip(hs, cs)):
        print('{:010X}: {:48}  {:16}'.format(i * 16, h, c))


def fletcher16(bytes):
    """16 bit Fletcher checksum"""
    a = list(bytes)
    b = [sum(a[:i]) % 255 for i in range(len(a)+1)]
    return ((sum(b) << 8) & 0xFF00) | max(b)


class ESPSynchComms:

    CODES = {
        'STX': b'\x02',
        'ACK': b'\x06',
        'NAK': b'\x15',
        'CMD_SET_TIME': b'\x60',
        'CMD_FORMAT': b'\x61',
        'CMD_LIST': b'\x62',
        'CMD_REMOVE': b'\x63',
        'CMD_RENAME': b'\x64',
        'CMD_FILETX': b'\x65',
        'RPL_TIME_SET': b'\x70',
        'RPL_FORMATED': b'\x71',
        'RPL_LISTING': b'\x72',
        'RPL_REMOVED': b'\x73',
        'RPL_RENAMED': b'\x74',
        'RPL_RECEIVED': b'\x75',
        'PAD_5A': b'\x5A',
        'PAD_A5': b'\xA5'
    }

    RXSTATES = {}

    def __init__(self, comm):
        """ port = Serial Instance to communicate on """
        self._comm = comm
        self.cmn = 0
        self._debug = False

        self._completeMsg = None
        self._rxstate = 0
        self._checksum = 0
        self._thisMsg = None
        self._rxdbuf = bytes()

    def setDebug(self):
        self._debug = True

    def hard_reset(self):
        self._comm.setDTR(False)    # Enter BOOT Loader - Disabled (HiZ)
        self._comm.setRTS(True)     # EN/RESET->LOW
        time.sleep(0.1)
        self._comm.setRTS(False)    # EN/RESET->HiZ
        self._comm.setDTR(self._comm.dtr)  # Buggy Windows driver work around

    def TXCMN(self):
        return (bytes([self.cmn + 0x20]))

    def RXCMN(self):
        return (bytes([self.cmn + 0x40]))

    def incCMN(self):
        self.cmn = (self.cmn + 1) & 0x1F

    def NBO8(self, value):
        return (bytes([(value & 0xFF)]))

    def NBO16(self, value):
        return (self.NBO8(value >> 8) + self.NBO8(value))

    def NBO24(self, value):
        return (self.NBO8(value >> 16) + self.NBO16(value))

    def NBO32(self, value):
        return (self.NBO16(value >> 16) + self.NBO16(value))

    def OPT_ACK(self, value):
        return (((value & 0xFFFF) << 8) | 0x5A)

    def OPT_NAK(self, value):
        return (((value & 0xFF) << 16) | 0xA55A)

    def TXHeader(self, fcode, size=0):
        header = self.CODES['STX'] + \
            self.TXCMN() + \
            self.CODES[fcode] + \
            self.NBO24(size)
        header = header + self.NBO16(fletcher16(header))
        self._comm.write(header)

        if (self._debug):
            print("TX: ")
            hexdump(header)

    def TXData(self, data):
        data = data + self.NBO32(adler32(data))
        self._comm.write(data)
        if (self._debug):
            print("TX: ")
            hexdump(data)

    def RXFlush(self):
        # Flush Receive buffer
        self._comm.reset_input_buffer()
        self._rxdbuf = bytes()

    def RXTimeout(self, ms):
        self._comm.timeout = float(ms) / 1000.0

    def ResetRX(self):
        self._rxstate = 0

    def RXMessage(self, data):
        def GetStx(byte):
            if (byte == self.CODES['STX']):
                self._checksum = 0x0202
                self._rxstate += 1
                self._thisMsg = {}
                self._thisMsg['HEADER'] = b'\x02'

        def GetCMN(byte):
            if (byte == self.RXCMN()):
                self._rxstate += 1
                self._thisMsg['CMN'] = ord(byte) - 0x40
                self._thisMsg['HEADER'] += byte
            else:
                self._rxstate = 0

        def GetDelay1(byte):
            self._thisMsg['HEADER'] += byte
            self._thisMsg['DELAY'] = ord(byte) << 8
            self._rxstate += 1

        def GetDelay2(byte):
            self._thisMsg['HEADER'] += byte
            self._thisMsg['DELAY'] |= ord(byte)
            self._rxstate += 1

        def GetErrorCode(byte):
            self._thisMsg['HEADER'] += byte
            self._thisMsg['ERROR'] = ord(byte)
            self._rxstate += 1

        def GetPad1(byte):
            if (byte == self.CODES['PAD_A5']):
                self._thisMsg['HEADER'] += byte
                self._rxstate += 1
            else:
                self._rxstate == 0

        def GetPad2(byte):
            if (byte == self.CODES['PAD_5A']):
                self._thisMsg['HEADER'] += byte
                self._thisMsg['SIZE'] = 0
                self._rxstate += 1
            else:
                self._rxstate == 0

        def GetSizeHi(byte):
            self._thisMsg['HEADER'] += byte
            self._thisMsg['SIZE'] = ord(byte) << 16
            self._rxstate += 1

        def GetSizeMid(byte):
            self._thisMsg['HEADER'] += byte
            self._thisMsg['SIZE'] |= ord(byte) << 8
            self._rxstate += 1

        def GetSizeLow(byte):
            self._thisMsg['HEADER'] += byte
            self._thisMsg['SIZE'] |= ord(byte)
            self._rxstate += 1

        def GetFunc(byte):
            self._thisMsg['HEADER'] += byte
            self._rxstate += 1
            self._thisMsg['FUNC'] = byte

            if (byte == self.CODES['ACK']):
                self.RXSTATES[3] = GetDelay1
                self.RXSTATES[4] = GetDelay2
                self.RXSTATES[5] = GetPad2
            elif (byte == self.CODES['NAK']):
                self.RXSTATES[3] = GetErrorCode
                self.RXSTATES[4] = GetPad1
                self.RXSTATES[5] = GetPad2
            elif ((byte >= self.CODES['RPL_TIME_SET']) and
                  (byte <= self.CODES['RPL_RECEIVED'])):
                self.RXSTATES[3] = GetSizeHi
                self.RXSTATES[4] = GetSizeMid
                self.RXSTATES[5] = GetSizeLow
            else:
                self._rxstate = 0

        def GetChkHi(byte):
            self._thisMsg['CHK'] = ord(byte) << 8
            self._rxstate += 1

        def GetChkLo(byte):
            self._thisMsg['CHK'] |= ord(byte)

            if (fletcher16(self._thisMsg['HEADER']) == self._thisMsg['CHK']):
                if (self._thisMsg['SIZE'] == 0):
                    self._completeMsg = self._thisMsg
                    self._rxstate = 0
                else:
                    self._rxstate += 1
                    self._bcount = self._thisMsg['SIZE'] - 4
                    self._thisMsg['DATA'] = b''
            else:
                self._rxstate = 0

        def GetDataBody(byte):
            self._thisMsg['DATA'] += byte
            self._bcount -= 1
            if (self._bcount == 0):
                self._rxstate += 1

        def GetChk2_24(byte):
            self._thisMsg['CHK2'] = ord(byte) << 24
            self._rxstate += 1

        def GetChk2_16(byte):
            self._thisMsg['CHK2'] |= ord(byte) << 16
            self._rxstate += 1

        def GetChk2_8(byte):
            self._thisMsg['CHK2'] |= ord(byte) << 8
            self._rxstate += 1

        def GetChk2_0(byte):
            self._thisMsg['CHK2'] |= ord(byte)
            self._rxstate == 0

            if (adler32(self._thisMsg['DATA']) == self._thisMsg['CHK2']):
                self._completeMsg = self._thisMsg

        if (len(self.RXSTATES) == 0):  # Initialise the state table
            self.RXSTATES[0] = GetStx
            self.RXSTATES[1] = GetCMN
            self.RXSTATES[2] = GetFunc
            self.RXSTATES[3] = None  # State depends on result of GetFunc
            self.RXSTATES[4] = None  # State depends on result of GetFunc
            self.RXSTATES[5] = None  # State depends on result of GetFunc
            self.RXSTATES[6] = GetChkHi
            self.RXSTATES[7] = GetChkLo
            self.RXSTATES[8] = GetDataBody
            self.RXSTATES[9] = GetChk2_24
            self.RXSTATES[9] = GetChk2_16
            self.RXSTATES[9] = GetChk2_8
            self.RXSTATES[9] = GetChk2_0

        bcount = 0
        for byte in data:
            bcount += 1
            byte = bytes([byte])
            self.RXSTATES[self._rxstate](byte)
            if (self._completeMsg is not None):
                break
        bytesleft = data[bcount:]
        msg = self._completeMsg
        self._completeMsg = None

        return (msg, bytesleft)

    def getNextMessage(self, timeout):
        msg = None
        self.RXTimeout(timeout)

        if (len(self._rxdbuf) == 0):
            self._rxdbuf = self._comm.read(32)
            if (self._debug):
                if (len(self._rxdbuf) > 0):
                    print('RX: ')
                    hexdump(self._rxdbuf)

        while ((len(self._rxdbuf) > 0) and msg is None):
            msg, self._rxdbuf = self.RXMessage(self._rxdbuf)
            if ((msg is None) and (len(self._rxdbuf) == 0)):
                self._rxdbuf = self._comm.read(32)
                if (self._debug):
                    if (len(self._rxdbuf) > 0):
                        print('RX: ')
                        hexdump(self._rxdbuf)

        return msg


def CMD_synch(protocol, args):
    pass


def CMD_watch(protocol, args):
    pass


def CMD_list(protocol, args):
    pass


def CMD_rm(protocol, args):
    pass


def CMD_mv(protocol, args):
    pass


def CMD_format(protocol, args):
    print(
        ("Formatting SPIFFS on ESP8266/32 at {}").format(
            protocol._comm.name))

    # Flush Receive buffer and protocol handler
    protocol.RXFlush()
    protocol.ResetRX()

    # 250ms should be an eon in which to receive a reply to an ack.
    protocol.RXTimeout(20)

    protocol.TXHeader('CMD_FORMAT')

    # 20ms should be an eon in which to receive a reply.
    protocol.RXTimeout(20)

    # Get a reply
    # Should be an ACK/NAK or Formatted.  Anything else is an error

    # IF it was an ack, wait for the specified duration for an
    # Formatted reply, or a NAK.  Anything else is an error.


def CMD_ping(protocol, args):
    success = 0
    tries = 0

    print(
        ("Pinging ESP8266/32 {} times on {}").format(
            args['-t'], protocol._comm.name))

    # Send 10 (or whatever number) Acks, wait for replies or timeout
    for x in range(int(args['-t'])):
        # Flush Receive buffer
        protocol.RXFlush()
        # Reset Receive state machine
        protocol.ResetRX()

        if (args['--verbose']):
            print("PING " + str(x+1))
        protocol.TXHeader('ACK', protocol.OPT_ACK(0))

        msg = protocol.getNextMessage(20)

        if (msg is not None):
            if (msg['FUNC'] == protocol.CODES['ACK']):
                success += 1
                protocol.incCMN()
            else:
                if (args['--verbose']):
                    print("UNEXPECTED REPLY '{}'".format(msg['FUNC']))
                time.sleep(1)
        else:
            if (args['--verbose']):
                print("NO REPLY")
            time.sleep(1)

        tries += 1

    print("{} Pings, {} Replies.".format(tries, success))


def main():
    args = docopt(__doc__, version='0.1')
    if (args['--debug']):
        print(args)

    try:
        ser = serial.Serial(port=args['<port>'],
                            baudrate=args['-b'])  # open serial port
    except ValueError:
        print("Invalid Data Rate : ", args['-b'])
        exit(64)
    except serial.serialutil.SerialException as err:
        print("Port can not be configured : ", args['<port>'])
        print(err)
        exit(74)

    protocol = ESPSynchComms(ser)
    if (args['--debug']):
        protocol.setDebug()

    if (args['synch']):
        CMD_synch(protocol, args)
    elif (args['watch']):
        CMD_watch(protocol, args)
    elif (args['list']):
        CMD_list(protocol, args)
    elif (args['rm']):
        CMD_rm(protocol, args)
    elif (args['mv']):
        CMD_mv(protocol, args)
    elif (args['format']):
        CMD_format(protocol, args)
    elif (args['ping']):
        CMD_ping(protocol, args)
    else:
        print("Error: Unknown Command!")
    pass

    ser.close()             # close port


if __name__ == '__main__':
    main()
