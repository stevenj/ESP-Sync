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
    toHex = lambda c: '{:02X}'.format(c)    # noqa: E731
    toChr = lambda c: chr(c) if 32 <= c < 127 else '.'   # noqa: E731
    make = lambda f, *cs: join(group(list(map(f, data)), 8, 2), *cs)  # noqa: E731
    hs = make(toHex, '  ', ' ')
    cs = make(toChr, ' ', '')
    for i, (h, c) in enumerate(zip(hs, cs)):
        print('{:010X}: {:48}  {:16}'.format(i * 16, h, c))


def fletcher16(bytes):
    """16 bit Fletcher checksum"""
    a = list(bytes)
    b = [sum(a[:i]) % 255 for i in range(len(a)+1)]
    return (sum(b) << 8) | max(b)


class ESPSynchComms:

    CODES = {
      'STX'          : b'\x02',  # noqa: E203
      'ACK'          : b'\x06',  # noqa: E203
      'NAK'          : b'\x15',  # noqa: E203
      'CMD_SET_TIME' : b'\x60',  # noqa: E203
      'CMD_FORMAT'   : b'\x61',  # noqa: E203
      'CMD_LIST'     : b'\x62',  # noqa: E203
      'CMD_REMOVE'   : b'\x63',  # noqa: E203
      'CMD_RENAME'   : b'\x64',  # noqa: E203
      'CMD_FILETX'   : b'\x65',  # noqa: E203
      'RPL_TIME_SET' : b'\x70',  # noqa: E203
      'RPL_FORMATED' : b'\x71',  # noqa: E203
      'RPL_LISTING'  : b'\x72',  # noqa: E203
      'RPL_REMOVED'  : b'\x73',  # noqa: E203
      'RPL_RENAMED'  : b'\x74',  # noqa: E203
      'RPL_RECEIVED' : b'\x75',  # noqa: E203
      'PAD_5A'       : b'\x5A',  # noqa: E203
      'PAD_A5'       : b'\xA5'   # noqa: E203
    }

    def __init__(self, comm):
        """ port = Serial Instance to communicate on """
        self._comm = comm
        self.cmn = 0
        self._debug = False

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

    def TXHeader(self, fcode, size):
        header = self.CODES['STX'] + \
                  self.TXCMN() + \
                  self.CODES[fcode] + \
                  self.NBO24(size)
        header = header + self.NBO16(fletcher16(header))
        self._comm.write(header)

        if (self._debug):
            print("TX: " + header.hex())

    def TXData(self, data):
        data = data + self.NBO32(adler32(data))
        self._comm.write(data)
        if (self._debug):
            print("TX: " + data.hex)

    def RXFlush(self):
        # Flush Receive buffer
        self._comm.reset_input_buffer()

    def RXTimeout(self, ms):
        self._comm.timeout = float(ms) / 1000.0


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
    pass


def CMD_ping(protocol, args):
    print("Pinging ESP8266/32 on " + protocol._comm.name)

    # Flush Receive buffer
    protocol.RXFlush()
    # 250ms should be an eon in which to receive a reply to an ack.
    protocol.RXTimeout(250)

    # Send 10 Acks, wait for replies or timeout

    for x in range(int(args['-t'])):
        if (args['--verbose']):
            print("PING " + str(x+1))
        protocol.TXHeader('ACK', protocol.OPT_ACK(0))

        print('RX: ')
        data = bytes()
        xdata = protocol._comm.read(32)
        while (len(xdata) > 0):
            data = data + xdata
            xdata = protocol._comm.read(32)

        hexdump(data)

        time.sleep(1)


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
