# -*- coding: iso-8859-1 -*-

##########################################################################
# USPP Library (Universal Serial Port Python Library)
#
# Copyright (C) 2006 Isaac Barona <ibarona@gmail.com>
# 
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
# 
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
##########################################################################

"""
SerialPort_win.py - Handle low level access to serial port in windows.

See also uspp module docstring.

"""

#required pywin32
import win32file
import pywintypes
import win32event
import win32con

__version__ = "2.0"
__license__ = "lgpl"

class SerialPortException(Exception):
    """Exception raise in the SerialPort methods"""
    def __init__(self, args=None):
        self.parameter = args
    def __str__(self):
        return repr(self.parameter)


class SerialPort(object):
    """Encapsulate methods for accesing to a serial port."""

    supported_baud_rates = {
        110:        win32file.CBR_110,
        300:        win32file.CBR_300,
        600:        win32file.CBR_600,
        1200:       win32file.CBR_1200,
        2400:       win32file.CBR_2400,
        4800:       win32file.CBR_4800, 
        9600:       win32file.CBR_9600,
        19200:      win32file.CBR_19200,
        38400:      win32file.CBR_38400,
        57600:      win32file.CBR_57600,
        115200:     win32file.CBR_115200,
        128000:     win32file.CBR_128000,
        256000:     win32file.CBR_256000
    }

    def __init__(self, dev, params=None, timeout=1000, mode='232'):
        """Open the serial port named by the string 'dev'

        'dev' can be any of the following strings: 'COM1', 'COM2', ... 'COMX'
        
        'timeout' specifies the inter-byte timeout or first byte timeout
        (in milliseconds) for all subsequent reads on SerialPort.
        If we specify None time-outs are not used for reading operations
        (blocking reading).
        If 'timeout' is 0 then reading operations are non-blocking. It
        specifies that the reading operation is to return immediately
        with the bytes that have already been received, even if
        no bytes have been received.
        
        'speed' is an integer that specifies the input and output baud rate to
        use. Possible values are: 110, 300, 600, 1200, 2400, 4800, 9600,
        19200, 38400, 57600 and 115200.
        If None a default speed of 9600 bps is selected.
        
        'mode' specifies if we are using RS-232 or RS-485. The RS-485 mode
        is half duplex and use the RTS signal to indicate the
        direction of the communication (transmit or receive).
        Default to RS232 mode (at moment, only the RS-232 mode is
        implemented).

        'params' is a dictionary that specifies properties of the serial 
        communication.
        If params = None it uses default values otherwise uses the dictionary
        where identifies must have the same names as in PyDCB.

        """
        
        #to allow to digits ports 
        self.__dev_name, self.__timeout = "\\\\.\\%s" % dev, timeout
        self.__mode, self.__params = mode, params
        try:
            self.__handle = win32file.CreateFile(self.__dev_name,
                                  win32con.GENERIC_READ|win32con.GENERIC_WRITE,
                                  #exclusive access
                                  0, 
                                  #no security
                                  None, 
                                  win32con.OPEN_EXISTING,
                                  win32con.FILE_ATTRIBUTE_NORMAL,
                                  None)
        except pywintypes.error:
            raise SerialPortException('Unable to open port')
            exit(-1)

        self.__configure()

    def __configure(self):
        """Configure the serial port.
        
        Private method called in the class constructor that configure the 
        serial port with the characteristics given in the constructor.
        """
        
        defaults = {
            "BaudRate":     9600,
            "ByteSize":     8,
            "Parity":       win32file.NOPARITY,
            "StopBits":     win32file.ONESTOPBIT,
            "fDtrControl":  0,
            "fRtsControl":  0,
            "fDtrControl":  win32file.DTR_CONTROL_HANDSHAKE,
            "fRtsControl":  win32file.RTS_CONTROL_TOGGLE
        }
        
        parameter = {
            "baud_rate":  "BaudRate",
            "byte_size":  "ByteSize",
            "parity":     "Parity",
            "stop_bits":  "StopBits",
            "dtr":        "fDtrControl",
            "rts":        "fRtsControl",
            "xon_char":   "XonChar",
            "xoff_char":  "XoffChar",
            "dtr_control":"fDtrControl",
            "rts_control": "fRtsControl"
        }
        # Tell the port we want a notification on each char
        win32file.SetCommMask(self.__handle, win32file.EV_RXCHAR)
        # Setup a 4k buffer
        win32file.SetupComm(self.__handle, 4096, 4096)
        # Remove anything that was there
        win32file.PurgeComm(self.__handle, win32file.PURGE_TXABORT|win32file.PURGE_RXABORT|
                            win32file.PURGE_TXCLEAR|win32file.PURGE_RXCLEAR)

        # Setup the timeouts parameters for the port
        # timeouts is a tuple with the following items:
        # [0] int : ReadIntervalTimeout
        # [1] int : ReadTotalTimeoutMultiplier
        # [2] int : ReadTotalTimeoutConstant
        # [3] int : WriteTotalTimeoutMultiplier
        # [4] int : WriteTotalTimeoutConstant

        if self.__timeout == None:
            timeouts = 0, 0, 0, 0, 0
        elif self.__timeout == 0:
            timeouts = win32con.MAXDWORD, 0, 0, 0, 1000
        else:
            timeouts = self.__timeout, 0, self.__timeout, 0, 1000
        win32file.SetCommTimeouts(self.__handle, timeouts)

        #setup the connection info
        dcb = win32file.GetCommState(self.__handle)
        #set default values
        for entry in defaults.items():
            setattr(dcb, entry[0], entry[1])
        if self.__params:
            for entry in self.__params.items():
                try:
                    setattr(dcb, parameter[entry[0]], entry[1])
                except KeyError:
                    print("ignored parameter", entry)
        win32file.SetCommState(self.__handle, dcb)
        

    def close(self):
        """Close the serial port
        
        To close the serial port we have to do explicity: del s
        (where s is an instance of SerialPort)
        """
        try:
            win32file.CloseHandle(self.__handle)
        except IOError:
            raise SerialPortException('Unable to close port')
            exit(-1)


    def _in_waiting(self):
        """return the number of bytes waiting to be read"""
        flags, comstat = win32file.ClearCommError(self.__handle)
        print(comstat.cbInQue)
        return comstat.cbInQue

    def read(self, num=1):
        """Read num bytes from the serial port.

        If self.__timeout != 0 and != None and the number of read bytes is less
        than num an exception is generated because a timeout has expired.
        If self.__timeout == 0 read is non-blocking and inmediately returns
        up to num bytes that have previously been received.
        """
        hr, buf = win32file.ReadFile(self.__handle, num)
        return buf

    def set_dtr(self, level=True):
        """Set DTR line to specified logic level"""
        win32file.EscapeCommFunction(self.__handle, win32file.SETDTR if level else win32file.CLRDTR)
        
    def set_rts(self, level=True):
        """Set RTS line to specified logic level"""
        win32file.EscapeCommFunction(self.__handle, win32file.SETRTS if level else win32file.CLRRTS)

    def set_break(self, level=True):
        """"""
        win32file.EscapeCommFunction(self.__handle, win32file.SETBREAK if level else win32file.CLRBREAK)

    def write(self, text):
        """Write the string s to the serial port"""
        overlapped = win32file.OVERLAPPED()
        overlapped.hEvent = win32event.CreateEvent(None, 0, 0, None)
        err, n = win32file.WriteFile(self.__handle, text, overlapped)
        # Wait for the write to complete
        if err:
            win32event.WaitForSingleObject(overlapped.hEvent, win32event.INFINITE)

    def flush_input(self):
        """Discards all bytes from the intput buffer"""
        win32file.PurgeComm(self.__handle, win32file.PURGE_RXABORT|win32file.PURGE_RXCLEAR)
        
    def flush_output(self):
        """Discards all bytes from the output buffer"""
        win32file.PurgeComm(self.__handle, win32file.PURGE_TXABORT|win32file.PURGE_TXCLEAR)

    def flush(self):
        """Discards all bytes from the output or input buffer"""
        win32file.PurgeComm(self.__handle, win32file.PURGE_RXABORT|win32file.PURGE_RXCLEAR|
                            win32file.PURGE_TXABORT|win32file.PURGE_TXCLEAR)

