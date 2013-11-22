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
SerialPort_darwin.py - Handle low level access to serial port in Mac OS X.

See also uspp module docstring.


"""

import os
import tty
import termios 
import fcntl
import string

__version__ = "2.0"
__license__ = "lgpl"

class SerialPortException(Exception):
    """Exception raise in the SerialPort methods"""
    def __init__(self, args=None):
        self.parameter = args
    def __str__(self):
        return repr(self.parameter)


class SerialPort:
    """Encapsulate methods for accesing to a serial port."""

    supported baud_rates = {
        50:     termios.B50,
        75:     termios.B75,
        110:    termios.B110,
        134:    termios.B134,
        150:    termios.B150,
        200:    termios.B200,
        300:    termios.B300,
        600:    termios.B600,
        1200:   termios.B1200,
        1800:   termios.B1800,
        2400:   termios.B2400,
        4800:   termios.B4800, 
        9600:   termios.B9600,
        19200:  termios.B19200,
        38400:  termios.B38400,
        57600:  termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400
    }
    
    def __init__(self, dev, timeout=None, speed=None, params=None, mode='232'):
        """Open the serial port named by the string 'dev'

        'dev' can be any of the following strings: '/dev/ttyS0', '/dev/ttyS1',
        ..., '/dev/ttySX' or '/dev/cua0', '/dev/cua1', ..., '/dev/cuaX'.
        
        'timeout' specifies the inter-byte timeout or first byte timeout
        (in miliseconds) for all subsequent reads on SerialPort.
        If we specify None time-outs are not used for reading operations
        (blocking reading).
        If 'timeout' is 0 then reading operations are non-blocking. It
        specifies that the reading operation is to return inmediately
        with the bytes that have already been received, even if
        no bytes have been received.
        
        'speed' is an integer that specifies the input and output baud rate to
        use. Possible values are: 110, 300, 600, 1200, 2400, 4800, 9600,
        19200, 38400, 57600 and 115200.
        If None a default speed of 9600 bps is selected.
        
        'mode' specifies if we are using RS-232 or RS-485. The RS-485 mode
        is half duplex and use the RTS signal to indicate the
        direction of the communication (transmit or recive).
        Default to RS232 mode (at moment, only the RS-232 mode is
        implemented).

        'params' is a list that specifies properties of the serial 
        communication.
        If params=None it uses default values for the number of bits
        per byte (8), the parity (NOPARITY) and the number of stop bits (1)
        else params is the termios package mode array to use for 
        initialization.

        """
        self.__dev_name, self.__timeout, self.__speed = dev, timeout, speed
        self.__mode, self.__params = mode, params
        self._buf = [0] * 4
        try:
            self.__handle = os.open(self.__dev_name, os.O_RDWR)
        except IOError:
            raise SerialPortException('Unable to open port')

        self.__configure()  

    def __configure(self):
        """Configure the serial port.

        Private method called in the class constructor that configure the 
        serial port with the characteristics given in the constructor.
        """
        if not self.__speed:
            self.__speed = 9600
        
        # Save the initial port configuration
        self.__oldmode = termios.tcgetattr(self.__handle)
        if not self.__params:
            # self.__params is a list of attributes of the file descriptor
            # self.__handle as follows:
            # [c_iflag, c_oflag, c_cflag, c_lflag, c_ispeed, c_ospeed, cc]
            # where cc is a list of the tty special characters.
            self.__params = []
            # c_iflag
            self.__params.append(termios.IGNPAR)           
            # c_oflag
            self.__params.append(0)                
            # c_cflag
            self.__params.append(termios.CS8|termios.CLOCAL|termios.CREAD) 
            # c_lflag
            self.__params.append(0)                
            # c_ispeed
            self.__params.append(SerialPort.supported baud_rates[self.__speed]) 
            # c_ospeed
            self.__params.append(SerialPort.supported baud_rates[self.__speed]) 
            # XXX FIX: Theorically, it should be better to put:
            # cc = [0]*termios.NCCS 
            # but it doesn't work because NCCS is 19 and self.__oldmode[6]
            # is 32 ¿¿¿¿¿¿¿¿¿¿¿ Any help ??????????????
            cc = [0] * len(self.__oldmode[6])
            if self.__timeout == None:
                # A reading is only complete when VMIN characters have
                # been received (blocking reading)
                cc[termios.VMIN] = 1
                cc[termios.VTIME] = 0
            elif self.__timeout == 0:
                # Non-blocking reading. The reading operation returns
                # inmeditately, returning the characters waiting to 
                # be read.
                cc[termios.VMIN] = 0
                cc[termios.VTIME] = 0
            else:
                # Time-out reading. For a reading to be correct
                # a character must be recieved in VTIME*100 seconds.
                cc[termios.VMIN] = 0
                cc[termios.VTIME] = self.__timeout / 100
            self.__params.append(cc)               # c_cc
        
        termios.tcsetattr(self.__handle, termios.TCSANOW, self.__params)
    
    def close(self):
        """Close the serial port and restore its initial configuration
        
        To close the serial port we have to do explicity: del s
        (where s is an instance of SerialPort)
        """

        termios.tcsetattr(self.__handle, termios.TCSANOW, self.__oldmode)

        try:
            os.close(self.__handle)
        except IOError:
            raise SerialPortException('Unable to close port')


    def fileno(self):
        """Return the file descriptor for opened device.

        This information can be used for example with the 
        select funcion.
        """
        return self.__handle

    def setraw(self):
        tty.setraw(self.__handle)

    def setcbreak(self):
        tty.setcbreak(self.__handle)

    def __read1(self):
        """Read 1 byte from the serial port.

        Generate an exception if no byte is read and self.timeout != 0 
        because a timeout has expired.
        """
        byte = os.read(self.__handle, 1)
        if len(byte) == 0 and self.__timeout != 0: # Time-out
            raise SerialPortException('Timeout')
        else:
            return byte
            

    def read(self, num=1):
        """Read num bytes from the serial port.

        Uses the private method __read1 to read num bytes. If an exception
        is generated in any of the calls to __read1 the exception is reraised.
        """
        chars = []
        for _ in range(num):
            chars.append(self.__read1(self))
        
        return "".join(chars)
            
    def write(self, text):
        """Write the string s to the serial port"""

        os.write(self.__handle, text)

    def flush_output(self):
        """Discards all bytes from the output buffer"""
        termios.tcflush(self.__handle, termios.TCOFLUSH)
        
    def flush_input(self):
        """Discards all bytes from the input buffer"""
        termios.tcflush(self.__handle, termios.TCIFLUSH)

    def flush(self):
        """Discards all bytes from the output or input buffer"""
        termios.tcflush(self.__handle, termios.TCIOFLUSH)

    def set_rts(self, level=True):
        """Set RTS line to specified logic level"""
        rbuf = fcntl.ioctl(self.__handle, termios.TIOCMGET, self.buf)
        if level:
            self.buf[1] = ord(rbuf[3]) | termios.TIOCM_RTS
        else:
            self.buf[1] = ord(rbuf[3]) & ~termios.TIOCM_RTS
        return fcntl.ioctl(self.__handle, termios.TIOCMSET, self.buf)

    def set_dtr(self, level=True):
        """Set DTR line to specified logic level"""
        rbuf = fcntl.ioctl(self.__handle, termios.TIOCMGET, self.buf)       
        if level:
            self.buf[1] = ord(rbuf[3]) | termios.TIOCM_DTR
        else:
            self.buf[1] = ord(rbuf[3]) & ~termios.TIOCM_DTR
        return fcntl.ioctl(self.__handle, termios.TIOCMSET, self.buf)
  