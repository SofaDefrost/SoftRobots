/*!
\file    Serial.h
\brief   Serial library to communicate throught serial port, or any device emulating a serial port.
\author  Philippe Lucidarme (University of Angers) <serialib@googlegroups.com>
\version 1.2
\date    28 avril 2011
This Serial library is used to communicate through serial port.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE X CONSORTIUM BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

This is a licence-free software, it can be used by anyone who try to build a better world.
*/

#ifndef SERIAL_H
#define SERIAL_H

#if defined (_WIN32) || defined( _WIN64)
    #include "SysTimeWin.h"
#endif

#if defined (__linux__) || defined (__APPLE__)
	#include <sys/time.h>
    #include <stdlib.h>
    #include <sys/types.h>
    #include <sys/shm.h>
    #include <termios.h>
    #include <string.h>
    #include <iostream>
    // File control definitions
    #include <fcntl.h>
    #include <unistd.h>
    #include <sys/ioctl.h>
#endif

/*!  \class Serial
     \brief     This class can manage a serial port. The class allows basic operations (opening the connection, reading, writing data and closing the connection).
   */

class Serial
{
public:
    Serial    ();
    ~Serial   ();

    char    Open        (const char *Device,const unsigned int Bauds);
    void    Close();

    char    WriteChar   (char);
    char    ReadChar    (char *pByte,const unsigned int TimeOut_ms=0);
    char    WriteString (const char *String);
    int     ReadString  (   char *String,
                            char FinalChar,
                            unsigned int MaxNbBytes,
                            const unsigned int TimeOut_ms=0);
    char    Write       (const void *Buffer, const unsigned int NbBytes);
    int     Read        (void *Buffer,unsigned int MaxNbBytes,const unsigned int TimeOut_ms=0);

    void    FlushReceiver();

    // Return the number of bytes in the received buffer
    int     Peek();

private:
    int     ReadStringNoTimeOut  (char *String,char FinalChar,unsigned int MaxNbBytes);

#if defined (_WIN32) || defined( _WIN64)
    HANDLE          hSerial;
    COMMTIMEOUTS    timeouts;
#endif
#if defined (__linux__) || defined (__APPLE__)
    int             fd;
#endif

};

/*!  \class     TimeOut
     \brief     This class can manage a timer which is used as a timeout.
   */

class TimeOut
{
public:
    TimeOut();

    void                InitTimer();
    unsigned long int   ElapsedTime_ms();

private:    
    struct timeval      PreviousTime;
};

/*!
  \mainpage Serial class

  \brief

    The class Serial offers simple access to the serial port devices for windows and linux. It can be used for any serial device (Built-in serial port, USB to RS232 converter, arduino board or any hardware using or emulating a serial port)
    The class can be used under Windows and Linux.
    The class allows basic operations like :
    - opening and closing connection
    - reading data (characters, array of bytes or strings)
    - writing data (characters, array of bytes or strings)
    - non-blocking functions (based on timeout).

  \author   Philippe Lucidarme (University of Angers) <serialib@googlegroups.com>
  \date     1th may 2011 (Last update: 25th september 2012)
  \version  1.2

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE X CONSORTIUM BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

This is a licence-free software, it can be used by anyone who try to build a better world.
*/

#endif // SERIAL_H
