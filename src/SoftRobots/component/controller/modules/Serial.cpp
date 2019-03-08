/*!
 \file    Serial.cpp
 \brief   Class to manage the serial port renamed from Serialib
 \author  Philippe Lucidarme (University of Angers) <serialib@googlegroups.com>
 \version 1.2
 \date    28 avril 2011

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE X CONSORTIUM BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


This is a licence-free software, it can be used by anyone who try to build a better world.
 */

#include "Serial.h"

/*!
    \brief      Constructor of the class Serial.
*/

Serial::Serial()
{}

/*!
    \brief      Destructor of the class Serial. It close the connection
*/

Serial::~Serial()
{
    Close();
}

/*!
     \brief Open the serial port
     \param Device : Port name (COM1, COM2, ... for Windows ) or (/dev/ttyS0, /dev/ttyACM0, /dev/ttyUSB0 ... for linux)
     \param Bauds : Baud rate of the serial port.

                \n Supported baud rate for Windows :
                        - 110
                        - 300
                        - 600
                        - 1200
                        - 2400
                        - 4800
                        - 9600
                        - 14400
                        - 19200
                        - 38400
                        - 56000
                        - 57600
                        - 115200
                        - 128000
                        - 256000

               \n Supported baud rate for Linux :\n
                        - 110
                        - 300
                        - 600
                        - 1200
                        - 2400
                        - 4800
                        - 9600
                        - 19200
                        - 38400
                        - 57600
                        - 115200

     \return 1 success
     \return -1 device not found
     \return -2 error while opening the device
     \return -3 error while getting port parameters
     \return -4 Speed (Bauds) not recognized
     \return -5 error while writing port parameters
     \return -6 error while writing timeout parameters
  */
char Serial::Open(const char *Device,const unsigned int Bauds)
{
#if defined (_WIN32) || defined( _WIN64)

    hSerial = CreateFileA(  Device,GENERIC_READ | GENERIC_WRITE,0,0,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,0);
    if(hSerial==INVALID_HANDLE_VALUE) {
        if(GetLastError()==ERROR_FILE_NOT_FOUND)
            return -1;                                                  // Device not found
        return -2;                                                      // Error while opening the device
    }

    DCB dcbSerialParams = {0};                                          // Structure for the port parameters
    dcbSerialParams.DCBlength=sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams))                       // Get the port parameters
        return -3;                                                      // Error while getting port parameters
    switch (Bauds)                                                      // Set the speed (Bauds)
    {
    case 110  :     dcbSerialParams.BaudRate=CBR_110; break;
    case 300  :     dcbSerialParams.BaudRate=CBR_300; break;
    case 600  :     dcbSerialParams.BaudRate=CBR_600; break;
    case 1200 :     dcbSerialParams.BaudRate=CBR_1200; break;
    case 2400 :     dcbSerialParams.BaudRate=CBR_2400; break;
    case 4800 :     dcbSerialParams.BaudRate=CBR_4800; break;
    case 9600 :     dcbSerialParams.BaudRate=CBR_9600; break;
    case 14400 :    dcbSerialParams.BaudRate=CBR_14400; break;
    case 19200 :    dcbSerialParams.BaudRate=CBR_19200; break;
    case 38400 :    dcbSerialParams.BaudRate=CBR_38400; break;
    case 56000 :    dcbSerialParams.BaudRate=CBR_56000; break;
    case 57600 :    dcbSerialParams.BaudRate=CBR_57600; break;
    case 115200 :   dcbSerialParams.BaudRate=CBR_115200; break;
    case 128000 :   dcbSerialParams.BaudRate=CBR_128000; break;
    case 256000 :   dcbSerialParams.BaudRate=CBR_256000; break;
    default : return -4;
}    
    dcbSerialParams.ByteSize=8;                                         // 8 bit data
    dcbSerialParams.StopBits=ONESTOPBIT;                                // One stop bit
    dcbSerialParams.Parity=NOPARITY;                                    // No parity
    if(!SetCommState(hSerial, &dcbSerialParams))                        // Write the parameters
        return -5;                                                      // Error while writing

    timeouts.ReadIntervalTimeout=0;                                     // Set the Timeout parameters
    timeouts.ReadTotalTimeoutConstant=MAXDWORD;                         // No TimeOut
    timeouts.ReadTotalTimeoutMultiplier=0;
    timeouts.WriteTotalTimeoutConstant=MAXDWORD;
    timeouts.WriteTotalTimeoutMultiplier=0;
    if(!SetCommTimeouts(hSerial, &timeouts))                            // Write the parameters
        return -6;                                                      // Error while writting the parameters
    return 1;                                                           // Opening successfull

#endif
#if defined (__linux__) || defined (__APPLE__)
    struct termios options;                                             // Structure with the device's options

    fd = open(Device, O_RDWR | O_NOCTTY | O_NDELAY);                    // Open port
    if (fd == -1) return -2;                                            // If the device is not open, return -1
    fcntl(fd, F_SETFL, FNDELAY);                                        // Open the device in nonblocking mode

    tcgetattr(fd, &options);                                            // Get the current options of the port
    bzero(&options, sizeof(options));                                   // Clear all the options
    speed_t         Speed;
    switch (Bauds)                                                      // Set the speed (Bauds)
    {
    case 110  :     Speed=B110; break;
    case 300  :     Speed=B300; break;
    case 600  :     Speed=B600; break;
    case 1200 :     Speed=B1200; break;
    case 2400 :     Speed=B2400; break;
    case 4800 :     Speed=B4800; break;
    case 9600 :     Speed=B9600; break;
    case 19200 :    Speed=B19200; break;
    case 38400 :    Speed=B38400; break;
    case 57600 :    Speed=B57600; break;
    case 115200 :   Speed=B115200; break;
    default : return -4;
}
    cfsetispeed(&options, Speed);                                       // Set the baud rate at 115200 bauds
    cfsetospeed(&options, Speed);
    options.c_cflag |= ( CLOCAL | CREAD |  CS8);                        // Configure the device : 8 bits, no parity, no control
    options.c_iflag |= ( IGNPAR | IGNBRK );
    options.c_cc[VTIME]=0;                                              // Timer unused
    options.c_cc[VMIN]=0;                                               // At least on character before satisfy reading
    tcsetattr(fd, TCSANOW, &options);                                   // Activate the settings
    return (1);                                                         // Success
#endif
}

/*!
     \brief Close the connection with the current device
*/
void Serial::Close()
{
#if defined (_WIN32) || defined( _WIN64)
    CloseHandle(hSerial);
#endif
#if defined (__linux__) || defined (__APPLE__)
    close (fd);
#endif
}

/*!
     \brief Write a char on the current serial port
     \param Byte : char to send on the port (must be terminated by '\0')
     \return 1 success
     \return -1 error while writting data
  */
char Serial::WriteChar(const char Byte)
{
#if defined (_WIN32) || defined( _WIN64)
    DWORD dwBytesWritten;                                               // Number of bytes written
    if(!WriteFile(hSerial,&Byte,1,&dwBytesWritten,NULL))                // Write the char
        return -1;                                                      // Error while writing
    return 1;                                                           // Write operation successfull
#endif
#if defined (__linux__) || defined (__APPLE__)
    if (write(fd,&Byte,1)!=1)                                           // Write the char
        return -1;                                                      // Error while writting
    return 1;                                                           // Write operation successfull
#endif
}

/*!
     \brief Write a string on the current serial port
     \param String : string to send on the port (must be terminated by '\0')
     \return 1 success
     \return -1 error while writting data
  */
char Serial::WriteString(const char *String)
{
#if defined (_WIN32) || defined( _WIN64)
    DWORD dwBytesWritten;                                               // Number of bytes written
    if(!WriteFile(hSerial,String,strlen(String),&dwBytesWritten,NULL))  // Write the string
        return -1;                                                      // Error while writing
    return 1;                                                           // Write operation successfull
#endif
#if defined (__linux__) || defined (__APPLE__)
    int Lenght=strlen(String);                                          // Lenght of the string
    if (write(fd,String,Lenght)!=Lenght)                                // Write the string
        return -1;                                                      // error while writing
    return 1;                                                           // Write operation successfull
#endif
}

/*!
     \brief Write an array of data on the current serial port
     \param Buffer : array of bytes to send on the port
     \param NbBytes : number of byte to send
     \return 1 success
     \return -1 error while writting data
  */
char Serial::Write(const void *Buffer, const unsigned int NbBytes)
{
#if defined (_WIN32) || defined( _WIN64)
    DWORD dwBytesWritten;                                               // Number of byte written
    if(!WriteFile(hSerial, Buffer, NbBytes, &dwBytesWritten, NULL))     // Write data
        return -1;                                                      // Error while writing
    return 1;                                                           // Write operation successfull
#endif
#if defined (__linux__) || defined (__APPLE__)
    if (write (fd,Buffer,NbBytes)!=(ssize_t)NbBytes)                              // Write data
        return -1;                                                      // Error while writing
    return 1;                                                           // Write operation successfull
#endif
}

/*!
     \brief Wait for a byte from the serial device and return the data read
     \param pByte : data read on the serial device
     \param TimeOut_ms : delay of timeout before giving up the reading
            If set to zero, timeout is disable (Optional)
     \return 1 success
     \return 0 Timeout reached
     \return -1 error while setting the Timeout
     \return -2 error while reading the byte
  */
char Serial::ReadChar(char *pByte,unsigned int TimeOut_ms)
{
#if defined (_WIN32) || defined(_WIN64)

    DWORD dwBytesRead = 0;
    timeouts.ReadTotalTimeoutConstant=TimeOut_ms;                       // Set the TimeOut
    if(!SetCommTimeouts(hSerial, &timeouts))                            // Write the parameters
        return -1;                                                      // Error while writting the parameters
    if(!ReadFile(hSerial,pByte, 1, &dwBytesRead, NULL))                 // Read the byte
        return -2;                                                      // Error while reading the byte
    if (dwBytesRead==0) return 0;                                       // Return 1 if the timeout is reached
    return 1;                                                           // Success
#endif
#if defined (__linux__) || defined (__APPLE__)
    TimeOut         Timer;                                              // Timer used for timeout
    Timer.InitTimer();                                                  // Initialise the timer
    while (Timer.ElapsedTime_ms()<TimeOut_ms || TimeOut_ms==0)          // While Timeout is not reached
    {
        switch (read(fd,pByte,1)) {                                     // Try to read a byte on the device
        case 1  : return 1;                                             // Read successfull
        case -1 : return -2;                                            // Error while reading
        }
    }
    return 0;
#endif
}

/*!
     \brief Read a string from the serial device (without TimeOut)
     \param String : string read on the serial device
     \param FinalChar : final char of the string
     \param MaxNbBytes : maximum allowed number of bytes read
     \return >0 success, return the number of bytes read
     \return -1 error while setting the Timeout
     \return -2 error while reading the byte
     \return -3 MaxNbBytes is reached
  */
int Serial::ReadStringNoTimeOut(char *String,char FinalChar,unsigned int MaxNbBytes)
{
    unsigned int    NbBytes=0;                                          // Number of bytes read
    while (NbBytes<MaxNbBytes)                                          // While the buffer is not full
    {                                                                   // Read a byte with the restant time
        char ret=ReadChar(&String[NbBytes]);                            // Returned value from Read
        if (ret==1)                                                     // If a byte has been read
        {
            if (String[NbBytes]==FinalChar)                             // Check if it is the final char
            {
                String  [++NbBytes]=0;                                  // Yes : add the end character 0
                return NbBytes;                                         // Return the number of bytes read
            }
            NbBytes++;                                                  // If not, just increase the number of bytes read
        }
        if (ret<0) return ret;                                          // Error while reading : return the error number
    }
    return -3;                                                          // Buffer is full : return -3
}

/*!
     \brief Read a string from the serial device (with timeout)
     \param String : string read on the serial device
     \param FinalChar : final char of the string
     \param MaxNbBytes : maximum allowed number of bytes read
     \param TimeOut_ms : delay of timeout before giving up the reading (optional)
     \return  >0 success, return the number of bytes read
     \return  0 timeout is reached
     \return -1 error while setting the Timeout
     \return -2 error while reading the byte
     \return -3 MaxNbBytes is reached
  */
int Serial::ReadString(char *String,char FinalChar,unsigned int MaxNbBytes,unsigned int TimeOut_ms)
{
    if (TimeOut_ms==0)
        return ReadStringNoTimeOut(String,FinalChar,MaxNbBytes);

    unsigned int    NbBytes=0;                                          // Number of bytes read
    char            ret;                                                // Returned value from Read
    TimeOut         Timer;                                              // Timer used for timeout
    Timer.InitTimer();                                                  // Initialize the timer

    while (NbBytes<MaxNbBytes)                                          // While the buffer is not full
    {                                                                   // Read a byte with the restant time
        long int TimeOutParam=TimeOut_ms-Timer.ElapsedTime_ms();                 // Compute the TimeOut for the call of ReadChar
        if (TimeOutParam>0)                                             // If the parameter is higher than zero
        {
            ret=ReadChar(&String[NbBytes],TimeOutParam);                // Wait for a byte on the serial link            
            if (ret==1)                                                 // If a byte has been read
            {
                if (String[NbBytes]==FinalChar)                         // Check if it is the final char
                {
                    String  [++NbBytes]=0;                              // Yes : add the end character 0
                    return NbBytes;                                     // Return the number of bytes read
                }
                NbBytes++;                                              // If not, just increase the number of bytes read
            }
            if (ret<0) return ret;                                      // Error while reading : return the error number
        }
        if (Timer.ElapsedTime_ms()>TimeOut_ms) {                        // Timeout is reached
            String[NbBytes]=0;                                          // Add the end caracter
            return 0;                                                   // Return 0
        }
    }
    return -3;                                                          // Buffer is full : return -3
}

/*!
     \brief Read an array of bytes from the serial device (with timeout)
     \param Buffer : array of bytes read from the serial device
     \param MaxNbBytes : maximum allowed number of bytes read
     \param TimeOut_ms : delay of timeout before giving up the reading
     \return 1 success, return the number of bytes read
     \return 0 Timeout reached
     \return -1 error while setting the Timeout
     \return -2 error while reading the byte
  */
int Serial::Read (void *Buffer,unsigned int MaxNbBytes,unsigned int TimeOut_ms)
{
#if defined (_WIN32) || defined(_WIN64)
    DWORD dwBytesRead = 0;
    timeouts.ReadTotalTimeoutConstant=(DWORD)TimeOut_ms;                // Set the TimeOut
    if(!SetCommTimeouts(hSerial, &timeouts))                            // Write the parameters
        return -1;                                                      // Error while writting the parameters
    if(!ReadFile(hSerial,Buffer,(DWORD)MaxNbBytes,&dwBytesRead, NULL))  // Read the bytes from the serial device
        return -2;                                                      // Error while reading the byte
    if (dwBytesRead!=(DWORD)MaxNbBytes) return 0;                       // Return 0 if the timeout is reached
    return 1;                                                           // Success
#endif
#if defined (__linux__) || defined (__APPLE__)
    TimeOut          Timer;                                             // Timer used for timeout
    Timer.InitTimer();                                                  // Initialise the timer
    unsigned int     NbByteRead=0;
    while (Timer.ElapsedTime_ms()<TimeOut_ms || TimeOut_ms==0)          // While Timeout is not reached
    {
        unsigned char* Ptr=(unsigned char*)Buffer+NbByteRead;           // Compute the position of the current byte
        int Ret=read(fd,(void*)Ptr,MaxNbBytes-NbByteRead);              // Try to read a byte on the device
        if (Ret==-1) return -2;                                         // Error while reading
        if (Ret>0) {                                                    // One or several byte(s) has been read on the device
            NbByteRead+=Ret;                                            // Increase the number of read bytes
            if (NbByteRead>=MaxNbBytes)                                 // Success : bytes has been read
                return 1;
        }
    }
    return 0;                                                           // Timeout reached, return 0
#endif
}

/*!
    \brief Empty receiver buffer (UNIX only)
*/

void Serial::FlushReceiver()
{
#if defined (__linux__) || defined (__APPLE__)
    tcflush(fd,TCIFLUSH);
#endif
}

/*!
    \brief  Return the number of bytes in the received buffer (UNIX only)
    \return The number of bytes in the received buffer
*/
int Serial::Peek()
{
    int Nbytes=0;
#if defined (__linux__) || defined (__APPLE__)
    ioctl(fd, FIONREAD, &Nbytes);
#endif
    return Nbytes;
}

/*!
    \brief      Constructor of the class TimeOut.
*/

TimeOut::TimeOut()
{}

/*!
    \brief      Initialise the timer. It writes the current time of the day in the structure PreviousTime.
*/

void TimeOut::InitTimer()
{
    gettimeofday(&PreviousTime, NULL);
}

/*!
    \brief      Returns the time elapsed since initialization.  It write the current time of the day in the structure CurrentTime.
                Then it returns the difference between CurrentTime and PreviousTime.
    \return     The number of microseconds elapsed since the functions InitTimer was called.
  */

unsigned long int TimeOut::ElapsedTime_ms()
{
    struct timeval CurrentTime;
    int sec,usec;
    gettimeofday(&CurrentTime, NULL);                                   // Get current time
    sec=CurrentTime.tv_sec-PreviousTime.tv_sec;                         // Compute the number of second elapsed since last call
    usec=CurrentTime.tv_usec-PreviousTime.tv_usec;                      // Compute
    if (usec<0) {                                                       // If the previous usec is higher than the current one
        usec=1000000-PreviousTime.tv_usec+CurrentTime.tv_usec;          // Recompute the microseonds
        sec--;                                                          // Substract one second
    }
    return sec*1000+usec/1000;
}
