#include "mavlink/tmtc_ladgnss.h"

mavlink_system_t mavlink_system = {
  .sysid = 0,
  .compid = 0
};

/* Hardware Code -------------------------------------------------------------*/
#ifdef __GNUC__
/* Codes for GCC */

/** 
*   @brief  Initialize UART, for Linux.
*  
*   @param  Serial port handler
*   @return True if init success, false if init fail
*/  
bool InitUart(SerialPort* pSerialPort)
{
  
  struct termios newtio;  
  
  pSerialPort->fd = open(pSerialPort->uartName, O_RDWR | O_NOCTTY );
  if(pSerialPort->fd<0) { fprintf(stderr,"ERR MavLink\n"); exit(-1); }
  memset( &newtio, 0, sizeof(newtio) );
  

  newtio.c_iflag = IGNPAR;
  newtio.c_iflag = ICRNL;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  //newtio.c_cflag = B115200;
  
  switch(pSerialPort->baudRate)
  {
  case 115200 : newtio.c_cflag = B115200;
  break;
  case 57600 : newtio.c_cflag = B57600;
  break;
  case 38400 : newtio.c_cflag = B38400;
  break;
  case 9600 : newtio.c_cflag = B9600;
  break;
  default : newtio.c_cflag = B115200;
  break;
  
  }
  newtio.c_cflag |= CS8;
  newtio.c_cflag |= CLOCAL;
  newtio.c_cflag |= CREAD;
  
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 1;
  tcflush (pSerialPort->fd, TCIFLUSH );
  tcsetattr(pSerialPort->fd, TCSANOW, &newtio );
  fcntl(pSerialPort->fd, F_SETFL, FNDELAY);
  
  return pSerialPort->fd;    
}

/** 
*   @brief  Receive bytes by UART, for Linux.
*           The data is stored in serial port handler.
*           After recv, see pSerialPort->rxBuf & pSerialPort->nBytesRecv.
*  
*   @param  Serial port handler.
*   @return True if any bytes received, false if no bytes received.
*/  
bool RecvBytes(SerialPort* pSerialPort)
{
  pSerialPort->nBytesRecv = read(
    pSerialPort->fd,
    pSerialPort->rxBuf,
    sizeof(pSerialPort->rxBuf));

  return (pSerialPort->nBytesRecv >=1 ? TRUE : FALSE);  
}

/** 
*   @brief  Send bytes by UART, for Linux.
*           Send bytes in serial port handler. 
*           Please set pSerialPort->txBuf and pSerialPort->nBytesSend.
*  
*   @param  Serial port handler.
*   @return True if success, false when failed.
*/
bool SendBytes(SerialPort* pSerialPort)
{ 
  uint32_t nBytesWritten = write(
    pSerialPort->fd,
    pSerialPort->txBuf,
    pSerialPort->nBytesSend);

  return (nBytesWritten == pSerialPort->nBytesSend ? TRUE : FALSE); 
}

#elif _MSC_VER

/* Codes for Visual C++ */

/** 
*   @brief  Initialize UART, for Windows.
*  
*   @param  Serial port handler.
*   @return True if init success, false if init fail.
*/   
bool InitUart(SerialPort* pSerialPort)
{
  wchar_t uartName[32] = { 0 };
  MultiByteToWideChar(0, 0,
    pSerialPort->uartName,
    strlen(pSerialPort->uartName),
    uartName,
    strlen(pSerialPort->uartName));
  
  pSerialPort->hFile = CreateFile(
    uartName,                           // Port name
    GENERIC_READ | GENERIC_WRITE,       // Read/Write
    0,                                  // No Sharing
    NULL,                               // No Security
    OPEN_EXISTING,                      // Open existing port only
    0,                                  // Non Overlapped I/O
    NULL);                              // Null for Comm Devices
  
  if (pSerialPort->hFile == INVALID_HANDLE_VALUE)
    return false;
  
  DCB dcbSerialParams = { 0 };            // Initializing DCB structure
  dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
  if (!GetCommState(pSerialPort->hFile, &dcbSerialParams))
    return false;
  
  dcbSerialParams.BaudRate = pSerialPort->baudRate; // Setting BaudRate
  dcbSerialParams.ByteSize = 8;           // Setting ByteSize = 8
  dcbSerialParams.StopBits = ONESTOPBIT;  // Setting StopBits = 1
  dcbSerialParams.Parity = NOPARITY;      // Setting Parity = None
  
  return SetCommState(pSerialPort->hFile, &dcbSerialParams);
}

/** 
*   @brief  Receive bytes by UART, for Windows.
*           The data is stored in serial port handler.
*           After recv, see pSerialPort->rxBuf & pSerialPort->nBytesRecv
*  
*   @param  Serial port handler
*   @return True if any bytes received, false if no bytes received.
*/ 
bool RecvBytes(SerialPort* pSerialPort)
{
  return ReadFile(
    pSerialPort->hFile,
    pSerialPort->rxBuf,
    sizeof(pSerialPort->rxBuf),
    (LPDWORD)&pSerialPort->nBytesRecv,
    NULL);
}

/** 
*   @brief  Send bytes by UART, for Linux.
*           Send bytes in serial port handler. 
*           Please set pSerialPort->txBuf and pSerialPort->nBytesSend.
*  
*   @param  Serial port handler
*   @return True if success, false when failed.
*/
bool SendBytes(SerialPort* pSerialPort)
{
  uint32_t bytesWritten;
  return WriteFile(
   pSerialPort->hFile,
   pSerialPort->txBuf,
   pSerialPort->nBytesSend,
   (LPDWORD)&bytesWritten,
   NULL);
}


#endif

/** 
*   @brief  Send mavlink msg with serial port handler.
*  
*   @param  Serial port handler
*   @param  Pointer of mavlink message.
*   @return True if success, false when failed.
*/
bool SendMavlink(SerialPort* pSerialPort, mavlink_message_t* pMsg)
{
  pSerialPort->nBytesSend = mavlink_msg_to_send_buffer(
    pSerialPort->txBuf,
    pMsg);
  return SendBytes(pSerialPort);
}
