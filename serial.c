//**************************************************************************************************************************************
/**
 * @file serial.c
 * @author Ibrahim (Benewake Drone LiDAR Engineer)
 * @brief This C file and its corresponding header file mainly implement the functions for communicating with RS232/UART sensor.
 *
 * @version 0.1
 * @date 2024-11-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */
//**************************************************************************************************************************************

//**************************************************************************************************************************************
#include "serial.h"
//**************************************************************************************************************************************


#if defined(__linux__) || defined(__FreeBSD__)   /****** For Linux & FreeBSD ******/

#define SERIAL_PORTNR  38


int Cport[SERIAL_PORTNR], error;

struct termios new_port_settings, old_port_settings[SERIAL_PORTNR];

//-----------------------------------------------------------------------------------------------------------------
const char *comports[SERIAL_PORTNR]={"/dev/ttyS0","/dev/ttyS1","/dev/ttyS2","/dev/ttyS3","/dev/ttyS4","/dev/ttyS5",
                                    "/dev/ttyS6","/dev/ttyS7","/dev/ttyS8","/dev/ttyS9","/dev/ttyS10","/dev/ttyS11",
                                    "/dev/ttyS12","/dev/ttyS13","/dev/ttyS14","/dev/ttyS15","/dev/ttyUSB0",
                                    "/dev/ttyUSB1","/dev/ttyUSB2","/dev/ttyUSB3","/dev/ttyUSB4","/dev/ttyUSB5",
                                    "/dev/ttyAMA0","/dev/ttyAMA1","/dev/ttyACM0","/dev/ttyACM1",
                                    "/dev/rfcomm0","/dev/rfcomm1","/dev/ircomm0","/dev/ircomm1",
                                    "/dev/cuau0","/dev/cuau1","/dev/cuau2","/dev/cuau3",
                                    "/dev/cuaU0","/dev/cuaU1","/dev/cuaU2","/dev/cuaU3"};
//-----------------------------------------------------------------------------------------------------------------

//**************************************************************************************************************************************
/**
 * @brief This function (for Linux OS) tries to open COM port with specified port-number, baud-rate, mode, and flow control parameters.
 * 
 * @param comport_number 
 * @param baudrate 
 * @param mode 
 * @param flowctrl 
 * @return int 
 */
int SERIAL_OpenComport(int comport_number, int baudrate, const char *mode, int flowctrl) // Function-1
{
  int baudr, status;
 
  if((comport_number>=SERIAL_PORTNR)||(comport_number<0))
  {
    printf("Unknown COM port number\n");
    return(1);
  }

  //-----------------------------------------------------------------------------------------------------------------
  //---------------------------------------------Baud-rate selection-------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------
  switch(baudrate)
  {
    //-------------------------------------------For Non-Linux OS----------------------------------------------------
    case      50 : baudr = B50;
                   break;
    case      75 : baudr = B75;
                   break;
    case     110 : baudr = B110;
                   break;
    case     134 : baudr = B134;
                   break;
    case     150 : baudr = B150;
                   break;
    case     200 : baudr = B200;
                   break;
    case     300 : baudr = B300;
                   break;
    case     600 : baudr = B600;
                   break;
    case    1200 : baudr = B1200;
                   break;
    case    1800 : baudr = B1800;
                   break;
    case    2400 : baudr = B2400;
                   break;
    case    4800 : baudr = B4800;
                   break;
    case    9600 : baudr = B9600;
                   break;
    case   19200 : baudr = B19200;
                   break;
    case   38400 : baudr = B38400;
                   break;
    case   57600 : baudr = B57600;
                   break;
    case  115200 : baudr = B115200;
                   break;
    case  230400 : baudr = B230400;
                   break;
    case  460800 : baudr = B460800;
                   break;
    //-----------------------------------------------------------------------------------------------------------------

    //-------------------------------------------For Linux OS----------------------------------------------------------
#if defined(__linux__)
    case  500000 : baudr = B500000;
                   break;
    case  576000 : baudr = B576000;
                   break;
    case  921600 : baudr = B921600;
                   break;
    case 1000000 : baudr = B1000000;
                   break;
    case 1152000 : baudr = B1152000;
                   break;
    case 1500000 : baudr = B1500000;
                   break;
    case 2000000 : baudr = B2000000;
                   break;
    case 2500000 : baudr = B2500000;
                   break;
    case 3000000 : baudr = B3000000;
                   break;
    case 3500000 : baudr = B3500000;
                   break;
    case 4000000 : baudr = B4000000;
                   break;
    //-----------------------------------------------------------------------------------------------------------------
#endif
    default      : printf("Invalid baudrate\n");
                   return(1);
                   break;
  }
  //-----------------------------------------------------------------------------------------------------------------
  //---------------------------------------------Baud-rate selection-------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------

  int cbits=CS8, cpar=0, ipar=IGNPAR, bstop=0;

  if(strlen(mode) != 3) // Mode could 0, 1, or 2
  {
    printf("Invalid mode \"%s\"\n", mode);
    return(1);
  }

  //-----------------------------------------------------------------------------------------------------------------
  //---------------------------------------------Control Modes selection-------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------
  switch(mode[0])
  {
    case '8': cbits = CS8;
              break;
    case '7': cbits = CS7;
              break;
    case '6': cbits = CS6;
              break;
    case '5': cbits = CS5;
              break;
    default : printf("Invalid number of data-bits '%c'\n", mode[0]);
              return(1);
              break;
  }
  //-----------------------------------------------------------------------------------------------------------------

  //-----------------------------------------------------------------------------------------------------------------
  switch(mode[1])
  {
    // Parity None.
    case 'N':
    case 'n': cpar = 0;
              ipar = IGNPAR;
              break;
    // Parity enable.
    case 'E':
    case 'e': cpar = PARENB;
              ipar = INPCK;
              break;
    // Odd parity, else even.
    case 'O':
    case 'o': cpar = (PARENB | PARODD);
              ipar = INPCK;
              break;
    default : printf("Invalid parity '%c'\n", mode[1]);
              return(1);
              break;
  }
  //-----------------------------------------------------------------------------------------------------------------

  //-----------------------------------------------------------------------------------------------------------------
  switch(mode[2])
  {
    case '1': bstop = 0;
              break;
    case '2': bstop = CSTOPB;
              break;
    default : printf("Invalid number of stop bits '%c'\n", mode[2]);
              return(1);
              break;
  }
  //-----------------------------------------------------------------------------------------------------------------

  //-----------------------------------------------------------------------------------------------------------------
  Cport[comport_number] = open(comports[comport_number], O_RDWR | O_NOCTTY | O_NDELAY);
  if(Cport[comport_number] == -1)
  {
    perror("Unable to open COM port ");
    return(1);
  }
  //-----------------------------------------------------------------------------------------------------------------

  //-----------------------------------------------------------------------------------------------------------------
  if(flock(Cport[comport_number], LOCK_EX | LOCK_NB) != 0)
  {
    close(Cport[comport_number]);
    perror("Another process has locked the comport already.");
    return(1);
  }
  //-----------------------------------------------------------------------------------------------------------------

  //-----------------------------------------------------------------------------------------------------------------
  error = tcgetattr(Cport[comport_number], old_port_settings + comport_number);
  if(error == -1)
  {
    close(Cport[comport_number]); /* Close the port. */
    flock(Cport[comport_number], LOCK_UN);  /* Free the port so that others can use it. */
    perror("Unable to read port settings ");
    return(1);
  }
  //-----------------------------------------------------------------------------------------------------------------
  memset(&new_port_settings, 0, sizeof(new_port_settings));  /* Clear the struct "new_port_settings".*/

  //-----------------------------------------------------------------------------------------------------------------
  new_port_settings.c_cflag = cbits | cpar | bstop | CLOCAL | CREAD;
  if(flowctrl)
  {
    new_port_settings.c_cflag |= CRTSCTS;
  }
  //-----------------------------------------------------------------------------------------------------------------
  new_port_settings.c_iflag = ipar; 
  new_port_settings.c_oflag = 0;
  new_port_settings.c_lflag = 0;
  new_port_settings.c_cc[VMIN] = 0;
  new_port_settings.c_cc[VTIME] = 0;

  //-----------------------------------------------------------------------------------------------------------------
  cfsetispeed(&new_port_settings, baudr);
  cfsetospeed(&new_port_settings, baudr);
  //-----------------------------------------------------------------------------------------------------------------

  //-----------------------------------------------------------------------------------------------------------------
  error = tcsetattr(Cport[comport_number], TCSANOW, &new_port_settings);
  if(error == -1)
  {
    tcsetattr(Cport[comport_number], TCSANOW, old_port_settings + comport_number);
    close(Cport[comport_number]);
    flock(Cport[comport_number], LOCK_UN);
    perror("unable to adjust portsettings ");
    return(1);
  }
  //-----------------------------------------------------------------------------------------------------------------

  //-----------------------------------------------------------------------------------------------------------------
  if(ioctl(Cport[comport_number], TIOCMGET, &status) == -1)
  {
    tcsetattr(Cport[comport_number], TCSANOW, old_port_settings + comport_number);
    flock(Cport[comport_number], LOCK_UN);
    perror("Unable to get port status");
    return(1);
  }
  //-----------------------------------------------------------------------------------------------------------------

  status |= TIOCM_DTR; /* Turn on DTR */
  status |= TIOCM_RTS; /* Turn on RTS */

  //-----------------------------------------------------------------------------------------------------------------
  if(ioctl(Cport[comport_number], TIOCMSET, &status) == -1)
  {
    tcsetattr(Cport[comport_number], TCSANOW, old_port_settings + comport_number);
    flock(Cport[comport_number], LOCK_UN);
    perror("Unable to set port status");
    return(1);
  }
  //-----------------------------------------------------------------------------------------------------------------

  return(0);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
/**
 * @brief This function is used to poll the device (to check the status or send some bytes) connected to Host.
 * 
 * @param comport_number 
 * @param buf 
 * @param size 
 * @return int 
 */
int SERIAL_PollComport(int comport_number, unsigned char *buf, int size) // Function-2
{
  int n;

  n = read(Cport[comport_number], buf, size);

  //-----------------------------------------------------------------------------------------------------------------
  if(n < 0)
  {
    if(errno == EAGAIN)  return 0;
  }
  //-----------------------------------------------------------------------------------------------------------------

  return(n);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
/**
 * @brief This function sends/writes a single byte to the connected device. 
 * 
 * @param comport_number 
 * @param byte 
 * @return int 
 */
int SERIAL_SendByte(int comport_number, unsigned char byte) // Function-3
{
  int n = write(Cport[comport_number], &byte, 1);
  //-----------------------------------------------------------------------------------------------------------------
  if(n < 0)
  {
    if(errno == EAGAIN)
    {
      return 0;
    }
    else
    {
      return 1;
    }
  }
  //-----------------------------------------------------------------------------------------------------------------

  return(0);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
/**
 * @brief This function sends/writes a number of bytes equal to "size" to the connected device. 
 * 
 * @param comport_number 
 * @param buf
 * @param size 
 * @return int 
 */
int SERIAL_SendBuf(int comport_number, unsigned char *buf, int size) // Function-4
{
  int n = write(Cport[comport_number], buf, size);
  //-----------------------------------------------------------------------------------------------------------------
  if(n < 0)
  {
    if(errno == EAGAIN)
    {
      return 0;
    }
    else
    {
      return -1;
    }
  }
  //-----------------------------------------------------------------------------------------------------------------

  return(n);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
/**
 * @brief This function closes the COM port. 
 * 
 * @param comport_number
 * @return void 
 */
void SERIAL_CloseComport(int comport_number) // Function-5
{
  int status;
  //-----------------------------------------------------------------------------------------------------------------
  if(ioctl(Cport[comport_number], TIOCMGET, &status) == -1)
  {
    perror("Unable to get portstatus");
  }
  //-----------------------------------------------------------------------------------------------------------------

  status &= ~TIOCM_DTR;
  status &= ~TIOCM_RTS;

  //-----------------------------------------------------------------------------------------------------------------
  if(ioctl(Cport[comport_number], TIOCMSET, &status) == -1)
  {
    perror("Unable to set port status");
  }
  //-----------------------------------------------------------------------------------------------------------------

  //-----------------------------------------------------------------------------------------------------------------
  tcsetattr(Cport[comport_number], TCSANOW, old_port_settings + comport_number);
  close(Cport[comport_number]);
  flock(Cport[comport_number], LOCK_UN);
  //-----------------------------------------------------------------------------------------------------------------
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
int RS232_IsDCDEnabled(int comport_number)
{
  int status;
  // Try to get the port/connection status.
  ioctl(Cport[comport_number], TIOCMGET, &status);

  if(status&TIOCM_CAR) return(1); // If DC (data carrier) is detected then return 1 otherwise, return 0.
  else return(0);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************

//**************************************************************************************************************************************
int RS232_IsRINGEnabled(int comport_number)
{
  int status;

  ioctl(Cport[comport_number], TIOCMGET, &status);

  if(status&TIOCM_RNG) return(1);
  else return(0);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
RS232_IsCTSEnabled(int comport_number)
{
  int status;

  ioctl(Cport[comport_number], TIOCMGET, &status);

  if(status&TIOCM_CTS) return(1);
  else return(0);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
int RS232_IsDSREnabled(int comport_number)
{
  int status;

  ioctl(Cport[comport_number], TIOCMGET, &status);

  if(status&TIOCM_DSR) return(1); // If flag/status is set to DSR (Data Set Ready).
  else return(0);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_enableDTR(int comport_number)
{
  int status;

  if(ioctl(Cport[comport_number], TIOCMGET, &status) == -1)
  {
    perror("Unable to get port status");
  }

  status |= TIOCM_DTR;

  if(ioctl(Cport[comport_number], TIOCMSET, &status) == -1)
  {
    perror("Unable to set port status");
  }
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_disableDTR(int comport_number)
{
  int status;

  if(ioctl(Cport[comport_number], TIOCMGET, &status) == -1)
  {
    perror("Unable to get port status");
  }

  status &= ~TIOCM_DTR;

  if(ioctl(Cport[comport_number], TIOCMSET, &status) == -1)
  {
    perror("Unable to set port status");
  }
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_enableRTS(int comport_number)
{
  int status;

  if(ioctl(Cport[comport_number], TIOCMGET, &status) == -1)
  {
    perror("Unable to get port status");
  }

  status |= TIOCM_RTS; /* Turn on RTS */

  if(ioctl(Cport[comport_number], TIOCMSET, &status) == -1)
  {
    perror("Unable to set port status");
  }
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_disableRTS(int comport_number)
{
  int status;

  if(ioctl(Cport[comport_number], TIOCMGET, &status) == -1)
  {
    perror("Unable to get port status");
  }

  status &= ~TIOCM_RTS; /* Turn off RTS */

  if(ioctl(Cport[comport_number], TIOCMSET, &status) == -1)
  {
    perror("Unable to set port status");
  }
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_flushRX(int comport_number)
{
  tcflush(Cport[comport_number], TCIFLUSH);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_flushTX(int comport_number)
{
  tcflush(Cport[comport_number], TCOFLUSH);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_flushRXTX(int comport_number)
{
  tcflush(Cport[comport_number], TCIOFLUSH);
}
//**************************************************************************************************************************************


#else  /****** For Windows OS ******/

#define SERIAL_PORTNR  32

HANDLE Cport[SERIAL_PORTNR];

const char *comports[SERIAL_PORTNR]={"\\\\.\\COM1",  "\\\\.\\COM2",  "\\\\.\\COM3",  "\\\\.\\COM4",
                                    "\\\\.\\COM5",  "\\\\.\\COM6",  "\\\\.\\COM7",  "\\\\.\\COM8",
                                    "\\\\.\\COM9",  "\\\\.\\COM10", "\\\\.\\COM11", "\\\\.\\COM12",
                                    "\\\\.\\COM13", "\\\\.\\COM14", "\\\\.\\COM15", "\\\\.\\COM16",
                                    "\\\\.\\COM17", "\\\\.\\COM18", "\\\\.\\COM19", "\\\\.\\COM20",
                                    "\\\\.\\COM21", "\\\\.\\COM22", "\\\\.\\COM23", "\\\\.\\COM24",
                                    "\\\\.\\COM25", "\\\\.\\COM26", "\\\\.\\COM27", "\\\\.\\COM28",
                                    "\\\\.\\COM29", "\\\\.\\COM30", "\\\\.\\COM31", "\\\\.\\COM32"};

char mode_str[128]; // For storing string data 

//**************************************************************************************************************************************
int SERIAL_OpenComport(int comport_number, int baudrate, const char *mode, int flowctrl) // Function-1
{
  if((comport_number>=SERIAL_PORTNR)||(comport_number<0))
  {
    printf("Unknown comport number\n");
    return(1);
  }

  //-------------------------------------------For Windows OS----------------------------------------------------
  switch(baudrate)
  {
    case     110 : strcpy(mode_str, "baud=110");
                   break;
    case     300 : strcpy(mode_str, "baud=300");
                   break;
    case     600 : strcpy(mode_str, "baud=600");
                   break;
    case    1200 : strcpy(mode_str, "baud=1200");
                   break;
    case    2400 : strcpy(mode_str, "baud=2400");
                   break;
    case    4800 : strcpy(mode_str, "baud=4800");
                   break;
    case    9600 : strcpy(mode_str, "baud=9600");
                   break;
    case   19200 : strcpy(mode_str, "baud=19200");
                   break;
    case   38400 : strcpy(mode_str, "baud=38400");
                   break;
    case   57600 : strcpy(mode_str, "baud=57600");
                   break;
    case  115200 : strcpy(mode_str, "baud=115200");
                   break;
    case  128000 : strcpy(mode_str, "baud=128000");
                   break;
    case  256000 : strcpy(mode_str, "baud=256000");
                   break;
    case  500000 : strcpy(mode_str, "baud=500000");
                   break;
    case  921600 : strcpy(mode_str, "baud=921600");
                   break;
    case 1000000 : strcpy(mode_str, "baud=1000000");
                   break;
    case 1500000 : strcpy(mode_str, "baud=1500000");
                   break;
    case 2000000 : strcpy(mode_str, "baud=2000000");
                   break;
    case 3000000 : strcpy(mode_str, "baud=3000000");
                   break;
    default      : printf("invalid baudrate\n");
                   return(1);
                   break;
  }
  //-------------------------------------------For Windows OS----------------------------------------------------

  if(strlen(mode) != 3) // Mode could 0, 1, or 2
  {
    printf("Invalid mode \"%s\"\n", mode);
    return(1);
  }

  //-----------------------------------------------------------------------------------------------------------------
  //---------------------------------------------Control Modes selection-------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------
  switch(mode[0])
  {
    case '8': strcat(mode_str, " data=8");
              break;
    case '7': strcat(mode_str, " data=7");
              break;
    case '6': strcat(mode_str, " data=6");
              break;
    case '5': strcat(mode_str, " data=5");
              break;
    default : printf("Invalid number of data-bits '%c'\n", mode[0]);
              return(1);
              break;
  }
  //-----------------------------------------------------------------------------------------------------------------

  //-----------------------------------------------------------------------------------------------------------------
  switch(mode[1])
  {
    // Parity None.
    case 'N':
    case 'n': strcat(mode_str, " parity=n");
              break;
    // Even parity enable.
    case 'E':
    case 'e': strcat(mode_str, " parity=e");
              break;
    // Odd parity enable.
    case 'O':
    case 'o': strcat(mode_str, " parity=o");
              break;
    default : printf("Invalid parity '%c'\n", mode[1]);
              return(1);
              break;
  }
  //-----------------------------------------------------------------------------------------------------------------

  //-----------------------------------------------------------------------------------------------------------------
  // Stop bits selection
  switch(mode[2])
  {
    case '1': strcat(mode_str, " stop=1");
              break;
    case '2': strcat(mode_str, " stop=2");
              break;
    default : printf("Invalid number of stop bits '%c'\n", mode[2]);
              return(1);
              break;
  }
  //-----------------------------------------------------------------------------------------------------------------

  //-----------------------------------------------------------------------------------------------------------------
  if(flowctrl)
  {
    strcat(mode_str, " xon=off to=off odsr=off dtr=on rts=off");
  }
  else
  {
    strcat(mode_str, " xon=off to=off odsr=off dtr=on rts=on");
  }
  //-----------------------------------------------------------------------------------------------------------------


  Cport[comport_number] = CreateFileA(comports[comport_number],
                      GENERIC_READ|GENERIC_WRITE,
                      0,                          /* no share  */
                      NULL,                       /* no security */
                      OPEN_EXISTING,
                      0,                          /* no threads */
                      NULL);                      /* no templates */

  if(Cport[comport_number] == INVALID_HANDLE_VALUE)
  {
    printf("unable to open comport\n");
    return(1);
  }

  DCB port_settings;
  memset(&port_settings, 0, sizeof(port_settings));  /* clear the new struct  */
  port_settings.DCBlength = sizeof(port_settings);

  if(!BuildCommDCBA(mode_str, &port_settings))
  {
    printf("unable to set comport dcb settings\n");
    CloseHandle(Cport[comport_number]);
    return(1);
  }

  if(flowctrl)
  {
    port_settings.fOutxCtsFlow = TRUE;
    port_settings.fRtsControl = RTS_CONTROL_HANDSHAKE;
  }

  if(!SetCommState(Cport[comport_number], &port_settings))
  {
    printf("unable to set comport cfg settings\n");
    CloseHandle(Cport[comport_number]);
    return(1);
  }

  COMMTIMEOUTS Cptimeouts;

  Cptimeouts.ReadIntervalTimeout         = MAXDWORD;
  Cptimeouts.ReadTotalTimeoutMultiplier  = 0;
  Cptimeouts.ReadTotalTimeoutConstant    = 0;
  Cptimeouts.WriteTotalTimeoutMultiplier = 0;
  Cptimeouts.WriteTotalTimeoutConstant   = 0;

  if(!SetCommTimeouts(Cport[comport_number], &Cptimeouts))
  {
    printf("unable to set comport time-out settings\n");
    CloseHandle(Cport[comport_number]);
    return(1);
  }

  return(0);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
/**
 * @brief This function is used to poll the device (to check the status or send some bytes) connected to Host.
 * 
 * @param comport_number 
 * @param buf 
 * @param size 
 * @return int 
 */
int SERIAL_PollComport(int comport_number, unsigned char *buf, int size) // Function-2
{
  int n;

  //-----------------------------------------------------------------------------------------------------------------
  if(!ReadFile(Cport[comport_number], buf, size, (LPDWORD)((void *)&n), NULL))
  {
    return -1;
  }
  //-----------------------------------------------------------------------------------------------------------------

  return(n);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
/**
 * @brief This function sends/writes a single byte to the connected device.
 * 
 * @param comport_number 
 * @param byte 
 * @return int 
 */
int SERIAL_SendByte(int comport_number, unsigned char byte) // Function-3
{
  int n;

  if(!WriteFile(Cport[comport_number], &byte, 1, (LPDWORD)((void *)&n), NULL))
  {
    return(1);
  }

  if(n<0)  return(1);

  return(0);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
/**
 * @brief This function sends/writes a number of bytes equal to "size" to the connected device. 
 * 
 * @param comport_number 
 * @param buf
 * @param size 
 * @return int 
 */
int SERIAL_SendBuf(int comport_number, unsigned char *buf, int size) // Function-4
{
  int n;

  if(WriteFile(Cport[comport_number], buf, size, (LPDWORD)((void *)&n), NULL))
  {
    return(n);
  }

  return(-1);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
/**
 * @brief This function closes the COM port.
 * 
 * @param comport_number 
 */
void SERIAL_CloseComport(int comport_number) // Function-5
{
  CloseHandle(Cport[comport_number]);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
int RS232_IsDCDEnabled(int comport_number)
{
  int status;

  GetCommModemStatus(Cport[comport_number], (LPDWORD)((void *)&status));

  if(status&MS_RLSD_ON) return(1);
  else return(0);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
int RS232_IsRINGEnabled(int comport_number)
{
  int status;

  GetCommModemStatus(Cport[comport_number], (LPDWORD)((void *)&status));

  if(status&MS_RING_ON) return(1);
  else return(0);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
int RS232_IsCTSEnabled(int comport_number)
{
  int status;

  GetCommModemStatus(Cport[comport_number], (LPDWORD)((void *)&status));

  if(status&MS_CTS_ON) return(1); // MS_CTS_ON => The CTS (clear-to-send) signal is on.
  else return(0);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
int RS232_IsDSREnabled(int comport_number)
{
  int status;

  GetCommModemStatus(Cport[comport_number], (LPDWORD)((void *)&status));

  if(status&MS_DSR_ON) return(1); // MS_DSR_ON => The DSR (data-set-ready) signal is on.
  else return(0);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_enableDTR(int comport_number)
{
  EscapeCommFunction(Cport[comport_number], SETDTR);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_disableDTR(int comport_number)
{
  EscapeCommFunction(Cport[comport_number], CLRDTR);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_enableRTS(int comport_number)
{
  EscapeCommFunction(Cport[comport_number], SETRTS);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_disableRTS(int comport_number)
{
  EscapeCommFunction(Cport[comport_number], CLRRTS);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_flushRX(int comport_number)
{
  PurgeComm(Cport[comport_number], PURGE_RXCLEAR | PURGE_RXABORT);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_flushTX(int comport_number)
{
  PurgeComm(Cport[comport_number], PURGE_TXCLEAR | PURGE_TXABORT);
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_flushRXTX(int comport_number)
{
  PurgeComm(Cport[comport_number], PURGE_RXCLEAR | PURGE_RXABORT);
  PurgeComm(Cport[comport_number], PURGE_TXCLEAR | PURGE_TXABORT);
}
//**************************************************************************************************************************************

#endif

//**************************************************************************************************************************************
/**
 * @brief This function simply sends a string to serial port using already defined function SERIAL_SendByte().
 * 
 * @param comport_number 
 * @param text 
 */
void SERIAL_cputs(int comport_number, const char *text) // Function-6
{
  while(*text != 0)   SERIAL_SendByte(comport_number, *(text++));
}
//**************************************************************************************************************************************

//**************************************************************************************************************************************
/**
 * @brief This function returns index in comports (array) matching to device name or -1 if not found 
 * 
 * @param devname 
 * @return int 
 */
int SERIAL_GetPortnr(const char *devname) // Function-19
{
  int i;

  char str[32];

#if defined(__linux__) || defined(__FreeBSD__)   /**** Linux & FreeBSD ****/
  strcpy(str, "/dev/"); // Copy the string ""/dev/" into str array.
#else  /**** windows ****/
  strcpy(str, "\\\\.\\");
#endif

  strncat(str, devname, 16);
  str[31] = 0;
  //-----------------------------------------------------------------------------------------------------------------
  for(i=0; i<SERIAL_PORTNR; i++)
  {
	if(!strcmp(comports[i], str))
    {
      return i; // Return that index on which the string is matched.
    }
  }
  //-----------------------------------------------------------------------------------------------------------------

  return -1;  /* Device-name/COM-name not found */
}
//**************************************************************************************************************************************

