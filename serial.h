//**************************************************************************************************************************************
/**
 * @file serial.h
 * @author Ibrahim (Benewake Drone LiDAR Engineer)
 * @brief This header file and its corresponding C file mainly implement the functions for communicating with RS232/UART sensor.
 *
 * @version 0.1
 * @date 2024-11-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */
//**************************************************************************************************************************************

//************************************************************First ifndef**************************************************************
#ifndef SERIAL_INCLUDED
#define SERIAL_INCLUDED

//************************************************************extern "C"****************************************************************
#ifdef __cplusplus
extern "C" {
#endif

//**************************************************************************************************************************************
#include <stdio.h>
#include <string.h>
//**************************************************************************************************************************************

//******************************************************OS Specific Headers*************************************************************
#if defined(__linux__) || defined(__FreeBSD__)

#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>
#include <sys/file.h>
#include <errno.h>

#else

#include <windows.h>

#endif
//******************************************************OS Specific Headers*************************************************************

//**************************************************************************************************************************************
#define HEADER 0x59

typedef enum Sensor{
	Nova = 1,
	TFSeries, // TFmini-S, Plus, 02-Pro, Luna
	TF03, // TF03-100,180, 350
	TFS20L
}sensor;

struct TFdata{
	int checksum;
	int dist;
	int strength;
	int temp;
	int conf;
	int type;
};
//**************************************************************************************************************************************

//**************************************************************************************************************************************
/**
 * @brief The following are necessary functions/methods for UART/RS232 communication.
 */

//**************************************************************************************************************************************
/**
 * @brief This function tries to open COM port with specified port-number, baud-rate, mode, and flow control parameters.
 * 
 * @param comport_number 
 * @param baudrate 
 * @param mode 
 * @param flowctrl 
 * @return int 
 */
int SERIAL_OpenComport(int comport_number, int baudrate, const char *mode, int flowctrl); // Function-1
//**************************************************************************************************************************************

//**************************************************************************************************************************************
int SERIAL_PollComport(int, unsigned char *, int); // Function-2
//**************************************************************************************************************************************

//**************************************************************************************************************************************
int SERIAL_SendByte(int, unsigned char); // Function-3
//**************************************************************************************************************************************

//**************************************************************************************************************************************
int SERIAL_SendBuf(int, unsigned char *, int); // Function-4
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void SERIAL_CloseComport(int); // Function-5
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void SERIAL_cputs(int, const char *); // Function-6
//**************************************************************************************************************************************

//**************************************************************************************************************************************
int RS232_IsDCDEnabled(int);
//**************************************************************************************************************************************

//**************************************************************************************************************************************
int RS232_IsRINGEnabled(int);
//**************************************************************************************************************************************

//**************************************************************************************************************************************
int RS232_IsCTSEnabled(int);
//**************************************************************************************************************************************

//**************************************************************************************************************************************
int RS232_IsDSREnabled(int);
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_enableDTR(int);
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_disableDTR(int);
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_enableRTS(int);
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_disableRTS(int);
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_flushRX(int);
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_flushTX(int);
//**************************************************************************************************************************************

//**************************************************************************************************************************************
void RS232_flushRXTX(int);
//**************************************************************************************************************************************

//**************************************************************************************************************************************
int SERIAL_GetPortnr(const char *); // Function-19
//**************************************************************************************************************************************

#ifdef __cplusplus
} /* extern "C" */
#endif
//************************************************************End of extern "C"*********************************************************

#endif
//************************************************************End of first ifndef*******************************************************


