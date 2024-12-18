
//**************************************************************************************************************************************
/**
 * @file tf_series.c
 * @author Ibrahim (Benewake Drone LiDAR Engineer)
 * @brief This script is written for reading data from Benewwake sensors having UART interface and protocol as given below:
 * Byte0  Byte1  Byte2   Byte3   Byte4           Byte5       Byte6                Byte7                Byte8
 * 0x59   0x59   Dist_L  Dist_H  Strength_L      Strength_H  Reserved/ChipTemp_H  Reserved/ChipTemp_H  CheckSum
 *
 * For TF-Nova:
 * Byte0   Byte1   Byte2    Byte3    Byte4        Byte5        Byte6      Byte7        Byte8
 * 0x59    0x59    Dist_L   Dist_H   Strength_L   Strength_H   ChipTemp   Confidance   CheckSum
 *
 * Data Frame Header character: Hex 0x59, Decimal 89, or "Y".
 * 
 * The data is printed to the terminal, exit the program by pressing Ctrl-C.
 * Compile with the command: gcc tf_series.c serial.c -Wall -Wextra -o2 -o tf_series_uart
 * 
 * @version 0.1
 * @date 2024-12-09
 * 
 * @copyright Copyright (c) 2024
 */

//**************************************************************************************************************************************

//**************************************************************************************************************************************
#include <stdlib.h>
#include <stdio.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include "serial.h"
//**************************************************************************************************************************************

struct TFdata data = {0, 0, 0, 0, 0, 2};
struct TFdata *tfdata = &data;

//***************************************************Function to read serial data//****************************************************
int LidarData(int cport_nr)
{
   int n, i;
   unsigned char buf[4096];

   n = SERIAL_PollComport(cport_nr, buf, 4095);
   if(n > 0)
    {
      buf[n] = 0;
      for(i=0; i < n; i++)
      {
        if(buf[i] < 32)  /* replace unreadable control-codes by dots */
        {
          buf[i] = '.';
        }
        if ((buf[i] == HEADER) && (buf[i+1] == HEADER)){
          tfdata->checksum = buf[i] + buf[i+1] + buf[i+2] + buf[i+3] + buf[i+4] + buf[i+5] + buf[i+6] + buf[i+7];
          
          if((tfdata->checksum &0xff) == buf[i+8]){
            //-----------------------------------------------------------------------------------------------------------------
            tfdata->dist = buf[i+2] + buf[i+3]*256;
            tfdata->strength = buf[i+4] + buf[i+5]*256;
            printf("Distance: %d cm\t", tfdata->dist);
            printf("Strength: %d\t", tfdata->strength);
            switch(tfdata->type) {
              case 1 :
                tfdata->temp = buf[i+6];
                tfdata->conf = buf[i+7];
                printf("Confidance: %d\t", tfdata->conf);
                printf("ChipTemperature: %d Celcius\n", tfdata->temp);
                break;
              case 2 :
                tfdata->temp = buf[i+6] + buf[i+7]*256;
                tfdata->temp = tfdata->temp/8 - 256;
                printf("ChipTemperature: %d Celcius\n", tfdata->temp);
                break;
              case 3 :
                printf("TF03 has Reserved Bytes\n" );
                break;
              case 4 :
                tfdata->temp = buf[i+6] + buf[i+7]*256;
                tfdata->temp = tfdata->temp/8 - 256;
                printf("ChipTemperature: %d Celcius\n", tfdata->temp);
                break;
              default :
                printf("Invalid sensor type\n" );
            }
            //-----------------------------------------------------------------------------------------------------------------
          }

        }
      }

    }
    return 0;
}
//***************************************************Function to read serial data//****************************************************

//**************************************************************************************************************************************
/**
 * @brief Main function to open the port and read the data from the sensor.
 * 
 * @return int 
 */
int main()
{
  //****************************Port related settings**************************
  /** 
   * COM1 in case of windows OS is at index 0; you can choose the index accordingly.
   * For example, COM4 is at index 3 in comports[SERIAL_PORTNR] array and so on.*/
  int cport_nr = 3, bdrate = 115200; /* 115200 baud-rate */
  char mode[]={'8','N','1',0};
  //---------------------------------------------------------------------------
  if(SERIAL_OpenComport(cport_nr, bdrate, mode, 0))
  {
    printf("Can not open comport\n");

    return(0);
  }
  //****************************Port related settings**************************

  //***************************************************************************
  enum Sensor type = TFSeries;
  tfdata->type = type; 
  //---------------------------------------------------------------------------
  while(1)
  {

  LidarData(cport_nr);


#ifdef _WIN32
    Sleep(100);
#else
    usleep(100000);  /* Sleep for 100 milliSeconds */
#endif
  }
  //***************************************************************************

  return(0);
}
//**************************************************************************************************************************************

      

