# Benewake-UART-C-Library
This library can be used to read the data from Benewake TF-Series-UART LiDARs (TF-Nova, TFmini-S, Plus, 02-Pro, Luna, TF03, TF350.)

To compile the source code, you should have installed **make** on your system.
Open a terminal and go to the location where these files are located. From there, type the command:
**make all**

You should see an **exe** file in your folder. You can run that file from your terminal to see the data from sensor in terminal. The data should look like this:

![image](https://github.com/user-attachments/assets/206bb861-1c5b-4b6e-b71c-451edf8c9f05)


**NOTE**: You can adjust the following parameters if needed, especially the COM port **cport_nr** on your system might be different. **COM**1 in case of windows OS is at index 0; you can choose the index accordingly. For example, **COM4** is at index 3 in comports[SERIAL_PORTNR] array and so on.

![image](https://github.com/user-attachments/assets/8119dc9f-44cd-4132-9308-ba70be1f0900)
