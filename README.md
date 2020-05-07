# force-insole

Arduino file used to control my homemade FSR (force sensitive resistors)-based force insole. 

PC <===WiFI====> Arduino UNO + SD card module + WiFi module <===Wire===> Force Insole

The insole contains 4-6 FSRs (calibrated previously with calibration parameters contained in the file), connected via wire to an Arduino Uno. This file is capable of initiating the reading of force insole from a PC via WiFi, reading the force values from the FSR, and log them into an SD card.
