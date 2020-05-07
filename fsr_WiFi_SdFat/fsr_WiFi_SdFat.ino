/* FSR Programming with Data Logging into SD Card
 *  This program reads analog inputs from multiple FSR and display its estimated force values.
 *  It takes the analog reading from the force sensors and store them into the log file.
 *  
 *  Date: 11.17.2017
 *  Programmer: Astrini Sie
 *  
 *  Rev 1 dated 01.25.2017
 *  This code sends a VREF (PWM signal) to the op-amp which controls the FSR
 *  
 *  Rev 2 dated 02.07.2018
 *  Includes data logging function to be used with the Adafruit Data Logging Shield
 *  https://learn.adafruit.com/adafruit-data-logger-shield/overview
 *  The data logging portion of the code is based on the lighttemplogger.ino from Adafruit:
 *  https://github.com/adafruit/Light-and-Temp-logger/blob/master/lighttemplogger.ino
 *  
 *  Rev 3 dated 02.22.2018 finished 02.27.2018
 *  Includes WiFi connection to be used with the Adafruit WiFi Shield
 *  Note that the WiFi shield should be used independently from the Data Logging Shield
 *  Tutorial from https://www.arduino.cc/en/Guide/ArduinoWiFiShield
 *  
 *  Rev 4 dated 03.21.2018
 *  Cleanup un-used comments and sections from Rev 3
 *  Includes data collection and logging for 4 FSRs
 *  Updated 04.19.2018: 
 *  Nfsr = 10
 *  Added digital sync signal out to XSENS - not working
 *  
 *  Rev 4b dated 05.16.2018
 *  Includes a buffer before writing data to Serial Monitor
 *  
 *  Rev 4c dated 05.17.2018
 *  Changed SD library to SdFat library to allow for a faster SPI read/write
 *  Updated 06.13.2018
 *  Updated digital out sync from XSENS into Arduino. 
 */

#include <SdFat.h>
#include <SPI.h>
#include <Wire.h>
#include "RTClib.h"
#include <WiFi101.h>

// DEFINITIONS AND DECLARATIONS FOR FSR
#define Nfsr 10                            // number of FSRs attached
int fsrPins[Nfsr] = {0, 4, 1, 3, 2, 5, 9, 6, 8, 7};       
// array containing the pins connected to the FSR: FSR-L(1, 2, 3, 4, 5) to AD(0, 4, 1, 3, 2). FSR-R(1, 2, 3, 4, 5) to AD(5, 9, 6, 8, 7)
int fsrAnVal[Nfsr];                        // analog readings from the FSR
int fsrVolts[Nfsr];                        // voltage values from the FSR
double fsrNewtons[Nfsr];                   // Newton values from the FSR
// Conversion from volt to force values are obtained from calibration and curve fitting. Use fitallvals. fsrNewtons = p1*fsrVolts + p2
double p1[Nfsr] = {0.1039, 0.1549, 0.2891, 0.212, 0.1893, 0.1045, 0.2338, 0.2396, 0.1675, 0.1206};
double p2[Nfsr] = {-52.19, -41.64, -107.3, -57.81, -74.56, -73.6, -86.61, -108.3, -62.46, -49.69};
int i;

// DEFINITIONS AND DECLARATIONS FOR DATA LOGGING
// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  10 // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 1000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()
uint32_t logTime = 0; // time since last log

#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

RTC_DS1307 RTC; // define the Real Time Clock object

// for the data logging shield, we use digital pin 10 for the SD cs line
// Adafruit Data Logger Shield pin 10
// Adafruit WiFi Shield pin 4
const int chipSelect = 4;

// the logging file
SdFat sd;
SdFile logfile;

// user and experimental information
int hasRead = 0;
int subjectNo = 0;
char incomingByte;

// DEFINITIONS AND DECLARATIONS FOR WIFI
char ssid[] = "University of Washington";     // the name of your network
int status = WL_IDLE_STATUS;     // the Wifi radio's status
WiFiServer server(23);
int hasClient = 0;

// DEFINITIONS AND DECLARATIONS FOR DIGITAL SYNC
int hasSync = 0;
int digSync;

// -------------------- ERROR HANDLING -------------------------------------------------------- //
void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(13, HIGH);

  while(1);
}

// -------------------- WIFI STATUS PRINTING -------------------------------------------------- //
void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

// -------------------- SETUP ----------------------------------------------------------------- //
void setup(void) {
  Serial.begin(230400);   // We'll send debugging information via the Serial monitor

  // SETTING UP WIFI
  // ---------------------------------------------------------------------------------------------
  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid); //, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  server.begin();
  // you're connected now, so print out the status:
  printWiFiStatus();

  // SETTING UP SD CARD
  // ---------------------------------------------------------------------------------------------
  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(chipSelect, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!sd.begin(chipSelect, SD_SCK_MHZ(50))) {
    sd.initErrorHalt();
  }
  Serial.println("card initialized.");

  // SETTING UP RTC
  // ---------------------------------------------------------------------------------------------
  if (! RTC.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  // following line sets the RTC to the date & time this sketch was compiled
  RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  
  if (! RTC.isrunning()) {
    Serial.println("RTC is NOT running!");
  }

  // INCOMING WIFI CLIENTS & ENTERING USER AND EXPERIMENT INFO THROUGH WIFI
  // ---------------------------------------------------------------------------------------------
  while(!hasClient) {
    WiFiClient client = server.available();

    if (client) {
      hasClient = 1;
      Serial.println("New client detected");    
      client.flush();   // flushing the buffer otherwise weird characters will be displayed. however this does NOT seem to work???

      // print some random stuff that is currently in the buffer. 255 always shows up for don't know what reason.
      //Serial.println(client.read()); 
      
      client.print("Enter subject number (from 0 to 999): ");
      client.flush();

      while(client.available()) {
        while(!hasRead) {
          hasRead = 1;

          // this while loop makes sure that we get all the characters input from the keyboard before proceeding in the code.
          // the condition that breaks the loop is when ENTER key is pressed (user has done entering characters).
          while(1) {
            incomingByte = client.read();
            // printing these two lines to debug what are the things that actually exist in client and being sent to server
            //Serial.print("Incoming byte: ");
            //Serial.println(incomingByte);
            
            if (incomingByte == '\n') break;   // exit the while(1) upon the ENTER key, we're done receiving.
            if (incomingByte == -1) continue;  // if no characters are in the buffer read() returns -1

            // there is a bunch of junk "characters" sent from PuTTy to Serial. 
            // ignore all these characters and add to subjectNo only the ones that make sense (0 to 9 or 48 to 57 in ASCII)
            if (incomingByte >= 48 && incomingByte <= 57) { 
              subjectNo *= 10;  // shift left 1 decimal place. this line will only work when there is multiple digits (second encounter in while loop)
              // convert ASCII to integer, add, and shift left 1 decimal place
              subjectNo = ((incomingByte - 48) + subjectNo);
              // printing these two lines for debugging
              //Serial.print("Subject no: ");
              //Serial.println(subjectNo);
            }
            //delay(10);    // may not be useful. commented as of now. 
          }
          
          Serial.print("Subject number entered is: ");
          Serial.println(subjectNo);
          if (subjectNo > 999) {
            error("Max subject number is 999.");
          }
        }
      }
    }
  }

  // SETTING UP DATA LOG
  // ---------------------------------------------------------------------------------------------
  // create a new file
  char filename[] = "S000E000.CSV";
  filename[1] = subjectNo/100 + '0';
  filename[2] = (subjectNo%100)/10 + '0';
  filename[3] = (subjectNo%100)%10 + '0';
  for (uint8_t i = 0; i < 1000; i++) {
    filename[5] = i/100 + '0';
    filename[6] = (i%100)/10 + '0';
    filename[7] = (i%100)%10 + '0';
    if (! sd.exists(filename)) {
      break;
      // only assigns a name for a new file if it doesn't exist
    }
  }
  
  if (!logfile.open(filename, O_CREAT | O_WRITE | O_EXCL)) {
    error("couldnt create file");
  }
  
  Serial.print("Logging to: ");
  Serial.println(filename);

  // connect to RTC
  Wire.begin();  
  if (!RTC.begin()) {
    logfile.println("RTC failed");
#if ECHO_TO_SERIAL
    Serial.println("RTC failed");
#endif  //ECHO_TO_SERIAL
  }

    logfile.println("millis,epoch,sync,f1LV,f2LV,f3LV,f4LV,f5LV,f1RV,f2RV,f3RV,f4RV,f5RV");    
#if ECHO_TO_SERIAL
  Serial.println("millis,epoch,sync,f1LV,f2LV,f3LV,f4LV,f5LV,f1RV,f2RV,f3RV,f4RV,f5RV"); 
#endif //ECHO_TO_SERIAL

//  logfile.println("millis,stamp,datetime,sync,f1LV,f1LN,f2LV,f2LN,f3LV,f3LN,f4LV,f4LN,f5LV,f5LN,f1RV,f1RN,f2RV,f2RN,f3RV,f3RN,f4RV,f4RN,f5RV,f5RN");    
//#if ECHO_TO_SERIAL
//  Serial.println("millis,stamp,datetime,sync,f1LV,f1LN,f2LV,f2LN,f3LV,f3LN,f4LV,f4LN,f5LV,f5LN,f1RV,f1RN,f2RV,f2RN,f3RV,f3RN,f4RV,f4RN,f5RV,f5RN"); 
//#endif //ECHO_TO_SERIAL

//  logfile.println("millis,stamp,datetime,sync,f1LAn,f1LV,f2LAn,f2LV,f3LAn,f3LV,f4LAn,f4LV,f5LAn,f5LV,f1RAn,f1RV,f2RAn,f2RV,f3RAn,f3RV,f4RAn,f4RV,f5RAn,f5RV");    
//#if ECHO_TO_SERIAL
//  Serial.println("millis,stamp,datetime,sync,f1LAn,f1LV,f2LAn,f2LV,f3LAn,f3LV,f4LAn,f4LV,f5LAn,f5LV,f1RAn,f1RV,f2RAn,f2RV,f3RAn,f3RV,f4RAn,f4RV,f5RAn,f5RV"); 
//#endif //ECHO_TO_SERIAL
}


// -------------------- LOOP ------------------------------------------------------------------ //
void loop(void) {
if (millis() - logTime >= LOG_INTERVAL) {
  logTime = millis();
  
  // delay for the amount of time we want between readings
  // delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));

  // TIMING
  // ---------------------------------------------------------------------------------------------
  DateTime now;
  now = RTC.now();

  // log milliseconds since starting
  logfile.print(logTime);           // milliseconds since start
  logfile.print(", ");    
  logfile.print(now.unixtime()); // seconds since 1/1/1970
  logfile.print(", ");
//  logfile.print('"');
//  logfile.print(now.year(), DEC);
//  logfile.print("/");
//  logfile.print(now.month(), DEC);
//  logfile.print("/");
//  logfile.print(now.day(), DEC);
//  logfile.print(" ");
//  logfile.print(now.hour(), DEC);
//  logfile.print(":");
//  logfile.print(now.minute(), DEC);
//  logfile.print(":");
//  logfile.print(now.second(), DEC);
//  logfile.print('"');
//  logfile.print(", ");
//  Serial.print(logTime);         // milliseconds since start
//  Serial.print(", ");  
//  Serial.print(now.unixtime()); // seconds since 1/1/1970
//  Serial.print(", ");

  // DIGITAL SYNC TO XSENS
  // ---------------------------------------------------------------------------------------------
  // To make sure that the log file is started before a digital sync signal is sent, put the
  // digital out command in LOOP instead of SETUP

  digSync = analogRead(A15);
  digSync = map(digSync, 0, 1023, 0, 5000);
  logfile.print(digSync);
//  Serial.print(", ");
//  Serial.print(digSync);

  // SENSORS
  // ---------------------------------------------------------------------------------------------
  // Reading VOUT from Op-Amp
  for (i = 0; i < 10; i++) {
    fsrAnVal[i] = analogRead(fsrPins[i]);

    // Conversion from analog reading to voltage values
    fsrVolts[i] = map(fsrAnVal[i], 0, 1023, 0, 5000);

    // Conversion from voltage to force values
    // fsrNewtons[i] = p1[i]*fsrVolts[i] + p2[i];

    logfile.print(", ");    
    logfile.print(fsrVolts[i]);
//    logfile.print(", ");    
//    logfile.print(fsrNewtons[i]);
//    Serial.print(", ");    
//    Serial.print(fsrVolts[i]);
  }

  logfile.println();
//  Serial.println();


if ((millis() - syncTime) >= SYNC_INTERVAL) {
  syncTime = millis();
  
  // LOGGING TIMING
  // ---------------------------------------------------------------------------------------------
  
  Serial.print(logTime);         // milliseconds since start
  Serial.print(", ");  
  Serial.print(now.unixtime()); // seconds since 1/1/1970
  Serial.print(", ");
//  Serial.print('"');
//  Serial.print(now.year(), DEC);
//  Serial.print("/");
//  Serial.print(now.month(), DEC);
//  Serial.print("/");
//  Serial.print(now.day(), DEC);
//  Serial.print(" ");
//  Serial.print(now.hour(), DEC);
//  Serial.print(":");
//  Serial.print(now.minute(), DEC);
//  Serial.print(":");
//  Serial.print(now.second(), DEC);
//  Serial.print('"');
//  Serial.print(", ");
  
  Serial.print(digSync);

  for (i = 0; i < 10; i++) {
    Serial.print(", ");    
    Serial.print(fsrVolts[i]);
//    Serial.print(", ");    
//    Serial.print(fsrNewtons[i]);
  }

  Serial.println();

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time

  logfile.flush();
}
}
}
