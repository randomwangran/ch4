// A simple CH4, CO2, RH, T data logger for the Arduino analog pins
// By David Bastviken and Nguyen Thanh Duc, Linkoping University, Sweden.
//Thanks to cactus.io and Adafruit for code components. For setup of RTC, see separate logger shield documentation.
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>
//#include "RTClib.h" //RTC library
#include <cactus_io_DHT22.h>
//#include "cactus_io_DHT22.h" //RH_Temp library

#define LOG_INTERVAL 2000 // mills between logging entries (reduce to take more/faster data)
  
  // milliseconds before writing the logged data permanently to disk
  // LOG_INTERVAL write each time (safest)
#define SYNC_INTERVAL 5000 // mills between calls to flush() - to write data to the card

uint32_t syncTime = 0; // time of last sync()
#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

#define DHT22_PIN 8 // DHT22 (RH_T) data pin
DHT22 dht(DHT22_PIN);   // Initialize DHT sensor for normal 16mhz Arduino. 

// The analog pins that connect to the sensors
#define CH4sens A1 //CH4 sensor Vout
#define CH4ref A2 //CH4 sensor Vref
#define Vb A0 //Battery voltage
int CH4s = 0;
int CH4r = 0;
int Vbat = 0;
float CH4smV = 0;
float CH4rmV = 0;
float VbatmV = 0;
float mV = 5000;
float steps = 1024;

RTC_PCF8523 RTC; // define the Real Time Clock object

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;
// the logging file
File logfile;

void error(char *str) //Halt if error
{
  Serial.print("error: ");
  Serial.println(str);
  //digitalWrite(redLEDpin, HIGH); // red LED indicates error
  while(1); //halt command
}

double RHValue = 0;
double TempValue = 0;
double CO2Value = 0;
// This is the modified address of the CO2 sensor, 7bits shifted left (defaul 0x68, but collide with RTC chip)
int co2Addr = 0x7F;


/////////////////////////////////////////////////////////////////// 
// Function : void initPoll() 
// Executes : Tells sensor to take a measurement. 
// Notes    : A fuller implementation would read the register back and  
//            ensure the flag was set, but in our case we ensure the poll 
//            period is >25s and life is generally good. 
/////////////////////////////////////////////////////////////////// 
void initPoll() { 
 Wire.beginTransmission(co2Addr); 
 Wire.write(0x11); 
 Wire.write(0x00); 
 Wire.write(0x60); 
 Wire.write(0x35); 
 Wire.write(0xA6); 
  
 Wire.endTransmission(); 
 delay(20);  
 Wire.requestFrom(co2Addr, 2); 
   
 byte i = 0; 
 byte buffer[2] = {0, 0}; 

 while(Wire.available()) { 
     buffer[i] = Wire.read(); 
     i++; 
 }   
}

/////////////////////////////////////////////////////////////////// 
// Function : double readRH() 
// Returns  : The current RH Value, ?1 if error has occured 
/////////////////////////////////////////////////////////////////// 

double readRH() {
 int RH_value = 0;   // We will store the RH value inside this variable.  
 digitalWrite(13, HIGH);                
 
 ////////////////////////// 
 /* Begin Write Sequence */ 
 ////////////////////////// 
  
 Wire.beginTransmission(co2Addr); 
 Wire.write(0x22); //Command number 2 (Read Ram, 2 bytes)
 Wire.write(0x00); //Sensor address in ?? EEPROM ??
 Wire.write(0x14); //Two bytes starting from 0x14 (high byte) and 0x15 (low byte)
 Wire.write(0x36); //Checksum
  
 Wire.endTransmission(); 
 
 delay(20); 
 
 /////////////////////////   
 /* Begin Read Sequence */ 
 /////////////////////////  
   
 Wire.requestFrom(co2Addr, 4); 
   
 byte i = 0; 
 byte buffer[4] = {0, 0, 0, 0}; 
 
while(Wire.available()) { 
     buffer[i] = Wire.read(); 
     i++; 
 }   
   
 RH_value = 0; 
 RH_value |= buffer[1] & 0xFF;
 RH_value = RH_value << 8; 
 RH_value |= buffer[2] & 0xFF;
 Serial.print("RH  Data ");
 Serial.print(buffer[0], HEX);
 Serial.print("|"); 
 Serial.print(buffer[1], HEX);
 Serial.print("|");  
 Serial.print(buffer[2], HEX);
 Serial.print("|"); 
 Serial.println(buffer[3], HEX); 
 
 byte sum = 0;                              //Checksum Byte 
 sum = buffer[0] + buffer[1] + buffer[2];   //Byte addition utilizes overflow 
  
 if(sum == buffer[3]) { 
     // Success! 
     digitalWrite(13, LOW); 
     delay(10); 
     return ((double)RH_value / (double) 100); 
 }   
 else { 
  // Failure!  
  digitalWrite(13, LOW);
  delay(10);
  return ((double) -1); 
 }   
} 

/////////////////////////////////////////////////////////////////// 
// Function : double readTemp() 
// Returns  : The current Temperature Value, ?1 if error has occured 
/////////////////////////////////////////////////////////////////// 

double readTemp() {
 int Temp_value = 0;   // We will store the temperature value inside this variable.  
 digitalWrite(13, HIGH);                
 
 ////////////////////////// 
 /* Begin Write Sequence */ 
 ////////////////////////// 
  
 Wire.beginTransmission(co2Addr); //int K33 == 0x68
 Wire.write(0x22); //Command number 2 (Read Ram, 2 bytes)
 Wire.write(0x00); //Sensor address in ?? EEPROM ??
 Wire.write(0x12); //Two bytes starting from 0x12 (high byte) and 0x13 (low byte)
 Wire.write(0x34); //Checksum
  
 Wire.endTransmission(); 
 
 delay(20); 
 
  
 /////////////////////////   
 /* Begin Read Sequence */ 
 /////////////////////////  
   
 Wire.requestFrom(co2Addr, 4); 
   
 byte i = 0; 
 byte buffer[4] = {0, 0, 0, 0}; 
 
while(Wire.available()) { 
     buffer[i] = Wire.read(); 
     i++; 
 }   
   
 Temp_value = 0; 
 Temp_value |= buffer[1] & 0xFF;
 Temp_value = Temp_value << 8; 
 Temp_value |= buffer[2] & 0xFF;
 Serial.print("T   Data ");
 Serial.print(buffer[0], HEX);
 Serial.print("|"); 
 Serial.print(buffer[1], HEX);
 Serial.print("|");  
 Serial.print(buffer[2], HEX);
 Serial.print("|"); 
 Serial.println(buffer[3], HEX); 
 
 byte sum = 0;                              //Checksum Byte 
 sum = buffer[0] + buffer[1] + buffer[2];   //Byte addition utilizes overflow 
  
 if(sum == buffer[3]) { 
     // Success! 
     digitalWrite(13, LOW); 
     delay(10); 
     return ((double)Temp_value / (double) 100); 
 }   
 else { 
  // Failure!  
  digitalWrite(13, LOW);
  delay(10);
  return ((double) -1); 
 }   
} 


/////////////////////////////////////////////////////////////////// 
// Function : double readCO2() 
// Returns  : The current CO2 Value, ?1 if error has occured 
/////////////////////////////////////////////////////////////////// 

double readCO2() {
 int CO2_value = 0;   // We will store the temperature value inside this variable.  
 digitalWrite(13, HIGH);                
 
 ////////////////////////// 
 /* Begin Write Sequence */ 
 ////////////////////////// 
  
 Wire.beginTransmission(co2Addr); //int K33 == 0x68
 Wire.write(0x22); //Command number 2 (Read Ram, 2 bytes)
 Wire.write(0x00); //Sensor address in ?? EEPROM ??
 Wire.write(0x08); //Two bytes starting from 0x08 (high byte) and 0x09 (low byte). They contain the CO2 data
 Wire.write(0x2A); //Checksum
  
 Wire.endTransmission(); 
 
 delay(50); 
 
 /////////////////////////   
 /* Begin Read Sequence */ 
 /////////////////////////  
   
 Wire.requestFrom(co2Addr, 4); 
   
 byte i = 0; 
 byte buffer[4] = {0, 0, 0, 0}; 
 
while(Wire.available()) { 
     buffer[i] = Wire.read(); 
     i++; 
 }   
   
 CO2_value = 0; 
 CO2_value |= buffer[1] & 0xFF;
 CO2_value = CO2_value << 8; 
 CO2_value |= buffer[2] & 0xFF;
 Serial.print("CO2 Data ");
 Serial.print(buffer[0], HEX);
 Serial.print("|"); 
 Serial.print(buffer[1], HEX);
 Serial.print("|");  
 Serial.print(buffer[2], HEX);
 Serial.print("|"); 
 Serial.println(buffer[3], HEX); 
 
 byte sum = 0;                              //Checksum Byte 
 sum = buffer[0] + buffer[1] + buffer[2];   //Byte addition utilizes overflow 
  
 if(sum == buffer[3]) { 
     // Success! 
     digitalWrite(13, LOW); 
     delay(10); 
     return ((double)CO2_value * (double) 1); 
 }   
 else { 
  // Failure!  
  digitalWrite(13, LOW);
  delay(10);
  return ((double) -1); 
 }   
} 


void setup(void)
{
  Serial.begin(9600);
  Serial.println();
  dht.begin(); //start RH_T_sensor    
  
//#if WAIT_TO_START
//  Serial.println("Type any character to start");
//  while (!Serial.available());
//#endif //WAIT_TO_START

  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");
  
  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  if (! logfile) {
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
  if (! RTC.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
     RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
   
  logfile.println("millis,stampunix,datetime,RH%,tempC,CH4smV, CH4rmV, VbatmV, K33_RH, K33_Temp, K33_CO2");    
#if ECHO_TO_SERIAL
  Serial.println("millis,stampunix,datetime,RH%,tempC,CH4smV, CH4rmV, VbatmV, K33_RH, K33_Temp, K33_CO2");
#endif //ECHO_TO_SERIAL
}

char time_to_read_CO2 = 1;
char n_delay_wait = 0;  
void loop(){
  
  DateTime now;
  
  
  if (time_to_read_CO2 == 1) {
    initPoll();
    delay(50);
    CO2Value = readCO2(); 
    delay(20);
    RHValue = readRH();       
    delay(20);
    TempValue = readTemp();   

// if(RHValue >= 0) { 
//       Serial.print("RH: "); 
//       Serial.print(RHValue); 
//       Serial.print("% | Temp: ");
//       Serial.print(TempValue); 
//       Serial.print("C | CO2: ");
//       Serial.print(CO2Value, 0);
//       Serial.println("ppm"); 
//       Serial.println();       
// }     
// else { 
//       Serial.println(" | Checksum failed / Communication failure"); 
// }     
  time_to_read_CO2 = 0;
  }

  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
  
  // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m);           // milliseconds since start
  logfile.print(", ");    
#if ECHO_TO_SERIAL
  Serial.print(m);         // milliseconds since start
  Serial.print(", ");  
#endif

  // fetch the time
  now = RTC.now();
  // log time
  logfile.print(now.unixtime()); // seconds since 1/1/1970
  logfile.print(", ");
  //logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  //logfile.print('"');
#if ECHO_TO_SERIAL
  Serial.print(now.unixtime()); // seconds since 1/1/1970
  Serial.print(", ");
  //Serial.print('"');
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  //Serial.print('"');
#endif //ECHO_TO_SERIAL

  // Reading temperature or humidity takes about 250 milliseconds.
  // Sensor readings may also be up to 2 seconds 'old' (its a slow sensor)
  dht.readHumidity();
  dht.readTemperature();
    // Check if any reads failed and exit early (to try again).
  if (isnan(dht.humidity) || isnan(dht.temperature_C)) {
    Serial.print("DHT sensor read failure!");
    return;
  }
  CH4s = analogRead(CH4sens); //read CH4 Vout
  CH4smV = CH4s*(mV/steps); //convert pin reading to mV
  delay(10); //delay between reading of different analogue pins adviced.
  CH4r = analogRead(CH4ref); //read CH4 Vref
  CH4rmV = CH4r*(mV/steps); //convert pin reading to mV
  delay(10); //delay between reading of different analogue pins adviced.
  Vbat = analogRead(Vb); //read CH4 Vref
  VbatmV = Vbat *(mV/steps); //convert pin reading to mV, NOT YET correcting for the voltage divider.
  delay(10); //delay between reading of different analogue pins adviced.
   
  logfile.print(", ");    
  logfile.print(dht.humidity);
  logfile.print(", ");    
  logfile.print(dht.temperature_C);
  logfile.print(", ");    
  logfile.print(CH4smV);    
  logfile.print(", ");    
  logfile.print(CH4rmV);
  logfile.print(", ");
  logfile.print(VbatmV);    

  logfile.print(", ");
  logfile.print(RHValue);    
  logfile.print(", ");
  logfile.print(TempValue);    
  logfile.print(", ");
  logfile.print(CO2Value);    
  
#if ECHO_TO_SERIAL
  Serial.print(", ");    
  Serial.print(dht.humidity);
  Serial.print(", ");    
  Serial.print(dht.temperature_C);
  Serial.print(", ");    
  Serial.print(CH4smV);    
  Serial.print(", ");    
  Serial.print(CH4rmV);
  Serial.print(", ");
  Serial.print(VbatmV);
  Serial.print(", ");
  Serial.print(RHValue);
  Serial.print(", ");
  Serial.print(TempValue);
  Serial.print(", ");
  Serial.print(CO2Value);
#endif //ECHO_TO_SERIAL

  logfile.println();
#if ECHO_TO_SERIAL
  Serial.println();
#endif // ECHO_TO_SERIAL

if (time_to_read_CO2 == 0 ) {
  if (n_delay_wait < 29)
    n_delay_wait += 1;
    else {  
      time_to_read_CO2 = 1;   
      Serial.print("Time to read K33 sensor: ");
      n_delay_wait = 0;
  }
}
  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  logfile.flush();

}
