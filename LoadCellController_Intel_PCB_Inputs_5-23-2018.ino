/*
  The circuit:
  SD card attached to SPI bus as follows:
** UNO:  MOSI - pin 11, MISO - pin 12, CLK - pin 13, CS - pin 4 (CS pin can be changed)
  and pin #10 (SS) must be an output
** Mega: MISO - pin 50, MOSI - pin 51, CLK - pin 52, CS - pin 53 (CS pin can be changed)
  and pin #52 (SS) must be an output
*/
#include <EEPROM.h>
#include <HX711.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>    //For RTC Time
#include <Wire.h>      //For RTC data Logging
#include <TimeLib.h>   //For RTC data Logging
#include <Statistic.h>  // without trailing s
#include <LiquidCrystal_I2C.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
LiquidCrystal_I2C lcd(0x3F, 20, 4);// Set the LCD address to 0x27 for a 20 chars and 4 line display
#define FS610Relay   34
#define FS609Relay   35
#define FS601Relay   36
#define simRelay     37
#define FS610Pin     28
#define FS609Pin     27
#define FS601Pin     26
#define CS610Pin     29
#define CS609Pin     30
#define CS601Pin     31
//#define levelSensor A10
#define pump        8
#define drainPump   7
#define NUMREADINGS 10  //Determines number of readings for running average logging
const int chipSelect = 53;
//#define vccPin A11
//#define gndPin A12
HX711 scale1(48, 49);   // parameter "gain" is ommited; the default value 128 is used by the library
HX711 scale2(46, 47); // HX711 var(DAT,CLK); (GRN,WHT)
HX711 scale3(44, 45);
HX711 scale4(42, 43);
Statistic myStats;
Statistic load1Stats;
Statistic load2Stats;
Statistic load3Stats;
Statistic load4Stats;
Statistic grams3Stats;
Statistic grams4Stats;
Statistic lowLevel;
Statistic midLevel;
Statistic highLevel;
Statistic loadTotals;
Statistic grams1Stats;
Statistic grams2Stats;
//float levelSensorReading;
float waterHeight = 0.0;
bool debugMode;
long load1;
long load2;
long load3;
long load4;

//Added for running average calculation
long loadArray[4];    //Move load1, load2 etc to this array to easily perform running average
long readings[4][NUMREADINGS];    //Array which stores last X readings per load cell for running average
long total[4];      //Total of running average for each load sensor
long runLoad[4];    //Calculated running average for each load cell
long index = 0;     //Current reading being updated in the running average array

long loadTotal;
long loadTotalAVG;
long gramsTotal;
long gramsAvgSum;
long gramsMaxSum;
long gramsMinSum;
byte displayMode; //0 = raw, 1 = raw 2 = grams,
byte senseMode = 1;
long prevLowSetPoint;
long prevMidSetPoint;
long prevHighSetPoint;
long lowSetPoint =  8930961;
long midSetPoint =  9000000; //2582000
long highSetPoint = 9439989; //2630000
char ch = ' ';
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
char dateTime[6][7] = {"year", "month", "day", "hour", "minute", "second"};
char fileDate[] = "12-30-17.CSV";
char fileTime[] = "12.60.00.CSV";
String fileLocation = "/Logs/";
String fileName = "fileDate.csv";
char fileDir[] = "/fileDate/";
bool loggingMode;
bool FS610pos = 2;
bool FS609pos = 2;
bool FS601pos = 2;
bool FS610RelayState = 1;
bool FS609RelayState = 1;
bool FS601RelayState = 1;
bool prevFS610pos = 1;
bool prevFS609pos = 1;
bool prevFS601pos = 1;
bool CS610pos = 1;
bool CS609pos = 1;
bool CS601pos = 1;
bool FS601Sim = 0;
bool FS609Sim = 0;
bool FS610Sim = 0;
bool CSmode = 1;
bool FSmode = 0;
bool simulationMode; // Initiates relays to bypass level switches
byte numberCells = 4; // Number of Load Cells connected to controller
int calibrationFactor = 25; //2740; //-2740 for new LC, -2444 for old LC, 1980 works for right side up, -1530;
long grams1;
long grams2;
long grams3;
long grams4;
long gramsAvg1;
long gramsAvg2;
long gramsAvg3;
long gramsAvg4;
long zeroFactorSum;
long zeroFactor1;
long zeroFactor2;
long zeroFactor3;
long zeroFactor4;
unsigned long previousMillis = 0;
byte goodEnough = 1;
byte displayPages = 6; // The total number of display modes
const byte eepromValid = 420;    // If the first byte in eeprom is this then the data is valid.
const byte clearMemory = 69;  // Writes memory with values other than the validation numbers.
byte row = 0;
long loggingTimer;
bool menuMode = false;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
volatile boolean fired = false;
volatile boolean menu = false;
volatile long rotaryCount = 0;
bool rotateCW;
bool rotateCC;
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
const word updateLCD = 250; //LCD refresh speed in miliseconds
word clearLCD =  0;
unsigned long drainCycle = 0;
unsigned long pumpCycle = 0;
unsigned long idleCycle = 0;
bool drainMode = LOW;
bool pumpMode = LOW;
bool pumpStatus = 0;
bool drainPumpStatus = 0;
byte mac[] = { 0x90, 0xA2, 0xDA, 0x11, 0x43, 0x6B };
// NTP Servers:
IPAddress timeServer(132, 163, 97, 3); // time-c-wwv.nist.gov Location: WWV, Fort Collins, Colorado
const int timeZone = -8;  // Pacific Standard Time (USA) **Note Daylight savings is not apply
//const int timeZone = -7;  // Pacific Daylight Time (USA)
EthernetUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets
RTC_DS3231 rtc;        //For RTC Time
File Log;
//-------- NTP code ----------//
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets
time_t getNtpTime() {
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  if ( debugMode == true) {
    Serial.print(" Transmit NTP Request sent...");
  }
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      if ( debugMode == true) {
        Serial.print("Receiving NTP Response...");
      }
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  if ( debugMode == true) {
    Serial.print("No NTP Response at ");
  }
  return 0; // return 0 if unable to get the time
}
void sendNTPpacket(IPAddress &address) { // send an NTP request to the time server at the given address
  memset(packetBuffer, 0, NTP_PACKET_SIZE);  // set all bytes in the buffer to 0
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
void setup() {
  Serial.begin(115200);
//  pinMode(relayPower, OUTPUT);
  // pinMode(vccPin, OUTPUT);
  pinMode(simRelay, OUTPUT);
  pinMode(FS610Pin, INPUT);
  pinMode(FS609Pin, INPUT);
  pinMode(FS601Pin, INPUT);
  pinMode(FS610Relay, OUTPUT);
  pinMode(FS609Relay, OUTPUT);
  pinMode(FS601Relay, OUTPUT);
//  digitalWrite(relayPower, HIGH);
  digitalWrite(simRelay, HIGH);
  digitalWrite(FS610Relay, HIGH);
  digitalWrite(FS609Relay, HIGH);
  digitalWrite(FS601Relay, HIGH);
  pinMode(pump, OUTPUT);
  pinMode(drainPump, OUTPUT);
  digitalWrite(pump, HIGH);
  digitalWrite(drainPump, HIGH);
  attachInterrupt (0, menuButton, FALLING);   // pin 3. Button triggers on release with 10K soldered to R1
  attachInterrupt (4, isr, CHANGE);   // pin 19 encoder DT  pinB
  attachInterrupt (5, isr, CHANGE);   // pin 18 encoder CLK pinA
  //digitalWrite(gndPin, LOW);
  //digitalWrite(vccPin, HIGH);
  fired = true;
  long displayTime = 3500;
  byte reading = EEPROM.read(1);//  read the value of unlocked from EEPROM only read EEPROM if the signature byte is correct
  if (reading == eepromValid) {
    reading = EEPROM.read(0);
    if (reading == true) {
      Serial.println("logging is active");
      loggingMode = true;
    }
  } else {
    Serial.print("EEPROM memory for loggingMode was invalid and stored as ");
    Serial.println(reading);
  }
  reading = EEPROM.read(3);
  if (reading == eepromValid) {//  read the value of debugMode from EEPROM only read EEPROM if the signature byte is correct
    reading = EEPROM.read(2);
    if (reading == true) {
      Serial.println("debugMode is active");
      debugMode = true;
    }
  } else {
    Serial.print("EEPROM memory for debugMode was invalid and stored as ");
    Serial.println(reading);
  }
  reading = EEPROM.read(5);
  if (reading == eepromValid) {//  read the value of debugMode from EEPROM only read EEPROM if the signature byte is correct
    reading = EEPROM.read(4);
    if (reading == true) {
      Serial.println("simulationMode is active");
      simulationMode = true;
    }
  } else {
    Serial.print("EEPROM memory for simulationMode was invalid and stored as ");
    Serial.println(reading);
  }
  reading = EEPROM.read(7);//  read the value of displayMode from EEPROM only read EEPROM if the signature byte is correct
  if (reading == eepromValid) {
    reading = EEPROM.read(6);
    displayMode = reading;
  } else {
    Serial.print("EEPROM memory for displayMode was invalid and stored as ");
    Serial.println(reading);
    displayMode = 4;
  }
  lcd.begin();
  lcd.setCursor(0, row);
  lcd.print("Level Switch Control");
  row++;
  if (!SD.begin(chipSelect)) { // if the card is not present and can not be initialized;
    Serial.println("SD card hardware communication error");
    lcd.setCursor(0, row);
    lcd.print("SD card comms error");
    row++;
    displayTime = displayTime + 3300;
    // if (debugMode == false) return; // don't do anything more:
  }
  Wire.begin();
  //while (!Serial); //do not wait until Arduino Serial Monitor opens. Needed for native USB port only
  if (! rtc.begin()) {
    Serial.println("Real Time Clock communication error");
    lcd.setCursor(0, row);
    lcd.print("RTC comm error      ");
    row++;
    displayTime = displayTime + 3300;
  }
  if (rtc.lostPower()) {
    Serial.println("Real Time Clock lost power, please replace clock battery");
    updateClock();
  } else {
    reading = EEPROM.read(9);
    if (reading == eepromValid) {//  read the command to update clock from EEPROM  signature byte is correct
      reading = EEPROM.read(8);
      if (reading == 1) {
        Serial.println("Real Time Clock was scheduled to update time on bootup");
        updateClock();
        EEPROM.write(9, clearMemory);            // Change out the write mem signature
        EEPROM.write(8, 0);            // Write the value of schedule
        EEPROM.write(9, eepromValid);  // Write the signature for confirmation of successful memory write
      }
    }
    getfileDate(fileDate);
    fileName = fileDate;
  }
  if (row >> 1) {
    lcd.setCursor(0, row);
    lcd.print("Contact Calvin Cox ");
    row++;
  } else {
    lcd.setCursor(0, row);
    lcd.print("Calvin Cox Designes");
    row++;
  }
  if ( !SD.exists("/Logs/") ) { // see if the directory exists, create it if not.
    if ( SD.mkdir("/Logs/") ) {
      Serial.println("/Logs/ directory created");
    } else {
      lcd.setCursor(0, row);
      lcd.print("dirDNE");
      row++;
      displayTime = displayTime + 3300;
      Serial.print("/Logs/");
      Serial.println(" directory still does not exist");
    }
  } else {
    lcd.setCursor(0, row);
    row++;
    lcd.print("/Logs/");
    lcd.print(fileName);
  }
  Serial.print("Level Switch Controller initialized at ");
  printDate();
  Serial.println("Enter 0 for available commands");
  if (loggingMode == false) {
    unsigned long currentMillis = millis();
    while (millis() <= currentMillis + displayTime) {
      lcd.setCursor(0, row);
      DateTime now = rtc.now();
      lcdPrintTime();
      lcd.print(' ');
      lcd.print(now.month(), DEC);
      lcd.print('/');
      lcd.print(now.day(), DEC);
      lcd.print('/');
      lcd.print(now.year(), DEC);
    }
  }
  lcd.clear();
}
void loop() {
  if (fired) {
    if (rotateCW == true) cycleDisplayMode();
    if (rotateCC == true) cycleDisplayModeMinus();
    fired = false;
    updateDisplay();
  }
  if (menuMode == true) {
    menuMode = false;
    switch (displayMode) {
      case 1: {
          toggleLogging();
          break;
        }
      case 2: {
          toggleSimMode();
          break;
        }
      case 3: {
          toggleDebugMode();
          break;
        }
      case 4: {
          //toggleSenseMode();
          break;
        }
      case 5: {
          clearAll();
          break;
        }
      case 6: {
          goodEnough = 1;
          clearAll();
          zeroScale();
          break;
        }
      default: {
          break;
        }
    }
    lcd.clear();
  }
  if (Serial.available()) ch = Serial.read();
  recieveCommands();
  getReadings();
  updateDisplay();
  if (senseMode == 1) { // Float Switches
    if (FS610pos == LOW && FS609pos == HIGH) { // conditional error
      if (simulationMode == false) idle(); 
    }
    if (FS610pos == HIGH && FS609pos == HIGH) {
      lowLevel.add(loadTotal);
      lowSetPoint = lowLevel.average();
      if (simulationMode == true)
        fill();
    }
    if (FS610pos == LOW && FS609pos == LOW) {
      midLevel.add(loadTotal);
      midSetPoint = midLevel.average();
      /*
        if (simulationMode == true)
        drain();
      */
    }
    if (FS601pos == LOW && FS609pos == LOW) {
      highLevel.add(loadTotal);
      highSetPoint = highLevel.average();
      if (simulationMode == true)
        drain();
    }
    if (FS610pos == HIGH && FS601pos == LOW) {
      idle();
    }
  }
  if (senseMode == 2) { // Loadcells
    if (simulationMode == true) {
      while (loadTotal <= highSetPoint) {
        if (Serial.available()) ch = Serial.read();
        recieveCommands();
        getReadings();
        updateDisplay();
        if (loggingMode == true) writeSD();
        fill();
      }
      while (loadTotal >= lowSetPoint) {
        if (Serial.available()) ch = Serial.read();
        recieveCommands();
        getReadings();
        updateDisplay();
        if (loggingMode == true) writeSD();
        drain();
      }
      /*if (loadTotal >= midSetPoint) {
        drain();
        }

        if (loadTotal >= highSetPoint) {
        drain();
        }*/
    }
  }
  if (senseMode == 3) { // Capacitor
    if (CS610pos == HIGH && CS601pos == HIGH) {
      if (simulationMode == true) fill();
    }
    if (CS610pos == LOW && CS601pos == LOW) {
      if (simulationMode == true) drain();
    }
    if (CS610pos == HIGH && CS601pos == LOW) {
      idle(); //Serial.println("Either your float switches are plugged in backwards or your bottom level switch is unplugged, jammed, or wired incorrectly.");
    }
  }
  if (loggingMode == true) writeSD();
}
void recieveCommands() {
  if (Serial.available()) ch = Serial.read();
  if (ch == '0') {
    Serial.println("");
    Serial.println("1. Toggle Data Logging");
    Serial.println("2. Toggle Simulation Mode");
    Serial.println("3. Toggle Debug Mode");
    Serial.println("4. Alternate Display Mode");
    Serial.println("5. Toggle Sense Mode");
    Serial.println("8. Toggle between 1 and 4 cells");
    Serial.println("9. Zero Scale");
    Serial.println("a. AutoCalibrate to 10 grams");
    Serial.println("b. to sync NTP time");
    Serial.println("c. Clear All");
    Serial.print  ("d. Serial print / Logs / "); Serial.println(fileName);
    Serial.println("e to schedule RTC update on next boot sequence");
    Serial.println("f. Print clock times");
    Serial.println("g. Toggle Fill Pump");
    Serial.println("h. Toggle Drain Pump");
    Serial.println("j. Toggle Relay #1");
    Serial.println("k. Toggle Relay #2");
    Serial.println("l. Toggle Relay #3");
    ch = ' ';
  }
  if (ch == '1') {
    toggleLogging();
    ch = ' ';
  }
  if (ch == '2') {
    toggleSimMode();
    ch = ' ';
  }
  if (ch == '3') {
    toggleDebugMode();
    ch = ' ';
  }
  if (ch == '4') {
    cycleDisplayMode();
    ch = ' ';
  }
  if (ch == '5') {
    toggleSenseMode();
    ch = ' ';
  }
  if (ch == 'l') {
    Serial.print("Changed FS610 Relay state to ");
    FS610RelayState = ! FS610RelayState;
    digitalWrite(FS610Relay, FS610RelayState);
    Serial.println(FS610RelayState);
    ch = ' ';
  }
  if (ch == 'k') {
    Serial.print("Changed FS609 Relay state to ");
    FS609RelayState = ! FS609RelayState;
    digitalWrite(FS609Relay, FS609RelayState);
    Serial.println(FS609RelayState);
    ch = ' ';
  }
  if (ch == 'j') {
    Serial.print("Changed FS601 Relay state to ");
    FS601RelayState = ! FS601RelayState;
    digitalWrite(FS601Relay, FS601RelayState);
    Serial.println(FS601RelayState);
    ch = ' ';
  }
  if (ch == '8' ) {
    Serial.print("Changed to ");
    if (numberCells == 1) {
      numberCells = 4;
      Serial.println(numberCells);
    } else {
      numberCells = 1;
      Serial.println(numberCells);
    }
    ch = '9';
  }
  if (ch == '9' ) {
    goodEnough = 1;
    clearAll();
    zeroScale();
    ch = ' ';
  }
  if (ch == 'a') {
    grams1Stats.clear();
    lcd.clear();
    Serial.print("Auto Calibrating");
    autoCalibrate();
    ch = ' ';
  }
  if (ch == 'b') {
    Serial.println("Syncing NTP time from government time server");
    setSyncProvider(getNtpTime);
    Serial.print("NTP time is now");
    ntpTime(); // Display NTP time
  }
  if (ch == 'c') {
    clearAll();
    ch = ' ';
  }
  if (ch == 'd') {
    Serial.print("Initiating serial print of ");
    Serial.print(fileLocation);
    Serial.println(fileName);
    readLog();
    ch = ' ';
  }
  if (ch == 'e') {
    int reading = EEPROM.read(4);
    if (reading == 0) {
      Serial.println("RTC update is scheduled for next boot");
      EEPROM.write(9, 2);            // Change out the write mem signature
      EEPROM.write(8, 1);     // Write the value of unlocked
      EEPROM.write(9, eepromValid);  // Write the signature for confirmation of successful memory write
    }
    if (reading == 1) {
      Serial.println("RTC update was already scheduled");
    }
    ch = ' ';
  }
  if (ch == 'f') {
    Serial.print("RTC time is set");
    printDate();
    Serial.print("NTP time is set");
    ntpTime();
    ch = ' ';
  }
  if (ch == 'g') {
    pumpStatus = !pumpStatus;
    digitalWrite(pump, !pumpStatus);
    if (pumpStatus == 1) {
      fill();
      Serial.println("Fill pump is on");
    }
    if (pumpStatus == 0) {
      idle();
      Serial.println("Fill pump is off");
    }
    ch = ' ';
  }
  if (ch == 'h') {
    drainPumpStatus = !drainPumpStatus;
    digitalWrite(drainPump, !drainPumpStatus);
    if (drainPumpStatus == 1) {
      drain();
      Serial.println("Drain pump is on");
    }
    if (drainPumpStatus == 0) {
      idle();
      Serial.println("Drain pump is off");
    }
    ch = ' ';
  }
}
void updateDisplay() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= updateLCD) {
    previousMillis = currentMillis;
    clearLCD = clearLCD + updateLCD;
    switch (displayMode) {
      case 7: {
          lcd.setCursor(0, 0);
          lcd.print("#1 ");
          lcd.print(grams1);
          lcd.print("g");
          lcd.setCursor(10, 0);
          lcd.print("#2 ");
          lcd.print(grams2);
          lcd.print("g");
          lcd.setCursor(0, 1);
          lcd.print("#3 ");
          lcd.print(grams3);
          lcd.print("g");
          lcd.setCursor(10, 1);
          lcd.print("#4 ");
          lcd.print(grams4);
          lcd.print("g");
          lcd.setCursor(0, 2);
          lcd.print("Total Grams ");
          lcd.print(gramsTotal);
          lcd.print("g");
          if (pumpStatus == 1 && drainPumpStatus == 0) {
            lcd.setCursor(0, 3);
            lcd.print("Fill cycle ");
            lcd.print(drainCycle);
          }
          if (pumpStatus == 0 && drainPumpStatus == 1) {
            lcd.setCursor(0, 3);
            lcd.print("Drain cycle ");
            lcd.print(pumpCycle);
          }
          if (pumpStatus == 0 && drainPumpStatus == 0) {
            lcd.setCursor(0, 3);
            lcd.print("Pumps Idle       ");
          }
          break;
        }
      case 6: {
          lcd.setCursor(0, 0);
          lcd.print("H20 ");
          lcd.print(waterHeight);
          lcd.print("cm  ");
          lcd.print((loadTotal / calibrationFactor) / 1000);
          lcd.setCursor(0, 3);
          lcd.print("LS610 LL: ");
          lcd.print((lowSetPoint / calibrationFactor) / 1000);
          lcd.print("Kg");
          lcd.setCursor(0, 2);
          lcd.print("LS609 LH: ");
          lcd.print((midSetPoint / calibrationFactor) / 1000);
          lcd.print("Kg");
          lcd.setCursor(0, 1);
          lcd.print("LS601 HH: ");
          lcd.print((highSetPoint / calibrationFactor) / 1000);
          lcd.print("Kg");
          break;
        }
      case 3: {
          lcd.setCursor(0, 2);
          lcd.print("610 ");
          lcd.print(lowSetPoint);
          lcd.setCursor(0, 1);
          lcd.print("609 ");
          lcd.print(midSetPoint);
          lcd.setCursor(0, 0);
          lcd.print("601 ");
          lcd.print(highSetPoint);
          if (pumpStatus == 1 && drainPumpStatus == 0) {
            lcd.setCursor(0, 3);
            lcd.print("Fill cycles ");
            lcd.print(drainCycle);
          }
          if (pumpStatus == 0 && drainPumpStatus == 1) {
            lcd.setCursor(0, 3);
            lcd.print("Drain cycle ");
            lcd.print(pumpCycle);
          }
          if (pumpStatus == 0 && drainPumpStatus == 0) {
            lcd.setCursor(0, 3);
            lcd.print("Pumps Idle       ");
          }
          if (debugMode == true) {
            if (numberCells == 1) {
              Serial.print("#1: ");
              Serial.print(grams1);
              Serial.print("grams1Avg ");
              Serial.print(grams1Stats.average());
              Serial.print("load1 ");
              Serial.println(load1);
            }
            if (numberCells == 4) {
              Serial.print(" LowSet : ");
              Serial.print(lowSetPoint);
              Serial.print(" MidSet : ");
              Serial.print(midSetPoint);
              Serial.print(" HighSet : ");
              Serial.print(highSetPoint);
              Serial.print(" RawTotal ");
              Serial.print(loadTotal);
              //         Serial.print(" gramTotal ");
              //         Serial.print(gramsTotal);
              Serial.print(" 610 ");
              Serial.print(FS610pos);
              Serial.print(" 609 ");
              Serial.print(FS609pos);
              Serial.print(" 601 ");
              Serial.println(FS601pos);
              /*   //          Serial.print("H2OAnalog ");
                 //          Serial.print(levelSensorReading);
                 //          Serial.print("H20Level ");
                 //          Serial.println(waterHeight);
                           Serial.print("L1 ");
                           Serial.print(load1);
                           Serial.print(" L2 ");
                           Serial.print(load2);
                           Serial.print(" L3 ");
                           Serial.print(load3);
                           Serial.print(" L4 ");
                           Serial.print(load4);
                           Serial.print(" g1 ");
                           Serial.print(grams1);
                           Serial.print(" g2 ");
                           Serial.print(grams2);
                           Serial.print(" g3 ");
                           Serial.print(grams3);
                           Serial.print(" g4 ");
                           Serial.print(grams4);
                         Serial.print(" FS610R ");
                         Serial.print(FS610Relay);
                         Serial.print(" FS609R ");
                         Serial.print(FS609Relay);
                         Serial.print(" FS601R ");
                         Serial.print(FS601Relay);
              */
            }
          }
          break;
        }
      case 1: {
          lcd.setCursor(0, 0);
          lcd.print(load1);
          lcd.setCursor(10, 0);;
          lcd.print(load2);
          lcd.setCursor(0, 1);
          lcd.print(load3);
          lcd.setCursor(10, 1);
          lcd.print(load4);
          lcd.setCursor(0, 2);
          lcd.print("Sense: ");
          switch (senseMode) {
            case 1:    // Float Switches
              lcd.print("FloatSwitch");
              break;
            case 2:    // Loadcells
              lcd.print("LoadCell");
              break;
            case 3:    // Capacitor Sensors
              lcd.print("Capacitors");
              break;
            case 4:    // Optical Sensors
              lcd.print("Opticals");
              break;
            default:
              lcd.print("Unknown!");
              break;
          }
          if (senseMode == 3){ //Capacitive Sensor position display
          lcd.setCursor(0, 3);
          lcd.print("LL ");
          lcd.print(CS610pos);
          lcd.print(" LH ");
          lcd.print(CS609pos);
          lcd.print(" HH ");
          lcd.print(CS601pos);
          lcd.print("");
          }
          if (senseMode == 1){ //Float switch position display
          lcd.setCursor(0, 3);
          lcd.print("LL ");
          lcd.print(FS610pos);
          lcd.print(" LH ");
          lcd.print(FS609pos);
          lcd.print(" HH ");
          lcd.print(FS601pos);
          lcd.print("");
          }          
          if (debugMode == true) {
            if (numberCells == 4) {
              Serial.print(" LowSet : ");
              Serial.print(lowSetPoint);
              Serial.print(" MidSet : ");
              Serial.print(midSetPoint);
              Serial.print(" HighSet : ");
              Serial.print(highSetPoint);
              Serial.print(" RawTotal ");
              Serial.print(loadTotal);
              Serial.print(" 610 ");
              Serial.print(FS610pos);
              Serial.print(" 609 ");
              Serial.print(FS609pos);
              Serial.print("L1 ");
              Serial.print(load1);
              Serial.print(" L2 ");
              Serial.print(load2);
              Serial.print(" L3 ");
              Serial.print(load3);
              Serial.print(" L4 ");
              Serial.println(load4);
            }
          }
          break;
        }
      case 4: {
          lcd.setCursor(0, 0);
          DateTime now = rtc.now();
          lcdPrintTime();
          lcd.print(' ');
          lcd.print(now.month(), DEC);
          lcd.print('/');
          lcd.print(now.day(), DEC);
          lcd.print('/');
          lcd.print(now.year(), DEC);
          lcd.setCursor(0, 1);
          lcd.print("Logger Timer: ");
          lcd.setCursor(0, 2);
          lcd.print(now.hour() - now.hour(), DEC);
          lcd.print(':');
          lcd.print(now.minute() - now.minute(), DEC);
          lcd.print(':');
          lcd.print(now.second() - now.second(), DEC);
          lcd.print(' ');
          lcd.print(now.day() - now.day(), DEC);
          lcd.print(" days");
          lcd.setCursor(0, 3);
          lcd.print("Sense: ");
          switch (senseMode) {
            case 1:    // Float Switches
              lcd.print("FloatSwitch");
              break;
            case 2:    // Loadcells
              lcd.print("LoadCell");
              break;
            case 3:    // Capacitor Sensors
              lcd.print("Capacitors");
              break;
            case 4:    // Optical Sensors
              lcd.print("Opticals");
              break;
            default:
              lcd.print("Unknown!");
              break;
          }
          break;
        }
      case 5: {
          lcd.setCursor(0, 0);
          lcd.print("zF1");
          lcd.print(zeroFactor1);
          lcd.setCursor(10, 0);
          lcd.print("zF2");
          lcd.print(zeroFactor2);
          lcd.setCursor(0, 1);
          lcd.print("zF3");
          lcd.print(zeroFactor3);
          lcd.setCursor(10, 1);
          lcd.print("zF4");
          lcd.print(zeroFactor4);
          lcd.setCursor(0, 2);
          lcd.print("zFT");
          lcd.print(zeroFactorSum);
          lcd.setCursor(0, 3);
          lcd.print("ZFS / CF ");
          lcd.print(zeroFactorSum / 25);
          lcd.print("Kg");
          break;
        }
      case 2: {
          load1 = scale1.read();
          load2 = scale2.read();
          load3 = scale3.read();
          load4 = scale4.read();
          load1Stats.add(load1);
          load2Stats.add(load2);
          load3Stats.add(load3);
          load4Stats.add(load4);
          lcd.setCursor(0, 0);
          lcd.print("AVG #1 ");
          lcd.print(runLoad[0]);
          lcd.setCursor(0, 1);
          lcd.print("AVG #2 ");
          lcd.print(runLoad[1]);
          lcd.setCursor(0, 2);
          lcd.print("AVG #3 ");
          lcd.print(runLoad[2]);
          lcd.setCursor(0, 3);
          lcd.print("AVG #4 ");
          lcd.print(runLoad[3]);
          zeroFactor1 = load1Stats.average();
          zeroFactor2 = load2Stats.average();
          zeroFactor3 = load3Stats.average();
          zeroFactor4 = load4Stats.average();
          zeroFactorSum = (load1Stats.average() + load2Stats.average() + load3Stats.average() + load4Stats.average());
          Serial.print("AVG Sum : "); //This can be used to remove the need to zeroFactor1 the scale. Useful in permanent scale projects.
          Serial.println(zeroFactorSum);
          break;
        }
      default: displayMode = 4;
    }
    if (loggingMode == true) {
      lcd.setCursor(19, 0);
      lcd.print("L");
    } else {
      lcd.setCursor(19, 0);
      lcd.print(" ");
    }
    if (simulationMode == true) {
      lcd.setCursor(19, 1);
      lcd.print("S");
    } else {
      lcd.setCursor(19, 1);
      lcd.print(" ");
    }
    if (debugMode == true) {
      lcd.setCursor(19, 2);
      lcd.print("D");
    } else {
      lcd.setCursor(19, 2);
      lcd.print(" ");
    }
    lcd.setCursor(19, 3);
    lcd.print(displayMode);
    //lcd.print("/");
    //lcd.print(displayPages);
  }
}
void getReadings() {
  FS610pos = digitalRead(FS610Pin);
  FS609pos = digitalRead(FS609Pin);
  FS601pos = digitalRead(FS601Pin);
  CS610pos = digitalRead(CS610Pin);
  CS609pos = digitalRead(CS609Pin);
  CS601pos = digitalRead(CS601Pin);
  if (numberCells == 1) {
    load1 = scale1.read();
    grams1 = 100 * ((zeroFactor1 - load1) / calibrationFactor);
    load1Stats.add(load1);
    grams1Stats.add(grams1);
  }
  if (numberCells == 4) {
    load1 = scale1.read();
    load2 = scale2.read();
    load3 = scale3.read();
    load4 = scale4.read();
    runAverage();   // Added to perform running average logging
    loadTotal = (load1 + load2 + load3 + load4);
    loadTotals.add(loadTotal);
    loadTotalAVG = loadTotals.average();
    load1Stats.add(load1);
    load2Stats.add(load2);
    load3Stats.add(load3);
    load4Stats.add(load4);
    grams1 = (load1 - zeroFactor1) / calibrationFactor;
    grams2 = (load2 - zeroFactor2) / calibrationFactor;
    grams3 = (load3 - zeroFactor3) / calibrationFactor;
    grams4 = (load4 - zeroFactor4) / calibrationFactor;
    gramsTotal = (grams1 + grams2 + grams3 + grams4);
    grams1Stats.add(grams1);
    grams2Stats.add(grams2);
    grams3Stats.add(grams3);
    grams4Stats.add(grams4);
    gramsAvgSum = (grams1Stats.average() + grams2Stats.average() + grams3Stats.average() + grams4Stats.average());
    gramsMaxSum = (grams1Stats.maximum() + grams2Stats.maximum() + grams3Stats.maximum() + grams4Stats.maximum());
    gramsMinSum = (grams1Stats.minimum() + grams2Stats.minimum() + grams3Stats.minimum() + grams4Stats.minimum());
    /*
      if (grams1 < 0) grams1 = 0;
      if (grams2 < 0) grams2 = 0;
      if (grams3 < 0) grams3 = 0;
      if (grams4 < 0) grams4 = 0;
    */
  }
}
void runAverage() {       //Added for running average calculation
  loadArray[0] = load1;   //Copy the most recent load cell readings into an array for easy handling
  loadArray[1] = load2;
  loadArray[2] = load3;
  loadArray[3] = load4;

  for (int loadNum = 0; loadNum < 4; loadNum++) {
    total[loadNum] -= readings[loadNum][index];       //Subtract the last reading
    readings[loadNum][index] = loadArray[loadNum];                 //Place the current reading in the array
    total[loadNum] += readings[loadNum][index];       //Add the reading to the total
    runLoad[loadNum] = total[loadNum] / NUMREADINGS;  //Calculate the running average for the given load cell
  }
  
  index = (index + 1);        //Advance to the next index
  if (index >= NUMREADINGS)    //if we're at the end of the array
  index = 0;                    //...wrap around to the beginning
}
void clearAverage() {   //Clear the running average arrays when logging is stopped
  for (int loadNum = 0; loadNum < 4; loadNum++) {
    total[loadNum] = 0;   //Clear total value
  }
  memset(readings,0,4*NUMREADINGS*sizeof(readings[0][0]));  //Reset the running average array to all zero
  index = 0;    //Reset index
}
void updateClock() {
  if (Ethernet.begin(mac) == 0) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println("RTC was updated to compile time");
  } else {
    setSyncProvider(getNtpTime);
    rtc.adjust(getNtpTime());
    Serial.println("RTC was updated to internet time");
  }
}
void fill() {
  if (drainMode == true && pumpMode == false) pumpCycle++;
  drainMode = false;
  pumpMode = true;
  digitalWrite(pump, LOW );
  pumpStatus = 1;
  digitalWrite(drainPump, HIGH );
  drainPumpStatus = 0;
}
void drain() {
  if (drainMode == false && pumpMode == true) drainCycle++;
  drainMode = true;
  pumpMode = false;
  digitalWrite(pump, HIGH );
  pumpStatus = 0;
  digitalWrite(drainPump, LOW );
  drainPumpStatus = 1;
}
void idle () {
  if (drainMode == true || pumpMode == true ) idleCycle++;
  drainMode = false;
  pumpMode = false;
  digitalWrite(pump, HIGH );
  pumpStatus = 0;
  digitalWrite(drainPump, HIGH );
  drainPumpStatus = 0;
}
void printIPAddress() {
  Serial.print("My IP address: ");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    Serial.print(".");
  }
  Serial.println();
}
void ethernetMaintain() {
  switch (Ethernet.maintain())  {
    case 1: //renewed fail
      Serial.println("Error: renewed fail");
      break;

    case 2: //renewed success
      Serial.println("Renewed success");
      printIPAddress(); //print your local IP address:
      break;

    case 3: //rebind fail
      Serial.println("Error: rebind fail");
      break;
    case 4: //rebind success
      Serial.println("Rebind success");
      printIPAddress(); //print your local IP address:
      break;
    default:
      //nothing happened
      break;
  }
}
void syncTime() {
  /*
    memset(packetBuffer, 0, NTP_PACKET_SIZE);  // set all bytes in the buffer to 0
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12]  = 49;
    packetBuffer[13]  = 0x4E;
    packetBuffer[14]  = 49;
    packetBuffer[15]  = 52;
    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    Udp.beginPacket((132, 163, 4, 101), 123); //NTP requests are to port 123
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();

    Udp.read(packetBuffer, NTP_PACKET_SIZE);
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);  // timestamp starts at byte 40 of packet. It is 2 words (4 bytes) long
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);   // Extract each word and...
    unsigned long secsSince1900 = highWord << 16 | lowWord;             // ... combine into long: NTP time (seconds since Jan 1 1900):
    const unsigned long seventyYears = 2208988800UL;                    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    unsigned long epoch = secsSince1900 - seventyYears;                 // subtract seventy years to get to 1 Jan. 1900:
    DateTime gt(epoch - (1000 * 60 * 60 * 24) - 1635); //Unix Time 1497494460000 = 1497494460000/1000/60/60/24+DATE(1970,1,1) = Thursday, June 15, 2017
    rtc.adjust(gt);   // obtain date & time based on NTP-derived epoch...
    Serial.println("RTC time has been set to NTP time");*/
}
void ntpTime() {  // digital clock display of the time
  Serial.print(" at ");
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" / ");
  Serial.print(day());
  Serial.print(" / ");
  Serial.println(year());;
}
void menuButton() {  // Interrupt Service Routine. DO NOT CALL UPON FUNCTIONS!
  if ((millis() - lastDebounceTime) > debounceDelay) {
    menuMode = true;
    if (debugMode == 1) {
      Serial.println("Button pressed");
    }
  }
  lastDebounceTime = millis();
}
void isr () { // Interrupt Service Routine. DO NOT CALL UPON FUNCTIONS!
  static boolean ready;
  static unsigned long lastFiredTime;
  static byte pinA, pinB;
  // wait for main program to process it
  if (fired)
    return;
  byte newPinA = digitalRead (18);
  byte newPinB = digitalRead (19);
  // Forward is: LH/HH or HL/LL
  // Reverse is: HL/HH or LH/LL
  // so we only record a turn on both the same (HH or LL)
  if (newPinA == 1 || newPinB == 1) {
    if (ready) {
      byte increment = 1;
      // if they turn the encoder faster, make the count go up more
      unsigned long now = millis ();
      unsigned long interval = now - lastFiredTime;
      lastFiredTime = now;
      if (interval <= 4)
        increment = 5;
      else if (interval <= 10)
        increment = 3;
      else if (interval <= 20)
        increment = 2;
      if (newPinA == HIGH) { // must be HH now
        if (pinA == LOW) {

          rotaryCount += increment;
          rotateCW = false;
          rotateCC = true;
        }
        else {
          rotaryCount -= increment;
          rotateCW = true;
          rotateCC = false;
        }
      }
      else { // must be LL now
        if (pinA == LOW) {
          rotaryCount -= increment;
          rotateCW = true;
          rotateCC = false;
        }
        else {
          rotaryCount += increment;
          rotateCW = false;
          rotateCC = true;
        }
      }
      fired = true;
      ready = false;
      if (debugMode == true) {
        Serial.print ("Count = ");
        Serial.print (rotaryCount);
        Serial.print (" rotateCW: ");
        Serial.print (rotateCW);
        Serial.print (" rotateCC: ");
        Serial.print (rotateCC);
        Serial.print(" displayMode = ");
        Serial.println(displayMode);
      }
    }  // end of being ready
  }  // end of completed click
  else
    ready = true;
  pinA = newPinA;
  pinB = newPinB;
}
void readLog() {
  getfileDate(fileDate);
  fileName = fileDate;
  File Log = SD.open("/Logs/" + fileName);
  if (Log) { // if the file is available, open it
    while (Log.available()) {
      Serial.write(Log.read());
    }
    Log.close();
  }
  else { // if the file isn't open, print an error:
    Serial.print("Error reading ");
    Serial.print(fileName);
    Serial.println(", file does not exist.");
  }
}
void zeroScale() {
  while (goodEnough <= 40) {
    load1 = scale1.read();
    load2 = scale2.read();
    load3 = scale3.read();
    load4 = scale4.read();
    load1Stats.add(load1);
    load2Stats.add(load2);
    load3Stats.add(load3);
    load4Stats.add(load4);
    if (numberCells == 1) {
      lcd.setCursor(0, 0);
      lcd.print("AVG #1 ");
      lcd.print(zeroFactorSum);
      zeroFactorSum = load1Stats.average();
      goodEnough ++;
    } else {
      lcd.setCursor(0, 0);
      lcd.print("AVG #1 ");
      lcd.print(zeroFactor1);
      lcd.setCursor(0, 1);
      lcd.print("AVG #2 ");
      lcd.print(zeroFactor2);
      lcd.setCursor(0, 2);
      lcd.print("AVG #3 ");
      lcd.print(zeroFactor3);
      lcd.setCursor(0, 3);
      lcd.print("AVG #4 ");
      lcd.print(zeroFactor4);
      zeroFactor1 = load1Stats.average();
      zeroFactor2 = load2Stats.average();
      zeroFactor3 = load3Stats.average();
      zeroFactor4 = load4Stats.average();
      zeroFactorSum = (load1Stats.average() + load2Stats.average() + load3Stats.average() + load4Stats.average());
      goodEnough ++;
    }
  }
  Serial.print("Zero factor Sum : "); //This can be used to remove the need to zeroFactor1 the scale. Useful in permanent scale projects.
  Serial.println(zeroFactorSum);
  unsigned long currentMillis = millis() + 2000;
  //lcd.clear();
  while (millis() <= currentMillis) {
    lcd.setCursor(0, 2);
    lcd.print("Average Zero Factor");
    lcd.setCursor(0, 3);
    lcd.print(zeroFactorSum);
  }
  clearAll();
}
void clearAll() {
  load1Stats.clear();
  load2Stats.clear();
  load3Stats.clear();
  load4Stats.clear();
  grams1Stats.clear();
  grams2Stats.clear();
  grams3Stats.clear();
  grams4Stats.clear();
  lowLevel.clear();
  midLevel.clear();
  highLevel.clear();
  lcd.clear();
  goodEnough = 1;
  lowSetPoint =  2364000;
  midSetPoint =  2500000; //2582000
  highSetPoint = 2582000; //2630000
  Serial.println("Cleared stats, statics, and LCD.");
}
void toggleSenseMode() {
  if (senseMode == 4) senseMode = 0;
  senseMode ++;
  Serial.print("Sense Mode is ");
  switch (senseMode) {
    case 1:    // Float Switches
      Serial.println("Float Switche Mode");
      break;
    case 2:    // Loadcells
      Serial.println("LoadCell Mode");
      break;
    case 3:    // Capacitor Sensors
      Serial.println("Capacitor Sensor Mode");
      break;
    case 4:    // Optical Sensors
      Serial.println("Optical Sensor Mode");
      break;
    default:
      Serial.println("Unknown Sense Mode selected!");
      break;
  }
}
void toggleLogging() {
  loggingMode = !loggingMode;
  if (loggingMode == true) {
    EEPROM.write(1, clearMemory);
    EEPROM.write(0, loggingMode);     // Write the value of loggingMode
    EEPROM.write(1, eepromValid);  // Write the signature for confirmation of successful memory write
    getfileDate(fileDate);
    fileName = fileDate;
    DateTime loggingTime = rtc.now();
    loggingTimer = (loggingTime.unixtime());
    File Log = SD.open("/Logs/" + fileName, FILE_WRITE); // open the file. note that only one file can be open at a time,
    if (Log) {//FAT32 files name can only be 8 characters max + extension
      Log.print("#1 Raw");
      Log.print(",");
      Log.print("#2 Raw");
      Log.print(",");
      Log.print("#3 Raw");
      Log.print(",");
      Log.print("#4 Raw");
      Log.print(",");
      Log.print("Raw Total");
      Log.print(",");
      
      // Added to record running average values
      Log.print("#1 RunAvg");
      Log.print(",");
      Log.print("#2 RunAvg");
      Log.print(",");
      Log.print("#3 RunAvg");
      Log.print(",");
      Log.print("#4 RunAvg");
      Log.print(",");
      
      Log.print("FS610Pos");
      Log.print(",");
      Log.print("FS609Pos");
      Log.print(",");
      Log.print("FS601Pos");
      Log.print(",");
      Log.print("CS610Pos");
      Log.print(",");
      Log.print("CS609Pos");
      Log.print(",");
      Log.print("CS601Pos");
      Log.print(",");
      Log.print("pumpStatus");
      Log.print(",");
      Log.print("drainPumpStatus");
      Log.print(",");
      Log.print("drainCycles");
      Log.print(",");
      Log.print("pumpCycles");
      Log.print(",");
      Log.print("Low Set Point");
      Log.print(", ");
      Log.print("High Set Point");
      Log.print(", ");
      Log.print("Delta");
      Log.print(", ");
      Log.print("Sense Mode");
      Log.print(", ");
      Log.print("Time");
      Log.print(",");
      Log.print("Date: ");
      DateTime now = rtc.now();
      Log.print(' ');
      Log.print(now.month(), DEC);
      Log.print('/');
      Log.print(now.day(), DEC);
      Log.print('/');
      Log.print(now.year(), DEC);
      Log.println(",");
      Log.close();
      Serial.print("Logging on recording data to /Logs/");
      Serial.println(fileName);
    }
    if (debugMode == true) {
      Serial.println("Error writing data headers to the log file");
    }
  }
  if (loggingMode == false) {
    EEPROM.write(0, loggingMode);   // Write the value of loggingMode
    EEPROM.write(1, eepromValid);   // Write the signature for confirmation of successful memory write
    clearAverage();                 // Clear all running average sub-arrays to zero out running average
    Serial.println("Logging off");
    lcd.clear();
  }
}
void toggleDebugMode() {
  debugMode = !debugMode;
  if (debugMode == false) {
    EEPROM.write(3, clearMemory);  // clear out the signature. That way we know if we didn't finish the write successfully.
    EEPROM.write(2, debugMode);  // Write the value of the variable to EEPROM memory
    EEPROM.write(3, eepromValid);  // all good. Write the signature so we'll know it's all good.
    Serial.println("DebugMode off");
    lcd.clear();
  }
  if (debugMode == true) {
    EEPROM.write(3, clearMemory);  // clear out the signature. That way we know if we didn't finish the write successfully.
    EEPROM.write(2, debugMode);  // Write the value of the variable to EEPROM memory
    EEPROM.write(3, eepromValid);  // all good. Write the signature so we'll know it's all good.
    Serial.println("DebugMode on");
  }
}
void cycleDisplayMode() {
  displayMode ++;
  if (displayMode == (displayPages + 1)) displayMode = 1;
  EEPROM.write(6, displayMode);  // Write the value of the variable to EEPROM memory
  EEPROM.write(7, eepromValid);  // all good. Write the signature so we'll know it's all good.
  lcd.clear();
}
void cycleDisplayModeMinus() {
  displayMode--;
  if (displayMode == (displayPages + 1)) displayMode = 1;
  EEPROM.write(6, displayMode);  // Write the value of the variable to EEPROM memory
  EEPROM.write(7, eepromValid);  // all good. Write the signature so we'll know it's all good.
  lcd.clear();
}
void toggleSimMode() {
  simulationMode = ! simulationMode;
  if (simulationMode == false) {
    digitalWrite(FS610Relay, HIGH);
    digitalWrite(FS609Relay, HIGH);
    digitalWrite(FS601Relay, HIGH);
    digitalWrite(simRelay, LOW);
    EEPROM.write(5, clearMemory);  // clear out the signature. That way we know if we didn't finish the write successfully.
    EEPROM.write(4, simulationMode);  // Write the value of the variable to EEPROM memmory
    EEPROM.write(5, eepromValid);  // all good. Write the signature so we'll know it's all good.
    idle(); //idle pumps
    Serial.println("Simulation Mode Off");
  }
  if (simulationMode == true) {
    EEPROM.write(5, clearMemory);  // clear out the signature. That way we know if we didn't finish the write successfully.
    EEPROM.write(4, simulationMode);  // Write the value of the variable to EEPROM memmory
    EEPROM.write(5, eepromValid);  // all good. Write the signature so we'll know it's all good.
    Serial.println("Simulation Mode On");
  }
}
void autoCalibrate() {
  if (Serial.available()) ch = Serial.read();
  recieveCommands();
  getReadings();
  updateDisplay();
  drain();
  if (FS610pos == HIGH && FS609pos == HIGH) {
    if (simulationMode == true) fill();
  }
  if (FS610pos == LOW && FS609pos == LOW) {
    midLevel.add(loadTotal);
    midSetPoint = midLevel.average();
    if (simulationMode == true) drain();
  }
  if (FS610pos == HIGH && FS609pos == LOW) {
    //Serial.println("Either your float switches are plugged in backwards or your bottom level switch is unplugged, jammed, or wired incorrectly.");
    idle();
  }
  if (FS610pos == LOW && FS609pos == HIGH) {
    if (simulationMode == false) idle();
  }
}
/*
  while (goodEnough <= 50) {
  if (Serial.available()) ch = Serial.read();
  getReadings();
  if (numberCells == 1) {
    lcd.setCursor(0, 0);
    lcd.print("avg ");
    if (grams1 > 1000) {
      lcd.print(grams1Stats.average() / 1000);
      lcd.print("Kg");
    } else {
      lcd.print(grams1Stats.average());
      lcd.print("g");
    }
    lcd.setCursor(0, 1);
    lcd.print("grams1 ");
    if (grams1 > 1000) {
      lcd.print(grams1);
      lcd.print("Kg");
    } else {
      lcd.print(grams1);
      lcd.print("g");
    }
    lcd.setCursor(0, 3);
    lcd.print("Calbra  ");
    lcd.print(calibrationFactor);
    if (grams1Stats.average() > 1100) {
      calibrationFactor -= 5;
      goodEnough = 1;
    }
    if (grams1Stats.average() < 900) {
      calibrationFactor += 5;
      goodEnough = 1;
    }
    if (grams1Stats.average() < 1100 && grams1Stats.average() > 1010 ) {
      calibrationFactor -= 1;
      goodEnough = 1;
    }
    if (grams1Stats.average() > 900 && grams1Stats.average() < 990 ) {
      calibrationFactor += 1;
      goodEnough = 1;
    }
    if (grams1Stats.average() < 1010 && grams1Stats.average() > 1002 ) {
      calibrationFactor -= .01;
      goodEnough = 1;
    }
    if (grams1Stats.average() > 990 && grams1Stats.average() < 998 ) {
      calibrationFactor += .01;
      goodEnough = 1;
    }
    if (grams1Stats.average() <= 1002 && grams1Stats.average() >= 998) {
      goodEnough = goodEnough + 1;
      Serial.print(".");
    }
  }
  if (numberCells == 4) {
    lcd.setCursor(0, 0);
    lcd.print("avg ");
    if (gramsAvgSum > 1000) {
      lcd.print(gramsAvgSum);
      lcd.print("Kg");
    } else {
      lcd.print(gramsAvgSum);
      lcd.print("g");
    }
    lcd.setCursor(0, 1);
    lcd.print("gramsTotal");
    if (grams1 > 1000) {
      lcd.print(gramsTotal);
      lcd.print("Kg");
    } else {
      lcd.print(gramsTotal);
      lcd.print("g");
    }
    lcd.setCursor(0, 3);
    lcd.print("Calbra  ");
    lcd.print(calibrationFactor);
    if (gramsAvgSum > 1100) {
      calibrationFactor -= 5;
      goodEnough = 1;
    }
    if (gramsAvgSum < 900) {
      calibrationFactor += 5;
      goodEnough = 1;
    }
    if (gramsAvgSum < 1100 && gramsAvgSum > 1010 ) {
      calibrationFactor -= 1;
      goodEnough = 1;
    }
    if (gramsAvgSum > 900 && gramsAvgSum < 990 ) {
      calibrationFactor += 1;
      goodEnough = 1;
    }
    if (gramsAvgSum < 1010 && gramsAvgSum > 1002 ) {
      calibrationFactor -= .01;
      goodEnough = 1;
    }
    if (gramsAvgSum > 990 && gramsAvgSum < 998 ) {
      calibrationFactor += .01;
      goodEnough = 1;
    }
    if (gramsAvgSum <= 1002 && gramsAvgSum >= 998) {
      goodEnough = goodEnough + 1;
      Serial.print(".");
    }
  }
  }
  Serial.print("Calibration Factor set to ");
  Serial.println(calibrationFactor);
  goodEnough = 1;
  }*/
void writeSD() {
  getfileDate(fileDate);
  fileName = fileDate;
  File Log = SD.open("/Logs/" + fileName, FILE_WRITE); // open the file. note that only one file can be open at a time
  if (Log) {
    Log.print(load1);
    Log.print(", "); // ", " is the delimited character for data columns
    Log.print(load2);
    Log.print(", ");
    Log.print(load3);
    Log.print(", ");
    Log.print(load4);
    Log.print(", ");
    Log.print(loadTotal);
    Log.print(", ");

    // Added to record running average values
    Log.print(runLoad[0]);
    Log.print(", ");
    Log.print(runLoad[1]);
    Log.print(", ");
    Log.print(runLoad[2]);
    Log.print(", ");
    Log.print(runLoad[3]);
    Log.print(", ");

    Log.print(FS610pos);
    Log.print(", ");
    Log.print(FS609pos);
    Log.print(", ");
    Log.print(FS601pos);
    Log.print(", ");
    Log.print(CS610pos);
    Log.print(", ");
    Log.print(CS609pos);
    Log.print(", ");
    Log.print(CS601pos);
    Log.print(", ");
    Log.print(pumpStatus);
    Log.print(",");
    Log.print(drainPumpStatus);
    Log.print(",");
    Log.print(drainCycle);
    Log.print(",");
    Log.print(pumpCycle);
    Log.print(",");
    if (lowSetPoint != prevLowSetPoint || midSetPoint != prevMidSetPoint) {
      zeroFactorSum = lowSetPoint;
      Log.print(lowSetPoint);
      Log.print(", ");
      /*Log.print(midSetPoint);
        Log.print(", ");
        Log.print(midSetPoint - lowSetPoint);
        Log.print(", ");
      */
      Log.print(highSetPoint);
      Log.print(", ");
      Log.print(highSetPoint - lowSetPoint);
      Log.print(", ");
    } else {
      Log.print(", ");
      Log.print(", ");
      Log.print(", ");
    }
    prevLowSetPoint = lowSetPoint;
    prevMidSetPoint = midSetPoint;
    prevMidSetPoint = highSetPoint;
    switch (senseMode) {
      case 1:    // Loadcells
        Log.print("LC");
        break;
      case 2:    // Float Switches
        Log.print("FS");
        break;
      case 3:    // Capacitor Sensors
        Log.print("CS");
        break;
      case 4:    // Optical Sensors
        Log.print("OS");
        break;
    }
    Log.print(", ");
    DateTime now = rtc.now();
    Log.print(now.hour(), DEC);
    Log.print(':');
    Log.print(now.minute(), DEC);
    Log.print(':');
    Log.println(now.second(), DEC);
    Log.close(); // close the file:
  } else {
    Serial.println("There is still an Error logging data to the log file");
    toggleLogging();
  }
}
void printDigits(int digits) {  // utility for digital clock display: prints preceding colomn and leading 0
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
void lcdPrintDigits(int digits) {  // utility for digital clock display: prints preceding colomn and leading 0
  if (digits < 10)
    lcd.print('0');
  lcd.print(digits);
}
void printTime() {
  DateTime now = rtc.now();
  Serial.print(now.hour(), DEC);
  printDigits(now.minute());
  printDigits(now.second());
  Serial.print(" ");
}
void lcdPrintTime() {
  DateTime now = rtc.now();
  lcdPrintDigits(now.hour());
  lcd.print(":");
  lcdPrintDigits(now.minute());
  lcd.print(":");
  lcdPrintDigits(now.second());
}
void printDate() {
  DateTime now = rtc.now();
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.print(" on ");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(" ");
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print('/');
  Serial.println(now.year(), DEC);
}
void getfileDir(char *fileDir) {
  DateTime now = rtc.now();
  int year = now.year();
  int month = now.month();
  int day = now.day();
  fileDate[0] = month / 10 + '0';
  fileDate[1] = month % 10 + '0';
  fileDate[2] = '-';
  fileDate[3] = day / 10 + '0';
  fileDate[4] = day % 10 + '0';
  fileDate[5] = '-';
  fileDate[6] = '1';
  fileDate[7] = '7';
  return;
}
void getfileDate(char *fileDate) {
  DateTime now = rtc.now();
  int year = now.year();
  int month = now.month();
  int day = now.day();
  fileDate[0] = month / 10 + '0';
  fileDate[1] = month % 10 + '0';
  fileDate[2] = '-';
  fileDate[3] = day / 10 + '0';
  fileDate[4] = day % 10 + '0';
  fileDate[5] = '-';
  fileDate[6] = '1';
  fileDate[7] = '7';
  fileTime[8] = '.';
  fileTime[9] = 'C';
  fileTime[10] = 'S';
  fileTime[11] = 'V';
  return;
}
void getfileTime(char *fileTime) {
  DateTime now = rtc.now();
  int hour = now.hour();
  int minute = now.minute();
  int second = now.second();
  fileTime[0] = hour / 10 + '0';
  fileTime[1] = hour % 10 + '0';
  fileTime[2] = '.';
  fileTime[3] = minute / 10 + '0';
  fileTime[4] = minute % 10 + '0';
  fileTime[5] = '.';
  fileTime[6] = second / 10 + '0';
  fileTime[7] = second / 10 + '0';
  fileTime[8] = '.';
  fileTime[9] = 'C';
  fileTime[10] = 'S';
  fileTime[11] = 'V';
  return;
}
