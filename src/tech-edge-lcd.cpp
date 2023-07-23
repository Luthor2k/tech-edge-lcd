//  Vacuum Fluorescent Display for use with Tech Edge wideband 2Y1 controller
//  Dec 2017 Arthur Hazleden

//  https://github.com/Luthor2k/tech-edge-lcd

#include <Arduino.h>
#include <sensor-tables.h>
#include <RTClib.h>
#include <SerLCD.h> //Click here to get the library: http://librarymanager/All#SparkFun_SerLCD
#include <HardwareSerial.h>
#include <SD.h>
#include <SPI.h>

//Pins:
static const uint8_t SETTING_LDRPIN = A0; //LDR input pin, set to wherever connected
static const uint8_t SETTING_SD_CARD_CS = 15; //chip select pin 
static const uint8_t SETTING_STATUS_LED = 13; //chip select pin 

//settings for engine config:
const uint8_t SETTING_ENGINE_PULSES_PER_REV = 1;
const float SETTING_STOIC_AFR = 14.5; //AFR of 14.5 is correct for a diesel, 14.7 for gas

const uint8_t SETTING_LCD_CONTRAST = 50;
int LDRreading;     //light dependent resistor for display brightness, raw values are about 700 when dark, under 20 when bright
int LDRState = 1;  // will be set to one of four brightness levels, start out at 1



//program function timing
unsigned long prevDisplayUpdateTime = 0;
const uint64_t SETTING_DISPLAY_UPDATE_INTERVAL_MS = 1000;

unsigned long prevBrightnessUpdateTime = 0;
const uint64_t SETTING_BRIGHTNESS_UPDATE_INTERVAL_MS = 5000; //set this to how often to update the brightness in milliseconds

unsigned long DAQserialFrameStartTime = 0;
const uint64_t SETTING_DAQ_SERIAL_FRAME_WAIT_MS = 50; //approx time for the serial frame to definetly complete

//Serial stream from controller
const uint8_t DATA_FRAME_frameheader1 = 0x5a;
const uint8_t DATA_FRAME_frameheader2 = 0xa5;
const uint8_t DATA_FRAME_RX_LENGTH_BYTES = 28;   //there are 28 bytes in a correctly formatted frame
byte incomingSerial[20];
uint8_t DAQbytesAvailable = 0;
bool DAQsetToRead = false;

// Raw data coming in from controller
uint8_t frameheader1 = 0x5A;
uint8_t frameheader2 = 0xA5;

uint16_t sequence_counter;

uint16_t parsingLoopCounter;

uint16_t DAQRawLambda16;

uint16_t DAQRawUser1;
uint16_t DAQRawUser2;
uint16_t DAQRawUser3;

uint16_t DAQRawThermocouple1;
uint16_t DAQRawThermocouple2;
uint16_t DAQRawThermocouple3;

uint16_t DAQRawOnboardThermistor;

uint16_t DAQRawRPMCount;

uint8_t DAQRawSensorState;
uint8_t DAQRawHeaterState;

//status messages
/*
String = "Sensor Missing";
String status_sensor_cold = "Sensor Cold";
String status_sensor_warm = "Sensor Warm";

String status_heater_normal = "Heater Normal";
String status_heater_lowb = "Low Battery";
String status_heater_highb = "High Battery";
String status_heater_short = "Heater Short";
String status_heater_open = "Heater Open";

String status_lean = "LEAN";
*/

// Corrected data values:
float lambda;
float AFR;

float intakePressurePSI;
//float oilPressurePSI;
float intakeTemperatureC;
float throttlePositionPercent;

uint16_t exhuastTemperatureC;
uint16_t oilTemperatureC;
uint16_t turboTemperatureC;

float DAQTemperatureC;

uint16_t engineRPM;

uint16_t WbStatus;

uint16_t crcSum;
uint8_t crcComp;

bool crcGood;

SerLCD lcd; // Initialize the library with default I2C address 0x72

RTC_DS3231 rtc;

HardwareSerial PCSerialPort(0);
HardwareSerial DAQSerialPort(1);  //UART1, connected to the rs232 converter and then to the DAQ

//function declarations
void check_serial();
void update_averages();
void update_display();
void update_brightness();
void remap_raw_values();

void setup() {
  //initialize ports
  PCSerialPort.begin(115200);
  Wire.begin();
  pinMode(16, INPUT);
  DAQSerialPort.begin(19200, SERIAL_8N1, 16, 17);
  pinMode(SETTING_STATUS_LED, OUTPUT);

  //lcd setup
  lcd.begin(Wire);
  delay(20);
  Wire.setClock(400000); //Optional - set I2C SCL to High Speed Mode of 400kHz
  //lcd.setBacklight(255, 255, 255); //bright white
  lcd.setContrast(SETTING_LCD_CONTRAST); //Set contrast. Lower to 0 for higher contrast.
  lcd.clear();
  /*
  lcd.setCursor(0, 0);
  lcd.print("tech-edge-esp32-test");
  lcd.setCursor(0, 2);
  lcd.print("compiled ");
  lcd.print(String(__DATE__));
  */
  
  //SD card setup
  if(!SD.begin(SETTING_SD_CARD_CS)){
      PCSerialPort.println("Card Mount Failed");
      return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
      PCSerialPort.println("No SD card attached");
      return;
  }

  PCSerialPort.print("SD Card Type: ");
  if(cardType == CARD_MMC){
      PCSerialPort.println("MMC");
  } else if(cardType == CARD_SD){
      PCSerialPort.println("SDSC");
  } else if(cardType == CARD_SDHC){
      PCSerialPort.println("SDHC");
  } else {
      PCSerialPort.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  PCSerialPort.printf("SD Card Size: %lluMB\n", cardSize);

  //rtc setup
  if (! rtc.begin()) {
    PCSerialPort.println("Couldn't find RTC");
    PCSerialPort.flush();
    while (1) delay(10);
  }

  if (rtc.lostPower()) {
    PCSerialPort.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

  


  /*
  DateTime now = rtc.now();
  PCSerialPort.print(now.year(), DEC);
  PCSerialPort.print('/');
  PCSerialPort.print(now.month(), DEC);
  PCSerialPort.print('/');
  PCSerialPort.print(now.day(), DEC);
  PCSerialPort.print(" ");
  PCSerialPort.print(now.hour(), DEC);
  PCSerialPort.print(':');
  PCSerialPort.print(now.minute(), DEC);
  PCSerialPort.print(':');
  PCSerialPort.print(now.second(), DEC);
  PCSerialPort.println();
  */
}

void loop() {
  unsigned long currentMillis = millis();

  //DAQ sending 28chars (1 frame) at 10Hz, 19200 baud
  //(1/19200) * 28 = 1.458333ms per frame **seems to take longer than 2ms.. wait 20 instead
  //1 frame sent every 100ms; 100 - 1.45833 = 0.0985416s = 98ms seconds between frames
  if (DAQSerialPort.available() && DAQsetToRead == false) {
    DAQserialFrameStartTime = currentMillis;
    DAQbytesAvailable = DAQSerialPort.available();
    PCSerialPort.println(DAQbytesAvailable);
    DAQsetToRead = true;
  }

  if (currentMillis - DAQserialFrameStartTime >= SETTING_DAQ_SERIAL_FRAME_WAIT_MS && DAQsetToRead == true) {
    DAQbytesAvailable = DAQSerialPort.available();
    PCSerialPort.println(DAQbytesAvailable);
    DAQSerialPort.readBytes(incomingSerial, DAQbytesAvailable);
    int totalBytes = DAQbytesAvailable;
    while (DAQbytesAvailable > 0) {
      PCSerialPort.print(incomingSerial[totalBytes - DAQbytesAvailable], HEX);
      incomingSerial[totalBytes - DAQbytesAvailable] = 0;
      DAQbytesAvailable--;
    }
    PCSerialPort.print("\n");
    DAQsetToRead = false;
  }
  

  if (crcGood == true){
    PCSerialPort.println("serial frame sync");
    //update_averages();
    //remap_raw_values();
    
  }


  if (currentMillis - prevDisplayUpdateTime >= SETTING_DISPLAY_UPDATE_INTERVAL_MS) {
    prevDisplayUpdateTime = currentMillis;
    //update_display();
  }
  
  if (currentMillis - prevBrightnessUpdateTime >= SETTING_BRIGHTNESS_UPDATE_INTERVAL_MS) {
    prevBrightnessUpdateTime = currentMillis;
    //update_brightness();
  }
  
}

void check_serial() {
  crcGood = false;
  digitalWrite(SETTING_STATUS_LED, HIGH);
  delay(100);
  /*
  //delay(SETTING_SERIAL_FRAME_RX_DELAY_MS);  //pause for the time of 1 frame to complete
  DAQSerialPort.readBytesUntil(DATA_FRAME_frameheader1, incomingSerial, DATA_FRAME_RX_LENGTH_BYTES);
  Serial.println(F(incomingSerial));
  //this ^ would mean the frame is syncd from the back end not the front, can this be used as the index instead?
  DAQSerialPort.readBytesUntil(DATA_FRAME_frameheader1, incomingSerial, DATA_FRAME_RX_LENGTH_BYTES);
  Serial.println(F(incomingSerial));
  */
  //read until the end of the likely partial frame
  int bytesRead = DAQSerialPort.readBytes(incomingSerial, DATA_FRAME_RX_LENGTH_BYTES);
  //then read until the end of the known good frame
  //DAQSerialPort.readBytesUntil(DATA_FRAME_frameheader1, incomingSerial, DATA_FRAME_RX_LENGTH_BYTES);
  PCSerialPort.print("bytesRead: "+bytesRead);

  for (byte headerSearchIndex = 0; headerSearchIndex < DATA_FRAME_RX_LENGTH_BYTES; headerSearchIndex++) {
    PCSerialPort.print(incomingSerial[headerSearchIndex], HEX);    
  }
  PCSerialPort.println(" END");

  //find the start of frame header, a two byte code;   1   - Frame Header byte 1 (0x5A), 2   - Frame Header byte 2 (0xA5)
  bool headerSearchFirstByteFound = false;
  for (byte headerSearchIndex = 0; headerSearchIndex < DATA_FRAME_RX_LENGTH_BYTES; headerSearchIndex++) {

    PCSerialPort.print(headerSearchIndex, DEC);
    PCSerialPort.print(":");
    PCSerialPort.println(incomingSerial[headerSearchIndex], HEX);

    if (incomingSerial[headerSearchIndex] == DATA_FRAME_frameheader1) {

      PCSerialPort.println("found first header frame");

      if (incomingSerial[headerSearchIndex + 1] == DATA_FRAME_frameheader2) {

        PCSerialPort.println("found second header frame");
        headerSearchFirstByteFound = true;
        
      }

    }
  }

  digitalWrite(SETTING_STATUS_LED, LOW);
  /*
  if (incomingSerial[0] == byte(DATA_FRAME_frameheader2))    //should be the second of the 0x5A 0xA5 frame header
  {
    // CRC calculation
    crcSum = 0;
    for (uint8_t frameByteIndex = 0; frameByteIndex < 27; frameByteIndex++){
      crcSum += incomingSerial[frameByteIndex];
      crcComp = lowByte(crcSum);  //should ALWAYS be 0xFF. well, should always be 0xA5 because of how the frame sync is being done..
    }
    if (crcComp == DATA_FRAME_frameheader2){
      crcGood = true;

      /// Break apart the frame: ///
      sequence_counter = incomingSerial[1];
  
      DAQRawLambda16 = (int(incomingSerial[4]) * 256) + int(incomingSerial[5]);
  
      DAQRawUser1 = (int(incomingSerial[8]) * 256) + int(incomingSerial[9]);      //boost pressure
      DAQRawUser2 = (int(incomingSerial[10]) * 256) + int(incomingSerial[11]);    //throttle position
      DAQRawUser3 = (int(incomingSerial[12]) * 256) + int(incomingSerial[13]);    //intake temperature
  
      DAQRawThermocouple1 = (int(incomingSerial[14]) * 256) + int(incomingSerial[15]);  //egt, needs correcting curve to be applied
      DAQRawThermocouple2 = (int(incomingSerial[16]) * 256) + int(incomingSerial[17]);  //oil temp
      DAQRawThermocouple3 = (int(incomingSerial[18]) * 256) + int(incomingSerial[19]);  //turbo temp
  
      DAQRawOnboardThermistor = (int(incomingSerial[20]) * 256) + int(incomingSerial[21]);
      DAQRawRPMCount = (int(incomingSerial[22]) * 256) + int(incomingSerial[23]);
  
      DAQRawSensorState = byte(incomingSerial[24]) & byte(0x07);  //wideband pump cell pid state bits
      DAQRawHeaterState = byte(incomingSerial[25]) & byte(0x07);  //wideband heater pid state bits
    }
  }
  */
}

void update_averages()
{
  delay(1);
}

void remap_raw_values()
{
  /// remap all raw values to their real-world values: ///
  AFR = ((float(DAQRawLambda16) / 8192 ) + 0.5 ) * SETTING_STOIC_AFR;

  intakePressurePSI = float(map(DAQRawUser1, 818, 7366, 0, 3000)) / 100;  // 0 - 5 volts maps shows inthe dac as 0 - 8184, maps to 0 - "30" psi

  //oilPressurePSI = map(DAQRawUser2, 8184, 6465, 0, 110);  //oil pressure sensor

  /*
  //VW coolant temp sensor, not linear so lookup table needed
  for (parsingLoopCounter = 1; DAQRawUser2 > coolantSenseTable[parsingLoopCounter]; parsingLoopCounter = parsingLoopCounter + 2) { }
  coolant_temperature = map(DAQRawUser2, coolantSenseTable[parsingLoopCounter - 2], coolantSenseTable[parsingLoopCounter], coolantSenseTable[parsingLoopCounter - 3], coolantSenseTable[parsingLoopCounter - 1]);
  */

  //VW intake temp sensor, not linear so lookup table needed
  for (parsingLoopCounter = 1; DAQRawUser3 > coolantSenseTable[parsingLoopCounter]; parsingLoopCounter = parsingLoopCounter + 2) { }
  intakeTemperatureC = map(DAQRawUser3, coolantSenseTable[parsingLoopCounter - 2], coolantSenseTable[parsingLoopCounter], coolantSenseTable[parsingLoopCounter - 3], coolantSenseTable[parsingLoopCounter - 1]);

  //before dealing with the thermocouples, we must first calc the CJC temperature
  //DAQ RTD temperature sensor, not linear so lookup table needed
  for (parsingLoopCounter = 1; DAQRawOnboardThermistor < DAQ_Temp_Table[parsingLoopCounter]; parsingLoopCounter = parsingLoopCounter + 2) { }
  DAQTemperatureC = map(DAQRawOnboardThermistor, DAQ_Temp_Table[parsingLoopCounter - 2], DAQ_Temp_Table[parsingLoopCounter], DAQ_Temp_Table[parsingLoopCounter - 3], DAQ_Temp_Table[parsingLoopCounter - 1]);

  for (parsingLoopCounter = 1; DAQRawThermocouple1 > thermocouple_Table[parsingLoopCounter]; parsingLoopCounter = parsingLoopCounter + 2) { }
  exhuastTemperatureC = map(DAQRawThermocouple1, thermocouple_Table[parsingLoopCounter - 2], thermocouple_Table[parsingLoopCounter], thermocouple_Table[parsingLoopCounter - 3], thermocouple_Table[parsingLoopCounter - 1]) + DAQTemperatureC;

  for (parsingLoopCounter = 1; DAQRawThermocouple2 > thermocouple_Table[parsingLoopCounter]; parsingLoopCounter = parsingLoopCounter + 2) { }
  oilTemperatureC = map(DAQRawThermocouple2, thermocouple_Table[parsingLoopCounter - 2], thermocouple_Table[parsingLoopCounter], thermocouple_Table[parsingLoopCounter - 3], thermocouple_Table[parsingLoopCounter - 1]) + DAQTemperatureC;
  
  for (parsingLoopCounter = 1; DAQRawThermocouple3 > thermocouple_Table[parsingLoopCounter]; parsingLoopCounter = parsingLoopCounter + 2) { }
  turboTemperatureC = map(DAQRawThermocouple3, thermocouple_Table[parsingLoopCounter - 2], thermocouple_Table[parsingLoopCounter], thermocouple_Table[parsingLoopCounter - 3], thermocouple_Table[parsingLoopCounter - 1]) + DAQTemperatureC;

  engineRPM = 12000000 / (DAQRawRPMCount * SETTING_ENGINE_PULSES_PER_REV);  //1.2 million is appropriate for one pulse per crankshaft revolution
}

void update_display() //called once per ~100ms to refresh the display
{
  //lcd.clear();  //causes flickering issues if used

  char printable_data[5];

  ///FIRST LINE///
  lcd.setCursor(0, 0);
  
  lcd.write("AFR:");
  if (AFR < 50)
  {
    dtostrf(AFR, 4, 1, printable_data);
    lcd.print(printable_data); 
  }
  else
  {
    lcd.write("LEAN");
  }

  lcd.setCursor(11, 0);
  lcd.write("EGT:");
  dtostrf(exhuastTemperatureC, 3, 0, printable_data);
  lcd.print(printable_data);
  lcd.write(0xDF);
  lcd.write("C");  
  
  ///SECOND LINE///
  lcd.setCursor(0, 1);
  lcd.write("INTAKE:");
  dtostrf(intakePressurePSI, 4, 1, printable_data);
  lcd.write(printable_data);
  lcd.write(" PSI");
  dtostrf(intakeTemperatureC, 3, 0, printable_data);
  lcd.print(printable_data);
  lcd.write(0xDF); //0xDF is the degree sign
  lcd.write("C");

  //brightness troubleshooting
  //lcd.setCursor(13, 1);
  //lcd.print(LDRreading, DEC);
  
  ///THIRD LINE///
  
  lcd.setCursor(0, 2);
  lcd.write("OIL:");
  //dtostrf(oilPressurePSI, 3, 0, printable_data);    //OIL PSI
  //lcd.print(printable_data);
  //lcd.write("PSI ");
  dtostrf(oilTemperatureC, 3, 0, printable_data); //oil temperature
  lcd.print(printable_data);
  lcd.write(0xDF); //0xDF is the degree sign
  lcd.write("C");
  
  lcd.setCursor(13, 2);
  dtostrf(engineRPM, 4, 0, printable_data);
  lcd.print(printable_data); //0xDF is the degree sign
  lcd.write("RPM");

  ///FOURTH LINE///
  lcd.setCursor(0, 3);
  lcd.write("METER:");
  dtostrf(turboTemperatureC, 3, 0, printable_data);
  lcd.print(printable_data);
  lcd.write(0xDF); //0xDF is the degree sign
  lcd.write("C");

  //CRC troubleshooting
  lcd.setCursor(13, 3);
  if (crcGood == 1){
      lcd.write("DiG");
  }
  if (crcGood == 0){
      lcd.write("CRC");
  }

  lcd.setCursor(17, 3);
  //dtostrf(DAQRawSensorState, 2, 0, printable_data); //print pump cell pid state
  //lcd.print(printable_data);
  lcd.print(String(DAQRawSensorState));

  lcd.setCursor(19, 3);
  //dtostrf(DAQRawHeaterState, 2, 0, printable_data); //print heater pid state
  //lcd.print(printable_data);
  lcd.print(String(DAQRawHeaterState));  
  
}

void update_brightness()
{
  LDRreading = analogRead(SETTING_LDRPIN);
  LDRreading = 700;

  //assuming LDR at brightest is 700, darkest is 0
  float ambientLightLevel = map(LDRreading, 0, 700, 0, 1);

  //lcd.setBacklight(255, 255, 255); //bright white
  lcd.setBacklight(int(255*ambientLightLevel), int(120*ambientLightLevel), int(90*ambientLightLevel));

}
