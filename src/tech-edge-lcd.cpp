//  Vacuum Fluorescent Display for use with Tech Edge wideband 2Y1 controller
//  Dec 2017 Arthur Hazleden

//  https://github.com/Luthor2k/tech-edge-lcd

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <sensor_tables.h>

//settings for engine config:
const uint8_t SETTING_ENGINE_PULSES_PER_REV = 1;

const int16_t SETTING_STOIC_AFR = 14.5; //AFR of 14.5 is correct for a diesel, 14.7 for gas

//light dependent resistor for display brightness
static const uint8_t SETTING_LDRPIN = A0; //LDR input pin, set to wherever connected
int LDRreading;     //raw LDR value; 700 when dark, under 20 when bright
int LDRState = 1;  // will be set to one of four brightness levels, start out at 1

LiquidCrystal lcd(13, 11, 5, 4, 3, 2);

//program function timing
unsigned long prevDisplayUpdateTime = 0;
const uint64_t SETTING_DISPLAY_UPDATE_INTERVAL_MS = 100;

unsigned long prevBrightnessUpdateTime = 0;
const uint64_t SETTING_BRIGHTNESS_UPDATE_INTERVAL_MS = 5000; //set this to how often to update the brightness in milliseconds

//Serial stream from controller
const uint8_t DATA_FRAME_frameheader1 = 0x5a;
const uint8_t DATA_FRAME_frameheader2 = 0xa5;
const uint8_t DATA_FRAME_RX_LENGTH_BYTES = 28;
const uint8_t SETTING_SERIAL_FRAME_RX_DELAY_MS = 15; //approx time for the serial frame to definetly complete
uint8_t incomingSerial[DATA_FRAME_RX_LENGTH_BYTES];
uint8_t serialStreamCount = 0;

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
int32_t AFR;

int32_t boostPressure;
int32_t oilPressure;
int32_t intakeTemperature;
int32_t coolant_temperature;
int32_t throttle_position;

uint16_t exhuastTemperature;
uint16_t oilTemperature;
uint16_t turboTemperature;

uint16_t DAQTemperature;

uint16_t engineRPM;

uint16_t WbStatus;

uint16_t crcSum;
uint8_t crcComp;

bool crcGood;

//function declarations
bool check_serial();
void update_averages();
void update_display();
void update_brightness();
void remap_raw_values();

void setup() {
  Serial.begin(19200);

  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);

  lcd.begin(20, 4);
  lcd.clear();
}

void loop() {
  unsigned long currentMillis = millis();

  if (check_serial() == true){
    update_averages();
    remap_raw_values();
  }

  if (currentMillis - prevDisplayUpdateTime >= SETTING_DISPLAY_UPDATE_INTERVAL_MS) {
    prevDisplayUpdateTime = currentMillis;
    update_display();
  }

  if (currentMillis - prevBrightnessUpdateTime >= SETTING_BRIGHTNESS_UPDATE_INTERVAL_MS) {
    prevBrightnessUpdateTime = currentMillis;
    update_brightness();
  }

}

bool check_serial()
{
  if (Serial.available() > 0) {

    delay(SETTING_SERIAL_FRAME_RX_DELAY_MS);  //pause for the time of 1 frame to complete

    Serial.readBytesUntil(DATA_FRAME_frameheader1, incomingSerial, DATA_FRAME_RX_LENGTH_BYTES);

    if (incomingSerial[0] == byte(DATA_FRAME_frameheader2))    //should be the second of the 0x5A 0xA5 frame header
    {
      // CRC calculation
      crcSum = 0;
      for (uint8_t frameByteIndex = 0; frameByteIndex < 27; frameByteIndex++){
        crcSum += incomingSerial[frameByteIndex];
        crcComp = lowByte(crcSum);  //should ALWAYS be 0xFF. well, should always be 0xA5 because of how the frame sync is being done..
      }

      crcGood = false;
      if (crcComp == DATA_FRAME_frameheader2){
        crcGood = true;
  
        /// Break apart the frame: ///
        sequence_counter = incomingSerial[1];
    
        DAQRawLambda16 = (int(incomingSerial[4]) * 256) + int(incomingSerial[5]);
    
        DAQRawUser1 = (int(incomingSerial[8]) * 256) + int(incomingSerial[9]);      //boost pressure
        DAQRawUser2 = (int(incomingSerial[10]) * 256) + int(incomingSerial[11]);    //oil pressure
        DAQRawUser3 = (int(incomingSerial[12]) * 256) + int(incomingSerial[13]);    //intake temperature
    
        DAQRawThermocouple1 = (int(incomingSerial[14]) * 256) + int(incomingSerial[15]);  //egt, needs correcting curve to be applied
        DAQRawThermocouple2 = (int(incomingSerial[16]) * 256) + int(incomingSerial[17]);  //oil temp
        DAQRawThermocouple3 = (int(incomingSerial[18]) * 256) + int(incomingSerial[19]);  //turbo temp
    
        DAQRawOnboardThermistor = (int(incomingSerial[20]) * 256) + int(incomingSerial[21]);
        DAQRawRPMCount = (int(incomingSerial[22]) * 256) + int(incomingSerial[23]);
    
        DAQRawSensorState = byte(incomingSerial[24]) & byte(0x07);  //wideband pump cell pid state bits
        DAQRawHeaterState = byte(incomingSerial[25]) & byte(0x07);  //wideband heater pid state bits
        return true;
      }
    }
  }
  return false;
}

void update_averages()
{
  delay(1);
}

void remap_raw_values()
{
  /// remap all raw values to their real-world values: ///
  lambda = float(DAQRawLambda16);
  //AFR = (( lambda / 8192 ) + 0.5 ) * SETTING_STOIC_AFR;
  AFR = (( lambda / 8192 ) + 0.5 ) * 14.5; //AFR of 14.5 is correct for a diesel, 14.7 for gas

  //boostPressure = DAQRawUser1 * 0.0024438; //0-30psi sensor
  boostPressure = map(DAQRawUser1, 818, 7366, 0, 3000);  // 0 - 5 volts maps shows inthe dac as 0 - 8184, maps to 0 - "30" psi
  boostPressure = boostPressure / 100;  //move the decimal place

  oilPressure = map(DAQRawUser2, 8184, 6465, 0, 110);  //oil pressure sensor

  /*
  //VW coolant temp sensor, not linear so lookup table needed
  for (parsingLoopCounter = 1; DAQRawUser2 > coolantSenseTable[parsingLoopCounter]; parsingLoopCounter = parsingLoopCounter + 2) { }
  coolant_temperature = map(DAQRawUser2, coolantSenseTable[parsingLoopCounter - 2], coolantSenseTable[parsingLoopCounter], coolantSenseTable[parsingLoopCounter - 3], coolantSenseTable[parsingLoopCounter - 1]);
  */

  //VW intake temp sensor, not linear so lookup table needed
  for (parsingLoopCounter = 1; DAQRawUser3 > coolantSenseTable[parsingLoopCounter]; parsingLoopCounter = parsingLoopCounter + 2) { }
  intakeTemperature = map(DAQRawUser3, coolantSenseTable[parsingLoopCounter - 2], coolantSenseTable[parsingLoopCounter], coolantSenseTable[parsingLoopCounter - 3], coolantSenseTable[parsingLoopCounter - 1]);

  //before dealing with the thermocouples, we must first calc the CJC temperature
  //DAQ RTD temperature sensor, not linear so lookup table needed
  for (parsingLoopCounter = 1; DAQRawOnboardThermistor < DAQ_Temp_Table[parsingLoopCounter]; parsingLoopCounter = parsingLoopCounter + 2) { }
  DAQTemperature = map(DAQRawOnboardThermistor, DAQ_Temp_Table[parsingLoopCounter - 2], DAQ_Temp_Table[parsingLoopCounter], DAQ_Temp_Table[parsingLoopCounter - 3], DAQ_Temp_Table[parsingLoopCounter - 1]);

  for (parsingLoopCounter = 1; DAQRawThermocouple1 > thermocouple_Table[parsingLoopCounter]; parsingLoopCounter = parsingLoopCounter + 2) { }
  exhuastTemperature = map(DAQRawThermocouple1, thermocouple_Table[parsingLoopCounter - 2], thermocouple_Table[parsingLoopCounter], thermocouple_Table[parsingLoopCounter - 3], thermocouple_Table[parsingLoopCounter - 1]) + DAQTemperature;

  for (parsingLoopCounter = 1; DAQRawThermocouple2 > thermocouple_Table[parsingLoopCounter]; parsingLoopCounter = parsingLoopCounter + 2) { }
  oilTemperature = map(DAQRawThermocouple2, thermocouple_Table[parsingLoopCounter - 2], thermocouple_Table[parsingLoopCounter], thermocouple_Table[parsingLoopCounter - 3], thermocouple_Table[parsingLoopCounter - 1]) + DAQTemperature;
  
  for (parsingLoopCounter = 1; DAQRawThermocouple3 > thermocouple_Table[parsingLoopCounter]; parsingLoopCounter = parsingLoopCounter + 2) { }
  turboTemperature = map(DAQRawThermocouple3, thermocouple_Table[parsingLoopCounter - 2], thermocouple_Table[parsingLoopCounter], thermocouple_Table[parsingLoopCounter - 3], thermocouple_Table[parsingLoopCounter - 1]) + DAQTemperature;

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
    //status_lean.toCharArray(printable_data, 5); //if running in a lean condition, paste lean test into lcd buffer
    lcd.write("LEAN");
  }
   
  //lcd.setCursor(10, 0);
  //lcd.write("RAW:");
  //lcd.setCursor(14, 0);
  
  //dtostrf(DAQRawLambda16, 5, 0, printable_data);
  //lcd.print(printable_data);
  //sprintf(printable_data, "RAW: %d", DAQRawLambda16);
  //lcd.print(printable_data);
  //lcd.print(String(DAQRawLambda16));
  /*
  char str_temp[6];
  // 4 is mininum width, 2 is precision; float value is copied onto str_temp
  AFR = 14.9;
  dtostrf(AFR, 4, 1, str_temp);   //arduino doesnt do floats
  sprintf (printable_data, "AFR: %s RAW: %d", str_temp, DAQRawLambda16);
  lcd.print(printable_data);
  */

  lcd.setCursor(10, 0);
  lcd.write("EGT:");
  dtostrf(exhuastTemperature, 3, 0, printable_data);
  lcd.print(printable_data);
  lcd.write(0xDF);
  lcd.write("C ");  
  
  ///SECOND LINE///
  lcd.setCursor(0, 1);
  lcd.write("MAP:");
  dtostrf(boostPressure, 4, 1, printable_data);
  lcd.write(printable_data);
  lcd.write(" PSI  ");
  //lcd.setCursor(12, 1);
  //intakeTemperature = 55;
  //dtostrf(intakeTemperature, 3, 0, printable_data);
  //lcd.print(printable_data);
  //lcd.write(0xDF); //0xDF is the degree sign
  //lcd.write("C  ");

  //brightness troubleshooting
  //lcd.setCursor(13, 1);
  //lcd.print(LDRreading, DEC);
  
  ///THIRD LINE///
  lcd.setCursor(0, 2);
  lcd.write("OIL:");
  //oilPressure = 34.8;
  //dtostrf(oilPressure, 3, 0, printable_data);    //OIL PSI
  //lcd.print(printable_data);
  //lcd.write(" PSI");
  //lcd.setCursor(12, 2);
  //oilTemperature = 0;
  dtostrf(oilTemperature, 3, 0, printable_data); //oil temperature
  lcd.print(printable_data);
  lcd.write(0xDF); //0xDF is the degree sign
  lcd.write("C  ");
  
  lcd.setCursor(13, 2);
  //DAQRawRPMCount = 1500;
  dtostrf(engineRPM, 4, 0, printable_data);
  lcd.print(printable_data); //0xDF is the degree sign
  lcd.write("RPM");
  
  ///FOURTH LINE///
  //turboTemperature = 999;
  lcd.setCursor(0, 3);
  lcd.write("METER:");
  dtostrf(turboTemperature, 3, 0, printable_data);
  lcd.print(printable_data);
  lcd.write(0xDF); //0xDF is the degree sign
  lcd.write("C  ");

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

  if (LDRreading > 700)
  {
    LDRState = 1;
    lcd.command((byte)0x20);  //tell VFD to go into 'funtion set' mode
    lcd.write((byte)0x03);    //set to lowest brightness when dark
  }

  if ((LDRreading < 650) && (LDRreading > 300))
  {
    LDRState = 2;
    lcd.command((byte)0x20);
    lcd.write((byte)0x02);
  }

  if ((LDRreading < 250) && (LDRreading > 30))
  {
    LDRState = 3;
    lcd.command((byte)0x20);
    lcd.write((byte)0x01);
  }

  if (LDRreading < 20)
  {
    LDRState = 4;
    lcd.command((byte)0x20);
    lcd.write((byte)0x00);    //set to highest brightness in daylight
  }

}
