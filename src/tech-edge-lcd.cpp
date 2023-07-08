//  Vacuum Fluorescent Display for use with Tech Edge wideband 2Y1 controller
//  Dec 2017 Arthur Hazleden

//  https://github.com/Luthor2k/tech-edge-lcd

#include <Arduino.h>
#include <LiquidCrystal.h>
 
static const uint8_t SETTING_LDRPIN = A0; // light dependent resistor input pin, set to wherever connected
int LDRreading;     //raw LDR value; 700 when dark, under 20 when bright
int LDRState = 1;  // will be set to one of four brightness levels, start out at 1

//program funtion timing
unsigned long prevDisplayUpdateTime = 0;
const uint64_t SETTING_DISPLAY_UPDATE_INTERVAL_MS = 100;

unsigned long prevBrightnessUpdateTime = 0;
const uint64_t SETTING_BRIGHTNESS_UPDATE_INTERVAL_MS = 5000; //set this to how often to update the brightness in milliseconds

//Serial stream from controller
const uint8_t DATA_FRAME_HEADER_1 = 0x5a;
const uint8_t DATA_FRAME_HEADER_2 = 0xa5;
const uint8_t DATA_FRAME_RX_LENGTH_BYTES = 28;
const uint8_t SETTING_SERIAL_FRAME_RX_DELAY_MS = 15; //approx time for the serial frame to definetly complete
uint8_t incomingSerial[DATA_FRAME_RX_LENGTH_BYTES];
byte serialStreamCount = 0;

// Raw data coming in from controller:
byte header_1 = 0x5A;
byte header_2 = 0xA5;

byte sequence_counter;

byte tick_high;
byte tick_low;

unsigned int lambda16;

int user1;
int user2;
int user3;

int thermocouple1;
int thermocouple2;
int thermocouple3;

int thermistor;

int RPM_count;

byte status_high;
byte status_low;

byte sensor_state;
byte heater_state;

//lookup tables for non linear sensors:
int coolantSenseTable[22] = {
  3,  5622,
  10,  5493,
  20,  5139,
  30,  4429,
  40,  3294,
  50,  2765,
  60,  2208,
  70,  1700,
  80,  1323,
  90,  1010,
  100, 730,

};

int DAQ_Temp_Table[34] = {
  -63, 1024,
  -27, 960,
  -14, 896,
  -5, 882,
  2, 768,
  8, 704,
  14, 640,
  19, 576,
  25, 512,
  31, 448,
  37, 384,
  44, 320,
  51, 256,
  61, 192,
  75, 128,
  98, 64,
  161, 0
};

int thermocouple_Table[34] = {
  //Integer  ADC
  //Approx,  Count,
  0, 0,
  76,  64,
  151, 128,
  229, 192,
  304, 256,
  378, 320,
  452, 384,
  524, 448,
  597, 512,
  670, 576,
  744, 640,
  819, 704,
  896, 768,
  974, 832,
  1054, 896,
  1136, 960,
  1220,  1024
};

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

float boost_pressure;
float oil_pressure;
float intake_temperature;
//float coolant_temperature;
//float throttle_position; //more important for tuning than oil, if recorded

int exhuast_temperature;
int oil_temperature;
int turbo_temperature;

int DAQ_temperature;

int rpm_engine;

int WB_STATUS;

int crc_sum;
byte crc_comp;

bool crc_good;

LiquidCrystal lcd(13, 11, 5, 4, 3, 2);

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

  //some test data:
  //lambda16 = 10000;  //should be diesel stoic
  //float lambdaF = float(lambda16);
  //AFR = (( lambdaF / 8192 ) + 0.5 ) * 14.5; //AFR of 14.5 is correct for a diesel, 14.7 for gas

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

    Serial.readBytesUntil(DATA_FRAME_HEADER_1, incomingSerial, DATA_FRAME_RX_LENGTH_BYTES);

    if (incomingSerial[0] == byte(DATA_FRAME_HEADER_2))    //should be the second of the 0x5A 0xA5 frame header
    {
      // CRC calculation
      crc_sum = 0;
      for (byte frame_byte_number = 0; frame_byte_number < 27; frame_byte_number++){
        crc_sum += incomingSerial[frame_byte_number];
        crc_comp = lowByte(crc_sum);  //should ALWAYS be 0xFF. well, should always be 0xA5 because of how the frame sync is being done..
      }

      crc_good = false;
      if (crc_comp == DATA_FRAME_HEADER_2){
        crc_good = true;
  
        /// Break apart the frame: ///
        sequence_counter = incomingSerial[1];
    
        lambda16 = (int(incomingSerial[4]) * 256) + int(incomingSerial[5]);
    
        user1 = (int(incomingSerial[8]) * 256) + int(incomingSerial[9]);      //boost pressure
        user2 = (int(incomingSerial[10]) * 256) + int(incomingSerial[11]);    //oil pressure
        user3 = (int(incomingSerial[12]) * 256) + int(incomingSerial[13]);    //intake temperature
    
        thermocouple1 = (int(incomingSerial[14]) * 256) + int(incomingSerial[15]);  //egt, needs correcting curve to be applied
        thermocouple2 = (int(incomingSerial[16]) * 256) + int(incomingSerial[17]);  //oil temp
        thermocouple3 = (int(incomingSerial[18]) * 256) + int(incomingSerial[19]);  //turbo temp
    
        thermistor = (int(incomingSerial[20]) * 256) + int(incomingSerial[21]);
        RPM_count = (int(incomingSerial[22]) * 256) + int(incomingSerial[23]);
    
        sensor_state = byte(incomingSerial[24]) & byte(0x07);  //wideband pump cell pid state bits
        heater_state = byte(incomingSerial[25]) & byte(0x07);  //wideband heater pid state bits
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
  /*
  if (lambda16 < 36864) //check if condition is in regular tuning range or if its super lean
    {
      lambda = ( lambda16 / 8192 ) + 0.5;
    }
  else
    {
        lambda = 5.0 + ((lambda16 - 36864 ) / 128);
    }
` */
  float lambdaF = float(lambda16);

  AFR = (( lambdaF / 8192 ) + 0.5 ) * 14.5; //AFR of 14.5 is correct for a diesel, 14.7 for gas

  //boost_pressure = user1 * 0.0024438; //0-30psi sensor
  boost_pressure = map(user1, 818, 7366, 0, 3000);  // 0 - 5 volts maps shows inthe dac as 0 - 8184, maps to 0 - "30" psi
  boost_pressure = boost_pressure / 100;  //get a float

  oil_pressure = map(user2, 8184, 6465, 0, 110);  //oil pressure sensor

  /*
  int i = 1;  //VW coolant temp sensor, not linear so lookup table needed
  for (i = 1; user2 > coolantSenseTable[i]; i = i + 2) { }
  coolant_temperature = map(user2, coolantSenseTable[i - 2], coolantSenseTable[i], coolantSenseTable[i - 3], coolantSenseTable[i - 1]);
  */
  int i = 1;  //VW intake temp sensor, not linear so lookup table needed
  for (i = 1; user3 > coolantSenseTable[i]; i = i + 2) { }
  intake_temperature = map(user3, coolantSenseTable[i - 2], coolantSenseTable[i], coolantSenseTable[i - 3], coolantSenseTable[i - 1]);

  //before dealing with the thermocouples, we must first calc the CJC temperature
  i = 1;  //DAQ RTD temperature sensor, not linear so lookup table needed
  for (i = 1; thermistor < DAQ_Temp_Table[i]; i = i + 2) { }
  DAQ_temperature = map(thermistor, DAQ_Temp_Table[i - 2], DAQ_Temp_Table[i], DAQ_Temp_Table[i - 3], DAQ_Temp_Table[i - 1]);

  i = 1;
  for (i = 1; thermocouple1 > thermocouple_Table[i]; i = i + 2) { }
  exhuast_temperature = map(thermocouple1, thermocouple_Table[i - 2], thermocouple_Table[i], thermocouple_Table[i - 3], thermocouple_Table[i - 1]) + DAQ_temperature;

  i = 1;
  for (i = 1; thermocouple2 > thermocouple_Table[i]; i = i + 2) { }
  oil_temperature = map(thermocouple2, thermocouple_Table[i - 2], thermocouple_Table[i], thermocouple_Table[i - 3], thermocouple_Table[i - 1]) + DAQ_temperature;
  
  i = 1;
  for (i = 1; thermocouple3 > thermocouple_Table[i]; i = i + 2) { }
  turbo_temperature = map(thermocouple3, thermocouple_Table[i - 2], thermocouple_Table[i], thermocouple_Table[i - 3], thermocouple_Table[i - 1]) + DAQ_temperature;

  rpm_engine = 12000000 / RPM_count;  //1.2 million is appropriate for one pulse per crankshaft revolution
}

void update_display() //called once per ~100ms to refresh the display
{
  //Serial.print("display_update");
  //Serial.print(sequence_counter);

  //lcd.clear();

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
  
  //dtostrf(lambda16, 5, 0, printable_data);
  //lcd.print(printable_data);
  //sprintf(printable_data, "RAW: %d", lambda16);
  //lcd.print(printable_data);
  //lcd.print(String(lambda16));
  /*
  char str_temp[6];
  // 4 is mininum width, 2 is precision; float value is copied onto str_temp
  AFR = 14.9;
  dtostrf(AFR, 4, 1, str_temp);   //arduino doesnt do floats
  sprintf (printable_data, "AFR: %s RAW: %d", str_temp, lambda16);
  lcd.print(printable_data);
  */

  lcd.setCursor(10, 0);
  lcd.write("EGT:");
  //exhuast_temperature = 999;
  dtostrf(exhuast_temperature, 3, 0, printable_data);
  lcd.print(printable_data);
  lcd.write(0xDF);
  lcd.write("C ");  
  
  ///SECOND LINE///
  lcd.setCursor(0, 1);
  lcd.write("MAP:");
  dtostrf(boost_pressure, 4, 1, printable_data);
  lcd.write(printable_data);
  lcd.write(" PSI  ");
  //lcd.setCursor(12, 1);
  //intake_temperature = 55;
  //dtostrf(intake_temperature, 3, 0, printable_data);
  //lcd.print(printable_data);
  //lcd.write(0xDF); //0xDF is the degree sign
  //lcd.write("C  ");

  //brightness troubleshooting
  //lcd.setCursor(13, 1);
  //lcd.print(LDRreading, DEC);
  
  ///THIRD LINE///
  lcd.setCursor(0, 2);
  lcd.write("OIL:");
  //oil_pressure = 34.8;
  //dtostrf(oil_pressure, 3, 0, printable_data);    //OIL PSI
  //lcd.print(printable_data);
  //lcd.write(" PSI");
  //lcd.setCursor(12, 2);
  //oil_temperature = 0;
  dtostrf(oil_temperature, 3, 0, printable_data); //oil temperature
  lcd.print(printable_data);
  lcd.write(0xDF); //0xDF is the degree sign
  lcd.write("C  ");
  
  lcd.setCursor(13, 2);
  //RPM_count = 1500;
  dtostrf(rpm_engine, 4, 0, printable_data);
  lcd.print(printable_data); //0xDF is the degree sign
  lcd.write("RPM");
  
  ///FOURTH LINE///
  //turbo_temperature = 999;
  lcd.setCursor(0, 3);
  lcd.write("METER:");
  dtostrf(turbo_temperature, 3, 0, printable_data);
  lcd.print(printable_data);
  lcd.write(0xDF); //0xDF is the degree sign
  lcd.write("C  ");

  //CRC troubleshooting
  lcd.setCursor(13, 3);
  if (crc_good == 1){
      lcd.write("DiG");
  }
  if (crc_good == 0){
      lcd.write("CRC");
  }

  lcd.setCursor(17, 3);
  //dtostrf(sensor_state, 2, 0, printable_data); //print pump cell pid state
  //lcd.print(printable_data);
  lcd.print(String(sensor_state));

  lcd.setCursor(19, 3);
  //dtostrf(heater_state, 2, 0, printable_data); //print heater pid state
  //lcd.print(printable_data);
  lcd.print(String(heater_state));  
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
