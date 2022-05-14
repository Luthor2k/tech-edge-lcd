//  Vacuum Fluorescent Display for use with Tech Edge wideband 2Y1 controller
//  Dec 2017 Arthur Hazleden

/* LAYOUT:

   1250RPM   BATT:13.93
   AFR:14.7  COOL:102o
   OIL: 214o OIL: 24PSI
   IAT:12o   EGT:1294o

  eletrical connections:
  USER1:battery voltage / 4
  USER2:coolant temp sensor
  USER3:oil pressure sensor

  TC1:exhaust temperature
  TC2:post turbo IAT
  TC3:oil temp at filter
*/

#include <LiquidCrystal.h>


int LDRPin = A0;    // light dependent resistor input ping
int LDRreading;     //raw LDR value; 700 when dark, under 20 when bright
int LDRState = 1;  // will be set to one of four brightness levels

//program funtion timing
unsigned long prevDisplayUpdateTime = 0;
const long DisplayUpdateInterval = 100;

//Serial stream from controller
byte incomingSerial[56];
byte serialStreamCount = 0;

// Raw data coming in from controller:
byte header_1 = 0x5A;
byte header_2 = 0xA5;

byte sequence_counter;

byte tick_high;
byte tick_low;

int lambda16;

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

byte CRC;

//lookup look loop counter
int i = 0;

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
String status_sensor_missing = "Sensor Missing";
String status_sensor_cold = "Sensor Cold";
String status_sensor_warm = "Sensor Warm";

String status_heater_normal = "Heater Normal";
String status_heater_lowb = "Low Battery";
String status_heater_highb = "High Battery";
String status_heater_short = "Heater Short";
String status_heater_open = "Heater Open";

String status_lean = "LEAN";

// Corrected data values:
float lambda;
float AFR = 0;

float voltage1;
float coolant_T;
float voltage3;

int thermo1 = 999;
float thermo2;
float thermo3;

int DAQ_temperature;

int RPM = 999;

int WB_STATUS;


LiquidCrystal lcd(13, 11, 5, 4, 3, 2);

void setup() {
  Serial.begin(19200);

  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);


  lcd.begin(20, 4);

  lcd.clear();

  //Operating Values:
  lcd.setCursor(0, 0);
  //lcd.write("    RPM  B+:      V");
  lcd.write("     RPM");
  lcd.setCursor(0, 1);
  lcd.write("AFR:     COOL:    "); lcd.write(0xDF);lcd.write("C");
  lcd.setCursor(0, 2);
  lcd.write("OIL:   "); lcd.write(0xDF); lcd.write(" OIL:    PSI"); //oxDF is the degree sign
  lcd.setCursor(0, 3);
  lcd.write("DAQ:   "); lcd.write(0xDF); lcd.write("C EGT:    "); lcd.write(0xDF); lcd.write("C");
  //lcd.write("RAW:      "); lcd.write("EGT:    "); lcd.write(0xDF); lcd.write("C");
    
  /*
    //Troubleshooting Values:
    lcd.setCursor(0, 0);
    lcd.write("sCnt:   L:");
    lcd.setCursor(0, 1);
    lcd.write("U:    ,    ,");
    lcd.setCursor(0, 2);
    lcd.write("TC:    ,    ,");
    lcd.setCursor(0, 3);
    lcd.write("Tm:     RPM:");
  */
}

void loop() {
  unsigned long currentMillis = millis();

  check_serial();

  if (currentMillis - prevDisplayUpdateTime >= DisplayUpdateInterval) {
    prevDisplayUpdateTime = currentMillis;

    update_brightness();
    update_display();
  }

}

void check_serial()
{
  if (Serial.available() > 0) {

    delay(15);  //pause for the time of 1 frame to complete, 15ms?

    serialStreamCount = Serial.readBytesUntil(0x5A, incomingSerial, 30);

    //while(Serial.available() > 0){int garbage = Serial.read();}   //flush input buffer
    /*
        for (int i = 0; i <= 27; i++) {
          lcd.print(incomingSerial[i], HEX);
        }
    */

    /// Break apart the frame: ///
    sequence_counter = incomingSerial[1];

    lambda16 = (int(incomingSerial[4]) * 256) + int(incomingSerial[5]);

    user1 = (int(incomingSerial[8]) * 256) + int(incomingSerial[9]);      //B+
    user2 = (int(incomingSerial[10]) * 256) + int(incomingSerial[11]);    //coolant
    user3 = (int(incomingSerial[12]) * 256) + int(incomingSerial[13]);    //oil pressure

    thermocouple1 = (int(incomingSerial[14]) * 256) + int(incomingSerial[15]);  //egt, needs correcting curve to be applied
    thermocouple2 = (int(incomingSerial[16]) * 256) + int(incomingSerial[17]);  //IAT
    thermocouple3 = (int(incomingSerial[18]) * 256) + int(incomingSerial[19]);  //oil temp

    thermistor = (int(incomingSerial[20]) * 256) + int(incomingSerial[21]);
    RPM_count = (int(incomingSerial[22]) * 256) + int(incomingSerial[23]);

    sensor_state = byte(incomingSerial[24]) & byte(0x07);  //wideband pump cell pid state bits
    heater_state = byte(incomingSerial[25]) & byte(0x07);  //wideband heater pid state bits

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
    AFR = (( lambda16 / 8192 ) + 0.5 ) * 14.5; //AFR of 14.5 is correct for a diesel, 14.7 for gas
    
    //voltage1 = user1 * 0.00061095; //0-5V scale

    voltage1 = user1 * 0.0024438; //0-20V scale for use with 4:1 resistor divider

    int i = 1;  //VW coolant temp sensor, not linear so lookup table needed
    for (i = 1; user2 > coolantSenseTable[i]; i = i + 2) { }
    coolant_T = map(user2, coolantSenseTable[i - 2], coolantSenseTable[i], coolantSenseTable[i - 3], coolantSenseTable[i - 1]);

    voltage3 = map(user3, 818, 6465, 0, 110);  //oil pressure sensor


    //before dealing with the thermocouples, we must first calc the CJC temperature
    i = 1;  //DAQ RTD temperature sensor, not linear so lookup table needed
    for (i = 1; thermistor < DAQ_Temp_Table[i]; i = i + 2) { }
    DAQ_temperature = map(thermistor, DAQ_Temp_Table[i - 2], DAQ_Temp_Table[i], DAQ_Temp_Table[i - 3], DAQ_Temp_Table[i - 1]);
  
    i = 1;
    for (i = 1; thermocouple1 > thermocouple_Table[i]; i = i + 2) { }
    thermo1 = map(thermocouple1, thermocouple_Table[i - 2], thermocouple_Table[i], thermocouple_Table[i - 3], thermocouple_Table[i - 1]) + DAQ_temperature;

    i = 1;
    for (i = 1; thermocouple2 > thermocouple_Table[i]; i = i + 2) { }
    thermo2 = map(thermocouple2, thermocouple_Table[i - 2], thermocouple_Table[i], thermocouple_Table[i - 3], thermocouple_Table[i - 1]) + DAQ_temperature;
    
    i = 1;
    for (i = 1; thermocouple3 > thermocouple_Table[i]; i = i + 2) { }
    thermo3 = map(thermocouple3, thermocouple_Table[i - 2], thermocouple_Table[i], thermocouple_Table[i - 3], thermocouple_Table[i - 1]) + DAQ_temperature;

    //thermo3 = user3 * 0.00061095;

    serialStreamCount = 0;
  }
}


void update_display() //called once per ~100ms to refresh the display
{
  //Serial.print("display_update");
  //Serial.print(sequence_counter);

  //RPM = random(1, 9999);
  //user1 = random(100, 145) / 10.0;

  char printable_data[5];

  ///FIRST LINE///
  lcd.setCursor(0, 0);
  dtostrf(RPM_count, 4, 0, printable_data);
  //lcd.print(printable_data);

  lcd.setCursor(13, 0);
  //dtostrf(voltage1, 5, 2, printable_data);
  dtostrf(sensor_state, 2, 0, printable_data); //print pump cell pid state
  //lcd.print(printable_data);
  lcd.print(String(sensor_state));

  lcd.setCursor(16, 0);
  //dtostrf(voltage1, 5, 2, printable_data);
  dtostrf(heater_state, 2, 0, printable_data); //print heater pid state
  //lcd.print(printable_data);
  lcd.print(String(heater_state));

  ///SECOND LINE///
  lcd.setCursor(4, 1);
  if (AFR < 50)
  {
    dtostrf(AFR, 3, 1, printable_data);
  }
  else
  {
    status_lean.toCharArray(printable_data, 5);
  }
  
  lcd.print(printable_data);

  lcd.setCursor(13, 1);
  dtostrf(coolant_T, 3, 0, printable_data);    //Coolant
  //lcd.print(printable_data);

  ///THIRD LINE///
  lcd.setCursor(4, 2);
  dtostrf(thermo3, 4, 0, printable_data); //oil temperature
  //lcd.print(printable_data);

  lcd.setCursor(13, 2);
  lcd.print("   ");
  lcd.setCursor(13, 2);
  //dtostrf(voltage3, 2, 0, printable_data);    //OIL PSI
  //lcd.print(String(voltage3, 0));

  ///FOURTH LINE///
  lcd.setCursor(4, 3);
  dtostrf(DAQ_temperature, 3, 0, printable_data);
  //dtostrf(thermistor, 3, 0, printable_data);  //raw value for troubleshooting
  lcd.print(printable_data);

  lcd.setCursor(14, 3);
  dtostrf(thermo1, 4, 0, printable_data);
  lcd.print(printable_data);

}



void update_brightness()
{
  LDRreading = analogRead(LDRPin);

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
