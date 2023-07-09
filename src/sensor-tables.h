//lookup tables for non linear sensors

//  https://github.com/Luthor2k/tech-edge-lcd

//vw coolant temp sensor
uint16_t coolantSenseTable[22] = {
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


//tech edge DIY 2Y onboard thermistor
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

//type K thermocoupe
uint16_t thermocouple_Table[34] = {
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