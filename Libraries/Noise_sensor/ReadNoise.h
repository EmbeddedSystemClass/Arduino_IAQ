
#if defined(ARDUINO) && ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif



#define MEASURE_PIN  1


class NoiseRead {

public:
  NoiseRead();

  //Public Functions
  float readNoise(void);

private:
  
  //Private Variables
  int RawRead[5];
  int i;
  int Median;
  int k;
  int j;
  int x;
  float NoiseMedianVolts;
    
};
