
#if defined(ARDUINO) && ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif



#define MEASURE_PIN  3


class VocRead {

public:
  VocRead();

  //Public Functions
  float readVOC(void);

private:
  
  //Private Variables
  int RawRead;
  float AverageRead;
  int i;
  
    
};
