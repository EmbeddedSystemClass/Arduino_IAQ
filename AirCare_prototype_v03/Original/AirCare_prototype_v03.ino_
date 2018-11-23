
//sketch must be uploaded over WiFi
#include <Process.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Bridge.h>
#include <YunServer.h>
#include <YunClient.h>
//#include <Console.h>
//#include <Wire.h>
#include <EEPROM.h>
#include "SIm7013.h" 
//#include "SharpDust.h"
#include "Cozir.h"
#include "Inova.h"
#include "ReadNoise.h"
#include "TSL2561.h"
#include "ReadVoc.h"

YunServer server; 
Process resetmcu;

SI7013 rh_sensor;  //initiate an object of SI7013 class
//SharpDust dust_sensor;  //initiate an object of SharpDust class
COZIR co_sensor(11,8);  //initiate an object of COZIR class  (11 - Arduino rx & Cozir tx, 8 -Arduino tx & Cozir rx)
INOVA pm_sensor(10,9);  //initiate an object of INOVA class  (10 - Arduino rx & Inova tx, 9 -Arduino tx & Inova rx)
NoiseRead noise; // object to read data from the GTC noise sensor
VocRead voc_sensor; //object to read data from the VOC sensor (MiCS-5524)
TSL2561 al_sensor; // initiate an object of TSL2561 class (ambient light sensor)

double PM25=0; //var for PM2.5
double PM10=0; //var for PM10
int FanPin=5; // pin to drive the fan
int CO2conc=0;   //var for CO2 concentration
int i=0;
int light=0; //var for the lighting level
long TimeStart=0;
long TimeEnd=0;
long TimeToDelay=0;
long SamplingRate=3000;  //measurement rate in ms; must be not less than 3000
float humd=0;  //var for humidity
float temp=0;  //var for temperature
float VOCconc=0;  // var for VOC concentration
float noise_level=0; //var for the noise level
String StrToSend;    //message to log
char strarray[150]; //char array to be sended over WiFi
char newline=10; //for new line
char CR=13; // for carriage return
char bClientCommand=0; // incoming command from PC app: 0x63 reset mcu, 0x23 power on the fan, 0x22 power off the fan
bool STATE=HIGH;
bool ledstate=false;
bool FanState=false;
int eepromValue; // fan state data stored in eeprom: 0 or 255 - powered off, 1 - powered on
int addr=0; // eeprom address of the eepromValue
int ValToWrite; // var to be written to eeprom: 0- to power off the fan, 1 - to power on


void setup()
{
  
  Bridge.begin();
  rh_sensor.begin();
  co_sensor.begin();
  pm_sensor.begin();
  al_sensor.begin();
  server.begin();
  eepromValue=eepromValueRead(); //read fan state data from eeprom, if 1 - turn on the fan
  pinMode(FanPin, OUTPUT);
  if (eepromValue==1)
  {
    //digitalWrite(13, HIGH);
    digitalWrite(FanPin, LOW);
  }
  else
  {
    //digitalWrite(13, LOW);
    digitalWrite(FanPin, HIGH);
  }
  pinMode(13, OUTPUT);
  delay(2000);

}

void loop() 
{
  
  StrToSend="";
  TimeStart=0;
  TimeEnd=0;
  TimeToDelay=0;
  TimeStart=millis(); //time of start the data reading
  light=al_sensor.readVisAndIR(); //get data from TSL2561 sensor
  humd = rh_sensor.readHumidity();  //get data from Si7013 sensor
  temp = rh_sensor.readTemperature();  //get data from Si7013 sensor
  co_sensor.listen();  // since there are 2 sw serial ports 
  delay(100);
  while (!co_sensor.isListening())
  {
    delay(1);
  }
  if(co_sensor.isListening())
  {
    CO2conc=co_sensor.readCO2conc();  //get data from Cozir CO2 sensor
  }
  
  pm_sensor.listen();
  delay(100);
  while (!pm_sensor.isListening())
  {
    delay(1);
    
  }
  if(pm_sensor.isListening())
  {
    PM25=pm_sensor.readPM25conc();  //get data from Inova PM sensor
    PM10=pm_sensor.readPM10conc();
    
  }
  VOCconc=voc_sensor.readVOC(); // get data from VOC sensor
  noise_level=noise.readNoise(); // get data from noise sensor

  // start to form a string to be sended to the tcp client
  StrToSend+="V ";
  StrToSend+=String(VOCconc ,2);
  StrToSend+=" T ";
  StrToSend+=String(temp,1);
  StrToSend+=" H ";
  StrToSend+=String(humd,1);
  StrToSend+=" CO2 ";
  StrToSend+=String(CO2conc); 
  StrToSend+=" PM25 ";
  StrToSend+=String(PM25,1);
  StrToSend+=" PM10 ";
  StrToSend+=String(PM10,1);
  StrToSend+=" L ";
  StrToSend+=String(light);
  StrToSend+=" N ";
  StrToSend+=String(noise_level,2);
  StrToSend+=" ";
  StrToSend+=String(eepromValue);
  //Serial.println(StrToSend);

  // start sending the string
  int lentgh=0; // length of the string
  StrToSend.toCharArray(strarray,150);
  YunClient client = server.accept();
  for (i=0; i<150; i++)
  {
   if (int(strarray[i])!=0)
   {
     lentgh++;
   }
  }
  for (i=0; i<lentgh; i++)
   {
     server.write(strarray[i]);
     //Console.println(int(array[i]));
     delay(1);
   }
   server.write(CR);
   server.write(newline);
   for (i=0; i<150; i++)
    {
     strarray[i]=0;
    }  

  // listen a command from app
  if (client.available())
  {
    bClientCommand=client.read();
    if (bClientCommand==0x63) // command to reset mcu
    {
      resetmcu.runShellCommand("/usr/bin/reset-mcu");
    }
    if (bClientCommand==0x23) // command to turn on the fan
    {
      //ledstate=!ledstate; //MT
      ledstate = HIGH; //MT
      //digitalWrite(13, ledstate);
      digitalWrite(FanPin, LOW);
      ValToWrite=1;
      EEPROM.write(addr, ValToWrite);
      eepromValue=eepromValueRead();
    }
    if (bClientCommand==0x22)  //command to turn off the fan
    {
      //ledstate=!ledstate; //MT
      ledstate = LOW; //MT
      //digitalWrite(13, ledstate);
      digitalWrite(FanPin, HIGH);
      ValToWrite=0;
      EEPROM.write(addr, ValToWrite);
      eepromValue=eepromValueRead();
    }
  }
  
  
  TimeEnd=millis(); // time of the end of the cycle
  TimeToDelay=TimeEnd-TimeStart; // if elapsed time is less than sampling rate then wait 
  if ((SamplingRate-TimeToDelay)>0)   
  {
    delay(SamplingRate-TimeToDelay);
  }
  
  
  
}

int eepromValueRead() // function to read the value from EEPROM about fan state
{
  byte Value=EEPROM.read(0);
  return int(Value);
}


