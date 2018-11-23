//sketch should be uploaded over USB
//#include <I2C.h> //MT only for debug
#include <Process.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Bridge.h>
//#include <YunServer.h>
//#include <YunClient.h>
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

//YunServer server; 
Process resetmcu;
Process runCurl;

SI7013 rh_sensor;  //initiate an object of SI7013 class
//SharpDust dust_sensor;  //initiate an object of SharpDust class
COZIR co_sensor(11,8);  //initiate an object of COZIR class  (11 - Arduino rx & Cozir tx, 8 -Arduino tx & Cozir rx)
INOVA pm_sensor(10,9);  //initiate an object of INOVA class  (10 - Arduino rx & Inova tx, 9 -Arduino tx & Inova rx)
NoiseRead noise; // object to read data from the GTC noise sensor
VocRead voc_sensor; //object to read data from the VOC sensor (MiCS-5524)
TSL2561 al_sensor; // initiate an object of TSL2561 class (ambient light sensor)

int FanPin=5; // pin to drive the fan
int CO2conc=0;   //var for CO2 concentration
int i=0;
int light=0; //var for the lighting level
long TimeStart=0;
long TimeEnd=0;
long TimeToDelay=0;
long SamplingRate=5000;  //measurement rate in ms; must be not less than 3000
float humd=0;  //var for humidity
float temp=0;  //var for temperature
float VOCconc=0;  // var for VOC concentration
float noise_level=0; //var for the noise level
float PM25=0;
float PM10=0;
float CO2concAv=0; //averaged value of CO2 conc
float PM25Av=0;
float PM10Av=0;
float VOCconcAv=0;
float noise_levelAv=0;
float lightAv=0;
String StrToSend;    //message to log
String cmdRun; // parameter in curl command on Linux side
String cmd="sh /mnt/sda1/curl.sh"; // the cURL command. two parameters are needed
char strarray[150]; //char array to be sended over WiFi
char newline=10; //for new line
char CR=13; // for carriage return
char bClientCommand=0; // incoming command from app: 0x63 reset mcu, 0x23 power on the fan, 0x22 power off the fan
bool STATE=HIGH;
bool ledstate=false;
bool FanState=false;
int eepromValue; // fan state info stored in eeprom: 0 or 255 - powered off, 1 - powered on
int addr=0; // eeprom address of the eepromValue
int ValToWrite; // var to be written to eeprom: 0- to power off the fan, 1 - to power on


void setup()
{
  Serial.begin(9600); 
  
  //MT only for debug
  /*
  I2c.begin(); 
  Serial.println("I2C scan devices ...");
  I2c.scan();
  */
  Bridge.begin();
  rh_sensor.begin();
  co_sensor.begin();
  pm_sensor.begin();
  al_sensor.begin();
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
  cmdRun="";
  StrToSend="";
  light=0;
  humd=0.0;
  temp=0.0;
  CO2conc=0;
  PM25=0.0;
  PM10=0.0;
  VOCconc=0.0;
  noise_level=0.0;
  TimeStart=0;
  TimeEnd=0;
  TimeToDelay=0;
  TimeStart=millis(); //time of start the data reading
  //digitalWrite(13, HIGH);
  
  //start 5sec measurement cycle
    light=al_sensor.readVisAndIR(); //get data from TSL2561 sensor
    //Serial.println("light OK"); //MT
    humd = rh_sensor.readHumidity();  //get data from Si7013 sensor
    //Serial.println("rh OK"); //MT
    temp = rh_sensor.readTemperature();  //get data from Si7013 sensor
    //Serial.println("T OK"); //MT
    co_sensor.listen();  // since there are 2 sw serial ports 
    delay(100);
    CO2conc=co_sensor.readCO2conc();  //get data from Cozir CO2 sensor
    //Serial.println("CO2 OK"); //MT
    delay(1000);
    pm_sensor.listen();
    delay(100);
    PM25=pm_sensor.readPM25conc();  //get data from Inova PM sensor
    PM10=pm_sensor.readPM10conc();
    //Serial.println("PM OK"); //MT
    VOCconc=voc_sensor.readVOC(); // get data from VOC sensor
    //Serial.println("VOC OK"); //MT
    noise_level=noise.readNoise(); // get data from noise sensor
    Serial.println("Measurement cycle OK");
 // end measurement cycle
  
 //averaging
  CO2concAv=CO2concAv+CO2conc;
  PM25Av=PM25Av+PM25;
  PM10Av=PM10Av+PM10;
  VOCconcAv=VOCconcAv+VOCconc;
  noise_levelAv=noise_levelAv+noise_level;
  lightAv=lightAv+light;
 // end averaging

  //digitalWrite(13, LOW);
  TimeEnd=millis(); // time of the end of the cycle
  TimeToDelay=TimeEnd-TimeStart; // if elapsed time is less than sampling rate then wait 
  if ((SamplingRate-TimeToDelay)>0)   
  {
    delay(SamplingRate-TimeToDelay);
  }

  i++;
  if (i==12) //occurs once per appx. 30 sec
  {
    CO2concAv=CO2concAv/12.0;
    PM25Av=PM25Av/12.0;
    PM10Av=PM10Av/12.0;
    VOCconcAv=VOCconcAv/12.0;
    noise_levelAv=noise_levelAv/12.0;
    lightAv=lightAv/12.0;
    String ForTrim="";
    // start to form a string to be sent to the cloud
    StrToSend+="V";
    StrToSend+=String(VOCconcAv ,2);
    StrToSend+="_T";
    StrToSend+=String(temp,1);
    StrToSend+="_H";
    StrToSend+=String(humd,1);
    StrToSend+="_CO2";
    StrToSend+=String(CO2concAv,0); 
    StrToSend+="_PM25";
    StrToSend+=String(PM25Av,1);
    StrToSend+="_PM10";
    StrToSend+=String(PM10Av,1);
    StrToSend+="_L";
    ForTrim=String(lightAv,0);
    ForTrim.trim();
    StrToSend+=ForTrim;
    StrToSend+="_N";
    StrToSend+=String(noise_levelAv,2);
    StrToSend+="_F";
    StrToSend+=String(eepromValue);
    Serial.println(StrToSend);

    cmdRun=cmd+" "+String(StrToSend.length());
    cmdRun=cmdRun+" "+StrToSend;
    //send data to the IoT hub by cURL POST request
    runCurl.runShellCommand(cmdRun);
    while(runCurl.running())
    {
      
    }
    Serial.print("HTTP server response: ");
    while (runCurl.available() > 0) {
      char c = runCurl.read();
      Serial.print(c);
      }
    Serial.println();  
    CO2concAv=0.0;
    PM25Av=0.0;
    PM10Av=0.0;
    VOCconcAv=0.0;
    noise_levelAv=0.0;
    lightAv=0.0;
    i=0;
  }

  if (Serial.available()>0)
  {
      byte SerialCommandByte=Serial.read();
      /*if (SerialCommandByte==0x63) // command to reset mcu
      {
          resetmcu.runShellCommand("/usr/bin/reset-mcu");
      }*/
      if (SerialCommandByte==0x23) // command to turn on the fan
      {
        //ledstate=!ledstate; //MT
        //ledstate = HIGH; //MT
        //digitalWrite(13, ledstate);
        digitalWrite(FanPin, LOW);
        ValToWrite=1;
        EEPROM.write(addr, ValToWrite);
        eepromValue=eepromValueRead();
      }
      if (SerialCommandByte==0x22)  //command to turn off the fan
      {
        //ledstate=!ledstate; //MT
        //ledstate = LOW; //MT
        //digitalWrite(13, ledstate);
        digitalWrite(FanPin, HIGH);
        ValToWrite=0;
        EEPROM.write(addr, ValToWrite);
        eepromValue=eepromValueRead();
      }
  }
      
}

int eepromValueRead() // function to read the value from EEPROM about fan state
{
  byte Value=EEPROM.read(0);
  return int(Value);
}


