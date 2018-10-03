/*    _   _ _ _____ _    _              _____     _ _     ___ ___  _  __
 *   /_\ | | |_   _| |_ (_)_ _  __ _ __|_   _|_ _| | |__ / __|   \| |/ /
 *  / _ \| | | | | | ' \| | ' \/ _` (_-< | |/ _` | | / / \__ \ |) | ' <
 * /_/ \_\_|_| |_| |_||_|_|_||_\__, /__/ |_|\__,_|_|_\_\ |___/___/|_|\_\
 *                             |___/
 *
 * Copyright 2018 AllThingsTalk
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * This experiment shows how LoRa can be used to monitor the quality of
 * your surrounding environment. Measure in- and outdoor air quality, noise
 * levels and temperature to provide stakeholders a dashboard to support
 * their decision making to improve quality of living.
 */
 
// Select your preferred method of sending data
//#define CONTAINERS
//#define CBOR
#define BINARY

/***************************************************************************/



#include <Wire.h>
#include "Seeed_BME280.h"
#include "MutichannelGasSensor.h"
#include "ATT_SDS011.h"
#include "SI114X.h"
#include <ATT_LoRaWAN.h>
#include "keys.h"
#include <MicrochipLoRaModem.h>

#define SERIAL_BAUD 57600

#define debugSerial Serial
#define loraSerial Serial1

#define SoundSensorPin A4


#define SEND_EVERY 300000

void callback(const unsigned char* payload, unsigned int length );

MicrochipLoRaModem modem(&loraSerial, &debugSerial, callback);
ATTDevice device(&modem, &debugSerial, false, 7000);  // minimum time between 2 messages set at 7000 milliseconds

#ifdef CONTAINERS
  #include <Container.h>
  Container container(device);
#endif

#ifdef CBOR
  #include <CborBuilder.h>
  CborBuilder payload(device);
#endif

#ifdef BINARY
  #include <PayloadBuilder.h>
  PayloadBuilder payload(device);
#endif


//Sensor declarations
ATT_SDS011 sds011(20,21);

SI114X SI1145 = SI114X();

BME280 tph; // I2C

int soundValue;
float temp;
float hum;
float pres;
int Vis;
int IR;
float UV;
float carbonmonoxide;
float nitrogendioxide;
float hydrogen;
float ammonia;
float methane;
float propane;
float butane;
float ethanol;

void setup() 
{
  //pinMode(GROVEPWR, OUTPUT);  // turn on the power for the secondary row of grove connectors
  //digitalWrite(GROVEPWR, HIGH);

  debugSerial.begin(SERIAL_BAUD);
  while((!debugSerial) && (millis()) < 10000){}  // wait until the serial bus is available

  loraSerial.begin(modem.getDefaultBaudRate());  // set baud rate of the serial connection to match the modem
  while((!loraSerial) && (millis()) < 10000){}   // wait until the serial bus is available

  while(!device.initABP(DEV_ADDR, APPSKEY, NWKSKEY))

  delay(2000);
  
  // init SF to 7
  modem.setLoRaSF(7);
    
  debugSerial.println("Ready to send data");
  debugSerial.println();
  debugSerial.println("-- Environmental Sensing LoRa experiment --");
  debugSerial.println();

  initSensors();
}

void loop() 
{
  readSensors();
  displaySensorValues();
  sendSensorValues();
  
  debugSerial.print("Delay for: ");
  debugSerial.println(SEND_EVERY);
  debugSerial.println();
  delay(SEND_EVERY);
}


void initSensors()
{
  debugSerial.println("Initializing sensors, this can take a few seconds...");
  
  pinMode(SoundSensorPin, INPUT);
  
  tph.init();
  gas.begin(0x04);//the default I2C address of the slave is 0x04
  gas.powerOn();
  


  debugSerial.print("Firmware Version = ");
  debugSerial.println(gas.getVersion());
  debugSerial.println("Done");

  while (!SI1145.Begin()) {
     debugSerial.println("Si1145 is not ready!");
     delay(1000);
     }
}

void readSensors()
{
    debugSerial.println("Start reading sensors");
    debugSerial.println("---------------------");
    
    soundValue = analogRead(SoundSensorPin);
    
    temp = tph.getTemperature();
    hum = tph.getHumidity();
    pres = tph.getPressure()/100.0;

    
    // sunlight values

    Vis = SI1145.ReadVisible();
    IR = SI1145.ReadIR();
    UV = SI1145.ReadUV();
        

   // Air quality values
   
    ammonia = gas.measure_NH3();
    carbonmonoxide = gas.measure_CO();
    nitrogendioxide= gas.measure_NO2();
    propane = gas.measure_C3H8();
    butane = gas.measure_C4H10();
    methane = gas.measure_CH4();
    hydrogen = gas.measure_H2();
    ethanol = gas.measure_C2H5OH();

    Measurements(10);
}

#define MEASUREMENTS 30

//Global vars
typedef struct intMeasurement
{
  int dataBuffer[MEASUREMENTS];
  int average;
}intMeasurement_t;

typedef struct floatMeasurement
{
  float dataBuffer[MEASUREMENTS];
  float average;
} floatMeasurement_t;

intMeasurement_t pm25;
intMeasurement_t pm10;
byte numberOfMeasurementsDone = 0;

//void Measurements(int numberOfMeasurements, sensors_t* sensors)

void Measurements(int numberOfMeasurements )
{    
    //Start the measurements
    sds011.startMeasurement();
    
    //take # measurements
    while(numberOfMeasurementsDone < numberOfMeasurements)
    {
      //SDS011 if packet is valid we can do the other measurements to
      if(sds011.getMeasurement() == packetValid)
      {
         pm25.dataBuffer[numberOfMeasurementsDone] = sds011.pm25;
         pm10.dataBuffer[numberOfMeasurementsDone] = sds011.pm10;                
         numberOfMeasurementsDone++;
      }

    }    

    //stop the measurements
    sds011.stopMeasurement();

    //calculate average    
    pm25.average = 0;
    pm10.average = 0;
    
    for(int i=0; i < numberOfMeasurements; i++)
    {
      pm25.average += pm25.dataBuffer[i];
      pm10.average += pm10.dataBuffer[i];
    }

    pm25.average /= numberOfMeasurements;
    pm10.average /= numberOfMeasurements;

      
    numberOfMeasurementsDone = 0;
}

void process()
{
  while(device.processQueue() > 0)
  {
    debugSerial.print("QueueCount: ");
    debugSerial.println(device.queueCount());
    delay(10000);
  }
}

void sendSensorValues()
{
  #ifdef CONTAINERS
  debugSerial.println("Start sending data to the ATT cloud platform");
  debugSerial.println("--------------------------------------------");
  container.addToQueue(soundValue, LOUDNESS_SENSOR, false); process();
  container.addToQueue(temp, TEMPERATURE_SENSOR, false); process();
  container.addToQueue(hum, HUMIDITY_SENSOR, false); process();
  container.addToQueue(pres, PRESSURE_SENSOR, false); process();
  #endif

  #ifdef CBOR
  payload.reset();
  payload.map(5);
  payload.addNumber(soundValue, "S");
  payload.addNumber(temp, "T");
  payload.addNumber(hum, "H");
  payload.addNumber(pres, "P");
  payload.addToQueue(false);
  process();
  #endif

  #ifdef BINARY
  payload.reset();
  payload.addInteger(soundValue);
  payload.addNumber(temp);
  payload.addNumber(hum);
  payload.addNumber(pres);
  payload.addInteger(Vis);
  payload.addInteger(IR);
  payload.addInteger(UV);
  payload.addNumber(carbonmonoxide);
  payload.addNumber(nitrogendioxide);
  payload.addNumber(pm25.average);
  payload.addNumber(pm10.average);
  payload.addToQueue(false);
  process();
  #endif
}

void displaySensorValues()
{
  debugSerial.print("Sound level: ");
  debugSerial.print(soundValue);
  debugSerial.println(" Analog (0-1023)");
      
  debugSerial.print("Temperature: ");
  debugSerial.print(temp);
  debugSerial.println(" °C");
      
  debugSerial.print("Humidity: ");
  debugSerial.print(hum);
	debugSerial.println(" %");
      
  debugSerial.print("Pressure: ");
  debugSerial.print(pres);
	debugSerial.println(" hPa");

  debugSerial.print("Visible Light: ");
  debugSerial.print(Vis);
  debugSerial.println(" lm");

  debugSerial.print("IR Light: ");
  debugSerial.print(IR);
  debugSerial.println(" lm");

  debugSerial.print("UV index: ");
  debugSerial.print(float(UV/100));
  debugSerial.println(" ");

  debugSerial.print("The concentration of CO is ");
  if(carbonmonoxide>=0) debugSerial.print(carbonmonoxide);
  else debugSerial.print("invalid");
  debugSerial.println(" ppm");

  debugSerial.print("The concentration of NO2 is ");
  if(nitrogendioxide >=0) debugSerial.print(nitrogendioxide);
  else debugSerial.print("invalid");
  debugSerial.println(" ppm");

  debugSerial.print("The concentration of H2 is ");
  if(hydrogen >=0) debugSerial.print(hydrogen);
  else debugSerial.print("invalid");
  debugSerial.println(" ppm");

  debugSerial.print("The concentration of NH3 is ");
  if(ammonia >=0) debugSerial.print(ammonia);
  else debugSerial.print("invalid");
  debugSerial.println(" ppm");

  debugSerial.print("The concentration of CH4 is ");
  if(methane >=0) debugSerial.print(methane);
  else debugSerial.print("invalid");
  debugSerial.println(" ppm");
  
  debugSerial.print("The concentration of C3H8 is ");
  if(propane >=0) debugSerial.print(propane);
  else debugSerial.print("invalid");
  debugSerial.println(" ppm");
  
  debugSerial.print("The concentration of C4H10 is ");
  if(butane >=0) debugSerial.print(butane);
  else debugSerial.print("invalid");
  debugSerial.println(" ppm");
  
  debugSerial.print("The concentration of C2H5OH is ");
  if(ethanol >=0) debugSerial.print(ethanol);
  else debugSerial.print("invalid");
  debugSerial.println(" ppm");

  debugSerial.println("Average pm2.5 = " + String(pm25.average));
  debugSerial.println("Average pm10 = " + String(pm10.average));  
 
}

// callback function
// handle messages that were sent from the AllThingsTalk cloud to this device
void callback(const unsigned char* payload, unsigned int length) 
{
   debugSerial.println("========================================================");
   debugSerial.println("========================CALLBACK!!!=====================");
   debugSerial.println("========================================================");
   String msgString;
  
   char message_buff[length + 1];
   strncpy(message_buff, (char*)payload, length);
   message_buff[length] = '\0';
   msgString = String(message_buff);
   
   debugSerial.print("Payload: ");
   debugSerial.println(msgString);

   if (msgString.substring(2,3) == "s") {
      Serial.println("actuator s!");
      debugSerial.println(msgString.substring(4,6).toInt());
      if (msgString.substring(3,4) == "0x1A") {
         debugSerial.println(msgString.substring(4,8).toInt());
         }
      else if (msgString.substring(3,4) == "0x19") {
         debugSerial.println(msgString.substring(4,6).toInt());
         }   
      }

}
