// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include "DHT.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include "max32664.h"
#include <Adafruit_NeoPixel.h>

#define RESET_PIN 25
#define MFIO_PIN 24
#define RAWDATA_BUFFLEN 250

max32664 MAX32664(RESET_PIN, MFIO_PIN, RAWDATA_BUFFLEN);

#define DHTPIN 26     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11

DHT dht(DHTPIN, DHTTYPE);

float tempData[2] = {0,0};

SoftwareSerial HM10(27, 28); // RX = D27, TX = D28, RX on Sony Spresense -> TX on HM10, TX on Sony Spresense -> RX on HM10

char appData;  

String inData = "";

#define BUTTON 18

bool clockwise = true;
bool moving = false;
int counter = 0;
int previousPos = 0;

#define NEOPIXELPIN 17
#define NUMPIXELS 10

Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXELPIN, NEO_GRB + NEO_KHZ800);

void mfioInterruptHndlr(){
  //Serial.println("i");
}

void enableInterruptPin(){

  //pinMode(mfioPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MAX32664.mfioPin), mfioInterruptHndlr, FALLING);

}

void loadAlgomodeParameters(){

  algomodeInitialiser algoParameters;
  /*  Replace the predefined values with the calibration values taken with a reference spo2 device in a controlled environt.
      Please have a look here for more information, https://pdfserv.maximintegrated.com/en/an/an6921-measuring-blood-pressure-MAX32664D.pdf
      https://github.com/Protocentral/protocentral-pulse-express/blob/master/docs/SpO2-Measurement-Maxim-MAX32664-Sensor-Hub.pdf
  */

  algoParameters.calibValSys[0] = 120;
  algoParameters.calibValSys[1] = 122;
  algoParameters.calibValSys[2] = 125;

  algoParameters.calibValDia[0] = 80;
  algoParameters.calibValDia[1] = 81;
  algoParameters.calibValDia[2] = 82;

  algoParameters.spo2CalibCoefA = 1.5958422;
  algoParameters.spo2CalibCoefB = -34.659664;
  algoParameters.spo2CalibCoefC = 112.68987;

  MAX32664.loadAlgorithmParameters(&algoParameters);
}

void setup() {
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  
  Serial.begin(115200);

  dht.begin();

  HM10.begin(115200); // set HM10 serial at 115200 baud rate

  pinMode(16,INPUT);
  pinMode(17,INPUT);
  pinMode(19,INPUT);
  pinMode(18,INPUT);

  pinMode(LED0, OUTPUT); // onboard LED
  pinMode(LED1, OUTPUT); // onboard LED
  pinMode(LED2, OUTPUT); // onboard LED
  pinMode(LED3, OUTPUT); // onboard LED

  digitalWrite(LED0, LOW); // switch OFF LED
  digitalWrite(LED1, LOW); // switch OFF LED
  digitalWrite(LED2, LOW); // switch OFF LED
  digitalWrite(LED3, LOW); // switch OFF LED

  Wire.begin();

  loadAlgomodeParameters();

  int result = MAX32664.hubBegin();
  if (result == CMD_SUCCESS){
    Serial.println("Sensorhub begin!");
  }else{
    //stay here.
    while(1){
      Serial.println("Could not communicate with the sensor! please make proper connections");
      delay(5000);
    }
  }

  bool ret = MAX32664.startBPTcalibration();
  while(!ret){

    delay(10000);
    Serial.println("failed calib, please retsart");
    //ret = MAX32664.startBPTcalibration();
  }

  delay(1000);

  //Serial.println("start in estimation mode");
  ret = MAX32664.configAlgoInEstimationMode();
  while(!ret){

    //Serial.println("failed est mode");
    ret = MAX32664.configAlgoInEstimationMode();
    delay(10000);
  }

  //MAX32664.enableInterruptPin();
  Serial.println("Getting the device ready..");
  delay(1000);
}

void temp() {

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    tempData[0] = 0;
    tempData[1] = 0;
    return;
  }

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));

  tempData[0] = t;
  tempData[1] = h;

  return;
  
}

void loop() {

  pixels.clear(); // Set all pixel colors to 'off'

    for(int i=0; i<counter; i++) { // For each pixel...

      // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
      // Here we're using a moderately bright green color:
      pixels.setPixelColor(i, pixels.Color(255, 0, 0));

      pixels.show();   // Send the updated pixel colors to the hardware.

      delay(100);

  }

  if (digitalRead(21) == LOW && previousPos == 1) {
    clockwise = false;
    previousPos = 0;
    counter--;
  } else if (digitalRead(21) == LOW && previousPos == 2) {
    clockwise = true;
    previousPos = 0;
    counter++;
  }

  if (digitalRead(20) == LOW && previousPos == 0) {
    clockwise = true;
    previousPos = 1;
  } else if (digitalRead(20) == LOW && previousPos == 2) {
    clockwise = false;
    previousPos = 1;
  }

  if (digitalRead(19) == LOW && previousPos == 0) {
    clockwise = false;
    previousPos = 2;
    counter--;
  } else if (digitalRead(19) == LOW && previousPos == 1) {
    clockwise = true;
    previousPos = 2;
    counter++;
  }

  if (digitalRead(BUTTON) == HIGH) {
    inData = (String) counter;

    
  }

  HM10.listen();  // listen the HM10 port

  while (HM10.available() > 0) {   // if HM10 sends something then read

    appData = HM10.read();

    inData = String(appData);  // save the data in string format

    Serial.write(appData);

  }

  if ( inData == "0") {

    uint8_t num_samples = MAX32664.readSamples();
    temp();

    if(num_samples) {

      HM10.write(MAX32664.max32664Output.hr);
      HM10.write(",");
      HM10.write(MAX32664.max32664Output.spo2);
      HM10.write(",");
      HM10.write(MAX32664.max32664Output.sys);
      HM10.write(",");
      HM10.write(MAX32664.max32664Output.dia);
      HM10.write(",");
      HM10.write(tempData[0]);
      HM10.write(",");
      HM10.write(tempData[1]);
    
    
    }

    Serial.println("LED0 ON");
    digitalWrite(LED0, HIGH); // switch OFF LED
    delay(500);
    digitalWrite(LED0, LOW); // switch OFF LED
    Serial.println("LED0 OFF");
    

  } else if (inData == "1") {

    uint8_t num_samples = MAX32664.readSamples();

    if(num_samples) {

      HM10.write(MAX32664.max32664Output.hr);
    
    
    }

    Serial.println("LED1 ON");
    digitalWrite(LED1, HIGH); // switch OFF LED
    delay(500);
    digitalWrite(LED1, LOW); // switch OFF LED
    Serial.println("LED1 OFF");

    
    
  } else if (inData == "2") {

    uint8_t num_samples = MAX32664.readSamples();

    if(num_samples) {

      HM10.write(MAX32664.max32664Output.spo2);
    
    
    }

    Serial.println("LED2 ON");
    digitalWrite(LED2, HIGH); // switch OFF LED
    delay(500);
    digitalWrite(LED2, LOW); // switch OFF LED
    Serial.println("LED2 OFF");

    
    
  } else if (inData == "3") {

    uint8_t num_samples = MAX32664.readSamples();

    if(num_samples) {

      HM10.write(MAX32664.max32664Output.sys);
      HM10.write(",");
      HM10.write(MAX32664.max32664Output.dia);
    
    }

    Serial.println("LED3 ON");
    digitalWrite(LED3, HIGH); // switch OFF LED
    delay(500);
    digitalWrite(LED3, LOW); // switch OFF LED
    Serial.println("LED3 OFF");

    
    
  } else if (inData == "4") {

    temp();


     HM10.write(tempData[0]);
     HM10.write(",");
     HM10.write(tempData[1]);
    
    

    Serial.println("LED3 ON");
    digitalWrite(LED3, HIGH); // switch OFF LED
    delay(500);
    digitalWrite(LED3, LOW); // switch OFF LED
    Serial.println("LED3 OFF");
    
  } else {
    delay(500);
  }

  inData = "";
  
}
