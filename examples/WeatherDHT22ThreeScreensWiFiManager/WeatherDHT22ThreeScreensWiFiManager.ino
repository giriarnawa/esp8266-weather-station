/**The MIT License (MIT)
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Author Ken Taylor
Derived from software by Daniel Eichhorn http://blog.squix.ch
Derived from software by Jason Chen

This demo includes WiFiManager so that you can set up WiFi configuration without 
configuring the WiFi credentials in code and recompiling. WiFi credentials are 
entered using a web browser. You must use the version of WiFiManager from 
https://github.com/kentaylor/WiFiManager.

Add your own configuration parameters at each location labelled "Your secret" with values from the service providers. 
The configuration parameters are obtained from Weather Underground and ThingSpeak. Add your own configuration 
parameters at each location labelled "Yours" to match your location. Parameters labelled as "hardware dependant" vary 
according to how your wiring is configured. Parameters labelled as "configurable" can be varied to change device behaviour.
*/
#include "DHT.h" //https://github.com/adafruit/DHT-sensor-library 
#include <ESP8266WiFi.h>
#include "OneButton.h"
#include "Ticker.h"
#include <JsonListener.h>
#include "SSD1306.h"
#include "SSD1306Ui.h"
#include "Wire.h"
#include "WundergroundClient.h"
#include "WeatherStationFonts.h"
#include "WeatherStationImages.h"
#include "TimeClient.h"
#include "ThingspeakClient.h"
#include <OneWire.h> // Used for talking to DS18B20 temp sensor. 
// Include the libraries needed for DS18B20 temperature measurement from https://github.com/milesburton/Arduino-Temperature-Control-Library
#include <DallasTemperature.h> 
#include <WiFiManager.h>          //https://github.com/kentaylor/WiFiManager Must be this version.
#include <BMP280.h>              //https://github.com/mahfuz195/BMP280_Arduino_Library
/***************************
 * Begin Settings
 **************************/
/*Trigger for inititating config mode is Pin D3 and also flash button on NodeMCU
 * Flash button is convenient to use but if it is pressed it will stuff up the serial port device driver 
 * until the computer is rebooted on windows machines.
 */
const int TRIGGER_PIN = D6; // Trigger for putting up a configuration portal. Wake up pin for deep sleep mode NodeMCU and WeMos. Hardware dependant.
bool ConfigurationPortalRequired = false;
bool ConfigurationPortalQuestionRequired = false;
bool NexrScreen = false;

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define DHTPIN D2     // what digital pin we're connected to. Wemos and NodeMCU has a 10K pullup on D2 which removes need for an additional pullup resistor. Hardware dependant.
#define ONE_WIRE_BUS D4 // GPIO pin which DS18B20 is plugged into. Wemos and NodeMCU has a 10K pullup on D4 which removes need for an additional pullup resistor. Hardware dependant.
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature DS18B20sensors(&oneWire);

// arrays to hold device address
DeviceAddress Thermometer;

WiFiClient client;


// Setup
const int UPDATE_INTERVAL_SECS = 10 * 60; // Get data from weather underground every 10 minutes. Configurable.
const int SensorReadInterval = 15000; // Read sensors every 15 seconds. Configurable.

// Display Settings
const int I2C_DISPLAY_ADDRESS = 0x3c;
const int SDA_PIN = D3; //Hardware dependant.
const int SDC_PIN = D5; //Hardware dependant.

// TimeClient settings
const float UTC_OFFSET = 10; //Yours. Offset for your time zone.

// Wunderground Settings
const boolean IS_METRIC = true;
const String WUNDERGRROUND_API_KEY = ""; //Your secret
const String WUNDERGRROUND_LANGUAGE = "EN";  //Yours.
const String WUNDERGROUND_COUNTRY = "AU";  //Yours.
const String WUNDERGROUND_CITY = "Canberra";  //Yours.

//Thingspeak Settings
const String THINGSPEAK_CHANNEL_ID = ""; //Your secret
const String THINGSPEAK_API_READ_KEY = ""; //Your secret

void ConfigSavedScreen(); //Screen after WiFi config has been updated.

bool drawFrame1(SSD1306 *display, SSD1306UiState* state, int x, int y);
bool drawFrame2(SSD1306 *display, SSD1306UiState* state, int x, int y);
bool drawFrame3(SSD1306 *display, SSD1306UiState* state, int x, int y);
bool drawFrame4(SSD1306 *display, SSD1306UiState* state, int x, int y);

void updateData(SSD1306 *display);
void setReadyForWeatherUpdate();
void drawProgress(SSD1306 *display, int percentage, String label);
void drawForecast(SSD1306 *display, int x, int y, int dayIndex);
void readSensors();
void checkButton();
void ConfigurationPortalQuestion(SSD1306 *display);
void ConfigurationPortalQuestionFlag();
void NextScreenFlag(); 
void ConfigurationPortalFlag();
void updateThingSpeak(String);


// Initialize the oled display for address 0x3c
SSD1306   display(I2C_DISPLAY_ADDRESS, SDA_PIN, SDC_PIN);
SSD1306Ui ui     ( &display );
DHT dht(DHTPIN, DHTTYPE);

// Need to create an SFE_BMP180 object, here called "pressure":
BMP280 bmp;
#define ALTITUDE 8.0 // Yours. Altitude of belconnen in meters 

TimeClient timeClient(UTC_OFFSET);

// Set to false, if you prefere imperial/inches, Fahrenheit
WundergroundClient wunderground(IS_METRIC); //Yours

ThingspeakClient thingspeak;

// ThingSpeak Settings
char thingSpeakAddress[] = "api.thingspeak.com";
String writeAPIKey = ""; //Your secret
const int updateThingSpeakInterval = 120 * 1000;      // Time interval in milliseconds to update ThingSpeak (number of seconds * 1000 = interval)
#define MAXIMUM_REPORTING_INTERVAL 15*60*1000 //Maximum interval between Thingspeak uploads. (number of seconds * 1000 = interval)
#define MINIMUM_REPORTING_INTERVAL 2*60*1000  //Miniimum interval between Thingspeak uploads. (number of seconds * 1000 = interval)
// Temperature difference to trigger logging new data to the server
#define TEMPERATURE_TRIGGER 0.25

// this array keeps function pointers to all frames
// frames are the single views that slide from right to left
bool (*frames[])(SSD1306 *display, SSD1306UiState* state, int x, int y) = {  drawFrame2, drawFrame3, drawFrame4 };
int numberOfFrames = 3;

// flag changed in the ticker function every 10 minutes
bool readyForWeatherUpdate = false;

String lastUpdate = "--";
String localTemperature = "n/a";
String localHumidity = "n/a";
struct sensor {
  float value = NAN; //Value from last successful read
  boolean current = false; // Was last read successful
  byte invalidReadsCntr = 0; //When there is too many consecutive misreads mark sensor offline
  boolean online = false; //Assume sensors are not online until proved otherwise. Sensors are read even when offline in case they come back.
};
sensor DS18B20Sensor;
sensor DHThumiditySensor;
sensor DHTtemperatureSensor;
sensor BMPtemperatureSensor;
sensor BMPseaLevelPressureSensor;

unsigned long SensorReadTime = millis()-3000; //Do the first read 3 seconds after startup

// Setup a new OneButton on pin TRIGGER_PIN. Hardware dependant.
OneButton button(TRIGGER_PIN, true);
Ticker buttonTicker;
Ticker wundergroundTicker;
/***************************
 * End Settings
 **************************/

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");    
  pinMode(TRIGGER_PIN, INPUT_PULLUP); // Set up WiFi configuration button. Hardware dependant.
  if (WiFi.SSID()==""){
    Serial.println("We haven't got any access point credentials, so get them now");   
    ConfigurationPortalRequired = true;
  }
  else{
    WiFi.mode(WIFI_STA); // Force to station mode because if device was switched off while in access point mode it will start up next time in access point mode.
  }
  dht.begin();
  //Start the pressure sensor
  if(!bmp.begin(SDA_PIN,SDC_PIN)){
        Serial.println("BMP init failed!");
    while(1);
  }
  else Serial.println("BMP init success!");
    bmp.setOversampling(4);
  // locate DS18B20 devices on the bus. 
  DS18B20sensors.begin();
  DS18B20sensors.setResolution(12);
  int SensorCount = DS18B20sensors.getDeviceCount(); 
  DS18B20sensors.requestTemperatures(); // Send the command to get temperatures leave as 12 bit by default
  Serial.print("Found ");
  Serial.print(SensorCount);
  Serial.println(" DS18B20 temperature sensors.");
  for (int i =0; i<SensorCount; i++)
    {
    float temp = DS18B20sensors.getTempCByIndex(i);
    Serial.print("Temperature ");
    Serial.print(i);
    Serial.print(" ");
    Serial.println(temp);
    }
  DS18B20sensors.setWaitForConversion(FALSE); //This must be used with caution. See comments on this command in Dallas Temperature.cpp
  /*  DS18B20sensors.requestTemperatures() will become a non blocking call but it will take
   *  about 750ms for the temp sensor to complete the reading. DS18B20sensors.getTempCByIndex should not be called until the sensor
   *  has completed taking the reading i.e wait at least 1 second.
   */
   
  // initialize display
  display.init();
  display.clear();
  display.display();

  //display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setContrast(255);

  // link the ConfigurationPortal functions to button events.   
  button.attachClick(ConfigurationPortalFlag); //a button press starts a configuration portal wile connecting to WiFi
  button.attachDuringLongPress(ConfigurationPortalQuestionFlag); //long press always starts a configuration portal
  //Start timer to detect click events
  buttonTicker.attach(0.05, checkButton);
  
  //Wait for WiFi to get connected
  int counter = 0;
  unsigned long startedAt = millis();
  int status = WL_DISCONNECTED;
  while ((status != WL_CONNECTED) && (ConfigurationPortalRequired == false) && (ConfigurationPortalQuestionRequired == false) && (counter < 600)) { //Move on after 5 minutes if connection fails.
    status = WiFi.status();
    Serial.print(status);
    Serial.print(" . ");
    Serial.println(counter); 
    display.clear();
    switch (status)
     {
      // WL_NO_SHIELD 255
       //WL_SCAN_COMPLETED 2
       //WL_CONNECTION_LOST 5
      case WL_IDLE_STATUS: { //0
        display.drawString(64, 10, "Device WiFi Failed");
        String text = WiFi.SSID();
        text = text + " not visible";
        display.drawString(64, 40,text);
        display.drawString(64, 54, "Push button to configure WiFi");
        break;
      }
      case WL_DISCONNECTED: {  // 6
        display.drawString(64, 10, "Connecting to WiFi");
        String text = WiFi.SSID();
        if (counter>60) {
          display.drawString(64, 40,text);
          display.drawString(64, 54, "Push button to configure");
        }
        else {
          display.drawString(64, 45,text);
          }
        break;
        }
      case WL_NO_SSID_AVAIL: { // 1
        display.drawString(64, 10, "Connecting to WiFi");
        String text = WiFi.SSID();
        text = text + " not visible";
        display.drawString(64, 40,text);
        display.drawString(64, 54, "Push button to configure WiFi");
        break;
      }     
      case WL_CONNECTED: break; // 3
      case WL_CONNECT_FAILED: { //4
        display.drawString(64, 10, "Connecting to WiFi");
        display.drawString(64, 40, "Connect failed.");
        display.drawString(64, 54, "Try restarting");
        break;
      }
      default: display.drawString(64, 10, "Connecting to WiFi");
    }   
    display.drawXbm(46, 30, 8, 8, counter % 3 == 0 ? activeSymbole : inactiveSymbole);
    display.drawXbm(60, 30, 8, 8, counter % 3 == 1 ? activeSymbole : inactiveSymbole);
    display.drawXbm(74, 30, 8, 8, counter % 3 == 2 ? activeSymbole : inactiveSymbole);
    display.display(); 
    counter++;
    delay(500); 
  }
  Serial.print("After waiting ");
  float waited = (millis()- startedAt); 
  Serial.print(waited/1000);
  Serial.print(" secs in setup() connection result is ");
  Serial.println(WiFi.status());
    // Set the ConfigurationPortal function to be called on a LongPress event.   
  //This callback will fire every tick so to avoid simultaneous instances don't do much there 
  button.attachClick(NextScreenFlag); //Button click no longer will start a configuration portal.
  ui.setTargetFPS(30);
  ui.setActiveSymbole(activeSymbole);
  ui.setInactiveSymbole(inactiveSymbole);
  // You can change this to
  // TOP, LEFT, BOTTOM, RIGHT
  ui.setIndicatorPosition(BOTTOM);

  // Defines where the first frame is located in the bar.
  ui.setIndicatorDirection(LEFT_RIGHT);

  // You can change the transition that is used
  // SLIDE_LEFT, SLIDE_RIGHT, SLIDE_TOP, SLIDE_DOWN
  ui.setFrameAnimation(SLIDE_LEFT);

  // Add frames
  ui.setFrames(frames, numberOfFrames);
  ui.setTimePerFrame(6000); //6 seconds 
  // Inital UI takes care of initalising the display too.
  ui.init();

  Serial.println("");
  if (!ConfigurationPortalRequired) updateData(&display);  //Jump past display update if ConfigurationPortalRequired
  wundergroundTicker.attach(UPDATE_INTERVAL_SECS, setReadyForWeatherUpdate);
  //display.flipScreenVertically();
}

void loop() {
    if (ConfigurationPortalQuestionRequired) {
     ConfigurationPortalQuestion(&display);
  }
  if (ConfigurationPortalRequired) {
     Serial.println("Configuration portal requested.");
     display.clear();
     display.setTextAlignment(TEXT_ALIGN_CENTER);
     display.setFont(ArialMT_Plain_10);
     display.drawString(64, 5, "To configure WiFi. Go to");
     display.drawString(64, 20, "http://192.168.4.1 after");
     display.drawString(64, 35, "connecting computer to");
     String text = "ESP" + String(ESP.getChipId());
     text = text + " Wifi Network";
     display.drawString(64, 50, text);    
     display.display();
     //Local intialization. Once its business is done, there is no need to keep it around
     WiFiManager wifiManager;

    //sets timeout in seconds until configuration portal gets turned off.
    //If not specified device will remain in configuration mode until
    //switched off via webserver or device is restarted.
    //wifiManager.setConfigPortalTimeout(600);
    wifiManager.setSaveConfigCallback(* ConfigSavedScreen);
    //it starts an access point 
    //and goes into a blocking loop awaiting configuration
    if (!wifiManager.startConfigPortal()) {
      Serial.println("Not connected to WiFi but continuing anyway.");
    } else {
      //if you get here you have connected to the WiFi
      Serial.println("connected...yeey :)");
    }
    ConfigurationPortalRequired = false;
    ConfigurationPortalQuestionRequired = false;
    ESP.reset(); // This is a bit crude. For some unknown reason webserver can only be started once per boot up 
    // so resetting the device allows to go back into config mode again when it reboots.
    delay(5000);
  }
  
  if (readyForWeatherUpdate && ui.getUiState().frameState == FIXED) {
    updateData(&display);
    Serial.println("update display");   
  }
  int remainingTimeBudget = ui.update(); 
  //If less than remaining time budget which is a few milliseconds then animations will be smooth.
  //if ( remainingTimeBudget>0 ) {
    // You can do some work here
    // Don't do stuff if you are below your
  // time budget and you want smooth animation.}
  static float lastTemperatureUploaded = 1000;
  float temperatureDiff = 0; // Used to decide if temperature has changed enough to upload data to Thingspeak
  bool validTemperatureSensor = true;
  unsigned long TimeInterval = abs( millis() - SensorReadTime); //Will go huge when milis counter rolls over
  if (TimeInterval>SensorReadInterval ) { 
    readSensors();
    // Update humidity on screen
    if (DHThumiditySensor.current == true) { //DHT sensor misreads sometimes, leave unchanged unless last read was valid.
      localHumidity = String(DHThumiditySensor.value,0); //display precision is integers only.  
    }
    else {
      if (DHThumiditySensor.online == false) localHumidity = "n/a";
    }
    // Try all temperature sensors in preference order for temperature for upload test and temperature on screen. 
    float PreferredLocalTemperature = 2000;
    if (DS18B20Sensor.current == true) {
      PreferredLocalTemperature = DS18B20Sensor.value;  
    }
    else if (DHTtemperatureSensor.current == true) {
      PreferredLocalTemperature = DHTtemperatureSensor.value;       
    }
    else if (BMPtemperatureSensor.current == true) {
      PreferredLocalTemperature = BMPtemperatureSensor.value;      
    }
    if ((DS18B20Sensor.online == false) && (DHTtemperatureSensor.online == false) && (BMPtemperatureSensor.online == false)) {
      validTemperatureSensor = false;
      localTemperature = "n/a"; // If all temperature sensors are offline mark not available
    }
    else{
      temperatureDiff = PreferredLocalTemperature - lastTemperatureUploaded;
      if (temperatureDiff < 0) temperatureDiff = - temperatureDiff; //Make absolute so that upload occurs on increasing or decreasing temperatures
      String tt(PreferredLocalTemperature, 1);
      localTemperature = tt;
    }

    static long lastUploadAttempt = millis(); //First upload to thingspeeak will be sampling interval after bootup.
    if (((abs((millis()-lastUploadAttempt)) > MINIMUM_REPORTING_INTERVAL) && ((temperatureDiff > TEMPERATURE_TRIGGER)|| (!validTemperatureSensor))) || (abs((millis()-lastUploadAttempt)) > MAXIMUM_REPORTING_INTERVAL)) { // Prepare data to upload to thingspeak
      
      //Prepare data to be uploaded
      String uploadDHTtemperature = "NAN";
      String uploadHumidity = "NAN";
      String uploadDS18B20 = "NAN";
      String uploadBMPseaLevelPressure = "NAN";
      String uploadBMPtemperatureSensor = "NAN";
      
      if (DHTtemperatureSensor.online == true) {
        String t1(DHTtemperatureSensor.value, 1); // upload resolution precision is 1 decimal place 
        uploadDHTtemperature = t1;      
      }
      if (DHThumiditySensor.online == true) {
        String t2(DHThumiditySensor.value, 1); // upload resolution precision is 1 decimal place
        uploadHumidity = t2;       
      }
      if (DS18B20Sensor.online == true) {
        String t3(DS18B20Sensor.value, 1); // upload resolution precision is 1 decimal place 
        uploadDS18B20 = t3;      
      }
      if (BMPseaLevelPressureSensor.online == true) {
        String t4(BMPseaLevelPressureSensor.value, 1); // upload resolution precision is 1 decimal place  
        uploadBMPseaLevelPressure = t4;     
      }
      if (BMPtemperatureSensor.online == true) {
        String t5(BMPtemperatureSensor.value, 1); // upload resolution precision is 1 decimal place 
        uploadBMPtemperatureSensor = t5;      
      }     
      if ((abs((millis()-lastUploadAttempt)) > MINIMUM_REPORTING_INTERVAL) && (DHTtemperatureSensor.online == false)||(DHTtemperatureSensor.current == true) && (DHThumiditySensor.online == false)||(DHThumiditySensor.current == true)
      && (DS18B20Sensor.online == false)||(DS18B20Sensor.current == true) && (BMPseaLevelPressureSensor.online == false)||(BMPseaLevelPressureSensor.current == true) //If data is good and more than MINIMUM_REPORTING_INTERVAL upload.
      && (BMPseaLevelPressureSensor.online == false)||(BMPseaLevelPressureSensor.current == true)) { // Upload data to thingspeak
        Serial.println("upload temperature/humidity/pressure to thingspeak");
        lastUploadAttempt = millis(); // Only try once per minimum upload interval even if upload fails
        updateThingSpeak("field1="+uploadDHTtemperature+"&field2="+uploadHumidity+"&field3="+uploadDS18B20+"&field4="+uploadBMPseaLevelPressure+"&field5="+uploadBMPtemperatureSensor);  
        if (validTemperatureSensor) lastTemperatureUploaded =  PreferredLocalTemperature;
      } 
    }           
  }
  delay(500); // Make this delay less to get a sideways scrolling display, longer for a less precise screen update time  
}

void ConfigSavedScreen(){
   Serial.println("Configuration updated.");
   display.clear();
   display.setTextAlignment(TEXT_ALIGN_CENTER);
   display.setFont(ArialMT_Plain_10);
   display.drawString(64, 5,  "WiFi configured. Reboot");
   display.drawString(64, 20, "or select exit at");
   display.drawString(64, 35, "http://192.168.4.1 on");
   String text = "ESP" + String(ESP.getChipId());
   text = text + " Wifi Network";
   display.drawString(64, 50, text);    
   display.display();
}

void updateData(SSD1306 *display) {
    
  //display->flipScreenVertically();
  drawProgress(display, 10, "Updating time...");
  timeClient.updateTime();
  drawProgress(display, 30, "Updating conditions...");
  wunderground.updateConditions(WUNDERGRROUND_API_KEY, WUNDERGRROUND_LANGUAGE, WUNDERGROUND_COUNTRY, WUNDERGROUND_CITY);
  drawProgress(display, 50, "Updating forecasts...");
  wunderground.updateForecast(WUNDERGRROUND_API_KEY, WUNDERGRROUND_LANGUAGE, WUNDERGROUND_COUNTRY, WUNDERGROUND_CITY);
  drawProgress(display, 80, "Updating thingspeak...");
  thingspeak.getLastChannelItem(THINGSPEAK_CHANNEL_ID, THINGSPEAK_API_READ_KEY);
  //Serial.println(thingspeak.getFieldValue(0));
  lastUpdate = timeClient.getFormattedTime();
  readyForWeatherUpdate = false;
  drawProgress(display, 100, "Done...");
  delay(1000);
}

void drawProgress(SSD1306 *display, int percentage, String label) {
  display->clear();
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_10);
  display->drawString(64, 10, label);
  display->drawRect(10, 28, 108, 12);
  display->fillRect(12, 30, 104 * percentage / 100 , 9);
  display->display();
}

// Date and time only
bool drawFrame1(SSD1306 *display, SSD1306UiState* state, int x, int y) {
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_10);
  String date = wunderground.getDate();
  int textWidth = display->getStringWidth(date);
  display->drawString(64 + x, 10 + y, date);
  display->setFont(ArialMT_Plain_24);
  String time = timeClient.getFormattedTime();
  textWidth = display->getStringWidth(time);
  display->drawString(64 + x, 20 + y, time);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
}
//Current temperatures. Outside and inside.
bool drawFrame3(SSD1306 *display, SSD1306UiState* state, int x, int y) {
  display->setFont(ArialMT_Plain_10);
  String text = wunderground.getWeatherText();
  int space = text.indexOf(" "); //Show only first word and rely on picture to communicated following words.
  //For example if the phrase is scattered cloud the picture will be of clouds so scattered is enough.
  if (space>0) text.remove(space);
  display->drawString(15 + x - text.length()/2, 5 + y, text);

  display->setFont(ArialMT_Plain_24);
  text = wunderground.getCurrentTemp() + "°";
  display->drawString(60 + x, 5 + y, text);
  int textWidth = display->getStringWidth(text);
  display->setFont(ArialMT_Plain_16);
  display->drawString(60 + x + textWidth, 5 + y + 8, "C");

   display->setFont(ArialMT_Plain_24);
  text = localTemperature + "°";
  display->drawString(60 + x, 30 + y, text);
  textWidth = display->getStringWidth(text);
  display->setFont(ArialMT_Plain_16);
  display->drawString(60 + x+ textWidth, 30 + y + 8, "C");
  
  display->setFont(Meteocons_0_42);
  String weatherIcon = wunderground.getTodayIcon();
  int weatherIconWidth = display->getStringWidth(weatherIcon);
  display->drawString(30 + x - weatherIconWidth / 2, 20 + y, weatherIcon);
}
//Current humidities. Outside and inside.
bool drawFrame4(SSD1306 *display, SSD1306UiState* state, int x, int y) {
  display->setFont(ArialMT_Plain_10);
  display->drawString(5 + x, 25 + y, "Humidity");
  display->drawString(23 + x, 40 + y, "In");
  String rain = wunderground.getPrecipitation1Hour();
  String humid = wunderground.getHumidity();
  rain.remove(rain.length()-2);
  float rainNumber = rain.toFloat();
  display->setFont(ArialMT_Plain_10);
  int textWidth;
  if (rainNumber>0) { //If rain in last hour display rain otherwise humidity
    display->drawString(18 + x, y, "Rain");
    display->drawString(3 + x, 11 + y, "Last Hour");
    display->setFont(ArialMT_Plain_24);
    display->drawString(60 + x, 5 + y, rain);
    textWidth = display->getStringWidth(rain);
    display->setFont(ArialMT_Plain_10);
    display->drawString(60 + x + textWidth, 5 + y + 11, " mm"); 
     
  }
  else {
    display->drawString(20 + x, 10 + y, "Out");
    display->setFont(ArialMT_Plain_24);
    humid.remove((humid.length()-1));
    display->drawString(60 + x, 5 + y, humid);
    textWidth = display->getStringWidth(humid);
    display->setFont(ArialMT_Plain_16);
    display->drawString(60 + x + textWidth, 5 + y + 8, " %");
  }

   display->setFont(ArialMT_Plain_24);
   humid = localHumidity;
  display->drawString(60 + x, 30 + y, humid);
  textWidth = display->getStringWidth(humid);
  display->setFont(ArialMT_Plain_16);
  display->drawString(60 + x + textWidth, 30 + y + 8, " %");
}

// 3 day forecast
bool drawFrame2(SSD1306 *display, SSD1306UiState* state, int x, int y) {
  drawForecast(display, x, y, 0);
  drawForecast(display, x + 44, y, 2);
  drawForecast(display, x + 88, y, 4);
}

void drawForecast(SSD1306 *display, int x, int y, int dayIndex) {
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_10);
  String day = wunderground.getForecastTitle(dayIndex).substring(0, 3);
  day.toUpperCase();
  display->drawString(x + 20, y, day);

  display->setFont(Meteocons_0_21);
  display->drawString(x + 20, y + 15, wunderground.getForecastIcon(dayIndex));

  display->setFont(ArialMT_Plain_16);
  display->drawString(x + 20, y + 37, wunderground.getForecastLowTemp(dayIndex) + "/" + wunderground.getForecastHighTemp(dayIndex));
  display->setTextAlignment(TEXT_ALIGN_LEFT);
}

void setReadyForWeatherUpdate() {
  Serial.println("Setting readyForUpdate to true");
  readyForWeatherUpdate = true;
}

void readSensors(){
    SensorReadTime = millis();
    float humidity = dht.readHumidity();     
    if (humidity>0) { //Will be nan if no sensor response or read errot. Will get occasional read errors
      DHThumiditySensor.value  = humidity; 
      DHThumiditySensor.current = true;
      DHThumiditySensor.invalidReadsCntr = 0;
      DHThumiditySensor.online = true;
    }
    else {
      DHThumiditySensor.current = false;
      DHThumiditySensor.invalidReadsCntr = DHThumiditySensor.invalidReadsCntr + 1;
      if (DHThumiditySensor.invalidReadsCntr > 3) DHThumiditySensor.online = false;
    }
    float temperatureDHT = dht.readTemperature(); 
    if (temperatureDHT>0) { //Will be nan if no sensor response or read errot. Will get occasional read errors
      DHTtemperatureSensor.value  = temperatureDHT; 
      DHTtemperatureSensor.current = true;
      DHTtemperatureSensor.invalidReadsCntr = 0;
      DHTtemperatureSensor.online = true;
    }
    else {
      DHTtemperatureSensor.current = false;
      DHTtemperatureSensor.invalidReadsCntr = DHTtemperatureSensor.invalidReadsCntr + 1;
      if (DHTtemperatureSensor.invalidReadsCntr > 3) DHTtemperatureSensor.online = false;
    }
    float temp = DS18B20sensors.getTempCByIndex(0);
    if ((temp != 85.0) && (temp > -100)) { //Read 85 at sensor start up and -127 if no sensor response
      DS18B20Sensor.value  = temp; 
      DS18B20Sensor.current = true;
      DS18B20Sensor.invalidReadsCntr = 0;
      DS18B20Sensor.online = true;
    }
    else {
      DS18B20Sensor.current = false;
      DS18B20Sensor.invalidReadsCntr = DS18B20Sensor.invalidReadsCntr + 1;
      if (DS18B20Sensor.invalidReadsCntr > 3) DS18B20Sensor.online = false;
    }
    
    DS18B20sensors.requestTemperatures(); //initiate ds18B20 sampling which takes about 750 milliseconds to complete. Data will be 15 secs old when read.
    // Start a BMP temperature measurement:
    // If request is successful, the number of ms to wait is returned.
    // If request is unsuccessful, 0 is returned.
    char statusBMP = bmp.startMeasurment(); //
    if (statusBMP != 0)
    {
      // Wait for the measurement to complete:
      delay(statusBMP);
      // Retrieve the completed temperature measurement:
      // Note that the measurement is stored in the variable temperatureBMP.
      // Function returns 1 if successful, 0 if failure.
      double temperatureBMP, PressureBMP;
      statusBMP = bmp.getTemperatureAndPressure(temperatureBMP, PressureBMP); 
      if (statusBMP != 0){
          BMPtemperatureSensor.value  = temperatureBMP; 
          BMPtemperatureSensor.current = true;
          BMPtemperatureSensor.invalidReadsCntr = 0;
          BMPtemperatureSensor.online = true;
        // Print out the measurement:
        //Serial.print("temperatureBMP: ");
        //Serial.print(temperatureBMP,2);
        //Serial.print(" deg C, ");

        if (statusBMP != 0){
          delay(statusBMP);
            // The pressure sensor returns abolute pressure, which varies with altitude.
            // To remove the effects of altitude, use the sealevel function and your current altitude.
            // This number is commonly used in weather reports.
            // Parameters: PressureBMP = absolute pressure in mb, ALTITUDE = current altitude in m.
            // Result: seaLevelPressure = sea-level compensated pressure in mb

            double seaLevelPressure = bmp.sealevel(PressureBMP,ALTITUDE);
            BMPseaLevelPressureSensor.value  = seaLevelPressure; 
            BMPseaLevelPressureSensor.current = true;
            BMPseaLevelPressureSensor.invalidReadsCntr = 0;
            BMPseaLevelPressureSensor.online = true;
            //Serial.print("relative (sea-level) pressure: ");
            //Serial.print(seaLevelPressure,2);
            //Serial.print(" mb, ");
            //Serial.print(seaLevelPressure*0.0295333727,2);
            //Serial.println(" inHg");
          }
          else {
            Serial.println("error retrieving pressure measurement");
            BMPseaLevelPressureSensor.current = false;
            BMPseaLevelPressureSensor.invalidReadsCntr =  BMPseaLevelPressureSensor.invalidReadsCntr + 1;
            if ( BMPseaLevelPressureSensor.invalidReadsCntr > 3)  BMPseaLevelPressureSensor.online = false;
            BMPtemperatureSensor.current = false;
          }
      }
      else {
        Serial.println("error retrieving BMP temperature measurement");
        BMPseaLevelPressureSensor.current = false;
        BMPseaLevelPressureSensor.invalidReadsCntr =  BMPseaLevelPressureSensor.invalidReadsCntr + 1;
        if ( BMPseaLevelPressureSensor.invalidReadsCntr > 3)  BMPseaLevelPressureSensor.online = false;
        BMPtemperatureSensor.current = false;
        BMPtemperatureSensor.invalidReadsCntr = BMPtemperatureSensor.invalidReadsCntr + 1;
        if (BMPtemperatureSensor.invalidReadsCntr > 3) BMPtemperatureSensor.online = false;
      }
    }
    else {
      Serial.println("error starting BMP measurement");
      BMPseaLevelPressureSensor.current = false;
      BMPseaLevelPressureSensor.invalidReadsCntr =  BMPseaLevelPressureSensor.invalidReadsCntr + 1;
      if ( BMPseaLevelPressureSensor.invalidReadsCntr > 3)  BMPseaLevelPressureSensor.online = false;
      BMPtemperatureSensor.current = false;
      BMPtemperatureSensor.invalidReadsCntr = BMPtemperatureSensor.invalidReadsCntr + 1;
      if (BMPtemperatureSensor.invalidReadsCntr > 3) BMPtemperatureSensor.online = false;
    }
    Serial.print("  DHThumiditySensor = ");
    Serial.print(DHThumiditySensor.value);
    Serial.print("  DHThumiditySensor.current = ");
    Serial.print(DHThumiditySensor.current);
    Serial.print("  DHThumiditySensor.invalidReadsCntr = ");
    Serial.print(DHThumiditySensor.invalidReadsCntr);
    Serial.print("  DHThumiditySensor.online = ");
    Serial.println(DHThumiditySensor.online);

    Serial.print("  DHTtemperatureSensor = ");
    Serial.print(DHTtemperatureSensor.value);
    Serial.print("  DHTtemperatureSensor.current = ");
    Serial.print(DHTtemperatureSensor.current);
    Serial.print("  DHTtemperatureSensor.invalidReadsCntr = ");
    Serial.print(DHTtemperatureSensor.invalidReadsCntr);
    Serial.print("  DHTtemperatureSensor.online = ");
    Serial.println(DHTtemperatureSensor.online);
    
    Serial.print("  DS18B20temperature = ");
    Serial.print(temp);
    Serial.print("  DS18B20temperaturecurrent = ");
    Serial.print(DS18B20Sensor.current);
    Serial.print("  DS18B20Sensor.invalidReadsCntr = ");
    Serial.print(DS18B20Sensor.invalidReadsCntr);
    Serial.print("  DS18B20Sensor.online = ");
    Serial.println(DS18B20Sensor.online);

    Serial.print("  BMPtemperatureSensor = ");
    Serial.print(BMPtemperatureSensor.value);
    Serial.print("  BMPtemperatureSensor.current = ");
    Serial.print(BMPtemperatureSensor.current);
    Serial.print("  BMPtemperatureSensor.invalidReadsCntr = ");
    Serial.print(BMPtemperatureSensor.invalidReadsCntr);
    Serial.print("  BMPtemperatureSensor.online = ");
    Serial.println(BMPtemperatureSensor.online);

    Serial.print("  BMPseaLevelPressureSensor = ");
    Serial.print(BMPseaLevelPressureSensor.value);
    Serial.print("  BMPseaLevelPressureSensor.current = ");
    Serial.print(BMPseaLevelPressureSensor.current);
    Serial.print("  BMPseaLevelPressureSensor.invalidReadsCntr = ");
    Serial.print(BMPseaLevelPressureSensor.invalidReadsCntr);
    Serial.print("  BMPseaLevelPressureSensor.online = ");
    Serial.println(BMPseaLevelPressureSensor.online);
}

void updateThingSpeak(String tsData) {
  static int failedCounter = 0;
    if (client.connect(thingSpeakAddress, 80)) {    
           
        client.print("POST /update HTTP/1.1\n");
        client.print("Host: api.thingspeak.com\n");
        client.print("Connection: close\n");
        client.print("X-THINGSPEAKAPIKEY: "+writeAPIKey+"\n");
        client.print("Content-Type: application/x-www-form-urlencoded\n");
        client.print("Content-Length: ");
        client.print(tsData.length());
        client.print("\n\n");
        client.print(tsData);

        if (client.connected()) {
            Serial.println("Connecting to ThingSpeak...");
            Serial.println();
            failedCounter = 0;
        } else {
            failedCounter++;
            Serial.println("Connection to ThingSpeak failed ("+String(failedCounter, DEC)+")");   
            Serial.println();
        }
    } else {
        failedCounter++;
        Serial.println("Connection to ThingSpeak Failed ("+String(failedCounter, DEC)+")");   
        Serial.println();
    }
}

void checkButton() { //Called on ticker event. Update button state. 
  button.tick();
}
void ConfigurationPortalQuestionFlag() { //Called on every tick during a long button press
 ConfigurationPortalQuestionRequired = true;
}
void ConfigurationPortalFlag() { //Called on every tick during a long button press
 ConfigurationPortalRequired = true;
}
void NextScreenFlag() { //Called on button press
 NexrScreen = true;  //Doesn't do anything at the moment. Functionality to be added.
}
void ConfigurationPortalQuestion(SSD1306 *display) {
  delay(0); 
  Serial.println("ConfigQuestion");
  int counter=0;
  while ((button.isLongPressed()) && (counter<=10)) {  
    if (counter % 2 == 0) drawProgress(display, (counter*5+1), "Configure WiFi     "); 
    else drawProgress(display, (counter*5+1), "Configure WiFi ???");
    counter++;
    delay(500); 
  }
  if (button.isLongPressed()) ConfigurationPortalRequired = true;
  else ConfigurationPortalQuestionRequired = false;
}
