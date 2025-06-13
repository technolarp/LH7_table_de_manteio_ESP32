/*
   ----------------------------------------------------------------------------
   TECHNOLARP - https://technolarp.github.io/
   TABLE DE MANTEIO ESP32 - https://github.com/technolarp/LH7_table_de_manteio_ESP32
   version 1.0.0 - 06/2025
   ----------------------------------------------------------------------------
*/

#include <Arduino.h>

// WIFI
#include <AsyncTCP.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

char apName[20]="MANTEIO";
char apPassword[20]="manteio123";

// API
#include <HTTPClient.h>

// FASTLED
#include <FastLED.h>
#define NUM_REEDS 9
#define DATA_PIN 13

CRGB leds[NUM_REEDS];
uint8_t indexLed = 0;

// MCP23017
#include <Adafruit_MCP23X17.h>
Adafruit_MCP23X17 mcp;

uint8_t reedState[NUM_REEDS];
uint8_t reedPreviousState[NUM_REEDS];

// HEARTBEAT
uint32_t previousMillisHB;
uint32_t intervalHB;

// READ MCP
uint32_t previousMillisRead;
uint32_t intervalRead;

uint32_t previousMillisDebounce;
uint32_t intervalDebounce;

// FUNCTIONS
void httpPutRequest(uint16_t movie);

void setup()
{
  // SERIAL
  Serial.begin(115200);
  delay(500);
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("----------------------------------------------------------------------------"));
  Serial.println(F("TABLE DE MANTEIO ESP32 - https://github.com/technolarp/LH7_table_de_manteio_ESP32"));
  Serial.println(F("version 1.0.0 - 06/2025"));
  Serial.println(F("----------------------------------------------------------------------------"));

  // FASTLED
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_REEDS);

  // WIFI
  WiFi.disconnect(true);

  Serial.println(F(""));
  Serial.println(F("connecting WiFi"));

  WiFi.mode(WIFI_STA);
  WiFi.begin(apName, apPassword);

  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println(F("WiFi Failed!"));
    for (uint8_t i=0;i<NUM_REEDS;i++)
    {
      leds[i]=CRGB::Red;
    }
    FastLED.show();
    delay(3000);
    
    for (uint8_t i=0;i<NUM_REEDS;i++)
    {
      leds[i]=CRGB::Black;
    }
    FastLED.show();
  }
  else
  {
    Serial.println(F("WiFi OK"));
    for (uint8_t j=0;j<10;j++)
    {
      for (uint8_t i=0;i<NUM_REEDS;i++)
      {
        leds[i]=CRGB::Green;
      }
      FastLED.show();
      delay(100);
      
      for (uint8_t i=0;i<NUM_REEDS;i++)
      {
        leds[i]=CRGB::Black;
      }
      FastLED.show();
      delay(100);
    }    
  }

  // Print ESP Local IP Address
  Serial.print(F("localIP: "));
  Serial.println(WiFi.localIP());

  Serial.print(F("gatewayIP: "));
  Serial.println(WiFi.gatewayIP());

  // MCP23017
  if (!mcp.begin_I2C())
  {
    Serial.println(F("Error I2C"));
    while (1);
  }

  // configure pin for input with pull up
  for (int8_t i=0;i<NUM_REEDS;i++)
  {
    mcp.pinMode(i, INPUT_PULLUP);
    reedState[i]=0;
    reedPreviousState[i]=0;
  }

  // HEARTBEAT
  previousMillisHB = millis();
  intervalHB = 200;

  previousMillisRead = millis();
  intervalRead = 100;

  previousMillisDebounce = millis();
  intervalDebounce = 500;

  // START
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("START !!!"));
}

void loop()
{  
  // READ MCP
  if ( (millis() - previousMillisRead > intervalRead) && (millis() - previousMillisDebounce > intervalDebounce) )
  {
    previousMillisRead = millis();
    bool changeFlag = false;

    // read MCP
    for (int8_t i=0;i<NUM_REEDS;i++)
    {
      uint8_t readMcp = !mcp.digitalRead(i);
      reedState[i]=readMcp;

      if (reedState[i] != reedPreviousState[i])
      {
        changeFlag = true;
      }
    }

    if (changeFlag)
    {
      for (int8_t i=0;i<NUM_REEDS;i++)
      {
        Serial.print(reedState[i]);
        Serial.print(F(" "));
      }
      Serial.println(F(" "));

      int8_t check = 0;
      int8_t movie = 0;
      for (int8_t i=0;i<NUM_REEDS;i++)
      {
        check = check + reedState[i];
      }

      int8_t multiple = 10;
      if (check==2)
      {
        for (int8_t i=0;i<NUM_REEDS;i++)
        {
          if (reedState[i]==1)
          {
            movie=movie + (i+1)*multiple;
            multiple=1;
          }          
        }
      }
      Serial.print("movie: ");
      Serial.println(movie);

      httpPutRequest(movie);
      
      previousMillisDebounce=millis();
    }
  }

  if (millis() - previousMillisHB > intervalHB)
  {
    previousMillisHB = millis();

    indexLed+=1;
    indexLed%=NUM_REEDS;
    
    // maj array reed
    for (uint8_t i=0;i<NUM_REEDS;i++)
    {
      if (reedState[i] != reedPreviousState[i])
      {
        reedPreviousState[i]=reedState[i];
      }
    }

    // maj led
    for (uint8_t i=0;i<NUM_REEDS;i++)
    {
      if (reedState[i]==1)
      {
        leds[i] = CRGB::Red;
      }
      else
      {
        if(i==indexLed)
        {
          leds[i] = CRGB::Blue;
        }
        else
        {
          leds[i] = CRGB::Black;
        }
      }
    }
    FastLED.show();
  }
}

// send API PUT request to gateaway IP
void httpPutRequest(uint16_t movie)
{
  HTTPClient http;
  IPAddress apGW = WiFi.gatewayIP();

  String toSend =  "http://" + String(apGW[0]) + "." + String(apGW[1]) + "." + String(apGW[2]) + "." + String(apGW[3]) + ":8000/manteio?play=" + movie;
  Serial.println(toSend);
  
  http.begin(toSend);
  http.addHeader("Content-Type", "text/plain");
 
  int httpResponseCode = http.PUT("PUT sent from ESP32");

  Serial.print("httpResponseCode PUT: ");
  Serial.println(httpResponseCode);
}