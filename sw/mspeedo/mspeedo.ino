//Wi-Fi
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

//Websockets
#include <WebSockets.h>
#include <WebSocketsServer.h>

//Webserver and page to be served included in PROGMEM within guihtml.h
#include <ESP8266WebServer.h>
#include "guihtml.h"

//mDNS
#include <ESP8266mDNS.h>

//EEPROM
#include <EEPROM.h>

//OTA
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//SPI
#include <SPI.h>

//ADNS 
#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_PRODUCT_IDX           0x3F
#define ADNS3080_CONFIGURATION_BITS    0x0A
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_PRODUCT_ID_VALUE      0x17
#define ADNS3080_PRODUCT_IDX_VALUE     0xF8

//30 x 30 = 900 pixels
#define ADNS3080_PIXELS                900
#define MOTION_MEASURE_PERIOD_US       10000

//Pin Definitions
#define RESET_PIN                      2
#define CS_PIN                         4

//WiFi configuration
char ssid1[32] = "";
char pass1[32] = "";
char ssid2[32] = "";
char pass2[32] = "";
char hostn[32] = "";

//Mode globals
int8_t WS_image_num = -1;
int8_t WS_motion_num = -1;
uint32_t last_motion = 0;

//ADNS globals
uint8_t motion;
int8_t dx;
int8_t dy;
int32_t x=0;
int32_t y=0;
uint8_t surfq;
uint8_t image_buf[ADNS3080_PIXELS];

//JSON Allocation
#include <ArduinoJson.h>
StaticJsonBuffer<300> jsonBuffer;
JsonObject& jsonroot = jsonBuffer.createObject();
char charbuffer[300];

ESP8266WiFiMulti WiFiMulti;
ESP8266WebServer webServer = ESP8266WebServer(80);
WebSocketsServer webSocket = WebSocketsServer(81);

void ADNS_reset(void) {
  digitalWrite(RESET_PIN, HIGH);
  delayMicroseconds(15); //Minimum 10uS pulse
  digitalWrite(RESET_PIN, LOW);
  delayMicroseconds(500); //Reset delay
}

void ADNS_write(uint8_t reg, uint8_t data) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(data);
  digitalWrite(CS_PIN, HIGH);
}

uint8_t ADNS_read(uint8_t reg) {
  uint8_t data;
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg);
  delayMicroseconds(75); //Read delay (50 for most, 75 for motion/motionburst, use 75)
  data = SPI.transfer(0);
  digitalWrite(CS_PIN, HIGH);
  return data;
}

void ADNS_motionburst(void) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(ADNS3080_MOTION_BURST);
  delayMicroseconds(75); //Motion read delay
  motion = SPI.transfer(0);
  dx = SPI.transfer(0);
  dy = SPI.transfer(0);
  surfq = SPI.transfer(0);
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(4); //Exist burst mode
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t payload_len) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\r\n", num);
      if( WS_motion_num == num){
        WS_motion_num = -1; //Stop motion data
      }
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\r\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      }
      break;
    case WStype_TEXT:
      Serial.printf("[%u] Got text: %s\r\n", num, payload);

      if(payload[0] == '#') {
        Serial.println(F("Got image request"));
        WS_motion_num = -1; //Stop motion data
        bool first_pixel = true;

        ADNS_write(ADNS3080_FRAME_CAPTURE, 0x83);
        delayMicroseconds(1510);
        
        for (uint16_t i = 0; i < ADNS3080_PIXELS; i++) {
          uint8_t regValue = ADNS_read(ADNS3080_FRAME_CAPTURE);
          if (first_pixel && !(regValue & 0x40)) {
            Serial.println(F("Failed to find first pixel"));
            return;
          }
          first_pixel = false;
          image_buf[i] = regValue << 2;
        }
        webSocket.sendBIN(num, image_buf, ADNS3080_PIXELS);
      } else if (payload[0] == '?'){
        Serial.println(F("Got motion request"));
        ADNS_reset(); // Do a reset before starting back motion data
        WS_motion_num = num;
        
        //webSocket.sendBIN(num, payload, length);
      } else {
        return;
        //Random ack to not need the async...?
      }
      break;
    case WStype_BIN:
      Serial.printf("[%u] get binary length: %u\r\n", num, payload_len);
      //hexdump(payload, payload_len);
      break;
  }
}


void setup() {
  int retries = 0;
  
  Serial.begin(115200);
  delay(10);
  //Serial.setDebugOutput(true);

  //Load WiFi
  Serial.println("Loading Wi-Fi config");
  EEPROM.begin(512);
  EEPROM.get(0, ssid1);
  EEPROM.get(0+sizeof(ssid1), pass1);
  EEPROM.get(0+sizeof(ssid1)+sizeof(pass1), ssid2);
  EEPROM.get(0+sizeof(ssid1)+sizeof(pass1)+sizeof(ssid2), pass2);
  EEPROM.get(0+sizeof(ssid1)+sizeof(pass1)+sizeof(ssid2)+sizeof(pass2), hostn);

  Serial.println(ssid1);
  Serial.println(pass1);
  Serial.println(ssid2);
  Serial.println(pass2);
  Serial.println(hostn);
  WiFi.hostname(hostn);
  WiFi.mode(WIFI_AP_STA);
  WiFiMulti.addAP(ssid1, pass1);
  WiFiMulti.addAP(ssid2, pass2);

  while(retries < 20){
    if(WiFiMulti.run() == WL_CONNECTED) { 
      break;
    }
    delay(500);
    Serial.println("Attempting Wi-Fi connection");
    retries++;
  }
  
  if(WiFiMulti.run() == WL_CONNECTED){
    WiFi.mode(WIFI_STA);
    Serial.println("");
    Serial.println("Wi-Fi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    ArduinoOTA.setHostname(hostn);
  } else {
    WiFi.softAP("MSPEEDO-AP");
    Serial.println("");
    Serial.println("WiFi AP Started");
    Serial.println("IP address: ");
    Serial.println(WiFi.softAPIP());    
  }
  
  ArduinoOTA.onStart([]() {
    Serial.println("\nStart");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    ESP.restart();
  });

  ArduinoOTA.begin();
  Serial.println("OTA service started");
  
  
  SPI.setHwCs(false);
  SPI.setFrequency(2000000);
  SPI.setDataMode(SPI_MODE3);
  pinMode(RESET_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); //Disable chip

  delayMicroseconds(250); //Initial ADNS delay before any commands including rest takes effect
  ADNS_reset();
  SPI.begin();
  
  uint8_t id = ADNS_read(ADNS3080_PRODUCT_ID);
  uint8_t idx = ADNS_read(ADNS3080_PRODUCT_IDX);
  Serial.print("Product ID ");
  Serial.println(id);
  Serial.print("Revision ID ");
  Serial.println(ADNS_read(0x01));
  
  if ((id == ADNS3080_PRODUCT_ID_VALUE) && (idx = ADNS3080_PRODUCT_ID_VALUE)){
    Serial.println(F("ADNS-3080 found"));
    ADNS_write(ADNS3080_CONFIGURATION_BITS, 0x10); //Set resolution to 1600dpi
  } else {
    Serial.print(F("Could not find ADNS-3080: "));
    Serial.println(id, HEX);
    //Do something about it here....
  }
  
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  webServer.on("/", []() {
    webServer.send(200, "text/html", GUIHTML);
  });
  webServer.on("/config.js", []() {
    String output = "var ssid1='" + String(ssid1) + "';";
    output += "var pass1='" + String(pass1) + "';";
    output += "var ssid2='" + String(ssid2) + "';";
    output += "var pass2='" + String(pass2) + "';";
    output += "var hostname='" + String(hostn) + "';";
    webServer.send(200, "text/json", output);
  });
  webServer.on("/configwifi", []() {
    Serial.println("Saving WiFi Config");
    webServer.arg("ssid1").toCharArray(ssid1, sizeof(ssid1)-1);
    webServer.arg("pass1").toCharArray(pass1, sizeof(pass1)-1);
    webServer.arg("ssid2").toCharArray(ssid2, sizeof(ssid2)-1);
    webServer.arg("pass2").toCharArray(pass2, sizeof(pass2)-1);
    webServer.arg("hostname").toCharArray(hostn, sizeof(hostn)-1);
    
    EEPROM.put(0, ssid1);
    EEPROM.put(0+sizeof(ssid1), pass1);
    EEPROM.put(0+sizeof(ssid1)+sizeof(pass1), ssid2);
    EEPROM.put(0+sizeof(ssid1)+sizeof(pass1)+sizeof(ssid2), pass2);
    EEPROM.put(0+sizeof(ssid1)+sizeof(pass1)+sizeof(ssid2)+sizeof(pass2), hostn);
    EEPROM.commit();
    EEPROM.end();
    webServer.send(200, "text/html", "Config saved - Rebooting, please reconnect.");
    Serial.println("Rebooting...");
    ESP.reset();
  });
  webServer.begin();
    
  if(MDNS.begin(hostn)) {
    Serial.println("MDNS responder started");
  }
  MDNS.addService("http", "tcp", 80);
  MDNS.addService("ws", "tcp", 81);
}

void loop() {
  webSocket.loop();
  webServer.handleClient();
  ArduinoOTA.handle();
  if ( WS_motion_num != -1 ){
    uint32_t current_micros = micros();
    if ( (current_micros - last_motion) >= MOTION_MEASURE_PERIOD_US ){
      uint32_t period = current_micros - last_motion;
      last_motion = current_micros;
      ADNS_motionburst();
      if(motion & 0x80){
        x += dx;
        y += dy;
      }
      jsonroot["overflow"] = motion & 0x10;
      jsonroot["motion"] = motion & 0x80;
      jsonroot["dx"] = dx;
      jsonroot["dy"] = dy;
      jsonroot["x"] = x;
      jsonroot["y"] = y;
      jsonroot["surfq"] = surfq; 
      jsonroot["period"] = period;
      size_t size = jsonroot.printTo(charbuffer, sizeof(charbuffer));
      webSocket.sendTXT(WS_motion_num, charbuffer, size);
    }
  }
}
