/*
This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "WiFiBridge.h"
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "Configuration.h"
#include <EEPROM.h>

#include <FS.h>
#include <ArduinoJson.h>
#include <Hash.h>
#include <ESP8266WebServer.h>

int ledState = LOW;             // used to set the LED state
long previousMillis = 0;        // will store last time LED was updated
long ap_interval = 50;         //blink interval in ap mode
IPAddress default_IP(192,168,240,1);  //defaul IP Address
String HOSTNAME = DEF_HOSTNAME;

WiFiMode_t wifiModeStatus = WIFI_OFF;

ESP8266WebServer server(80);    //server UI

void setup() 
{
  pinMode(WIFI_LED, OUTPUT);      //initialize wifi LED
  digitalWrite(WIFI_LED, LOW);
  EEPROM.begin(1024);

  ArduinoOTA.begin();             //OTA ESP
  initMDNS();
  setWiFiConfig(WIFI_AP_STA);
  wifiBridge.begin();
  delayMicroseconds(10);

  SPIFFS.begin();
  initHostname();
  initWebServer();                 //UI begin
  delayMicroseconds(10);

  //wifiBridge.autoconnect();
  #ifdef DEBUG_SERIALE
  Serial.begin(115200);
  Serial.println("Ready!");
  wifiBridge._startAutoconnect = true;

  #endif
}

void loop() 
{
  ArduinoOTA.handle();
  wifiBridge.handle();
  handleWebServer();
  wifiLed();
}

void initMDNS()
{

}

void initHostname()
{
  //retrieve user defined hostname
  String tmpHostname = Config.getParam("hostname");
  if( tmpHostname!="" )
    HOSTNAME = tmpHostname;
  WiFi.hostname(HOSTNAME);
}

void wifiLed()
{
  uint32_t currentMillis = millis();
  int wifi_status = WiFi.status();
  if ((WiFi.getMode() == 1 || WiFi.getMode() == 3) && wifi_status == WL_CONNECTED) {    //wifi LED in STA MODE
    if (currentMillis - previousMillis > ap_interval) {
      previousMillis = currentMillis;
      if (ledState == LOW) {
        ledState = HIGH;
        ap_interval = 200;    //time wifi led ON
      }
      else {
        ledState = LOW;
        ap_interval = 2800;   //time wifi led OFF
      }
      digitalWrite(WIFI_LED, ledState);
    }
  }
  else { //if (WiFi.softAPgetStationNum() > 0 ) {   //wifi LED on in AP mode
    if (currentMillis - previousMillis > ap_interval) {
      previousMillis = currentMillis;
      if (ledState == LOW) {
        ledState = HIGH;
        ap_interval = 950;
      }
      else {
        ledState = LOW;
        ap_interval = 50;
      }
      digitalWrite(WIFI_LED, ledState);
    }
  }

}

void setWiFiConfig(WiFiMode_t mode)
{
  //set mode
  WiFi.mode(mode);
  WiFi.disconnect();
  delay(100);

  //set default AP
  if(mode == WIFI_AP || mode == WIFI_AP_STA){
    String mac = WiFi.macAddress();
    String apSSID = String(SSIDNAME) + "-" + String(mac[9])+String(mac[10])+String(mac[12])+String(mac[13])+String(mac[15])+String(mac[16]);
    char softApssid[18];
    apSSID.toCharArray(softApssid, apSSID.length()+1);
    //delay(1000);
    WiFi.softAP(softApssid);
    WiFi.softAPConfig(default_IP, default_IP, IPAddress(255, 255, 255, 0));   //set default ip for AP mode
  }
}
