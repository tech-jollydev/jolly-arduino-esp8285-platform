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

#include "Configuration.h"


bool Configuration::setParam(String param, String value){
  if(value.length()>1 && param.length()>1){
    String paramConfig = "";
    DynamicJsonDocument jsonBuffer(1024);
    File configFile = SPIFFS.open(CONFIGFILENAME,"r");
    if(configFile){     //read Config file
      while(configFile.available()){
        paramConfig = paramConfig + char(configFile.read());
      }
    }
    else{
      paramConfig = JSONEMPTY;
    }
    configFile.close();
    deserializeJson(jsonBuffer, paramConfig);
    //write param to config file
    configFile = SPIFFS.open(CONFIGFILENAME, "w");
    jsonBuffer[param] = value;
    serializeJson(jsonBuffer, configFile);
    configFile.close();
    return true;
  }
  else{
    return false;
  }
}

String Configuration::getParam(String param){
  if(param.length()>1){
    File configFile = SPIFFS.open(CONFIGFILENAME, "r");
    if (!configFile) {
      return EMPTY;
    }
    size_t size = configFile.size();
    std::unique_ptr<char[]> buf(new char[size]);
    configFile.readBytes(buf.get(), size);

    DynamicJsonDocument jsonBuffer(1024);
    DeserializationError err = deserializeJson(jsonBuffer, buf.get());

    if (err != DeserializationError::Ok) {
      return EMPTY;
    }
    configFile.close();
    return jsonBuffer[param];
  }
  else{
    return EMPTY;
  }
}

Configuration Config;
