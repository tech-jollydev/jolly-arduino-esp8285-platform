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

#include "SPISlave.h"
#include "WiFiBridge.h"
#include "CommCmd.h"

//check data ready to process (SPI)
volatile bool notify = false;
volatile uint32_t msgNum = 0;
InterfaceClass interface;
tsUdpData UdpData;

uint32_t srTimer = 0;
uint32_t srTimer2 = 0;

uint8_t temp[64] = {0};
bool srMem = false;
bool notifyAck = false;

void stopServer();
// ----------------------------------------------- InterfaceClass -----------------------------------------------
// ----------------------------------------------------- PRIVATE
/*
*
*/
InterfaceClass::InterfaceClass()
{
  _processing = 0;
  longCmd.totalLen = 0;
  longCmd.nParam = 0;
  recPacket.receivedLen = 0;
  recPacket.endReceived = true;
}

/*
*
*/
void InterfaceClass::_SPISlaveInit()
{
  //SPI Data register
  //--------------------------------------------------------------- SPI data received Callback
  SPISlave.onData([this](uint8_t *data, size_t len) {
    // copy received data from the SPI buffer into the internal packet buffer
    memcpy(_raw_pkt, data, len);
    // enable processing in handle function
    enableProcess();
  });

  //SPI Status register
  //--------------------------------------------------------------- SPI status received Callback
  SPISlave.onStatus([this](uint32_t data) {
  });

  SPISlave.onDataSent([this]() {
    // when the data has been sent, release the next write into the SPI buffer
    _sendingWait = false;
  });

  // Setup SPI Slave registers and pins
  //--------------------------------------------------------------- SPI slave init
  SPISlave.begin(); // point to hspi_slave

  /*SPI1WS = 0;
  for (uint8_t i = 0; i < 16; i++)
    SPI1W(i) = 0;*/

  pinMode(PIN_SPI_SR, OUTPUT);   //GPIO16 (XPD-DCDC) connected to ATMEGA328P's pin 13 (PD7 - IO7)
  digitalWrite(PIN_SPI_SR, LOW);
  pinMode(PIN_SPI_CS, SPECIAL);
}

/*
* 
*/
void InterfaceClass::_SPISlaveWrite(uint8_t *_rep, uint32_t len)
{
  // calculate the number of packets to be sent - each packet is 32 byte long
  uint8_t nPacket = (uint8_t)(len >> 5);
  uint8_t remainingByte = 0;
  uint32_t pktOffset = 0;
  uint32_t t = 0;
  uint8_t tempBuf[33] = {0};
  bool multipacket = false;
  uint16_t sentByte = 0 ;

  if ((remainingByte = (len % 32)) > 0)
    nPacket++;
  
  _sendingWait = false;

  while (nPacket > 0) {
    t = millis() + SPI_TIMEOUT;
    
    if(nPacket == 1 && remainingByte > 0) {
      memcpy(tempBuf, (uint8_t *)(_rep + pktOffset), remainingByte);
      SPISlave.setData(tempBuf, 32);
      sentByte += remainingByte;
    }
    else{
      SPISlave.setData((uint8_t *)(_rep + pktOffset), 32); //send response to MCU
      sentByte += 32;
    }
    // set the ISR trigger signal to fire a read process
    setSR(HIGH);

    _sendingWait = true;
    pktOffset += 32;
    nPacket--;

    t = millis() + SPI_TIMEOUT;
    while (_sendingWait) {
      delay(0);

      if (t < millis()) {
#ifdef DEBUG_SERIALE
        Serial.println("Stop waiting write");
        Serial.println("Byte written so far: " + String(sentByte));
#endif
        // lower the ISR signal to let the 328 know that ESP is already idle
        setSR(LOW);
        return;
      }
    }
    setSR(LOW);
    
    if(nPacket > 0) {
      multipacket = true;
      delayMicroseconds(10);
    }
  }
  #ifdef DEBUG_SERIALE
  Serial.println("byte written: " + String(sentByte));
  #endif
// lower the ISR signal to let the 328 know that ESP is already idle
  setSR(LOW);
  }

/*
* 
*/
int8_t InterfaceClass::_parsePkt(tsNewCmd *_pkt)
{
  // ---------------------------------------------------------------------------- receiving long command packets
  if (recPacket.receivedLen > 0) {
    uint8_t endOffset;

    for (uint8_t i = 0; i < SPI_BUFFER_SIZE; i++) {
      if (_raw_pkt[i] == END_CMD) {
        endOffset = i;

        if (recPacket.receivedLen + endOffset == recPacket.totalLen - 1) {
          recPacket.endReceived = true;
         
          _pkt->cmdType = recPacket.cmdType;
          _pkt->cmd = recPacket.cmd;
          _pkt->nParam = endOffset;
          _pkt->totalLen = recPacket.totalLen - DATA_HEAD_FOOT_LEN;
          _pkt->dataPtr = &_raw_pkt[0];

          recPacket.receivedLen = recPacket.totalLen = 0;

          return 0;
        }
      }
    }
    if (recPacket.endReceived == false) {
      recPacket.receivedLen += SPI_BUFFER_SIZE;

      if(recPacket.receivedLen >= recPacket.totalLen){
        recPacket.endReceived = true;
        recPacket.receivedLen = recPacket.totalLen = 0;
      }
      
      _pkt->cmdType = recPacket.cmdType;
      _pkt->cmd = recPacket.cmd;
      _pkt->totalLen = recPacket.totalLen - DATA_HEAD_FOOT_LEN;
      _pkt->nParam = SPI_BUFFER_SIZE;
      _pkt->dataPtr = &_raw_pkt[0];

      notifyAck = true;

      return 0;
    }
  }
  // ---------------------------------------------------------------------------- multipacket command
  else if (longCmd.nParam > 0){
    uint8_t endOffset;
    for (uint8_t i = 0; i < SPI_BUFFER_SIZE; i++) {
      if (_raw_pkt[i] == END_CMD) {
        endOffset = i;
         
        _pkt->cmdType = longCmd.cmdType;
        _pkt->cmd = longCmd.cmd;
        //_pkt->nParam = longCmd.nParam + endOffset;
        _pkt->totalLen = longCmd.totalLen - DATA_HEAD_FOOT_LEN;
        memcpy(&longCmd.dataPtr[longCmd.nParam], _raw_pkt, endOffset);
        _pkt->dataPtr = longCmd.dataPtr;

        longCmd.nParam = 0;

        return 0;
      }
    }
    // command continuation
    if((longCmd.nParam + SPI_BUFFER_SIZE > longCmd.totalLen) || ((longCmd.nParam + SPI_BUFFER_SIZE) >= (SPI_BUFFER_SIZE << 2))){
      // error - received length is greater than expected or it exceeds the maximum command packet length
      longCmd.nParam = 0;
      notifyAck = true;
      return -1;
    }
    // copy received data
    memcpy(&longCmd.dataPtr[longCmd.nParam], _raw_pkt, SPI_BUFFER_SIZE);
    longCmd.nParam += SPI_BUFFER_SIZE;
    notifyAck = true;
    return -1;
  }
    
  // ---------------------------------------------------------------------------- decoding command
  else if (_raw_pkt[0] == START_CMD) {
    _pkt->cmdType = _raw_pkt[0];
    _pkt->cmd = _raw_pkt[1];
    _pkt->totalLen = (uint32_t)_raw_pkt[2];
    _pkt->nParam = _raw_pkt[3];
    _pkt->dataPtr = &_raw_pkt[4];

    if (_pkt->totalLen > SPI_BUFFER_SIZE) {
      longCmd.cmdType = _pkt->cmdType;
      longCmd.cmd = _pkt->cmd;
      longCmd.totalLen = _pkt->totalLen;

      _payLoadLen = SPI_BUFFER_SIZE - 4;
      _rxLen = SPI_BUFFER_SIZE;

      longCmd.nParam = _payLoadLen;

      // copy total len - cmd header size
      memset(_longCmdPkt, 0, SPI_BUFFER_SIZE << 2); // clear 128 bytes, max cmd len = 4 * SPI_BUFFER_SIZE
      longCmd.dataPtr = _longCmdPkt;
      memcpy(longCmd.dataPtr, _pkt->dataPtr, _payLoadLen);

      pktRecvTimeout = millis() + 5000;
      
      notifyAck = true;

      return -1;
    }
    else {
      if (_raw_pkt[_pkt->totalLen - 1] != END_CMD)
        return -1;
      return 0;
    }
  }
  // ---------------------------------------------------------------------------- decoding data packets
  else if (_raw_pkt[0] == DATA_PKT)
  {
    _pkt->cmdType = recPacket.cmdType = _raw_pkt[0];
    _pkt->cmd = recPacket.cmd = _raw_pkt[1];
    recPacket.receivedLen = SPI_BUFFER_SIZE;
    recPacket.totalLen = (_raw_pkt[5] << 24) + (_raw_pkt[4] << 16) + (_raw_pkt[3] << 8) + _raw_pkt[2]; //_pkt->totalLen = recPacket.totalLen = (_raw_pkt[2] << 24) + (_raw_pkt[3] << 16) + (_raw_pkt[4] << 8) + _raw_pkt[5];
    recPacket.endReceived = false;
    _pkt->totalLen = recPacket.totalLen - DATA_HEAD_FOOT_LEN;
    recPacket.sockN = _raw_pkt[6];
    // 32 byte - (cmdType (1 byte) + cmd (1 byte) + totalLen (4 byte) + socketIndex (1 byte)) = 32 - 7 = 25
    _pkt->nParam = recPacket.receivedLen - (sizeof(recPacket.cmdType) + sizeof(recPacket.cmd) + sizeof(recPacket.totalLen) + sizeof(recPacket.sockN)); //_pkt->nParam = 26;
    _pkt->dataPtr = &_raw_pkt[7];

    for(uint8_t i = 7; i < SPI_BUFFER_SIZE; i++){
      if(_raw_pkt[i] == END_CMD){
        recPacket.endReceived = true;
        recPacket.receivedLen = recPacket.totalLen = 0;
        _pkt->nParam = 255; // this value is used as flag that indicates that data packet is all contained in a single packet

        return 0;
      }
    }

    pktRecvTimeout = millis() + 5000;

    notifyAck = true;

    return 0;
  }
  // error;
  else {
    return -1;
  }

  return 0;
}

// ----------------------------------------------------- PUBLIC
/*
*
*/
void InterfaceClass::begin()
{
  _SPISlaveInit();
}

/*
*
*/
void InterfaceClass::enableNotify()
{
  _notify = true;
}

/*
*
*/
void InterfaceClass::enableProcess()
{
  _processing = 1;
}

/*
*
*/
bool InterfaceClass::canNotify()
{
  return _notify;
}

/*
*
*/
uint8_t InterfaceClass::canProcess()
{
  return _processing;
}

/*
*
*/
void InterfaceClass::endNotify()
{
  _notify = false;
}

/*
*
*/
void InterfaceClass::endProcessing()
{
  _processing = 0;
}

/*
*
*/
void InterfaceClass::setSR(bool level)
{
    uint32_t deltaT = micros() - srTimer2;

    if(deltaT < 20)
      delayMicroseconds(20 - deltaT);
  
  digitalWrite(PIN_SPI_SR, level);
  srTimer2 = micros();
}

/*
*
*/
int8_t InterfaceClass::read(tsNewCmd *_pkt)
{
  return _parsePkt(_pkt);
}

/*
*
*/
void InterfaceClass::write(uint8_t *_pkt, uint32_t len)
{
  _SPISlaveWrite(_pkt, len);
}

/*
*
*/
bool InterfaceClass::checkMasterReady()
{
  return (digitalRead(SS));
}

uint32_t InterfaceClass::sockBuffInit(void)
{
  memset(_socketPkt, 0, MTU_SIZE);
  _socketPktPtr = 0;
}

uint32_t InterfaceClass::sockBuffAdd(uint8_t* data, uint32_t sz)
{
  if(_socketPktPtr + sz <= MTU_SIZE){
    memcpy((uint8_t*)(_socketPkt + _socketPktPtr), data, sz);
    _socketPktPtr += sz;
  }
  return _socketPktPtr;
}

// ----------------------------------------------- WiFiBridgeClass -----------------------------------------------
/*
*
*/

// ----------------------------------------------------- PRIVATE

void WiFiBridgeClass::_notifySTAConnStatus(uint8_t stat)
{
  _notifyMsgCntr = 0;
  memset(interface._notifyMsg, 0, SPI_BUFFER_SIZE);
  
  interface._notifyMsg[_notifyMsgCntr++] = START_CMD;

  if(stat == WL_CONNECTED){
    if(connectedIndex_temp < 0){
      int8_t ind = -1;
      
      // the network was already in the list
      if((ind = _checkSSIDInList(tempNetworkParam.strSSID)) < 0){
        // SSID not present make a new entry
        networkArray.lastConnected = _networkListEvalAging(0xff);
        memset(networkArray.netEntry[networkArray.lastConnected].strSSID, 0, 33);
        memset(networkArray.netEntry[networkArray.lastConnected].strPWD, 0, 65);
        strcpy(networkArray.netEntry[networkArray.lastConnected].strSSID, tempNetworkParam.strSSID);

        interface._notifyMsg[_notifyMsgCntr] = CONNECT_OPEN_AP | REPLY_FLAG;

        if (tempNetworkParam.strPWD[0] != 0xff){
          interface._notifyMsg[_notifyMsgCntr] = CONNECT_SECURED_AP | REPLY_FLAG;
          strcpy(networkArray.netEntry[networkArray.lastConnected].strPWD, tempNetworkParam.strPWD);
        }
        // update the aging
        _networkListEvalAging(networkArray.lastConnected);
        delay(0);
        _storeWiFiSSIDList();
        
        _notifyMsgCntr++;
      }
      else{
        networkArray.lastConnected = ind;
        if(networkArray.netEntry[networkArray.lastConnected].strPWD[0] != 0xff)
          interface._notifyMsg[_notifyMsgCntr] = CONNECT_SECURED_AP | REPLY_FLAG;
        else 
          interface._notifyMsg[_notifyMsgCntr] = CONNECT_OPEN_AP | REPLY_FLAG;

        _notifyMsgCntr++;
        _networkListEvalAging(networkArray.lastConnected);
      }
    }
    else{
      networkArray.lastConnected = connectedIndex_temp;
      if(networkArray.netEntry[networkArray.lastConnected].strPWD[0] != 0xff)
        interface._notifyMsg[_notifyMsgCntr] = CONNECT_SECURED_AP | REPLY_FLAG;
      else 
        interface._notifyMsg[_notifyMsgCntr] = CONNECT_OPEN_AP | REPLY_FLAG;

      _notifyMsgCntr++;
      _networkListEvalAging(connectedIndex_temp);
      
    }
    delay(0);
/*
    #ifdef DEBUG_SERIALE
      for(uint8_t i=0; i<MAX_NETWORK_LIST; i++){
        Serial.print("\nSSID: ");
        Serial.write(networkArray.netEntry[i].strSSID, 32);
        Serial.print("\nPWD: ");
        Serial.write(networkArray.netEntry[i].strPWD, 64);
        Serial.print("\naging: ");
        Serial.println(networkArray.netEntry[i].aging);
        
        delay(0);
      }
    #endif
*/
  }
  else if(stat == WL_DISCONNECTED){
    interface._notifyMsg[_notifyMsgCntr++] = DISCONNECT_CMD | REPLY_FLAG;
    #ifdef DEBUG_SERIALE
      Serial.println("disconnecting...");
    #endif
  }

  connectedIndex_temp = -1;

  interface._notifyMsg[_notifyMsgCntr++] = 0;
  interface._notifyMsg[_notifyMsgCntr++] = PARAM_NUMS_1;
  interface._notifyMsg[_notifyMsgCntr++] = PARAM_SIZE_1;
  interface._notifyMsg[_notifyMsgCntr++] = stat;
  interface._notifyMsg[_notifyMsgCntr++] = END_CMD;

  interface._notifyMsg[2] = _notifyMsgCntr;
/*
  #ifdef DEBUG_SERIALE
    Serial.write(interface._notifyMsg, interface._notifyMsg[2]);
  #endif
*/
}

/*
 *
 */
void WiFiBridgeClass::_prepareReplayBuf()
{
  _txPktSize = 0;
  memset(_repPkt, 0, MAX_CMD_LEN);
}

/*
*
*/
void WiFiBridgeClass::_createErrorResponse()
{
  _repPkt[_txPktSize++] = ERR_CMD;
  _repPkt[_txPktSize++] = 4;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = END_CMD;
}

/*
*
*/
bool WiFiBridgeClass::_process()
{
  bool rep = true;
  _prepareReplayBuf();

  switch (_tempPkt.cmd)
  {
  case GET_CONN_STATUS:
    _getStatus();
    break;
  case GET_FW_VERSION_CMD:
    _getFwVersion();
    break;
  case CONNECT_OPEN_AP:
    rep = _connect(0);
    break;
  case CONNECT_SECURED_AP:
    rep = _connect(1);
    break;
  case AUTOCONNECT_TO_STA:
    rep = _connect(2);
    break;
  case DISCONNECT_CMD:
    rep = _disconnect();
    break;
  case CANCEL_NETWORK_LIST:
    _clearWiFiSSIDList();
  break;
  case SET_IP_CONFIG_CMD:
    _config();
    break;
  case SET_DNS_CONFIG_CMD:
    _setDNS();
    break;
  case GET_HOSTNAME:
    _getHostname();
    break;
  case GET_IPADDR_CMD:
    _getNetworkData();
    break;
  case GET_MACADDR_CMD:
    _getMacAddress();
    break;
  case GET_CURR_SSID_CMD:
    _getCurrentSSID();
    break;
  case GET_CURR_BSSID_CMD:
    _getBSSID(1);
    break;
  case GET_CURR_RSSI_CMD:
    _getRSSI(1);
    break;
  case GET_IDX_RSSI_CMD:
    _getRSSI(0);
    break;
  case GET_CURR_ENCT_CMD:
    _getEncryption(1);
    break;
  case GET_IDX_ENCT_CMD:
    _getEncryption(0);
    break;
  case START_SCAN_NETWORKS:
    _startScanNetwork();
    break;
  case SCAN_NETWORK_RESULT:
    _scanNetwork(_tempPkt.dataPtr[1]);
    break;
  case START_SERVER_TCP_CMD:
    _startServer();
    break;
  case GET_DATA_TCP_CMD:
    _getData();
    break;
  case STOP_CLIENT_TCP_CMD:
    _stopClient();
    break;
  case GET_CLIENT_STATE_TCP_CMD:
    _clientStatus();
    break;
  case SEND_DATA_TCP_CMD:
    rep = _sendData();
    break;
  case GET_DATABUF_TCP_CMD: 
    _getDataBuf();
     break;
  case START_CLIENT_TCP_CMD:
    _startClient();
    break;
  case GET_STATE_TCP_CMD:
    _serverStatus();
    break;
  case GET_HOST_BY_NAME_CMD:
    _getHostByName();
    break;
  case SEND_DATA_UDP_CMD:
    _sendUdpData();
    break;
  case GET_REMOTE_DATA_CMD:
    _remoteData();
    break;
  case AVAIL_DATA_TCP_CMD:
    _availSocketData();
    break;
  case DISABLE_WEBPANEL:
    stopServer();
    rep = false;
    break;

  // Test multipacket transmission
  case TEST_DATA_TXRX:
    rep = _txrxMsg();
    break;
  default:
    _createErrorResponse();
    break;
  }
  return rep;
}

/*
*
*/
void WiFiBridgeClass::_getHostname()
{
  String host = WiFi.hostname();
  uint8_t sz = host.length();
   
  // set the response struct
  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_1;
  _repPkt[_txPktSize++] = sz;
  memcpy((_repPkt + _txPktSize), host.c_str(), sz);
  _txPktSize += sz;
  _repPkt[_txPktSize++] = END_CMD;

  _repPkt[2] = _txPktSize;
}

/*
*
*/
void WiFiBridgeClass::_getStatus()
{
  // Disconnet from the network
  uint8_t result = WiFi.status();

  // set the response struct
  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_1;
  _repPkt[_txPktSize++] = PARAM_SIZE_1;
  _repPkt[_txPktSize++] = result;
  _repPkt[_txPktSize++] = END_CMD;

  _repPkt[2] = _txPktSize;
}

/*
*
*/
void WiFiBridgeClass::_getFwVersion()
{
  // set the response struct
  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_1;
  _repPkt[_txPktSize++] = PARAM_SIZE_5;
  memcpy((_repPkt + _txPktSize), FW_VERSION, PARAM_SIZE_5);
  _txPktSize += PARAM_SIZE_5;
  _repPkt[_txPktSize++] = END_CMD;

  _repPkt[2] = _txPktSize;
}

/*
*
*/
void WiFiBridgeClass::_getMacAddress()
{
  uint8_t mac[PARAM_SIZE_6];

  //Retrive mac address
  WiFi.macAddress(mac);

  // set the response struct
  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_1;
  _repPkt[_txPktSize++] = PARAM_SIZE_6;
  for (int i = 0, j = PARAM_SIZE_6 - 1; i < PARAM_SIZE_6, j >= 0; i++, j--)
  {
    _repPkt[_txPktSize++] = mac[j];
  }
  _repPkt[_txPktSize++] = END_CMD;

  _repPkt[2] = _txPktSize;
}

/*
*
*/
void WiFiBridgeClass::_getCurrentSSID()
{
  //retrieve SSID of the current network
  String result = WiFi.SSID();

  // set the response struct
  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_1;
  _repPkt[_txPktSize++] = result.length();
  memcpy((_repPkt + _txPktSize), result.c_str(), result.length());
  _txPktSize += result.length();
  _repPkt[_txPktSize++] = END_CMD;

  _repPkt[2] = _txPktSize;
}

/*
*
*/
void WiFiBridgeClass::_getRSSI(uint8_t current)
{
  int32_t result;
  //retrieve RSSI
  if (current == 1)
  {
    result = WiFi.RSSI();
  }
  else
  {
    int8_t net_idx = _tempPkt.dataPtr[1];

    // NOTE: only for test this function
    // user must call scan network before
    // int num = WiFi.scanNetworks();
    result = WiFi.RSSI(net_idx);
  }

  // set the response struct
  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_1;
  _repPkt[_txPktSize++] = PARAM_SIZE_4;
  _repPkt[_txPktSize++] = (int8_t)result;
  _repPkt[_txPktSize++] = 0xFF;
  _repPkt[_txPktSize++] = 0xFF;
  _repPkt[_txPktSize++] = 0xFF;
  _repPkt[_txPktSize++] = END_CMD;

  _repPkt[2] = _txPktSize;
}

/*
*
*/
void WiFiBridgeClass::_getEncryption(uint8_t current)
{

  if (current == 1)
  {
    String currSSID = WiFi.SSID(); //get current SSID
    for (int i = 0; i < nets.nScannedNets; i++)
    {
      if (currSSID == WiFi.SSID(i))
      {
        nets.currNetIdx = i; //get the index of the current network
        break;
      }
    }
  }
  else
  {
    nets.currNetIdx = _tempPkt.dataPtr[1];
  }

  uint8_t result = WiFi.encryptionType(nets.currNetIdx);

  // set the response struct
  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_1;
  _repPkt[_txPktSize++] = PARAM_SIZE_1;
  _repPkt[_txPktSize++] = result;
  _repPkt[_txPktSize++] = END_CMD;

  _repPkt[2] = _txPktSize;
}

uint8_t WiFiBridgeClass::_networkListEvalAging(uint8_t connectedIndex)
{
  uint8_t agingVal = 0;
  uint8_t agingValStore = 0;
  uint8_t oldest = 0;

  // request an index to store a new network
  if(connectedIndex == 0xff){
    for(uint8_t i=0; i<MAX_NETWORK_LIST; i++){

      if(networkArray.netEntry[i].aging == 0){
        networkArray.netEntry[i].aging = 1;
        return i;
      }

      // find out the oldest netowrk
      if(agingVal < networkArray.netEntry[i].aging){
        agingVal = networkArray.netEntry[i].aging;
        oldest = i;
      }
    }
  }
  else{
    agingValStore = networkArray.netEntry[connectedIndex].aging;
    networkArray.netEntry[connectedIndex].aging = 1;
    
    for(uint8_t i=0; i<MAX_NETWORK_LIST; i++){
      if(connectedIndex == i)
        continue;
      if(networkArray.netEntry[i].aging != 0 && networkArray.netEntry[i].aging < agingValStore){
        networkArray.netEntry[i].aging++;
      }
    }
  }
  
  return oldest;
}

int8_t WiFiBridgeClass::_checkSSIDInList(char* targetSSID)
{
  int8_t index = -1;

  for(uint8_t j=0; j<MAX_NETWORK_LIST; j++){
    if(strcmp(targetSSID, networkArray.netEntry[j].strSSID) == 0) {
      index = j;
    }
  }
  return index;
}

bool WiFiBridgeClass::_connect(uint8_t option)
{
  bool ret = false;
  uint8_t result;
  uint32_t start = 0;

  connectedIndex_temp = -1;
 
  // autoconnect mode
  if(option == 2) {
    #ifdef DEBUG_SERIALE
      Serial.println("Autoconnect mode");
    #endif

    char ssidNet[33] = {0};
    int32_t rssi = -100;
    int32_t tempRssi = 0;
    bool found = false;

    start = millis() + CONNECTION_TIMEOUT;
    nets.nScannedNets = -1;

    //scanNetworks command
    do {
      nets.nScannedNets = WiFi.scanNetworks(false, false);
      delayMicroseconds(100);
    } while (nets.nScannedNets < 0 && start > millis());

    if (nets.nScannedNets <= 0){
      nets.nScannedNets = 0;
      result = WL_NO_SSID_AVAIL;
      #ifdef DEBUG_SERIALE
        Serial.println("no network found");
      #endif
    }
    else {
      for(uint8_t i=0; i<nets.nScannedNets; i++) {
        strcpy(ssidNet, WiFi.SSID(i).c_str());
        for(uint8_t j=0; j<MAX_NETWORK_LIST; j++){
          if(strcmp((char*)ssidNet, networkArray.netEntry[j].strSSID) == 0) {
            found = true;
            if(networkArray.lastConnected == j){  //last connected -> connect directly
              connectedIndex_temp = j;

              #ifdef DEBUG_SERIALE
                Serial.println("Connect to the last connected");
              #endif
              break;
            }
            tempRssi = WiFi.RSSI(i);
            if(rssi < tempRssi) {
              rssi = tempRssi;
              connectedIndex_temp = j;
            }
          }
        }
        if(found)
          break;
        memset(ssidNet, 0, 33);
      }
    }
    // a network match found
    if(connectedIndex_temp >= 0) {
      #ifdef DEBUG_SERIALE
        Serial.print("\nSSID: ");
        Serial.println((char *)networkArray.netEntry[connectedIndex_temp].strSSID);
        Serial.print("PWD: ");
        Serial.println((char *)networkArray.netEntry[connectedIndex_temp].strPWD);
      #endif

      if(WiFi.status() == WL_CONNECTED){
        //if(networkArray.netEntry[connectedIndex_temp].strSSID != Config.getParam("ssid").c_str() != ssidNet)
          WiFi.disconnect();
      }

      connectionTimeout = millis() + CONNECTION_TIMEOUT;
      connectionStatus = WL_IDLE_STATUS;

      result = WiFi.begin(networkArray.netEntry[connectedIndex_temp].strSSID, networkArray.netEntry[connectedIndex_temp].strPWD);
    }
    else{
      #ifdef DEBUG_SERIALE
        Serial.println("No network memorized or match found");
      #endif
    }
  }
  else {
    //get parameter
    tsParameter ssid = {
      .len = _tempPkt.dataPtr[0],
      .paramPtr = (uint8_t *)(_tempPkt.dataPtr + 1),
    };

    char pwdBuf[65];
    // string size + terminator
    char ssidBuf[ssid.len + 1];
    memset(ssidBuf, 0, ssid.len + 1);
    memcpy(ssidBuf, ssid.paramPtr, ssid.len);
    ssidBuf[ssid.len] = '\0';

    // Open network, connection without password
    if (option == 0) {
      if(WiFi.status() == WL_CONNECTED){
        //if(networkArray.netEntry[connectedIndex_temp].strSSID != Config.getParam("ssid").c_str() != ssidNet)
          WiFi.disconnect();
      }

      connectionTimeout = millis() + CONNECTION_TIMEOUT;
      connectionStatus = WL_IDLE_STATUS;
      
      //set network and retrieve result
      result = WiFi.begin(ssidBuf);
      memset(tempNetworkParam.strSSID, 0, 33);
      memset(tempNetworkParam.strPWD, 0, 65);
      tempNetworkParam.aging = 0;

      strcpy(tempNetworkParam.strSSID, ssidBuf);
      _connectionRequested = true;
    }
    // secured network, connection with password
    else if (option == 1) {
      //get parameter
      tsParameter pwd = {
        .len = _tempPkt.dataPtr[ssid.len + 1],
        .paramPtr = (uint8_t *)(_tempPkt.dataPtr + ssid.len + 2),
      };
      // string size + terminator
      //char pwdBuf[pwd.len + 1];
      memset(pwdBuf, 0, pwd.len + 1);
      memcpy(pwdBuf, pwd.paramPtr, pwd.len);
      pwdBuf[pwd.len] = '\0';

  #ifdef DEBUG_SERIALE
      Serial.print("\nSSID: ");
      Serial.println((char *)ssidBuf);
      Serial.print("PWD: ");
      Serial.println((char *)pwdBuf);
  #endif
      if(WiFi.status() == WL_CONNECTED){
        //if(networkArray.netEntry[connectedIndex_temp].strSSID != Config.getParam("ssid").c_str() != ssidNet)
          WiFi.disconnect();
      }

      connectionTimeout = millis() + CONNECTION_TIMEOUT;
      connectionStatus = WL_IDLE_STATUS;

      //set network and retrieve result
      result = WiFi.begin(ssidBuf, pwdBuf);
      memset(tempNetworkParam.strSSID, 0, 33);
      memset(tempNetworkParam.strPWD, 0, 65);
      tempNetworkParam.aging = 0;

      strcpy(tempNetworkParam.strSSID, ssidBuf);
      strcpy(tempNetworkParam.strPWD, pwdBuf);

      _connectionRequested = true;
    }
    else {
      _createErrorResponse();
      return ret;
    }
  }
  return ret;
}

/*
*
*/
bool WiFiBridgeClass::_disconnect()
{
  bool ret = false;
  //Disconnet from the network
  uint8_t result = WiFi.disconnect();
  return ret;
}

//ok
/*
* START_CMD, CMD | REPLY_FLAG, PARAM_NUMS_1, PARAM_SIZE_1, numNets, END_CMD
*/
void WiFiBridgeClass::_startScanNetwork()
{
  interface.timeout = millis() + 5000;

  //scanNetworks command
  do
  {
    nets.nScannedNets = WiFi.scanNetworks(false, false);
    delayMicroseconds(100);
  } while (nets.nScannedNets < 0 && interface.timeout > millis());

  if (nets.nScannedNets < 0)
    nets.nScannedNets = 0;

  //set the response struct
  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_1;
  _repPkt[_txPktSize++] = PARAM_SIZE_1;
  _repPkt[_txPktSize++] = nets.nScannedNets;
  _repPkt[_txPktSize++] = END_CMD;

  _repPkt[2] = _txPktSize;
}

/*
* START_CMD, CMD | REPLY_FLAG, NET_NUMBER, SSID_LEN, SSID_NAME, END_CMD
*/
void WiFiBridgeClass::_scanNetwork(uint8_t which)
{
  String ssidNet = WiFi.SSID(which);
  uint8_t enc = WiFi.encryptionType(which);
  int32_t rssi = WiFi.RSSI(which);
  delay(10);

  uint32_t result = ssidNet.length() + sizeof(enc) + sizeof(rssi) + 2;
  _repPkt[_txPktSize++] = DATA_PKT;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = (uint8_t)(result & 0xff);
  _repPkt[_txPktSize++] = (uint8_t)((result & 0xff00) >> 8);
  _repPkt[_txPktSize++] = (uint8_t)((result & 0xff0000) >> 16);
  _repPkt[_txPktSize++] = (uint8_t)((result & 0xff000000) >> 24);
  _repPkt[_txPktSize++] = which;
  _repPkt[_txPktSize++] = (uint8_t)rssi;
  _repPkt[_txPktSize++] = 0xff;
  _repPkt[_txPktSize++] = 0xff;
  _repPkt[_txPktSize++] = 0xff;
  _repPkt[_txPktSize++] = enc;
  _repPkt[_txPktSize++] = ssidNet.length();
  memcpy((_repPkt + _txPktSize), ssidNet.c_str(), ssidNet.length());
  _txPktSize += ssidNet.length();
  _repPkt[_txPktSize++] = END_CMD;
}

/*
*
*/
void WiFiBridgeClass::_getBSSID(uint8_t current)
{
  int paramLen = 6;
  uint8_t idx = 0; //network index
  uint8_t *result;

  //Retrive the BSSID
  result = WiFi.BSSID();

  //set the response struct
  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_1;
  _repPkt[_txPktSize++] = paramLen;
  for (int j = paramLen - 1; j >= 0; j--)
  {
    _repPkt[_txPktSize++] = result[j];
  }
  _repPkt[_txPktSize++] = END_CMD;

  _repPkt[2] = _txPktSize;
}

void WiFiBridgeClass::_config()
{
  bool result;
  uint8_t validParams = 0;

  uint8_t stip0, stip1, stip2, stip3,
    gwip0, gwip1, gwip2, gwip3,
    snip0, snip1, snip2, snip3;

  validParams = _tempPkt.dataPtr[1];

  //retrieve the static IP address
  stip0 = _tempPkt.dataPtr[3];
  stip1 = _tempPkt.dataPtr[4];
  stip2 = _tempPkt.dataPtr[5];
  stip3 = _tempPkt.dataPtr[6];
  _handyIp = new IPAddress(stip0, stip1, stip2, stip3);

  //retrieve the gateway IP address
  gwip0 = _tempPkt.dataPtr[8];
  gwip1 = _tempPkt.dataPtr[9];
  gwip2 = _tempPkt.dataPtr[10];
  gwip3 = _tempPkt.dataPtr[11];
  _handyGateway = new IPAddress(gwip0, gwip1, gwip2, gwip3);

  //retrieve the subnet mask
  snip0 = _tempPkt.dataPtr[13];
  snip1 = _tempPkt.dataPtr[14];
  snip2 = _tempPkt.dataPtr[15];
  snip3 = _tempPkt.dataPtr[16];
  _handySubnet = new IPAddress(snip0, snip1, snip2, snip3);

  result = WiFi.config(*_handyIp, *_handyGateway, *_handySubnet);

  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_1;
  _repPkt[_txPktSize++] = PARAM_SIZE_1;
  _repPkt[_txPktSize++] = result;
  _repPkt[_txPktSize++] = END_CMD;

  _repPkt[2] = _txPktSize;
}

void WiFiBridgeClass::_setDNS()
{
  bool result;
  uint8_t validParams = 0;

  validParams = _tempPkt.dataPtr[1];

  uint8_t dns1ip0, dns1ip1, dns1ip2, dns1ip3,
    dns2ip0, dns2ip1, dns2ip2, dns2ip3;

  //retrieve the dns 1 address
  dns1ip0 = _tempPkt.dataPtr[3];
  dns1ip1 = _tempPkt.dataPtr[4];
  dns1ip2 = _tempPkt.dataPtr[5];
  dns1ip3 = _tempPkt.dataPtr[6];
  IPAddress dns1(dns1ip0, dns1ip1, dns1ip2, dns1ip3);

  //retrieve the dns 2 address
  dns2ip0 = _tempPkt.dataPtr[8];
  dns2ip1 = _tempPkt.dataPtr[9];
  dns2ip2 = _tempPkt.dataPtr[10];
  dns2ip3 = _tempPkt.dataPtr[11];
  IPAddress dns2(dns2ip0, dns2ip1, dns2ip2, dns2ip3);

  result = WiFi.config(*_handyIp, *_handyGateway, *_handySubnet, dns1, dns2);

  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_1;
  _repPkt[_txPktSize++] = PARAM_SIZE_1;
  _repPkt[_txPktSize++] = result;
  _repPkt[_txPktSize++] = END_CMD;

  _repPkt[2] = _txPktSize;
}

void WiFiBridgeClass::_getHostByName()
{
  uint8_t result;
  IPAddress _reqHostIp;
  char host[_tempPkt.dataPtr[0]+1];
  //get the host name to look up
  //strncpy(host, _reqPckt.params[0].param, _reqPckt.params[0].paramLen);
  memcpy(host, &_tempPkt.dataPtr[1], _tempPkt.dataPtr[0]);
  host[_tempPkt.dataPtr[0]] = '\0';

  result = WiFi.hostByName(host, _reqHostIp); //retrieve the ip address of the host

  
  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_1;

  if(result == 0){ // call failed, return 0
    _repPkt[_txPktSize++] = PARAM_SIZE_1;
    _repPkt[_txPktSize++] = result;
    _repPkt[_txPktSize++] = END_CMD;
  }
  else{ //call succeded, return host's ip
    _repPkt[_txPktSize++] = PARAM_SIZE_4;
    _repPkt[_txPktSize++] = _reqHostIp.operator[](0);
    _repPkt[_txPktSize++] = _reqHostIp.operator[](1);
    _repPkt[_txPktSize++] = _reqHostIp.operator[](2);
    _repPkt[_txPktSize++] = _reqHostIp.operator[](3);
    _repPkt[_txPktSize++] = END_CMD;
  }
  _repPkt[2] = _txPktSize;
}

void WiFiBridgeClass::_getNetworkData()
{
  IPAddress localIp, subnetMask, gatewayIp; //, dnsIp1, dnsIp2;

  subnetMask = WiFi.subnetMask();
  gatewayIp = WiFi.gatewayIP();
  localIp = WiFi.localIP();
  
  uint32_t tim = millis() + 5000;

  while(localIp.operator[](0) == 0 && millis() < tim){
    delay(1);
    localIp = WiFi.localIP();
  }

  //set the response struct
  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_3;

  _repPkt[_txPktSize++] = PARAM_SIZE_4;
  _repPkt[_txPktSize++] = localIp.operator[](0);
  _repPkt[_txPktSize++] = localIp.operator[](1);
  _repPkt[_txPktSize++] = localIp.operator[](2);
  _repPkt[_txPktSize++] = localIp.operator[](3);

  _repPkt[_txPktSize++] = PARAM_SIZE_4;
  _repPkt[_txPktSize++] = subnetMask.operator[](0);
  _repPkt[_txPktSize++] = subnetMask.operator[](1);
  _repPkt[_txPktSize++] = subnetMask.operator[](2);
  _repPkt[_txPktSize++] = subnetMask.operator[](3);

  _repPkt[_txPktSize++] = PARAM_SIZE_4;
  _repPkt[_txPktSize++] = gatewayIp.operator[](0);
  _repPkt[_txPktSize++] = gatewayIp.operator[](1);
  _repPkt[_txPktSize++] = gatewayIp.operator[](2);
  _repPkt[_txPktSize++] = gatewayIp.operator[](3);
  _repPkt[_txPktSize++] = END_CMD;

  _repPkt[2] = _txPktSize;
}

/* WiFI Server */
void WiFiBridgeClass::_startServer()
{
  uint8_t result = 0;

  uint8_t _p1 = _tempPkt.dataPtr[1];
  uint8_t _p2 = _tempPkt.dataPtr[2];

  //retrieve the port to start server
  uint16_t _port = (_p1 << 8) + _p2;
  //retrieve sockets number
  uint8_t _sock = _tempPkt.dataPtr[4];
  //retrieve protocol mode (TCP/UDP)
  uint8_t _prot = _tempPkt.dataPtr[6];

  if (_sock < MAX_SOCK_NUMBER)
  {
    if (_prot == 0)
    { //TCP MODE
      if (mapWiFiServers[_sock] != NULL)
      {
        //mapWiFiServers[_sock]->stop();
        mapWiFiServers[_sock]->close();
        free(mapWiFiServers[_sock]);
      }
      if (_port == 80)
      {
        //UIserver.stop();      //stop UI SERVER
        UI_alert = true;
      }
      mapWiFiServers[_sock] = new WiFiServer(_port);
      mapWiFiServers[_sock]->begin();
      result = 1;
    }
    else
    { //UDP MODE
      if (mapWiFiUDP[_sock] != NULL)
      {
        mapWiFiUDP[_sock].stop();
      }
      mapWiFiUDP[_sock].begin(_port);
      _sockInUse[_sock].inUse = true;
      _sockInUse[_sock].conn = false;
      result = 1;
    }
  }

  //set the response struct
  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_1;
  _repPkt[_txPktSize++] = PARAM_SIZE_1;
  _repPkt[_txPktSize++] = result;
  _repPkt[_txPktSize++] = END_CMD;

  _repPkt[2] = _txPktSize;
}

void WiFiBridgeClass::_serverStatus()
{
  uint8_t result = 0;
  uint8_t _sock = 0;

  //retrieve socket index
  _sock = _tempPkt.dataPtr[1];
  if (mapWiFiServers[_sock] != NULL)
    result = mapWiFiServers[_sock]->status();
  else
    result = 0;

  //set the response struct
  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_1;
  _repPkt[_txPktSize++] = PARAM_SIZE_1;
  _repPkt[_txPktSize++] = result;
  _repPkt[_txPktSize++] = END_CMD;

  _repPkt[2] = _txPktSize;
}

void WiFiBridgeClass::_getData()
{
  uint8_t result = 0;
  //retrieve socket index
  uint8_t _sock = _tempPkt.dataPtr[1];
  //retrieve peek
  uint8_t _p1 = _tempPkt.dataPtr[3];
  uint8_t _p2 = _tempPkt.dataPtr[4];
  uint16_t _peek = (_p1 << 8) + _p2;

  if (mapWiFiClients[_sock]) {
    if (_peek > 0) {
      result = mapWiFiClients[_sock].peek();
    }
    else {
      result = mapWiFiClients[_sock].read();
    }
  }
  else if (mapWiFiUDP[_sock] != NULL) {
    if (_peek > 0) {
      if(UdpData.udpIndex < UdpData.udpBufLen)
        result = UdpData.udpBuf[UdpData.udpIndex];
    }
    else {
      if(UdpData.udpIndex < UdpData.udpBufLen)
        result = UdpData.udpBuf[UdpData.udpIndex++];
    }
  }

  //set the response struct
  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_1;
  _repPkt[_txPktSize++] = PARAM_SIZE_1;
  _repPkt[_txPktSize++] = result;
  _repPkt[_txPktSize++] = END_CMD;

  _repPkt[2] = _txPktSize;
}

void WiFiBridgeClass::_availSocketData()
{
  int result = 0;
  //retrieve socket index
  uint8_t _sock = _tempPkt.dataPtr[1];
  if (_sock < MAX_SOCK_NUMBER) {
    if(mapWiFiClients[_sock] != NULL){
      int res = mapWiFiClients[_sock].available();
      if(res > 0)
        result = res;
    }
    else if(mapWiFiUDP[_sock] != NULL){
      result = mapWiFiUDP[_sock].parsePacket();
      if (result > 0){ 
        // save data in udp struct and notify presence of new data
        mapWiFiUDP[_sock].read(UdpData.udpBuf, result);
        UdpData.udpBufLen = result;
        UdpData.udpAddress = mapWiFiUDP[_sock].remoteIP();
        UdpData.udpPort = mapWiFiUDP[_sock].remotePort();
        UdpData.udpIndex = 0;
      }
    }
  }

  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_2;
  _repPkt[_txPktSize++] = PARAM_SIZE_1;
  _repPkt[_txPktSize++] = _sock;
  _repPkt[_txPktSize++] = 2; // PARAM_SIZE
  _repPkt[_txPktSize++] = ((uint8_t *)&result)[0];
  _repPkt[_txPktSize++] = ((uint8_t *)&result)[1];

  _repPkt[_txPktSize++] = END_CMD;
  _repPkt[2] = _txPktSize;
}

/* WiFi Client */
void WiFiBridgeClass::_stopClient()
{
  uint8_t result = 0;
  uint8_t _sock = _tempPkt.dataPtr[1];
  if (_sock < MAX_SOCK_NUMBER)
  {
    if (mapWiFiClients[_sock])
    { //!= NULL
      mapWiFiClients[_sock].stop();
      _sockInUse[_sock].inUse = false;
      //mapWiFiClients[_sock] = NULL;
      result = 1;
    }
    else if (mapWiFiUDP[_sock] != NULL)
    {
      mapWiFiUDP[_sock].stop();
      _sockInUse[_sock].inUse = false;
      result = 1;
    }
  }

  //set the response struct
  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_1;
  _repPkt[_txPktSize++] = PARAM_SIZE_1;
  _repPkt[_txPktSize++] = result;
  _repPkt[_txPktSize++] = END_CMD;

  _repPkt[2] = _txPktSize;
}

void WiFiBridgeClass::_clientStatus()
{
  uint8_t result = 0;
  uint8_t _sock = (uint8_t)_tempPkt.dataPtr[1];

  if (_sock < MAX_SOCK_NUMBER) {
    if (!_sockInUse[_sock].inUse) {
      if (mapWiFiServers[_sock] != NULL) {
        mapWiFiClients[_sock] = mapWiFiServers[_sock]->available(); //Create the client from the server [Arduino as a Server]
        result = mapWiFiClients[_sock].status();
        if(result > 0)
          _sockInUse[_sock].inUse = true;
      }
    }
    else {
      result = mapWiFiClients[_sock].status();
    }
  }

  //set the response struct
  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_2;
  _repPkt[_txPktSize++] = PARAM_SIZE_1;
  _repPkt[_txPktSize++] = _sock;
  _repPkt[_txPktSize++] = PARAM_SIZE_1;
  _repPkt[_txPktSize++] = result;
  _repPkt[_txPktSize++] = END_CMD;

  _repPkt[2] = _txPktSize;
}

bool WiFiBridgeClass::_sendData()
{
  if(_tempPkt.cmdType == DATA_PKT && interface.recPacket.endReceived == false) {
    if(_tempPkt.nParam == 25) {
      interface.sockBuffInit();
      interface.sockBuffAdd(_tempPkt.dataPtr, _tempPkt.nParam);
    }
    else{
      interface.sockBuffAdd(_tempPkt.dataPtr, _tempPkt.nParam);
    }
  }
  else {
    if(_tempPkt.nParam == 255){ // 255 indicates a special case where data packet payload fits 1 packet
      interface.sockBuffInit();
      interface.sockBuffAdd(_tempPkt.dataPtr, _tempPkt.totalLen);
    }
    else{
      interface.sockBuffAdd(_tempPkt.dataPtr, _tempPkt.nParam);
    }

  #ifdef DEBUG_SERIALE
      Serial.print("\r\nsocket msg: ");
      Serial.write(interface._socketPkt, interface._socketPktPtr);
  #endif

    uint8_t _sock = interface.recPacket.sockN;
    uint32_t result = 0;
    uint8_t tcpResult = 0;
    
    if (_sock < MAX_SOCK_NUMBER && _sockInUse[_sock].inUse) {
      if(mapWiFiClients[_sock]){
        if (mapWiFiClients[_sock].status() == 4){
          result = mapWiFiClients[_sock].write((uint8_t *)(interface._socketPkt), interface._socketPktPtr);
        }
      }
      else if(mapWiFiUDP[_sock]){ 
        result = mapWiFiUDP[_sock].write((uint8_t *)(interface._socketPkt), interface._socketPktPtr);
      }
      if (result == interface._socketPktPtr)
        tcpResult = 1;
      else
        tcpResult = 0;
    }
  }
  
  return false;
}

void WiFiBridgeClass::_startClient()
{
  int result = 0;
  int _sock;
  uint16_t _port;
  uint8_t _prot;

  //retrieve the IP address to connect to
  uint8_t stip1 = _tempPkt.dataPtr[1];
  uint8_t stip2 = _tempPkt.dataPtr[2];
  uint8_t stip3 = _tempPkt.dataPtr[3];
  uint8_t stip4 = _tempPkt.dataPtr[4];
  IPAddress _ip(stip1, stip2, stip3, stip4);

  //retrieve the port to connect to
  uint8_t _p1 = _tempPkt.dataPtr[6];
  uint8_t _p2 = _tempPkt.dataPtr[7];
  _port = (_p1 << 8) + _p2;

  //retrieve sockets number
  _sock = (int)_tempPkt.dataPtr[9];

  //retrieve protocol mode (TCP/UDP)
  _prot = _tempPkt.dataPtr[11];

  if (_sock < MAX_SOCK_NUMBER) {
    if (_prot == 0) {
      //TCP MODE
      if (mapWiFiClients[_sock]) {
        WiFiClient wc;
        mapWiFiClients[_sock] = wc;
      }
      result = mapWiFiClients[_sock].connect(_ip, _port);
      if(result)
        _sockInUse[_sock].inUse = true;
    }
    else {
      //UDP MODE
      if (mapWiFiUDP[_sock] == NULL) {
        WiFiUDP wu;
        mapWiFiUDP[_sock] = wu;
      }
      result = mapWiFiUDP[_sock].beginPacket(_ip, _port);
    }
  }
  if (result) {
    _sockInUse[_sock].conn = true;
  }
  //set the response struct
  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_1;
  _repPkt[_txPktSize++] = PARAM_SIZE_1;
  _repPkt[_txPktSize++] = result;
  _repPkt[_txPktSize++] = END_CMD;

  _repPkt[2] = _txPktSize;  
}

/* WiFi UDP Client */
void WiFiBridgeClass::_remoteData()
{
  uint8_t _sock;

  //retrieve sockets number
  _sock = _tempPkt.dataPtr[1];

  if (_sock < MAX_SOCK_NUMBER && mapWiFiUDP[_sock] != NULL)
  {
    IPAddress remoteIp = UdpData.udpAddress;
    uint16_t remotePort = UdpData.udpPort;

      
    //set the response struct
    _repPkt[_txPktSize++] = _tempPkt.cmdType;
    _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
    _repPkt[_txPktSize++] = 0;
    _repPkt[_txPktSize++] = PARAM_NUMS_2;
    _repPkt[_txPktSize++] = PARAM_SIZE_4;
    _repPkt[_txPktSize++] = remoteIp.operator[](0);
    _repPkt[_txPktSize++] = remoteIp.operator[](1);
    _repPkt[_txPktSize++] = remoteIp.operator[](2);
    _repPkt[_txPktSize++] = remoteIp.operator[](3);
    _repPkt[_txPktSize++] = PARAM_SIZE_2;
    _repPkt[_txPktSize++] = (uint8_t)((remotePort & 0xff00) >> 8);
    _repPkt[_txPktSize++] = (uint8_t)(remotePort & 0xff);
    _repPkt[_txPktSize++] = END_CMD;

    _repPkt[2] = _txPktSize;      
  }
  else
  {
    // send 0 to report an error
    _repPkt[_txPktSize++] = _tempPkt.cmdType;
    _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
    _repPkt[_txPktSize++] = 0;
    _repPkt[_txPktSize++] = PARAM_NUMS_1;
    _repPkt[_txPktSize++] = PARAM_SIZE_1;
    _repPkt[_txPktSize++] = 0;
    _repPkt[_txPktSize++] = END_CMD;

    _repPkt[2] = _txPktSize;  
  }
}

void WiFiBridgeClass::_getDataBuf()
{
  int32_t result = 0;
  uint8_t _sock = _tempPkt.dataPtr[1];
  uint16_t _len = _tempPkt.dataPtr[4] + (_tempPkt.dataPtr[3] << 8);

  //set the response struct
  _repPkt[_txPktSize++] = DATA_PKT; //_tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  
  if (_sock < MAX_SOCK_NUMBER) { 
    if (mapWiFiUDP[_sock] != NULL){
      // read just available data
        uint32_t readLen = (_len > (UdpData.udpBufLen - UdpData.udpIndex)) ? (UdpData.udpBufLen - UdpData.udpIndex) : _len;
        result = readLen;
        memcpy(interface._longCmdPkt, UdpData.udpBuf + UdpData.udpIndex, readLen);
        UdpData.udpIndex += readLen; 
    }
    else if (mapWiFiClients[_sock]) {
      result = mapWiFiClients[_sock].read(interface._longCmdPkt, _len);
    }
  }
  
  #ifdef DEBUG_SERIALE
   //   Serial.print("\r\nget data buf ");
   //   Serial.println(result);
  #endif

  _repPkt[_txPktSize++] = (uint8_t)(result & 0xff);
  _repPkt[_txPktSize++] = (uint8_t)((result & 0xff00) >> 8);
  _repPkt[_txPktSize++] = (uint8_t)((result & 0xff0000) >> 16);
  _repPkt[_txPktSize++] = (uint8_t)((result & 0xff000000) >> 24);

  memcpy((_repPkt + _txPktSize), interface._longCmdPkt, result);
  _txPktSize += result; 
 
  _repPkt[_txPktSize++] = END_CMD;
  
  delay(0); // perform a yield to speed up process
}

/*
*
*/
void WiFiBridgeClass::_sendUdpData()
{
  int result = 0;
  uint8_t _sock = 0;

  //retrieve socket index
  _sock = _tempPkt.dataPtr[1];

  if (_sock < MAX_SOCK_NUMBER && mapWiFiUDP[_sock] != NULL)
  {
    //send data to client
    result = mapWiFiUDP[_sock].endPacket();
    _sockInUse[_sock].conn = false;
  }

  //set the response struct
  _repPkt[_txPktSize++] = _tempPkt.cmdType;
  _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_1;
  _repPkt[_txPktSize++] = PARAM_SIZE_1;
  _repPkt[_txPktSize++] = result;
  _repPkt[_txPktSize++] = END_CMD;

  _repPkt[2] = _txPktSize;  
}

/*
*
*/
bool WiFiBridgeClass::_txrxMsg()
{
  bool ret = false;

  if(_tempPkt.cmdType == DATA_PKT && interface.recPacket.endReceived == false) {
    if(_tempPkt.nParam == 25){
      interface.sockBuffInit();
      interface.sockBuffAdd(_tempPkt.dataPtr, _tempPkt.nParam);
    }
    else
      interface.sockBuffAdd(_tempPkt.dataPtr, _tempPkt.nParam);
  }
  else {
    interface.sockBuffAdd(_tempPkt.dataPtr, _tempPkt.nParam);

    msgNum++;

    // _longCmdPkt len + header & footer
    
    uint32_t extLen = interface._socketPktPtr + DATA_HEAD_FOOT_LEN;
    //set the response struct
    _repPkt[_txPktSize++] = _tempPkt.cmdType;
    _repPkt[_txPktSize++] = _tempPkt.cmd | REPLY_FLAG; //it should be | (REPLY_FLAG)
    _repPkt[_txPktSize++] = extLen & 0xFF;
    _repPkt[_txPktSize++] = (extLen >> 8) & 0xFF;
    _repPkt[_txPktSize++] = (extLen >> 16) & 0xFF;
    _repPkt[_txPktSize++] = (extLen >> 24) & 0xFF;
    _repPkt[_txPktSize++] = interface.recPacket.sockN;
    // I don't insert socket number because I didn't extract it from the received packet, hence I use that
    memcpy((_repPkt + _txPktSize), interface._socketPkt, interface._socketPktPtr);
    _txPktSize += interface._socketPktPtr;
    _repPkt[_txPktSize++] = END_CMD;

    ret = true;
/*
#ifdef DEBUG_SERIALE
    Serial.print(msgNum);
    //Serial.println(" msg: ");
    //Serial.println((char *)interface._socketPkt);
    //Serial.write(_repPkt, _txPktSize);
    Serial.println(_tempPkt.totalLen);
    Serial.write(interface._socketPkt, interface._socketPktPtr);
    delay(1);
#endif
*/    
  }
  return ret;
}

void WiFiBridgeClass::_fetchWiFiSSIDList()
{
  uint8_t val = 0;
  uint8_t* addr = (uint8_t*)&networkArray;

  for(uint16_t i=0; i<sizeof(tsNetArray); i++){
    val = EEPROM.read(i);
    memcpy((uint8_t*)(addr + i), (uint8_t*)&val, 1);
  }
}

void WiFiBridgeClass::_storeWiFiSSIDList()
{
  uint8_t* addr = (uint8_t*)&networkArray;

  for (uint16_t i = 0; i < 1024; i++) {
    delay(1);
    if(i <= sizeof(tsNetArray)){
      EEPROM.write(i, *((uint8_t*)(addr + i)));
    }
    else
      EEPROM.write(i, 0xff);
  }
  EEPROM.commit();
}

void WiFiBridgeClass::_clearWiFiSSIDList()
{
  for (uint16_t i = 0; i < 1024; i++) {
    EEPROM.write(i, 0xff);
  }
  EEPROM.commit();
}

// ----------------------------------------------------- PUBLIC
void WiFiBridgeClass::notifyEspReady()
{
  _txPktSize = 0;

  _repPkt[_txPktSize++] = START_CMD;
  _repPkt[_txPktSize++] = ESP_READY | REPLY_FLAG;
  _repPkt[_txPktSize++] = 0;
  _repPkt[_txPktSize++] = PARAM_NUMS_1;
  _repPkt[_txPktSize++] = PARAM_SIZE_1;
  _repPkt[_txPktSize++] = 1;
  _repPkt[_txPktSize++] = END_CMD;

  _repPkt[2] = _txPktSize;  
  
  interface.write(_repPkt, _txPktSize);

}

bool WiFiBridgeClass::autoconnect()
{
  uint8_t result;
  char ssidNet[33] = {0};
  int32_t rssi = -100;
  int32_t tempRssi = 0;
  uint32_t start = 0;
  bool found = false;

  start = millis() + CONNECTION_TIMEOUT;
  nets.nScannedNets = -1;

  //scanNetworks command
  do {
    nets.nScannedNets = WiFi.scanNetworks(false, false);
    delayMicroseconds(100);
  } while (nets.nScannedNets < 0 && start > millis());

  if (nets.nScannedNets <= 0){
    nets.nScannedNets = 0;
    result = WL_NO_SSID_AVAIL;
    #ifdef DEBUG_SERIALE
      Serial.println("no network found");
    #endif
  }
  else {
    for(uint8_t i=0; i<nets.nScannedNets; i++) {
      strcpy(ssidNet, WiFi.SSID(i).c_str());
      for(uint8_t j=0; j<MAX_NETWORK_LIST; j++){
        if(strcmp((char*)ssidNet, networkArray.netEntry[j].strSSID) == 0) {
          found = true;
          if(networkArray.lastConnected == j){  //last connected -> connect directly
            connectedIndex_temp = j;

            #ifdef DEBUG_SERIALE
              Serial.println("Connect to the last connected");
            #endif
            break;
          }
          tempRssi = WiFi.RSSI(i);
          if(rssi < tempRssi) {
            rssi = tempRssi;
            connectedIndex_temp = j;
          }
        }
      }
      if(found)
        break;
      memset(ssidNet, 0, 33);
    }
  }
  // a network match found
  if(connectedIndex_temp >= 0) {
    #ifdef DEBUG_SERIALE
      Serial.print("\nSSID: ");
      Serial.println((char *)networkArray.netEntry[connectedIndex_temp].strSSID);
      Serial.print("PWD: ");
      Serial.println((char *)networkArray.netEntry[connectedIndex_temp].strPWD);
    #endif
    
    if(WiFi.status() == WL_CONNECTED){
      //if(networkArray.netEntry[connectedIndex_temp].strSSID != Config.getParam("ssid").c_str() != ssidNet)
        WiFi.disconnect();
    }

    connectionTimeout = millis() + CONNECTION_TIMEOUT;
    connectionStatus = WL_IDLE_STATUS;

    result = WiFi.begin(networkArray.netEntry[connectedIndex_temp].strSSID, networkArray.netEntry[connectedIndex_temp].strPWD);
  }
  else{
    #ifdef DEBUG_SERIALE
      Serial.println("No network memorized or match found");
    #endif
  }

  return found;
}

/*
*
*/
WiFiBridgeClass::WiFiBridgeClass()
{
}

/*
*
*/
void WiFiBridgeClass::begin()
{
  interface.begin();

  //_clearWiFiSSIDList();

  // fetch WiFi network list from flash memory
  _fetchWiFiSSIDList();

/*
  #ifdef DEBUG_SERIALE
    for(uint8_t i=0; i<MAX_NETWORK_LIST; i++){
      Serial.print("\nSSID: ");
      Serial.write(networkArray.netEntry[i].strSSID, 32);
      Serial.print("\nPWD: ");
      Serial.write(networkArray.netEntry[i].strPWD, 64);
      Serial.print("\naging: ");
      Serial.println(networkArray.netEntry[i].aging);
    }
  #endif
*/
  _prepareReplayBuf();
  nets.nScannedNets = 0;
  nets.currNetIdx = 0;
  
  // _onConnected is the shared_pointer to onStationModeConnected event
  _onConnected = WiFi.onStationModeConnected([this](const WiFiEventStationModeConnected & event) {
    #ifdef DEBUG_SERIALE
      Serial.print("Station connected: ");
    #endif
    connectionStatus = WL_CONNECTED;
    interface.enableNotify();
    _connectionRequested = false;
    //_notifySTAConnStatus(WL_CONNECTED);
  });

  _onDisconnected = WiFi.onStationModeDisconnected([this](const WiFiEventStationModeDisconnected &event) {
    #ifdef DEBUG_SERIALE
      Serial.print("Station disconnected: ");
    #endif
    //if(connectionStatus == WL_CONNECTED){
    connectionStatus = WL_DISCONNECTED;
    interface.enableNotify();
    if(_connectionRequested){ // try to connect to a stored network if previous connection requested, by specifying ssid and pwd, failed
      _connectionRequested = false;
      _startAutoconnect = true;
    }
    //}
    //_notifySTAConnStatus(WL_DISCONNECTED);
  });
}

/*
*
*/
void WiFiBridgeClass::handle()
{
  if (interface.canProcess()) {
    uint32_t deltaT = 0;
    interface.endProcessing();

    srTimer = micros();
    // go into busy mode
    interface.setSR(LOW);
    if (interface.read(&_tempPkt) == 0){
      _prepareReply = _process();

      if(notifyAck){
        notifyAck = false;
        delayMicroseconds(10);
        interface.setSR(HIGH);
        interface.setSR(LOW);
      }
      if (_prepareReply){
        interface.write(_repPkt, _txPktSize);
        _txPktSize = 0;
      }
    }
    else{
      if(notifyAck){
        notifyAck = false;
        delayMicroseconds(10);
        interface.setSR(HIGH);
        interface.setSR(LOW);
      }
    }
  }

  if (connectionTimeout != 0 && connectionTimeout < millis() && connectionStatus != WL_CONNECTED){
    connectionTimeout = 0;
    WiFi.disconnect();
    #ifdef DEBUG_SERIALE
      Serial.println("Connection timeout");
    #endif
  }

  if(_startAutoconnect){
    _startAutoconnect = false;
    autoconnect();
  }

  if (interface.canNotify()) {
    interface.endNotify();
    _notifySTAConnStatus(connectionStatus);
    interface.write(interface._notifyMsg, _notifyMsgCntr);
  }

  if(interface.pktRecvTimeout < millis())
  {
    interface.pktRecvTimeout = 0;
    interface.recPacket.receivedLen = interface.longCmd.totalLen = 0;
  }
}

WiFiBridgeClass wifiBridge;
