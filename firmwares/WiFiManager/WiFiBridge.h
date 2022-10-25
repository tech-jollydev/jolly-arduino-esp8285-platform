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

#include "Arduino.h"
#include "config.h"
#include <ESP8266WiFi.h>
#include <inttypes.h>
#include <WiFiUdp.h>
#include <EEPROM.h>

#define SPI_TIMEOUT         1000

#define CONNECTION_TIMEOUT  10000

#define CMD_FLAG            0
#define REPLY_FLAG          1<<7
#define DATA_FLAG           0x40

#define WIFI_SPI_ACK        1
#define WIFI_SPI_ERR        0xFF

//#define TIMEOUT_CHAR      1000

#define MAX_SOCK_NUMBER     4 /**< Maxmium number of socket  */
#define NO_SOCKET_AVAIL     255

#define START_CMD           0xe0
#define DATA_PKT            0xf0
#define END_CMD             0xee
#define ERR_CMD             0xef

#define MAX_CMD_LEN         600 //256
#define MTU_SIZE            1401  // MTU size = 1400 + sizeof(sock) 
#define SPI_BUFFER_SIZE     32

#define MAX_NETWORK_LIST    10

#define DATA_HEAD_FOOT_LEN  8 //7 header: cmdType(1), cmd(1), size(4), sock(1); 1 footer: end(1)

enum numParams{
    PARAM_NUMS_0 = 0,
    PARAM_NUMS_1,
    PARAM_NUMS_2,
    PARAM_NUMS_3,
    PARAM_NUMS_4,
    PARAM_NUMS_5,
    MAX_PARAM_NUMS
};

enum sizeParams{
    PARAM_SIZE_0 = 0,
    PARAM_SIZE_1,
    PARAM_SIZE_2,
    PARAM_SIZE_3,
    PARAM_SIZE_4,
    PARAM_SIZE_5,
    PARAM_SIZE_6
};

typedef struct __attribute__((__packed__))
{
  char strSSID[33];
  char strPWD[65];
  uint8_t aging;
} tsNetParam;

typedef struct __attribute__((__packed__))
{
  tsNetParam netEntry[MAX_NETWORK_LIST];
  uint8_t lastConnected;
} tsNetArray;

#define MAX_PARAMS MAX_PARAM_NUMS-1

typedef struct __attribute__((__packed__))
{
  uint8_t cmdType;
  uint8_t cmd;
  uint32_t totalLen;
  uint32_t receivedLen;
  uint8_t sockN;
  bool endReceived;
} tsDataPacket;

typedef struct __attribute__((__packed__))
{
  uint8_t cmdType;
  uint8_t cmd;
  uint8_t nParam;
  uint32_t totalLen;
  uint8_t* dataPtr;
} tsNewCmd;

typedef struct __attribute__((__packed__))
{
  uint8_t len;
  uint8_t* paramPtr;
} tsParameter;

/*
*
*/
class InterfaceClass {
  private:
    uint8_t _raw_pkt[SPI_BUFFER_SIZE] = {0}; //SPI buffer (limited to 128 bytes)
    uint16_t _rxLen = 0;  // size doesn't match maximum dimension of data that could be received from a DATA_PKT
    uint16_t _payLoadLen = 0;
    volatile bool _sendingWait;
    uint8_t _processing;
    bool _notify;

    int8_t _parsePkt(tsNewCmd *_pkt);
    void _SPISlaveInit();
    void _SPISlaveWrite(uint8_t* _rep, uint32_t len);

  public:
    uint32_t timeout = 0;
    uint32_t pktRecvTimeout = 0;
    tsDataPacket recPacket;
    tsNewCmd longCmd;
    uint8_t* _notifyMsg = _longCmdPkt;
    uint8_t _longCmdPkt[MAX_CMD_LEN];
    uint8_t _socketPkt[MTU_SIZE];
    uint32_t _socketPktPtr = 0;

    InterfaceClass();
    void begin();
    bool canNotify();
    uint8_t canProcess();
    bool checkMasterReady();
    void enableNotify();
    void enableProcess();
    void end();
    void endNotify();
    void endProcessing();
    
    int8_t read(tsNewCmd *_pkt);
    void setSR(bool level);
    void triggerISR(void);
    void write(uint8_t *_pkt, uint32_t len);
  
    uint32_t sockBuffInit(void);
    uint32_t sockBuffAdd(uint8_t* data, uint32_t sz);
};

extern InterfaceClass interface;

typedef struct {
    int8_t nScannedNets;
    uint8_t currNetIdx;
} tsNetworkParams;

typedef struct{
  bool inUse;
  bool conn;
} tsSockStatus;

typedef struct{
  uint8_t udpBuf[512];
  uint16_t udpBufLen;
  uint16_t udpPort;
  IPAddress udpAddress;
  uint8_t udpIndex;
}tsUdpData;
/*
*
*/
class WiFiBridgeClass {
  friend InterfaceClass;

  private:
  uint8_t _repPkt[MAX_CMD_LEN]; //response array
  tsNewCmd _tempPkt;
  uint32_t _txPktSize;
  uint8_t _notifyMsgCntr;
  tsNetworkParams nets;
  bool _prepareReply = false;
  bool _connectionRequested = false;
  IPAddress* _handyIp;
  IPAddress* _handyGateway;
  IPAddress* _handySubnet;

  WiFiServer* mapWiFiServers[MAX_SOCK_NUMBER];
  WiFiClient mapWiFiClients[MAX_SOCK_NUMBER];
  WiFiUDP mapWiFiUDP[MAX_SOCK_NUMBER];
  tsSockStatus _sockInUse[MAX_SOCK_NUMBER] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};

  std::shared_ptr<WiFiEventHandlerOpaque> _onConnected;
  std::shared_ptr<WiFiEventHandlerOpaque> _onDisconnected;

  // WiFi Base
  void _config();
  bool _connect(uint8_t option);
  int8_t _checkSSIDInList(char* targetSSID);
  void _clearWiFiSSIDList();
  void _createErrorResponse();
  bool _disconnect();
  void _fetchWiFiSSIDList();
  void _getBSSID(uint8_t current);
  void _getCurrentSSID();
  void _getEncryption(uint8_t current);
  void _getFwVersion();
  void _getHostByName();
  void _getHostname();
  void _getMacAddress();
  void _getNetworkData();
  void _getRSSI(uint8_t current);
  void _getStatus();
  uint8_t _networkListEvalAging(uint8_t connectedIndex);
  void _notifySTAConnStatus(uint8_t stat);
  void _prepareReplayBuf();
  bool _process();
  void _reqHostByName();
  void _scanNetwork(uint8_t which);
  void _setDNS();
  void _startScanNetwork();
  void _storeWiFiSSIDList();
  
  // WiFi Server
  void _startServer();
  void availData();
  void _serverStatus();
  void _getData();
  bool _sendData();
  void checkDataSent();

  // WiFi Client
  void _startClient();
  void _stopClient();
  void _clientStatus();
  void _availSocketData();

  // WiFI UDP Client
  void _remoteData();
  void _getDataBuf();
  void insDataBuf();
  void _sendUdpData();

  // WiFi Test
  bool _txrxMsg();

public:
  bool _startAutoconnect = false;
  tsNetParam tempNetworkParam;
  tsNetArray networkArray;
  uint32_t connectionTimeout = 0;
  wl_status_t connectionStatus = WL_IDLE_STATUS;
  int8_t connectedIndex_temp = -1;

  bool UI_alert = false;

  WiFiBridgeClass();
  // Logic Functions
  bool autoconnect();
  void begin();
  void handle();

  void notifyEspReady();

};

extern WiFiBridgeClass wifiBridge;
