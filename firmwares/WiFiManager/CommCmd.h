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

enum {
  ESP_READY     = 0x09,
  CONNECT_OPEN_AP   = 0x10,
  CONNECT_SECURED_AP  = 0x11,
  SET_KEY_CMD         = 0x12,
  TEST_CMD          = 0x13,
  SET_IP_CONFIG_CMD = 0x14,
  SET_DNS_CONFIG_CMD  = 0x15,
  AUTOCONNECT_TO_STA  = 0x16,
  CANCEL_NETWORK_LIST = 0x17,

  GET_CONN_STATUS   = 0x20,
  GET_IPADDR_CMD    = 0x21,
  GET_MACADDR_CMD   = 0x22,
  GET_CURR_SSID_CMD = 0x23,
  GET_CURR_BSSID_CMD  = 0x24,
  GET_CURR_RSSI_CMD = 0x25,
  GET_CURR_ENCT_CMD = 0x26,
  SCAN_NETWORK_RESULT = 0x27,
  START_SERVER_TCP_CMD= 0x28,
  GET_STATE_TCP_CMD   = 0x29,
  DATA_SENT_TCP_CMD = 0x2A,
  AVAIL_DATA_TCP_CMD  = 0x2B,
  GET_DATA_TCP_CMD  = 0x2C,
  START_CLIENT_TCP_CMD= 0x2D,
  STOP_CLIENT_TCP_CMD = 0x2E,
  GET_CLIENT_STATE_TCP_CMD= 0x2F,
  DISCONNECT_CMD    = 0x30,
  GET_IDX_SSID_CMD  = 0x31,
  GET_IDX_RSSI_CMD  = 0x32,
  GET_IDX_ENCT_CMD  = 0x33,
  REQ_HOST_BY_NAME_CMD= 0x34,
  GET_HOST_BY_NAME_CMD= 0x35,
  START_SCAN_NETWORKS = 0x36,
  GET_FW_VERSION_CMD  = 0x37,
  GET_TEST_CMD    = 0x38,
  SEND_DATA_UDP_CMD = 0x39,
  GET_REMOTE_DATA_CMD = 0x3A,
  GET_HOSTNAME    = 0x3B,
  SET_HOSTNAME    = 0x3C,
  DISABLE_WEBPANEL = 0x3D,

  TEST_DATA_TXRX    = 0x40,

    // All command with DATA_FLAG 0x40 send a 16bit Len

  SEND_DATA_TCP_CMD   = 0x44,
    GET_DATABUF_TCP_CMD   = 0x45,
    INSERT_DATABUF_CMD    = 0x46,
};
