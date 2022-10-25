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


/*
 * Firmware version and build date
 */

#define BUILD_DATE "20210308"
#define FW_VERSION "1.0.0"
#define FW_NAME "wifibridge"

/*
 * Enable/Disable Debug
 */
#define DEBUG
//#define BAUDRATE_DEBUG 115200

/*
 * Define board hostname
 */
#define DEF_HOSTNAME "jolly"

#define BOARDMODEL "JOLLY"
#define ARDUINO_BOARD "jolly"
#define ESP_CH_SPI
#define WIFI_LED (9)
#define PIN_SPI_CS (15)
#define PIN_SPI_SR (16)
#define PIN_SPI_MOSI (13)
#define PIN_SPI_MISO (12)
#define PIN_SPI_SCK (14)
#define SSIDNAME "Jolly"

//#define DEBUG_SERIALE

#define MOSI PIN_SPI_MOSI
#define MISO PIN_SPI_MISO
#define SCK PIN_SPI_SCK
