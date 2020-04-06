/* OpenSprinkler Unified (AVR/RPI/BBB/LINUX/ESP8266) Firmware
 * Copyright (C) 2015 by Ray Wang (ray@opensprinkler.com)
 *
 * OpenSprinkler library
 * Feb 2015 @ OpenSprinkler.com
 *
 * This file is part of the OpenSprinkler library
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "OpenSprinkler.h"
#include "server.h"
#include "defines.h"

/** Declare static data members */
NVConData OpenSprinkler::nvdata;
ConStatus OpenSprinkler::status;
ConStatus OpenSprinkler::old_status;
byte OpenSprinkler::hw_type;

byte OpenSprinkler::nboards;
byte OpenSprinkler::nstations;
byte OpenSprinkler::station_bits[MAX_EXT_BOARDS+1];
uint16_t OpenSprinkler::baseline_current;

ulong OpenSprinkler::sensor_lasttime;
ulong OpenSprinkler::flowcount_log_start;
ulong OpenSprinkler::flowcount_rt;
volatile ulong OpenSprinkler::flowcount_time_ms;
ulong OpenSprinkler::raindelay_start_time;
byte OpenSprinkler::button_timeout;
ulong OpenSprinkler::checkwt_lasttime;
ulong OpenSprinkler::checkwt_success_lasttime;
ulong OpenSprinkler::powerup_lasttime;
byte OpenSprinkler::weather_update_flag;

char tmp_buffer[TMP_BUFFER_SIZE+1];       // scratch buffer

const char wtopts_filename[] PROGMEM = WEATHER_OPTS_FILENAME;
const char stns_filename[]   PROGMEM = STATION_ATTR_FILENAME;
const char ifkey_filename[]  PROGMEM = IFTTT_KEY_FILENAME;

const char wifi_filename[]   PROGMEM = WIFI_FILENAME;
byte OpenSprinkler::state = OS_STATE_INITIAL;
byte OpenSprinkler::prev_station_bits[MAX_EXT_BOARDS+1];
WiFiConfig OpenSprinkler::wifi_config = {WIFI_MODE_AP, "", ""};

extern ESP8266WebServer *wifi_server;
extern char ether_buffer[];

#include <FS.h>

/** Option json names (stored in progmem) */
// IMPORTANT: each json name is strictly 5 characters
// with 0 fillings if less
#define OP_JSON_NAME_STEPSIZE 5
const char op_json_names[] PROGMEM =
    "fwv\0\0"
    "tz\0\0\0"
    "ntp\0\0"
    "dhcp\0"
    "ip1\0\0"
    "ip2\0\0"
    "ip3\0\0"
    "ip4\0\0"
    "gw1\0\0"
    "gw2\0\0"
    "gw3\0\0"
    "gw4\0\0"
    "hp0\0\0"
    "hp1\0\0"
    "hwv\0\0"
    "ext\0\0"
    "seq\0\0"
    "sdt\0\0"
    "mas\0\0"
    "mton\0"
    "mtof\0"
    "urs\0\0" // todo: rename to sn1t
    "rso\0\0" // todo: rename to sn1o
    "wl\0\0\0"
    "den\0\0"
    "ipas\0"
    "devid"
    "con\0\0"
    "lit\0\0"
    "dim\0\0"
    "bst\0\0"
    "uwt\0\0"
    "ntp1\0"
    "ntp2\0"
    "ntp3\0"
    "ntp4\0"
    "lg\0\0\0"
    "mas2\0"
    "mton2"
    "mtof2"
    "fwm\0\0"
    "fpr0\0"
    "fpr1\0"
    "re\0\0\0"
    "dns1\0"
    "dns2\0"
    "dns3\0"
    "dns4\0"
    "sar\0\0"
    "ife\0\0"
    "sn2t\0"
    "sn2o\0"
    "reset";

/** Option promopts (stored in progmem, for LCD display) */
// Each string is strictly 16 characters
// with SPACE fillings if less
const char op_prompts[] PROGMEM =
    "Firmware version"
    "Time zone (GMT):"
    "Enable NTP sync?"
    "Enable DHCP?    "
    "Static.ip1:     "
    "Static.ip2:     "
    "Static.ip3:     "
    "Static.ip4:     "
    "Gateway.ip1:    "
    "Gateway.ip2:    "
    "Gateway.ip3:    "
    "Gateway.ip4:    "
    "HTTP Port:      "
    "----------------"
    "Hardware version"
    "# of exp. board:"
    "----------------"
    "Stn. delay (sec)"
    "Master 1 (Mas1):"
    "Mas1  on adjust:"
    "Mas1 off adjust:"
    "Sensor 1 type:  "
    "Normally open?  "
    "Watering level: "
    "Device enabled? "
    "Ignore password?"
    "Device ID:      "
    "LCD contrast:   "
    "LCD brightness: "
    "LCD dimming:    "
    "DC boost time:  "
    "Weather algo.:  "
    "NTP server.ip1: "
    "NTP server.ip2: "
    "NTP server.ip3: "
    "NTP server.ip4: "
    "Enable logging? "
    "Master 2 (Mas2):"
    "Mas2  on adjust:"
    "Mas2 off adjust:"
    "Firmware minor: "
    "Pulse rate:     "
    "----------------"
    "As remote ext.? "
    "DNS server.ip1: "
    "DNS server.ip2: "
    "DNS server.ip3: "
    "DNS server.ip4: "
    "Special Refresh?"
    "IFTTT Enable:   "
    "Sensor 2 type:  "
    "Normally open?  "
    "Factory reset?  ";

/** Option maximum values (stored in progmem) */
const byte op_max[] PROGMEM = {
  0,
  108,
  1,
  1,
  255,
  255,
  255,
  255,
  255,
  255,
  255,
  255,
  255,
  255,
  0,
  MAX_EXT_BOARDS,
  1,
  255,
  MAX_NUM_STATIONS,
  255,
  255,
  255,
  1,
  250,
  1,
  1,
  255,
  255,
  255,
  255,
  250,
  255,
  255,
  255,
  255,
  255,
  1,
  MAX_NUM_STATIONS,
  255,
  255,
  0,
  255,
  255,
  1,
  255,
  255,
  255,
  255,
  1,
  255,
  255,
  1,
  1
};

/** Option values (stored in RAM) */
byte OpenSprinkler::options[] = {
  OS_FW_VERSION, // firmware version
  28, // default time zone: GMT-5
  1,  // 0: disable NTP sync, 1: enable NTP sync
  1,  // 0: use static ip, 1: use dhcp
  0,  // this and next 3 bytes define static ip
  0,
  0,
  0,
  0,  // this and next 3 bytes define static gateway ip
  0,
  0,
  0,
  80, // on AVR, the default HTTP port is 80, this and next byte define http port number
  0,
  OS_HW_VERSION,
  0,  // number of 8-station extension board. 0: no extension boards
  1,  // the option 'sequential' is now retired
  120,// station delay time (-10 minutes to 10 minutes).
  0,  // index of master station. 0: no master station
  120,// master on time adjusted time (-10 minutes to 10 minutes)
  120,// master off adjusted time (-10 minutes to 10 minutes)
  1,  // sensor 1 type (see SENSOR_TYPE macro defines)
  1,  // sensor 1 option. 0: normally closed; 1: normally open.
  100,// water level (default 100%),
  1,  // device enable
  0,  // 1: ignore password; 0: use password
  0,  // device id
  150,// lcd contrast
  100,// lcd backlight
  50, // lcd dimming
  80, // boost time (only valid to DC and LATCH type)
  0,  // weather algorithm (0 means not using weather algorithm)
  50, // this and the next three bytes define the ntp server ip
  97,
  210,
  169,
  1,  // enable logging: 0: disable; 1: enable.
  0,  // index of master2. 0: no master2 station
  120,// master2 on adjusted time
  120,// master2 off adjusted time
  OS_FW_MINOR, // firmware minor version
  100,// this and next byte define flow pulse rate (100x)
  0,  // default is 1.00 (100)
  0,  // set as remote extension
  8,  // this and the next three bytes define the custom dns server ip
  8,
  8,
  8,
  0,  // special station auto refresh
  0,  // ifttt enable bits
  2,  // sensor 2 type
  0,  // sensor 2 option. 0: normally closed; 1: normally open.
  0   // reset
};

/** Weekday strings (stored in progmem, for LCD display) */
static const char days_str[] PROGMEM =
  "Mon\0"
  "Tue\0"
  "Wed\0"
  "Thu\0"
  "Fri\0"
  "Sat\0"
  "Sun\0";

/** Calculate local time (UTC time plus time zone offset) */
time_t OpenSprinkler::now_tz() {
  return now()+(int32_t)3600/4*(int32_t)(options[OPTION_TIMEZONE]-48);
}

/** read hardware MAC */
#define MAC_CTRL_ID 0x50
bool OpenSprinkler::read_hardware_mac() {
  WiFi.macAddress((byte*)tmp_buffer);
  return true;
}

void(* resetFunc) (void) = 0; // AVR software reset function

/** Initialize network with the given mac address and http port */
byte OpenSprinkler::start_network() {

  lcd_print_line_clear_pgm(PSTR("Starting..."), 1);
  if(wifi_server) delete wifi_server;
  if(get_wifi_mode()==WIFI_MODE_AP) {
    wifi_server = new ESP8266WebServer(80);
  } else {
    uint16_t httpport = (uint16_t)(options[OPTION_HTTPPORT_1]<<8) + (uint16_t)options[OPTION_HTTPPORT_0];
    wifi_server = new ESP8266WebServer(httpport);
  }
  status.has_hwmac = 1;
  return 1;
}

/** Reboot controller */
void OpenSprinkler::reboot_dev() {
  lcd_print_line_clear_pgm(PSTR("Rebooting..."), 0); 
  ESP.restart();
}

extern void flow_isr();

/** Initialize pins, controller variables, LCD */
void OpenSprinkler::begin() {

  hw_type = HW_TYPE_UNKNOWN;

	// Reset all stations
  clear_all_station_bits();
  apply_all_station_bits();

  pinMode(PIN_RELAY_1, OUTPUT);
  pinMode(PIN_RELAY_2, OUTPUT);
  pinMode(PIN_RELAY_3, OUTPUT);
  pinMode(PIN_RELAY_4, OUTPUT);
  pinMode(PIN_RELAY_5, OUTPUT);
  digitalWrite(PIN_RELAY_1, HIGH);
  digitalWrite(PIN_RELAY_2, HIGH);
  digitalWrite(PIN_RELAY_3, HIGH);
  digitalWrite(PIN_RELAY_4, HIGH);
  digitalWrite(PIN_RELAY_5, HIGH); // Turn Off External Relay
  pinMode(PIN_LED, OUTPUT);
  // Set up sensors
  /* todo: handle two sensors */
  pinMode(PIN_RAINSENSOR, INPUT_PULLUP);
  pinMode(PIN_FLOWSENSOR, INPUT_PULLUP);
  attachInterrupt(PIN_FLOWSENSOR, flow_isr, FALLING);

  // Default controller status variables
  // Static variables are assigned 0 by default
  // so only need to initialize non-zero ones
  status.enabled = 1;
  status.safe_reboot = 0;

  old_status = status;

  nvdata.sunrise_time = 360;  // 6:00am default sunrise
  nvdata.sunset_time = 1080;  // 6:00pm default sunset

  nboards = 0;
  nstations = 8;

  
    status.has_curr_sense = 0;  // OS3.0 has current sensing capacility
    // measure baseline current
    baseline_current = 100;

    if(!SPIFFS.begin()) {
      DEBUG_PRINTLN(F("SPIFFS failed"));
      status.has_sd = 0;
    } else {
      status.has_sd = 1;
    }

    state = OS_STATE_INITIAL;
  // set button pins
  // enable internal pullup
  pinMode(PIN_BUTTON_1, INPUT_PULLUP);
  pinMode(PIN_BUTTON_2, INPUT_PULLUP);
  pinMode(PIN_BUTTON_3, INPUT_PULLUP);
  pinMode(PIN_BUTTON_4, INPUT_PULLUP);
}

/** Apply all station bits
 * !!! This will activate/deactivate valves !!!
 */
void OpenSprinkler::apply_all_station_bits() {
    
  byte bid, s, sbits;  

  if(options[OPTION_SPE_AUTO_REFRESH]) {
    // handle refresh of RF and remote stations
    // we refresh the station whose index is the current time modulo MAX_NUM_STATIONS
    static byte last_sid = 0;
    byte sid = now() % MAX_NUM_STATIONS;
    if (sid != last_sid) {  // avoid refreshing the same station twice in a roll
      last_sid = sid;
      bid=sid>>3;
      s=sid&0x07;
      switch_special_station(sid, (station_bits[bid]>>s)&0x01);
    }
  }
}

/** Read rain sensor status */
void OpenSprinkler::rainsensor_status() {
  // options[OPTION_RS_TYPE]: 0 if normally closed, 1 if normally open
  if(options[OPTION_SENSOR1_TYPE]!=SENSOR_TYPE_RAIN) return;
  status.rain_sensed = (digitalRead(PIN_RAINSENSOR) == options[OPTION_SENSOR1_OPTION] ? 0 : 1);
}

/** Return program switch status */
bool OpenSprinkler::programswitch_status(ulong curr_time) {
  if(options[OPTION_SENSOR1_TYPE]!=SENSOR_TYPE_PSWITCH) return false;
  static ulong keydown_time = 0;
  byte val = digitalRead(PIN_RAINSENSOR);
  if(!val && !keydown_time) keydown_time = curr_time;
  else if(val && keydown_time && (curr_time > keydown_time)) {
    keydown_time = 0;
    return true;
  }
  return false;
}
/** Convert hex code to ulong integer */
static ulong hex2ulong(byte *code, byte len) {
  char c;
  ulong v = 0;
  for(byte i=0;i<len;i++) {
    c = code[i];
    v <<= 4;
    if(c>='0' && c<='9') {
      v += (c-'0');
    } else if (c>='A' && c<='F') {
      v += 10 + (c-'A');
    } else if (c>='a' && c<='f') {
      v += 10 + (c-'a');
    } else {
      return 0;
    }
  }
  return v;
}

/** Parse RF code into on/off/timeing sections */
uint16_t OpenSprinkler::parse_rfstation_code(RFStationData *data, ulong* on, ulong *off) {
  ulong v;
  v = hex2ulong(data->on, sizeof(data->on));
  if (!v) return 0;
  if (on) *on = v;
	v = hex2ulong(data->off, sizeof(data->off));
  if (!v) return 0;
  if (off) *off = v;
	v = hex2ulong(data->timing, sizeof(data->timing));
  if (!v) return 0;
  return v;
}

/** Get station name from NVM */
void OpenSprinkler::get_station_name(byte sid, char tmp[]) {
  tmp[STATION_NAME_SIZE]=0;
  nvm_read_block(tmp, (void*)(ADDR_NVM_STN_NAMES+(int)sid*STATION_NAME_SIZE), STATION_NAME_SIZE);
}

/** Set station name to NVM */
void OpenSprinkler::set_station_name(byte sid, char tmp[]) {
  tmp[STATION_NAME_SIZE]=0;
  nvm_write_block(tmp, (void*)(ADDR_NVM_STN_NAMES+(int)sid*STATION_NAME_SIZE), STATION_NAME_SIZE);
}

/** Save station attribute bits to NVM */
void OpenSprinkler::station_attrib_bits_save(int addr, byte bits[]) {
  nvm_write_block(bits, (void*)addr, MAX_EXT_BOARDS+1);
}

/** Load all station attribute bits from NVM */
void OpenSprinkler::station_attrib_bits_load(int addr, byte bits[]) {
  nvm_read_block(bits, (void*)addr, MAX_EXT_BOARDS+1);
}

/** Read one station attribute byte from NVM */
byte OpenSprinkler::station_attrib_bits_read(int addr) {
  return nvm_read_byte((byte*)addr);
}

/** verify if a string matches password */
byte OpenSprinkler::password_verify(char *pw) {
  byte *addr = (byte*)ADDR_NVM_PASSWORD;
  byte c1, c2;
  while(1) {
    if(addr == (byte*)ADDR_NVM_PASSWORD+MAX_USER_PASSWORD)
      c1 = 0;
    else
      c1 = nvm_read_byte(addr++);
    c2 = *pw++;
    if (c1==0 || c2==0)
      break;
    if (c1!=c2) {
      return 0;
    }
  }
  return (c1==c2) ? 1 : 0;
}

// ==================
// Schedule Functions
// ==================

/** Index of today's weekday (Monday is 0) */
byte OpenSprinkler::weekday_today() {
  //return ((byte)weekday()+5)%7; // Time::weekday() assumes Sunday is 1
  ulong wd = now_tz() / 86400L;
  return (wd+3) % 7;  // Jan 1, 1970 is a Thursday
}

/** Switch special station */
void OpenSprinkler::switch_special_station(byte sid, byte value) {
  // check station special bit
  if(station_attrib_bits_read(ADDR_NVM_STNSPE+(sid>>3))&(1<<(sid&0x07))) {
    // read station special data from sd card
    int stepsize=sizeof(StationSpecialData);
    read_from_file(stns_filename, tmp_buffer, stepsize, sid*stepsize);
    StationSpecialData *stn = (StationSpecialData *)tmp_buffer;
    // check station type
    if(stn->type==STN_TYPE_RF) {
      // transmit RF signal
      switch_rfstation((RFStationData *)stn->data, value);
    } else if(stn->type==STN_TYPE_REMOTE) {
      // request remote station
      switch_remotestation((RemoteStationData *)stn->data, value);
    }
    // GPIO and HTTP stations are only available for OS23 or OSPi
    else if(stn->type==STN_TYPE_GPIO) {
      // set GPIO pin
      switch_gpiostation((GPIOStationData *)stn->data, value);
    } else if(stn->type==STN_TYPE_HTTP) {
      // send GET command
      switch_httpstation((HTTPStationData *)stn->data, value);
    }
  }
}

/** Set station bit
 * This function sets/resets the corresponding station bit variable
 * You have to call apply_all_station_bits next to apply the bits
 * (which results in physical actions of opening/closing valves).
 */
byte OpenSprinkler::set_station_bit(byte sid, byte value) {
  byte *data = station_bits+(sid>>3);  // pointer to the station byte
  byte mask = (byte)1<<(sid&0x07); // mask
  if (value) {
    if((*data)&mask) return 0;  // if bit is already set, return no change
    else {
      (*data) = (*data) | mask;
      switch_special_station(sid, 1); // handle special stations
      switch (sid) {
        case 0:
          digitalWrite(PIN_RELAY_1, LOW);
          break;
        case 1:
          digitalWrite(PIN_RELAY_2, LOW);
          break;
        case 2:
          digitalWrite(PIN_RELAY_3, LOW);
          break;
        case 3:
          digitalWrite(PIN_RELAY_4, LOW);
          break;
        case 4:
          digitalWrite(PIN_RELAY_5, LOW);
          break;
      }
      return 1;
    }
  } else {
    if(!((*data)&mask)) return 0; // if bit is already reset, return no change
    else {
      (*data) = (*data) & (~mask);
      switch_special_station(sid, 0); // handle special stations
      switch (sid) {
        case 0:
          digitalWrite(PIN_RELAY_1, HIGH);
          break;
        case 1:
          digitalWrite(PIN_RELAY_2, HIGH);
          break;
        case 2:
          digitalWrite(PIN_RELAY_3, HIGH);
          break;
        case 3:
          digitalWrite(PIN_RELAY_4, HIGH);
          break;
        case 4:
          digitalWrite(PIN_RELAY_5, HIGH);
          break;
      }
      return 255;
    }
  }
  return 0;
}

/** Clear all station bits */
void OpenSprinkler::clear_all_station_bits() {
  byte sid;
  for(sid=0;sid<=MAX_NUM_STATIONS;sid++) {
    set_station_bit(sid, 0);
  }
}

/** Transmit one RF signal bit */
void transmit_rfbit(ulong lenH, ulong lenL) {
    digitalWrite(PIN_RFTX, 1);
    delayMicroseconds(lenH);
    digitalWrite(PIN_RFTX, 0);
    delayMicroseconds(lenL);
}

/** Transmit RF signal */
void send_rfsignal(ulong code, ulong len) {
  ulong len3 = len * 3;
  ulong len31 = len * 31;
  for(byte n=0;n<15;n++) {
    int i=23;
    // send code
    while(i>=0) {
      if ((code>>i) & 1) {
        transmit_rfbit(len3, len);
      } else {
        transmit_rfbit(len, len3);
      }
      i--;
    };
    // send sync
    transmit_rfbit(len, len31);
  }
}

/** Switch RF station
 * This function takes a RF code,
 * parses it into signals and timing,
 * and sends it out through RF transmitter.
 */
void OpenSprinkler::switch_rfstation(RFStationData *data, bool turnon) {
  ulong on, off;
  uint16_t length = parse_rfstation_code(data, &on, &off);
//  rfswitch.enableTransmit(PIN_RFTX);
//  rfswitch.setPulseLength(length);
//  rfswitch.setProtocol(1);
//  rfswitch.send(turnon ? on : off, 24);
}

/** Switch GPIO station
 * Special data for GPIO Station is three bytes of ascii decimal (not hex)
 * First two bytes are zero padded GPIO pin number.
 * Third byte is either 0 or 1 for active low (GND) or high (+5V) relays
 */
void OpenSprinkler::switch_gpiostation(GPIOStationData *data, bool turnon) {
  byte gpio = (data->pin[0] - '0') * 10 + (data->pin[1] - '0');
  byte activeState = data->active - '0';

  pinMode(gpio, OUTPUT);
  if (turnon)
    digitalWrite(gpio, activeState);
  else
    digitalWrite(gpio, 1-activeState);
}

/** Callback function for browseUrl calls */
void httpget_callback(byte status, uint16_t off, uint16_t len) {
#if defined(SERIAL_DEBUG)
  Ethernet::buffer[off+ETHER_BUFFER_SIZE-1] = 0;
  DEBUG_PRINTLN((const char*) Ethernet::buffer + off);
#endif
}

/** Switch remote station
 * This function takes a remote station code,
 * parses it into remote IP, port, station index,
 * and makes a HTTP GET request.
 * The remote controller is assumed to have the same
 * password as the main controller
 */
void OpenSprinkler::switch_remotestation(RemoteStationData *data, bool turnon) {

  ulong ip = hex2ulong(data->ip, sizeof(data->ip));
  ulong port = hex2ulong(data->port, sizeof(data->port));

  WiFiClient client;
  
  char *p = tmp_buffer + sizeof(RemoteStationData) + 1;
  BufferFiller bf = p;
  // MAX_NUM_STATIONS is the refresh cycle
  uint16_t timer = options[OPTION_SPE_AUTO_REFRESH]?2*MAX_NUM_STATIONS:64800;  
  bf.emit_p(PSTR("GET /cm?pw=$E&sid=$D&en=$D&t=$D"),
            ADDR_NVM_PASSWORD,
            (int)hex2ulong(data->sid, sizeof(data->sid)),
            turnon, timer);
  bf.emit_p(PSTR(" HTTP/1.0\r\nHOST: *\r\n\r\n"));
  
  byte cip[4];
  cip[0] = ip>>24;
  cip[1] = (ip>>16)&0xff;
  cip[2] = (ip>>8)&0xff;
  cip[3] = ip&0xff;

  if(!client.connect(IPAddress(cip), port))  return;
  client.write((uint8_t *)p, strlen(p));
  
  bzero(ether_buffer, ETHER_BUFFER_SIZE);
  
  time_t timeout = now_tz() + 5; // 5 seconds timeout
  while(!client.available() && now_tz() < timeout) {
  }

  bzero(ether_buffer, ETHER_BUFFER_SIZE);
  while(client.available()) {
    client.read((uint8_t*)ether_buffer, ETHER_BUFFER_SIZE);
  }
  client.stop();
  //httpget_callback(0, 0, ETHER_BUFFER_SIZE);    
}

/** Switch http station
 * This function takes an http station code,
 * parses it into a server name and two HTTP GET requests.
 */
void OpenSprinkler::switch_httpstation(HTTPStationData *data, bool turnon) {

  static HTTPStationData copy;
  // make a copy of the HTTP station data and work with it
  memcpy((char*)&copy, (char*)data, sizeof(HTTPStationData));
  char * server = strtok((char *)copy.data, ",");
  char * port = strtok(NULL, ",");
  char * on_cmd = strtok(NULL, ",");
  char * off_cmd = strtok(NULL, ",");
  char * cmd = turnon ? on_cmd : off_cmd;
 
  WiFiClient client;
  if(!client.connect(server, atoi(port))) return;
  
  char getBuffer[255];
  sprintf(getBuffer, "GET /%s HTTP/1.0\r\nHOST: *\r\n\r\n", cmd);
  
  DEBUG_PRINTLN(getBuffer);
  
  client.write((uint8_t *)getBuffer, strlen(getBuffer));
  
  bzero(ether_buffer, ETHER_BUFFER_SIZE);
  
  time_t timeout = now_tz() + 5; // 5 seconds timeout
  while(!client.available() && now_tz() < timeout) {
  }

  bzero(ether_buffer, ETHER_BUFFER_SIZE);
  while(client.available()) {
    client.read((uint8_t*)ether_buffer, ETHER_BUFFER_SIZE);
  }
  client.stop();
}

/** Setup function for options */
void OpenSprinkler::options_setup() {

  // add 0.25 second delay to allow nvm to stablize
  delay(250);

  byte curr_ver = nvm_read_byte((byte*)(ADDR_NVM_OPTIONS+OPTION_FW_VERSION));
  lcd_print_line_clear_pgm(PSTR("Reading1..."), 0);
  // check reset condition: either firmware version has changed, or reset flag is up
  // if so, trigger a factory reset
  if (curr_ver != OS_FW_VERSION || nvm_read_byte((byte*)(ADDR_NVM_OPTIONS+OPTION_RESET))==0xAA)  {

    lcd_print_line_clear_pgm(PSTR("Resetting..."), 0); //DEBUG
    lcd_print_line_clear_pgm(PSTR("Please Wait..."), 1); //DEBUG

    // ======== Reset NVM data ========
    int i, sn;

    //if(curr_ver!=0) // if SPIFFS has been written before, perform a full format
    SPIFFS.format();  // perform a SPIFFS format
    lcd_print_line_clear_pgm(PSTR("Formating..."), 0); //DEBUG
    // 0. wipe out nvm
    for(i=0;i<TMP_BUFFER_SIZE;i++) tmp_buffer[i]=0;
    for(i=0;i<NVM_SIZE;i+=TMP_BUFFER_SIZE) {
      int nbytes = ((NVM_SIZE-i)>TMP_BUFFER_SIZE)?TMP_BUFFER_SIZE:(NVM_SIZE-i);
      nvm_write_block(tmp_buffer, (void*)i, nbytes);
    }
    lcd_print_line_clear_pgm(PSTR("0.Wipeout...."), 0); //DEBUG
    // 1. write non-volatile controller status
    nvdata_save();
    lcd_print_line_clear_pgm(PSTR("1.Saving..."), 0); //DEBUG
    // 2. write string parameters
    nvm_write_block(DEFAULT_PASSWORD, (void*)ADDR_NVM_PASSWORD, strlen(DEFAULT_PASSWORD)+1);
    nvm_write_block(DEFAULT_LOCATION, (void*)ADDR_NVM_LOCATION, strlen(DEFAULT_LOCATION)+1);
    nvm_write_block(DEFAULT_JAVASCRIPT_URL, (void*)ADDR_NVM_JAVASCRIPTURL, strlen(DEFAULT_JAVASCRIPT_URL)+1);
    nvm_write_block(DEFAULT_WEATHER_URL, (void*)ADDR_NVM_WEATHERURL, strlen(DEFAULT_WEATHER_URL)+1);
    nvm_write_block(DEFAULT_WEATHER_KEY, (void*)ADDR_NVM_WEATHER_KEY, strlen(DEFAULT_WEATHER_KEY)+1);
    lcd_print_line_clear_pgm(PSTR("2.Writing..."), 0); //DEBUG
    // 3. reset station names and special attributes, default Sxx
    tmp_buffer[0]='S';
    tmp_buffer[3]=0;
    for(i=ADDR_NVM_STN_NAMES, sn=1; i<ADDR_NVM_MAS_OP; i+=STATION_NAME_SIZE, sn++) {
      tmp_buffer[1]='0'+(sn/10);
      tmp_buffer[2]='0'+(sn%10);
      nvm_write_block(tmp_buffer, (void*)i, strlen(tmp_buffer)+1);
    }
    remove_file(stns_filename);
    tmp_buffer[0]=STN_TYPE_STANDARD;
    tmp_buffer[1]='0';
    tmp_buffer[2]=0;
    int stepsize=sizeof(StationSpecialData);
    for(i=0;i<MAX_NUM_STATIONS;i++) {
        write_to_file(stns_filename, tmp_buffer, stepsize, i*stepsize, false);
    }
    lcd_print_line_clear_pgm(PSTR("3.Resetting station names..."), 0); //DEBUG
    // 4. reset station attribute bits
    // since we wiped out nvm, only non-zero attributes need to be initialized
    for(i=0;i<MAX_EXT_BOARDS+1;i++) {
      tmp_buffer[i]=0xff;
    }
    nvm_write_block(tmp_buffer, (void*)ADDR_NVM_MAS_OP, MAX_EXT_BOARDS+1);
    nvm_write_block(tmp_buffer, (void*)ADDR_NVM_STNSEQ, MAX_EXT_BOARDS+1);
    lcd_print_line_clear_pgm(PSTR("4.Resetting attributes..."), 0); //DEBUG
    // 6. write options
    options_save(true); // write default option values
    lcd_print_line_clear_pgm(PSTR("6.Writing options..."), 0); //DEBUG
    
    //======== END OF NVM RESET CODE ========

    // restart after resetting NVM.
    delay(500);
    reboot_dev();
  }

  {
    // load ram parameters from nvm
    // load options
    options_load();

    // load non-volatile controller data
    nvdata_load();
  }
  lcd_print_line_clear_pgm(PSTR("Buttons_init-Start..."), 0);
	byte button;// = button_read(BUTTON_WAIT_NONE);
  lcd_print_line_clear_pgm(PSTR("Buttons_init-Read..."), 0);
	switch(button & BUTTON_MASK) {

    case BUTTON_1:
      // if BUTTON_1 is pressed during startup, go to 'reset all options'
      lcd_print_line_clear_pgm(PSTR("Buttons_init-Read_Button1..."), 0);
      ui_set_options(OPTION_RESET);
      if (options[OPTION_RESET]) {
        reboot_dev();
      }
      break;

    case BUTTON_2:
      // if BUTTON_2 is pressed during startup, go to Test OS mode
      // only available for OS 3.0
      lcd_print_line_clear_pgm(PSTR("Buttons_init-Read_Button2..."), 0);
      lcd_print_line_clear_pgm(PSTR("===Test Mode==="), 0);
      lcd_print_line_clear_pgm(PSTR("  B3:proceed"), 1);
      do {
        button = button_read(BUTTON_WAIT_NONE);
      } while(!((button&BUTTON_MASK)==BUTTON_3 && (button&BUTTON_FLAG_DOWN)));
      // set test mode parameters
      wifi_config.mode = WIFI_MODE_STA;
      #ifdef TESTMODE_SSID
      wifi_config.ssid = TESTMODE_SSID;
      wifi_config.pass = TESTMODE_PASS;
      #else
      wifi_config.ssid = "ostest";
      wifi_config.pass = "opendoor";
      #endif
      button = 0;
    
      break;

    case BUTTON_3:
      // if BUTTON_3 is pressed during startup, enter Setup option mode
      lcd_print_line_clear_pgm(PSTR("Buttons_init-Read_Button3..."), 0);
      lcd_print_line_clear_pgm(PSTR("==Set Options=="), 0); 
      lcd_print_line_clear_pgm(PSTR("B1/B2:+/-, B3:->"), 0);  
      lcd_print_line_clear_pgm(PSTR("Hold B3 to save"), 1);
      do {
        button = button_read(BUTTON_WAIT_NONE);
      } while (!(button & BUTTON_FLAG_DOWN));

      ui_set_options(0);
      if (options[OPTION_RESET]) {
        reboot_dev();
      }
      break;
  }

  if (!button) {
    // flash screen
    lcd_print_line_clear_pgm(PSTR(" OpenSprinkler"), 0);
  /*  lcd_print_pgm(PSTR("v"));
    byte hwv = options[OPTION_HW_VERSION];
    lcd_print_pgm(PSTR((char)('0'+(hwv/10)));
    lcd_print_pgm(PSTR("."));
    lcd_print_pgm(PSTR((char)('0'+(hwv%10))));
    lcd_print_pgm(PSTR(" AC"));
    lcd_print_pgm(PSTR("FW "));
    lcd_print_pgm(PSTR((char)('0'+(OS_FW_VERSION/100))));
    lcd_print_pgm(PSTR("."));
    lcd_print_pgm(PSTR((char)('0'+((OS_FW_VERSION/10)%10))));
    lcd_print_pgm(PSTR("."));
    lcd_print_pgm(PSTR((char)('0'+(OS_FW_VERSION%10))));
    lcd_print_pgm(PSTR('('));
    lcd_print_pgm(PSTR(OS_FW_MINOR));
    lcd_print_pgm(PSTR(')')); */
  }
  lcd_print_line_clear_pgm(PSTR("Buttons_init-Finish..."), 0);
}

/** Load non-volatile controller status data from internal NVM */
void OpenSprinkler::nvdata_load() {
  lcd_print_line_clear_pgm(PSTR("nvdata_load-Start..."), 0);
  nvm_read_block(&nvdata, (void*)ADDR_NVM_NVCONDATA, sizeof(NVConData));
  old_status = status;
  lcd_print_line_clear_pgm(PSTR("nvdata_load-Finish..."), 0);
}

/** Save non-volatile controller status data to internal NVM */
void OpenSprinkler::nvdata_save() {
  nvm_write_block(&nvdata, (void*)ADDR_NVM_NVCONDATA, sizeof(NVConData));
}

/** Load options from internal NVM */
void OpenSprinkler::options_load() {
  lcd_print_line_clear_pgm(PSTR("Options_load-Start..."), 0);
  nvm_read_block(tmp_buffer, (void*)ADDR_NVM_OPTIONS, NUM_OPTIONS);
  for (byte i=0; i<NUM_OPTIONS; i++) {
    options[i] = tmp_buffer[i];
  }
  nboards = options[OPTION_EXT_BOARDS]+1;
  nstations = nboards * 8;
  status.enabled = options[OPTION_DEVICE_ENABLE];
  options[OPTION_FW_MINOR] = OS_FW_MINOR;

  // load WiFi config
  File file = SPIFFS.open(WIFI_FILENAME, "r");
  if(file) {
    String sval = file.readStringUntil('\n'); sval.trim();
    wifi_config.mode = sval.toInt();
    sval = file.readStringUntil('\n'); sval.trim();
    wifi_config.ssid = sval;
    sval = file.readStringUntil('\n'); sval.trim();
    wifi_config.pass = sval;
    file.close();
  }
  lcd_print_line_clear_pgm(PSTR("Options_load-Finish..."), 0);
}

/** Save options to internal NVM */
void OpenSprinkler::options_save(bool savewifi) {
  // save options in reverse order so version number is written the last
  for (int i=NUM_OPTIONS-1; i>=0; i--) {
    tmp_buffer[i] = options[i];
  }
  nvm_write_block(tmp_buffer, (void*)ADDR_NVM_OPTIONS, NUM_OPTIONS);
  nboards = options[OPTION_EXT_BOARDS]+1;
  nstations = nboards * 8;
  status.enabled = options[OPTION_DEVICE_ENABLE];
  if(savewifi) {
    // save WiFi config
    File file = SPIFFS.open(WIFI_FILENAME, "w");
    if(file) {
      file.println(wifi_config.mode);
      file.println(wifi_config.ssid);
      file.println(wifi_config.pass);
      file.close();
    }
  }
}

// ==============================
// Controller Operation Functions
// ==============================

/** Enable controller operation */
void OpenSprinkler::enable() {
  status.enabled = 1;
  options[OPTION_DEVICE_ENABLE] = 1;
  options_save();
}

/** Disable controller operation */
void OpenSprinkler::disable() {
  status.enabled = 0;
  options[OPTION_DEVICE_ENABLE] = 0;
  options_save();
}

/** Start rain delay */
void OpenSprinkler::raindelay_start() {
  status.rain_delayed = 1;
  nvdata_save();
}

/** Stop rain delay */
void OpenSprinkler::raindelay_stop() {
  status.rain_delayed = 0;
  nvdata.rd_stop_time = 0;
  nvdata_save();
}

/** print a program memory string to a given line with clearing */
void OpenSprinkler::lcd_print_line_clear_pgm(PGM_P str, byte line) {

  Serial.println("");
  uint8_t c;
  int8_t cnt = 0;
  while((c=pgm_read_byte(str++))!= '\0') {
    Serial.print((char)c);
    cnt++;
  }
  Serial.println("");
}

void OpenSprinkler::lcd_print_pgm(PGM_P str) {

  uint8_t c;
  int8_t cnt = 0;
  while((c=pgm_read_byte(str++))!= '\0') {
    Serial.print((char)c);
    cnt++;
  }
}

void OpenSprinkler::lcd_print_2digit(int v) {
  Serial.print((int)(v/10));
  Serial.print((int)(v%10));
}

/** print time to a given line */
void OpenSprinkler::lcd_print_time(time_t t) {
  
  lcd_print_2digit(hour(t));
  lcd_print_pgm(PSTR(":"));
  lcd_print_2digit(minute(t));
  lcd_print_pgm(PSTR("  "));
  // each weekday string has 3 characters + ending 0
  lcd_print_pgm(days_str+4*weekday_today());
  lcd_print_pgm(PSTR(" "));
  lcd_print_2digit(month(t));
  lcd_print_pgm(PSTR("-"));
  lcd_print_2digit(day(t));
  Serial.println("");
}

/** print ip address */
void OpenSprinkler::lcd_print_ip(const byte *ip, byte endian) {
  for (byte i=0; i<4; i++) {
    Serial.print(endian ? (int)ip[3-i] : (int)ip[i]);
    if(i<3) lcd_print_pgm(PSTR("."));
  }
}

/** print mac address */
void OpenSprinkler::lcd_print_mac(const byte *mac) {
  for(byte i=0; i<6; i++) {
    if(i)  lcd_print_pgm(PSTR("-"));
    Serial.print((mac[i]>>4), HEX);
    Serial.print((mac[i]&0x0F), HEX);
  }
  lcd_print_pgm(PSTR(" (MAC)"));
}

/** print station bits */
void OpenSprinkler::lcd_print_station(byte line, char c) {
  Serial.println("");
  if (status.display_board == 0) {
    lcd_print_pgm(PSTR("MC:"));  // Master controller is display as 'MC'
  }
  else {
    lcd_print_pgm(PSTR("E"));
    Serial.print((int)status.display_board);
    lcd_print_pgm(PSTR(":"));   // extension boards are displayed as E1, E2...
  }

  if (!status.enabled) {
  	lcd_print_line_clear_pgm(PSTR("-Disabled!-"), 1);
  } else {
	  byte bitvalue = station_bits[status.display_board];
	  for (byte s=0; s<8; s++) {
	    byte sid = (byte)status.display_board<<3;
	    sid += (s+1);
      if (sid == options[OPTION_MASTER_STATION]) {
	      Serial.print((bitvalue&1) ? c : 'M'); // print master station
      } else if (sid == options[OPTION_MASTER_STATION_2]) {
	      Serial.print((bitvalue&1) ? c : 'N'); // print master2 station
	    } else {
	      Serial.print((bitvalue&1) ? c : '_');
	    }
  	  bitvalue >>= 1;
	  }
	}
}

/** print a version number */
void OpenSprinkler::lcd_print_version(byte v) {
  if(v > 99) {
    Serial.print(v/100);
    Serial.print(".");
  }
  if(v>9) {
    Serial.print((v/10)%10);
    Serial.print(".");  }
  Serial.print(v%10);
}

/** Button functions */
/** wait for button */
byte OpenSprinkler::button_read_busy(byte pin_butt, byte waitmode, byte butt, byte is_holding) {

  int hold_time = 0;

  if (waitmode==BUTTON_WAIT_NONE || (waitmode == BUTTON_WAIT_HOLD && is_holding)) {
    if (digitalRead(pin_butt) != 0) return BUTTON_NONE;
    return butt | (is_holding ? BUTTON_FLAG_HOLD : 0);
  }

  while (digitalRead(pin_butt) == 0 &&
         (waitmode == BUTTON_WAIT_RELEASE || (waitmode == BUTTON_WAIT_HOLD && hold_time<BUTTON_HOLD_MS))) {
    delay(BUTTON_DELAY_MS);
    hold_time += BUTTON_DELAY_MS;
  }
  if (is_holding || hold_time >= BUTTON_HOLD_MS)
    butt |= BUTTON_FLAG_HOLD;
  return butt;

}

/** read button and returns button value 'OR'ed with flag bits */
byte OpenSprinkler::button_read(byte waitmode) {
  static byte old = BUTTON_NONE;
  byte curr = BUTTON_NONE;
  byte is_holding = (old&BUTTON_FLAG_HOLD);

  delay(BUTTON_DELAY_MS);

  if (digitalRead(PIN_BUTTON_1) == 0) {
    curr = button_read_busy(PIN_BUTTON_1, waitmode, BUTTON_1, is_holding);
  } else if (digitalRead(PIN_BUTTON_2) == 0) {
    curr = button_read_busy(PIN_BUTTON_2, waitmode, BUTTON_2, is_holding);
  } else if (digitalRead(PIN_BUTTON_3) == 0) {
    curr = button_read_busy(PIN_BUTTON_3, waitmode, BUTTON_3, is_holding);
  }

  // set flags in return value
  byte ret = curr;
  if (!(old&BUTTON_MASK) && (curr&BUTTON_MASK))
    ret |= BUTTON_FLAG_DOWN;
  if ((old&BUTTON_MASK) && !(curr&BUTTON_MASK))
    ret |= BUTTON_FLAG_UP;

  old = curr;
  
  return ret;
}

/** user interface for setting options during startup */
void OpenSprinkler::ui_set_options(int oid) {
  boolean finished = false;
  byte button;
  int i=oid;

  while(!finished) {
    button = button_read(BUTTON_WAIT_HOLD);

    switch (button & BUTTON_MASK) {
    case BUTTON_1:
      if (i==OPTION_FW_VERSION || i==OPTION_HW_VERSION || i==OPTION_FW_MINOR ||
          i==OPTION_HTTPPORT_0 || i==OPTION_HTTPPORT_1 ||
          i==OPTION_PULSE_RATE_0 || i==OPTION_PULSE_RATE_1) break; // ignore non-editable options
      if (pgm_read_byte(op_max+i) != options[i]) options[i] ++;
      break;

    case BUTTON_2:
      if (i==OPTION_FW_VERSION || i==OPTION_HW_VERSION || i==OPTION_FW_MINOR ||
          i==OPTION_HTTPPORT_0 || i==OPTION_HTTPPORT_1 ||
          i==OPTION_PULSE_RATE_0 || i==OPTION_PULSE_RATE_1) break; // ignore non-editable options
      if (options[i] != 0) options[i] --;
      break;

    case BUTTON_3:
      if (!(button & BUTTON_FLAG_DOWN)) break;
      if (button & BUTTON_FLAG_HOLD) {
        // if OPTION_RESET is set to nonzero, change it to reset condition value
        if (options[OPTION_RESET]) {
          options[OPTION_RESET] = 0xAA;
        }
        // long press, save options
        options_save();
        finished = true;
      }
      else {
        // click, move to the next option
        if (i==OPTION_USE_DHCP && options[i]) i += 9; // if use DHCP, skip static ip set
        else if (i==OPTION_HTTPPORT_0) i+=2; // skip OPTION_HTTPPORT_1
        else if (i==OPTION_PULSE_RATE_0) i+=2; // skip OPTION_PULSE_RATE_1
        else if (i==OPTION_SENSOR1_TYPE && options[i]!=SENSOR_TYPE_RAIN) i+=2; // if sensor1 is not rain sensor, skip sensor1 option
        else if (i==OPTION_SENSOR2_TYPE && options[i]!=SENSOR_TYPE_RAIN) i+=2; // if sensor2 is not rain sensor, skip sensor2 option
        else if (i==OPTION_MASTER_STATION && options[i]==0) i+=3; // if not using master station, skip master on/off adjust
        else if (i==OPTION_MASTER_STATION_2&& options[i]==0) i+=3; // if not using master2, skip master2 on/off adjust
        else  {
          i = (i+1) % NUM_OPTIONS;
        }
        if(i==OPTION_SEQUENTIAL_RETIRED) i++;
        else if (hw_type==HW_TYPE_AC && i==OPTION_BOOST_TIME) i++;  // skip boost time for non-DC controller
      }
      break;
    }
  }
}

void OpenSprinkler::reset_to_ap() {
  wifi_config.mode = WIFI_MODE_AP;
  options_save(true);
  reboot_dev();
}

void OpenSprinkler::config_ip() {
  if(options[OPTION_USE_DHCP] == 0) {
    byte *_ip = options+OPTION_STATIC_IP1;
    IPAddress dvip(_ip[0], _ip[1], _ip[2], _ip[3]);
    if(dvip==(uint32_t)0x00000000) return;
    
    _ip = options+OPTION_GATEWAY_IP1;
    IPAddress gwip(_ip[0], _ip[1], _ip[2], _ip[3]);
    if(gwip==(uint32_t)0x00000000) return;
    
    IPAddress subn(255,255,255,0);
    _ip = options+OPTION_DNS_IP1;
    IPAddress dnsip(_ip[0], _ip[1], _ip[2], _ip[3]);
    WiFi.config(dvip, gwip, subn, dnsip);
  }
}

void OpenSprinkler::led_toggle() {
		digitalWrite(PIN_LED,!digitalRead(PIN_LED));
}

void OpenSprinkler::led_on() {
		digitalWrite(PIN_LED, 0);
}
