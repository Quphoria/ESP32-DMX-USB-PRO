
// DMX Constants
// https://erg.abdn.ac.uk/users/gorry/eg3576/start-codes.html
#define DMX_START_CODE 0
#define RDM_START_CODE 0xcc

// Enttec DMX USB PRO API Constants
#define DMX_PRO_START_MSG 0x7E
#define DMX_PRO_END_MSG 0xE7

// Enttec DMX USB PRO API Labels
#define DMX_PRO_REPROGRAM_REQ       1
#define DMX_PRO_PROGRAM_FLASH       2
#define DMX_PRO_GET_WIDGET_PARAMS   3
#define DMX_PRO_SET_WIDGET_PARAMS   4
#define DMX_PRO_RECV_PACKET         5 // NYI
#define DMX_PRO_SEND_PACKET         6 // NYI "periodically send a DMX packet" mode
#define DMX_PRO_SEND_RDM            7 // NYI
#define DMX_PRO_RECV_DMX_ON_CHANGE  8
#define DMX_PRO_RECV_CHANGED_PACKET 9 // NYI
#define DMX_PRO_GET_SERIAL_NUMBER   10
#define DMX_PRO_SEND_RDM_DISCOVERY  11 // NYI

// API Protocol State Definitions
#define MSG_START         1
#define MSG_LABEL         2
#define MSG_DATA_LEN_LSB  3
#define MSG_DATA_LEN_MSB  4
#define MSG_DATA          5
#define MSG_END           6
#define MSG_ERROR         7 // Go to this state when packet breaks

// Firmware Variety
#define FIRMWARE_DMX        1
#define FIRMWARE_RDM        2
#define FIRMWARE_RDM_SNIFF  3

// Configuration
#define DMX_PORT = DMX_NUM_2
#define FIRMWARE_VERSION 144
#define SERIAL_NUMBER 0x0ffffffff // Not sure why the leading 0, should be 4 bytes, maybe documentation meant 0x
#define DMX_BAUD_RATE 250000 // typical baud rate - 250000

#include "esp_dmx.h"

QueueHandle_t dmx_queue;

// Device state

unsigned char widget_mode = FIRMWARE_DMX;
unsigned char recv_dmx_on_change = 0;

// Store in memory for now, move to eeprom later
// (it should all be persistent according to api reference)
// Should be able to squeeze 4 constants into 4 bytes???
unsigned char BreakTime = 9; // 9-127
unsigned char MaBTime = 1; // 1-127
unsigned char RefreshRate = 0; // 0-40
unsigned char[508] user_config;

// USB API state
unsigned char state;
unsigned char message_type;
unsigned int dataSize;
unsigned int index;
unsigned char data_buffer[600];

// DMX buffers
unsigned char dmx_buffer_select = 0;
unsigned char[513] dmx_buffer_a = {0};
unsigned char[513] dmx_buffer_b = {0};

// Function forward declarations
void loadEEPROMData();
void saveEEPROMData();
void setupDMX();
void processMessage();
void sendResponse(unsigned char label, unsigned int length, unsigned char *data);
uint8_t calculateBreakNum();
uint16_t calculateIdleNum();

void setup() {
  loadEEPROMData();
  Serial.begin(57600);
  // change the TX pin according to the DMX shield you're using
  setupDMX();
  state = MSG_START;
}

void loop() {
  unsigned char c;

  while(!Serial.available());
  c = Serial.read();

  switch (state)
  {
  case MSG_START:
    if (c == DMX_PRO_START_MSG) state = MSG_LABEL;
    break;
  case MSG_LABEL:
    message_type = c;
    state = MSG_DATA_LEN_LSB;
    break;
  case MSG_DATA_LEN_LSB:
    dataSize = c & 0xff;
    state = MSG_DATA_LEN_MSB;
    break;
  case MSG_DATA_LEN_MSB:
    dataSize += (c << 8) & 0xff00;
    index = 0;
    state = index == dataSize ? MSG_END : MSG_DATA;
    break;
  case MSG_DATA:
    data_buffer[index] = c;
    index++;
    if (index >= dataSize) state = MSG_END;
    break;
  case MSG_END:
    if (c == DMX_PRO_END_MSG) {
      state = MSG_START;
      processMessage();
    } else state = MSG_ERROR;
    break;
  case MSG_ERROR:
  default:
    // We have lost our state?
    // Wait for next possible end message (could be false positive?)
    if (c == DMX_PRO_END_MSG) {
      state = MSG_START;
    } else state = MSG_ERROR;
    break;
  }
}

void processMessage() {
  unsigned int user_config_size = 0;
  unsigned char resp_buffer[600];
  switch (message_type)
  {
  case DMX_PRO_REPROGRAM_REQ:
    dmx_set_mode(DMX_PORT, DMX_MODE_READ);
    break;
  case DMX_PRO_PROGRAM_FLASH:
    dmx_set_mode(DMX_PORT, DMX_MODE_READ);
    // Just say the firmware was programmed
    resp_buffer = "TRUE";
    sendResponse(DMX_PRO_PROGRAM_FLASH, 4, resp_buffer);
    break;
  case DMX_PRO_GET_WIDGET_PARAMS:
    user_config_size = (data_buffer[1] << 8) | data_buffer[0];
    resp_buffer[0] = FIRMWARE_VERSION;
    resp_buffer[1] = widget_mode;
    resp_buffer[2] = BreakTime;
    resp_buffer[3] = MABTime;
    resp_buffer[4] = RefreshRate;
    // Copy user config into response buffer
    if (user_config_size != 0) {
      memcpy(&resp_buffer[5], user_config, user_config_size);
    }
    sendResponse(DMX_PRO_GET_WIDGET_PARAMS, 5+user_config_size, resp_buffer);
    break;
  case DMX_PRO_SET_WIDGET_PARAMS:
    dmx_set_mode(DMX_PORT, DMX_MODE_READ);
    unsigned char param_changed = 0;
    user_config_size = (data_buffer[1] << 8) | data_buffer[0];
    param_changed |= BreakTime != data_buffer[2];
    BreakTime = data_buffer[2];
    param_changed |= MABTime != data_buffer[3];
    MABTime = data_buffer[3];
    param_changed |= RefreshRate != data_buffer[4];
    RefreshRate = data_buffer[4];
    if (user_config_size != 0) {
      param_changed |= memcmp(user_config, &data_buffer[5], user_config_size) != 0;
      memcpy(user_config, &data_buffer[5], user_config_size);
    }
    dmx_set_break_num(DMX_PORT, calculateBreakNum());
    dmx_set_idle_num(DMX_PORT, calculateIdleNum());
    if (param_changed) saveEEPROMData();
    break;
  case DMX_PRO_SEND_PACKET:
    dmx_set_mode(DMX_PORT, DMX_MODE_WRITE);
    if (dataSize > 0 && data_buffer[0] == DMX_START_CODE) {
      for (unsigned int i = 1; i < dataSize; i++) {
        // DmxMaster.write(i, data_buffer[i]);
      }
    }
    break;
  case DMX_PRO_SEND_RDM:
    // Send RDM then reset back to read mode
    dmx_set_mode(DMX_PORT, DMX_MODE_WRITE);

    dmx_set_mode(DMX_PORT, DMX_MODE_READ);
    break;
  case DMX_PRO_RECV_DMX_ON_CHANGE:
    dmx_set_mode(DMX_PORT, DMX_MODE_READ);
    recv_dmx_on_change = data_buffer[0] != 0; // Only 1 or 0
    break;
  case DMX_PRO_GET_SERIAL_NUMBER:
    dmx_set_mode(DMX_PORT, DMX_MODE_READ);
    resp_buffer[0] = SERIAL_NUMBER & 0xff
    resp_buffer[1] = (SERIAL_NUMBER >> 8) & 0xff
    resp_buffer[2] = (SERIAL_NUMBER >> 16) & 0xff
    resp_buffer[3] = (SERIAL_NUMBER >> 24) & 0xff
    sendResponse(DMX_PRO_GET_SERIAL_NUMBER, 4, resp_buffer)
    break;
  case DMX_PRO_SEND_RDM_DISCOVERY:
    // Send 38 byte RDM discover packet in data_buffer
    dmx_set_mode(DMX_PORT, DMX_MODE_WRITE);

    dmx_set_mode(DMX_PORT, DMX_MODE_READ);
    break;
  default:
    break;
  }
}

void sendResponse(unsigned char label, unsigned int length, unsigned char *data) {

}

// DMX Functions
uint8_t calculateBreakNum() {
  return (uint8_t)((((double)BreakTime) * 10.67e-6) * (double)DMX_BAUD_RATE);
}
uint16_t calculateIdleNum() {
  return (uint8_t)((((double)MABTime) * 10.67e-6) * (double)DMX_BAUD_RATE);
}

void setupDMX() {
  // first configure the UART...
  const dmx_config_t config = {
    .baud_rate = DMX_BAUD_RATE, 
    .break_num = calculateBreakNum(), 
    .idle_num = calculateIdleNum()
};
  const dmx_config_t config = DMX_DEFAULT_CONFIG;
  dmx_param_config(DMX_PORT, &config);

  // then set the communication pins...
  const int tx_io_num = 17, rx_io_num = 16, rts_io_num = 21;
  dmx_set_pin(DMX_PORT, tx_io_num, rx_io_num, rts_io_num);

  // and install the driver!
  dmx_driver_install(DMX_PORT, DMX_MAX_PACKET_SIZE, 10, &dmx_queue, 
        ESP_INTR_FLAG_IRAM);

  dmx_set_mode(DMX_PORT, DMX_MODE_READ);
}
