
#define DMX_PRO_START_MSG 0x7E
#define DMX_PRO_END_MSG 0xE7

// https://erg.abdn.ac.uk/users/gorry/eg3576/start-codes.html
#define DMX_START_CODE 0 

// Enttec DMX USB PRO API Labels
#define DMX_PRO_GET_WIDGET_PARAMS 3 // NYI
#define DMX_PRO_SET_WIDGET_PARAMS 4 // NYI
#define DMX_PRO_RECV_PACKET 5 // NYI
#define DMX_PRO_SEND_PACKET 6 // "periodically send a DMX packet" mode
#define DMX_PRO_RECV_DMX_ON_CHANGE 8
#define DMX_PRO_RECV_CHANGED_PACKET 9 // NYI
#define DMX_PRO_GET_SERIAL_NUMBER 10 // NYI

// API Protocol State Definitions
#define MSG_START         1
#define MSG_LABEL         2
#define MSG_DATA_LEN_LSB  3
#define MSG_DATA_LEN_MSB  4
#define MSG_DATA          5
#define MSG_END           6

#include "esp_dmx.h"

#define DMX_RX_PORT = DMX_NUM_1
#define DMX_TX_PORT = DMX_NUM_2

QueueHandle_t dmx_rx_queue, dmx_tx_queue;

unsigned char state;
unsigned char message_type;
unsigned int dataSize;
unsigned int index;
unsigned char data_buffer[601];

void setupDMX();

void setup() {
  Serial.begin(57600);
  // change the TX pin according to the DMX shield you're using
  setupDMX();
  state = MSG_START;
  
}

void processMessage();

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
    if (index >= dataSize) {
      state = MSG_END;
    }
    break;
  case MSG_END:
    if (c == DMX_PRO_END_MSG) {
      state = MSG_START;
      processMessage();
    }
    break;
  default:
    // We have lost our state?
    // Wait for next possible end message (could be false positive?)
    if (c == DMX_PRO_END_MSG) {
      state = MSG_START;
    }
    break;
  }
}

void processMessage() {
  switch (message_type)
  {
  case DMX_PRO_SEND_PACKET:
    if (dataSize > 0 && data_buffer[0] == DMX_START_CODE) {
      for (unsigned int i = 1; i < dataSize; i++) {
        // DmxMaster.write(i, data_buffer[i]);
      }
    }
    break;
  default:
    break;
  }
}

void setupDMX() {
  // first configure the UART...
  const dmx_config_t config = DMX_DEFAULT_CONFIG;
  dmx_param_config(DMX_RX_PORT, &config);
  dmx_param_config(DMX_TX_PORT, &config);

  // then set the communication pins...
  const int dmxrx_tx_io_num = 10, dmxrx_rx_io_num = 9, dmxrx_rts_io_num = 5;
  dmx_set_pin(DMX_RX_PORT, dmxrx_tx_io_num, dmxrx_rx_io_num, dmxrx_rts_io_num);
  const int dmxtx_tx_io_num = 17, dmxtx_rx_io_num = 16, dmxtx_rts_io_num = 21;
  dmx_set_pin(DMX_TX_PORT, dmxtx_tx_io_num, dmxtx_rx_io_num, dmxtx_rts_io_num);

  // and install the driver!
  dmx_driver_install(DMX_RX_PORT, DMX_MAX_PACKET_SIZE, 10, &dmx_rx_queue, 
        ESP_INTR_FLAG_IRAM);
  dmx_driver_install(DMX_TX_PORT, DMX_MAX_PACKET_SIZE, 10, &dmx_tx_queue, 
        ESP_INTR_FLAG_IRAM);

  dmx_set_mode(DMX_RX_PORT, DMX_MODE_READ);
  dmx_set_mode(DMX_TX_PORT, DMX_MODE_WRITE);
}