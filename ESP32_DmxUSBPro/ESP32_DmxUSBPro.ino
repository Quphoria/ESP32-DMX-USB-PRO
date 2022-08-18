
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
#define DMX_PRO_RECV_PACKET         5
#define DMX_PRO_SEND_PACKET         6 // "periodically send a DMX packet" mode
#define DMX_PRO_SEND_RDM            7 // NYI
#define DMX_PRO_RECV_DMX_ON_CHANGE  8
#define DMX_PRO_RECV_CHANGED_PACKET 9
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
#define DMX_PORT DMX_NUM_2
#define FIRMWARE_VERSION 144
#define SERIAL_NUMBER 0x0ffffffff // Not sure why the leading 0, should be 4 bytes, maybe documentation meant 0x
#define DMX_BAUD_RATE 250000 // typical baud rate - 250000
#define DMX_USB_BAUD_RATE 57600 // technically DMX USB Pro has no baud rate, but have seen others using this
#define NUM_SLOTS 100 // Number of slots for esp_dmx

// Pin Definitions
#define DMX_USB_RXD 9
#define DMX_USB_TXD 10
#define DMX_TX  17
#define DMX_RX  16
#define DMX_RTS 21

// TODO
/*
- Send RDM Discovery
- Handle RDM Messages
- EEPROM Load/Save Methods
*/

#include <esp_dmx.h>

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
unsigned char user_config[508];

// USB API state
unsigned char state;
unsigned char message_type;
unsigned int dataSize;
unsigned int data_index;
unsigned char data_buffer[600];

// DMX buffers
unsigned char dmx_buffer_select = 0;
unsigned char dmx_buffer_a[DMX_MAX_PACKET_SIZE] = {0};
unsigned char dmx_buffer_b[DMX_MAX_PACKET_SIZE] = {0};
unsigned char dmx_rx[DMX_MAX_PACKET_SIZE] = {0}; // Used to can generate dmx change messages

// Function forward declarations
void loadEEPROMData();
void saveEEPROMData();
void checkSerial();
void processMessage();
void sendResponse(unsigned char label, unsigned int length, unsigned char *data);
void sendDMXRecvChanged(unsigned int dmx_data_length, unsigned char *data);
uint8_t calculateBreakNum();
uint16_t calculateIdleNum();
unsigned int calculateRefreshTimerInterval();
void setupDMX();
void sendDMX(unsigned int length, unsigned char *data);
void DMXRefresh(TimerHandle_t pxTimer);
void DMXRecvTask(void *parameter);
void handleRecvDMXPacket(unsigned int length, unsigned char* data);

TimerHandle_t DMXRefreshTimer;
TaskHandle_t DMXRecvTaskHandle;
SemaphoreHandle_t USBAPIResponseMutex;

// Main code
void setup() {
  Serial.begin(115200); // For debugging
  Serial.println("Widget Starting");
  USBAPIResponseMutex = xSemaphoreCreateMutex();
  Serial.println("Setup - EEPROM");
  loadEEPROMData();
  Serial.println("Setup - Serial1");
  Serial1.begin(DMX_USB_BAUD_RATE, SERIAL_8N1, DMX_USB_RXD, DMX_USB_TXD);
  // change the TX pin according to the DMX shield you're using
  Serial.println("Setup - DMX");
  setupDMX();
  Serial.println("Setup - Recv Task");
  xTaskCreate(DMXRecvTask, "dmx_recv", 10000, NULL, 2, &DMXRecvTaskHandle);
  Serial.println("Setup - Refresh Timer");
  DMXRefreshTimer = xTimerCreate("dmx_refresh", calculateRefreshTimerInterval(), pdTRUE, 0, DMXRefresh);
  state = MSG_START;
  Serial.println("Widget Ready");
}

void loop() {
  checkSerial();
}

// EEPROM Functions
void loadEEPROMData() {}
void saveEEPROMData() {}

// Serial Functions
void checkSerial() {
  unsigned char c;

  while(!Serial1.available());
  c = Serial1.read();

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
      data_index = 0;
      state = data_index == dataSize ? MSG_END : MSG_DATA;
      break;
    case MSG_DATA:
      data_buffer[data_index] = c;
      data_index++;
      if (data_index >= dataSize) state = MSG_END;
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
      resp_buffer[0] = 'T';
      resp_buffer[1] = 'R';
      resp_buffer[2] = 'U';
      resp_buffer[3] = 'E';
      sendResponse(DMX_PRO_PROGRAM_FLASH, 4, resp_buffer);
      break;
    case DMX_PRO_GET_WIDGET_PARAMS:
      user_config_size = (data_buffer[1] << 8) | data_buffer[0];
      resp_buffer[0] = FIRMWARE_VERSION;
      resp_buffer[1] = widget_mode;
      resp_buffer[2] = BreakTime;
      resp_buffer[3] = MaBTime;
      resp_buffer[4] = RefreshRate;
      // Copy user config into response buffer
      if (user_config_size != 0) {
        memcpy(&resp_buffer[5], user_config, user_config_size);
      }
      sendResponse(DMX_PRO_GET_WIDGET_PARAMS, 5+user_config_size, resp_buffer);
      break;
    case DMX_PRO_SET_WIDGET_PARAMS: { // Put in scope for local variable
        dmx_set_mode(DMX_PORT, DMX_MODE_READ);
        unsigned char param_changed = 0;
        user_config_size = (data_buffer[1] << 8) | data_buffer[0];
        param_changed |= BreakTime != data_buffer[2];
        BreakTime = data_buffer[2];
        param_changed |= MaBTime != data_buffer[3];
        MaBTime = data_buffer[3];
        param_changed |= RefreshRate != data_buffer[4];
        RefreshRate = data_buffer[4];
        if (user_config_size != 0) {
          param_changed |= memcmp(user_config, &data_buffer[5], user_config_size) != 0;
          memcpy(user_config, &data_buffer[5], user_config_size);
        }
        dmx_set_break_num(DMX_PORT, calculateBreakNum());
        dmx_set_idle_num(DMX_PORT, calculateIdleNum());
        if (param_changed) {
          // Change timer period first, as if it breaks the device, the EEPROM data won't be changed
          xTimerChangePeriod(DMXRefreshTimer, calculateRefreshTimerInterval(), 100);
          saveEEPROMData();
        }
      }
      break;
    case DMX_PRO_SEND_PACKET:
      dmx_set_mode(DMX_PORT, DMX_MODE_WRITE);
      if (dataSize > 0) {
        sendDMX(dataSize, data_buffer);
      }
      break;
    case DMX_PRO_SEND_RDM:
      // Send RDM then reset back to read mode
      dmx_set_mode(DMX_PORT, DMX_MODE_WRITE);
      if (dataSize > 0) {
        sendDMX(dataSize, data_buffer);
      }
      dmx_set_mode(DMX_PORT, DMX_MODE_READ);
      break;
    case DMX_PRO_RECV_DMX_ON_CHANGE:
      dmx_set_mode(DMX_PORT, DMX_MODE_READ);
      recv_dmx_on_change = data_buffer[0] != 0; // Only 1 or 0
      break;
    case DMX_PRO_GET_SERIAL_NUMBER:
      dmx_set_mode(DMX_PORT, DMX_MODE_READ);
      resp_buffer[0] = SERIAL_NUMBER & 0xff;
      resp_buffer[1] = (SERIAL_NUMBER >> 8) & 0xff;
      resp_buffer[2] = (SERIAL_NUMBER >> 16) & 0xff;
      resp_buffer[3] = (SERIAL_NUMBER >> 24) & 0xff;
      sendResponse(DMX_PRO_GET_SERIAL_NUMBER, 4, resp_buffer);
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
  xSemaphoreTake(USBAPIResponseMutex, portMAX_DELAY); 
  Serial1.write(DMX_PRO_START_MSG);
  Serial1.write(label);
  Serial1.write(length & 0xff);
  Serial1.write((length >> 8) & 0xff);
  Serial1.write(data, length);
  Serial1.write(DMX_PRO_END_MSG);
  xSemaphoreGive(USBAPIResponseMutex); 
}

void sendDMXRecvChanged(unsigned int dmx_data_length, unsigned char *data) {
  // Wierd Compression algorithm, defined in API Documentation
  for (unsigned int i = 0; i < dmx_data_length; i++) {
    if (data[i] != dmx_rx[i]) {
      unsigned int start_byte_num = i >> 3; // Divide by 8
      unsigned int start_byte = start_byte_num << 3; // Multiply by 8
      unsigned int byte_offset = i - start_byte;
      unsigned char changed_bit_array[5] = {0};
      unsigned char num_changed = 1;
      unsigned char changed_data[40];
      changed_bit_array[byte_offset >> 3] |= 1 << (byte_offset & 0b111);
      changed_data[0] = data[i];
      dmx_rx[i] = data[i]; // Update rx buffer after we are done
      for (unsigned int x = i+1; x < start_byte + 40; x++) {
        byte_offset++;
        if (data[x] != dmx_rx[x]) {
          changed_data[num_changed] = data[x];
          dmx_rx[x] = data[x];
          changed_bit_array[byte_offset >> 3] |= 1 << (byte_offset & 0b111);
          num_changed++;
        }
      }
      unsigned char resp_data[1+5+40];
      resp_data[0] = start_byte_num;
      memcpy(&resp_data[1], changed_bit_array, 5);
      memcpy(&resp_data[6], changed_data, num_changed);
      sendResponse(DMX_PRO_RECV_CHANGED_PACKET, 6+num_changed, resp_data);
      i = start_byte + 39; // Skip bytes handled by the change packet
    }
  }
}

// DMX Functions
uint8_t calculateBreakNum() {
  return (uint8_t)((((double)BreakTime) * 10.67e-6) * (double)DMX_BAUD_RATE);
}
uint16_t calculateIdleNum() {
  // Round up
  return 1 + (uint8_t)((((double)MaBTime) * 10.67e-6) * (double)DMX_BAUD_RATE);
}
unsigned int calculateRefreshTimerInterval() {
  return (unsigned int)((1.0/(double)(min(40, (int)RefreshRate))) / (double)portTICK_PERIOD_MS);
}

void setupDMX() {
  // first configure the UART...
  const dmx_config_t config = {
    .baud_rate = DMX_BAUD_RATE, 
    .break_num = calculateBreakNum(), 
    .idle_num = calculateIdleNum()
  };
  dmx_param_config(DMX_PORT, &config);

  // then set the communication pins...
  dmx_set_pin(DMX_PORT, DMX_TX, DMX_RX, DMX_RTS);

  // and install the driver!
  dmx_driver_install(DMX_PORT, DMX_MAX_PACKET_SIZE, 10, &dmx_queue, 
    ESP_INTR_FLAG_IRAM);

  dmx_set_mode(DMX_PORT, DMX_MODE_READ);
}

void sendDMX(unsigned int length, unsigned char *data) {
  unsigned char dmx_packet[DMX_MAX_PACKET_SIZE] = {0};
  // Copy data into fixed length buffer
  memcpy(dmx_packet, data, length);

  if (data[0] == DMX_START_CODE) {
    // Copy dmx packet into buffer then switch the buffers
    memcpy(dmx_buffer_select ? dmx_buffer_a : dmx_buffer_b, data, length);
    dmx_buffer_select ^= 1;
  }

  // block until we are ready to send another packet
  dmx_wait_send_done(DMX_PORT, DMX_PACKET_TIMEOUT_TICK);
    
  dmx_write_packet(DMX_PORT, dmx_packet, NUM_SLOTS);
  dmx_send_packet(DMX_PORT, NUM_SLOTS);

  if (data[0] == DMX_START_CODE) {
    xTimerReset(DMXRefreshTimer, 10);
  }
}

void DMXRefresh(TimerHandle_t pxTimer) {
  dmx_mode_t dmx_mode = DMX_MODE_READ;
  dmx_get_mode(DMX_PORT, &dmx_mode);
  if (dmx_mode == DMX_MODE_WRITE) {
    // block until we are ready to send another packet
    dmx_wait_send_done(DMX_PORT, DMX_PACKET_TIMEOUT_TICK);

    // Read from opposite buffer to the one we write to (prevents reading a partially updated buffer)
    dmx_write_packet(DMX_PORT, dmx_buffer_select ? dmx_buffer_b : dmx_buffer_a, NUM_SLOTS);
    dmx_send_packet(DMX_PORT, NUM_SLOTS);
  }
}

void DMXRecvTask(void *parameter) {
  dmx_event_t event;
  while (1) {
    if (xQueueReceive(dmx_queue, &event, DMX_PACKET_TIMEOUT_TICK)) {
      switch (event.status) {
        case DMX_OK: { // Create scope for local variables
            printf("Received packet with start code: %02X and size: %i\n",
              event.start_code, event.size);
            // data is ok - read the packet into our buffer
            // Prefix RX packet with a 0
            unsigned char dmx_rx_packet[DMX_MAX_PACKET_SIZE+1];
            dmx_rx_packet[0] = 0; // Set Receive Status Byte
            dmx_read_packet(DMX_PORT, &dmx_rx_packet[1], event.size);
            handleRecvDMXPacket(event.size, dmx_rx_packet); // I assume the start code is still in the packet?
          }
          break;

        case DMX_ERR_IMPROPER_SLOT:
          printf("Received malformed byte at slot %i\n", event.size);
          // a slot in the packet is malformed - possibly a glitch due to the
          //  XLR connector? will need some more investigation
          // data can be recovered up until event.size
          break;

        case DMX_ERR_PACKET_SIZE:
          printf("Packet size %i is invalid\n", event.size);
          // the host DMX device is sending a bigger packet than it should
          // data may be recoverable but something went very wrong to get here
          break;

        case DMX_ERR_BUFFER_SIZE:
          printf("User DMX buffer is too small - received %i slots\n", 
            event.size);
          // whoops - our buffer isn't big enough
          // this code will not run if buffer size is set to DMX_MAX_PACKET_SIZE
          break;

        case DMX_ERR_DATA_OVERFLOW:
          printf("Data could not be processed in time\n");
          // the UART FIFO overflowed
          // this could occur if the interrupt mask is misconfigured or if the
          //  DMX ISR is constantly preempted
          break;
      }
    } else {
      printf("Lost DMX signal\n");
      // haven't received a packet in DMX_PACKET_TIMEOUT_TICK ticks
      // handle packet timeout...
    }
  }
}

void handleRecvDMXPacket(unsigned int dmx_data_length, unsigned char* data) {
  // Actual array length is dmx_data_length+1
  if (dmx_data_length == 0) return;
  unsigned char start_code = data[1];

  switch (start_code) {
    case DMX_START_CODE:
      if (recv_dmx_on_change) {
        // Skip recv status byte
        sendDMXRecvChanged(dmx_data_length, &data[1]);
      } else {
        sendResponse(DMX_PRO_RECV_PACKET, dmx_data_length+1, data);
      }
      break;
    case RDM_START_CODE:
      sendResponse(DMX_PRO_RECV_PACKET, dmx_data_length+1, data);
      break;
  }
}