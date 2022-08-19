
// DMX Constants
// https://erg.abdn.ac.uk/users/gorry/eg3576/start-codes.html
#define DMX_START_CODE 0
#define RDM_START_CODE 0xcc

// RDM Constants
#define RDM_SUB_START_CODE 0x01
#define RDM_UID_LENGTH 6
#define RDM_CC_DISCOVER         0x10
#define RDM_CC_DISCOVER_RESP    0x11
#define RDM_CC_GET_COMMAND      0x20
#define RDM_CC_GET_COMMAND_RESP 0x21
#define RDM_CC_SET_COMMAND      0x30
#define RDM_CC_SET_COMMAND_RESP 0x31
#define RDM_RESP_ACK        0x00
#define RDM_RESP_ACK_TIMER  0x01
#define RDM_RESP_NACK       0x02
#define RDM_RESP_ACK_OVERFL 0x03
#define RDM_PID_DISC_UNIQUE_BRANCH  0x0001
#define RDM_PID_DISC_MUTE           0x0002
#define RDM_PID_DISC_UNMUTE         0x0003
#define RDM_PID_QUEUED_MESSAGE      0x0020
#define RDM_DISC_UNIQUE_BRANCH_SLOTS 24

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
// #define DMX_RX_DEBUG
#define DMX_ENABLE_RDM
#define ACT_LED_DMX_IN_TICKS  125 / portTICK_PERIOD_MS // 8Hz
#define ACT_LED_DMX_OUT_TICKS 500 / portTICK_PERIOD_MS // 2Hz
#define ACTIVITY_LED 2

// Pin Definitions
#define DMX_USB_RXD 9
#define DMX_USB_TXD 10
#define DMX_TX  17
#define DMX_RX  16
#define DMX_RTS 21

// TODO
/*
- Send RDM Discovery
- Respond to RDM Discovery
- Mode switch
- RDM Sniff?
*/

#include <EEPROM.h>
#include <esp_dmx.h>

const unsigned char rdm_uid[RDM_UID_LENGTH] = {
  0x45,
  0x4E,
  (SERIAL_NUMBER >> 24) & 0xff,
  (SERIAL_NUMBER >> 16) & 0xff,
  (SERIAL_NUMBER >> 8) & 0xff,
  SERIAL_NUMBER & 0xff
}; 
const unsigned char rdm_broadcast_all_uid[RDM_UID_LENGTH] = 
  {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
const unsigned char rdm_broadcast_manufacturer_uid[RDM_UID_LENGTH] = 
  {0x45, 0x4E, 0xff, 0xff, 0xff, 0xff};
const unsigned char rdm_ack_timer_estimate[2] = {0x00, 0x01}; // 100ms
const unsigned char rdm_disc_mute_control_field[2] = {0x00, 0b00000010}; // Support sub devices
unsigned char rdm_disc_packet[24] = {
  0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xAA,
  rdm_uid[0] & 0xAA, rdm_uid[0] & 0x55,
  rdm_uid[1] & 0xAA, rdm_uid[1] & 0x55,
  rdm_uid[2] & 0xAA, rdm_uid[2] & 0x55,
  rdm_uid[3] & 0xAA, rdm_uid[3] & 0x55,
  rdm_uid[4] & 0xAA, rdm_uid[4] & 0x55,
  rdm_uid[5] & 0xAA, rdm_uid[5] & 0x55,
  0x00, 0x00, 0x00, 0xff
};

// Device state
unsigned char widget_mode = FIRMWARE_DMX;
unsigned char recv_dmx_on_change = 0;
unsigned char rdm_discovery_muted = 0;

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
unsigned char rdm_queue_message = 0;
unsigned int rdm_queued_message_ready = 0;
unsigned char rdm_queued_message[DMX_MAX_PACKET_SIZE] = {0};

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
void ActivityLed(TimerHandle_t pxTimer);
void DMXRecvTask(void *parameter);
void handleRecvDMXPacket(unsigned int length, unsigned char* data);
unsigned char RDMDestMatch(unsigned char *dest_uid);
unsigned char RDMDestMatchUID(unsigned char *dest_uid);
unsigned char RDMChecksumValid(unsigned char *rdm_data);
unsigned char HandleRDM(unsigned char *rdm_data);
void SendRDMResponse(unsigned char *req_data,
    unsigned char resp_type, unsigned char resp_length, unsigned char *resp_data);
void calculateRDMDiscoverChecksum();

QueueHandle_t dmx_queue;
TimerHandle_t DMXRefreshTimer, ActivityLEDTimer;
TaskHandle_t DMXRecvTaskHandle;
SemaphoreHandle_t USBAPIResponseMutex, RDMQueueMutex;

// Main code
void setup() {
  Serial.begin(115200); // For debugging
  Serial.println("Widget Starting");
  USBAPIResponseMutex = xSemaphoreCreateMutex();
  RDMQueueMutex = xSemaphoreCreateMutex();
  Serial.println("Setup - EEPROM");
  if (!EEPROM.begin(512)) {
    Serial.println("EEPROM failed to initialise");
  } else loadEEPROMData();
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
  Serial.println("Setup - Activity Led Timer");
  pinMode(ACTIVITY_LED, OUTPUT);
  ActivityLEDTimer = xTimerCreate("activity_led", ACT_LED_DMX_OUT_TICKS, pdFALSE, 0, ActivityLed);
  Serial.println("Widget Ready");
}

void loop() {
  checkSerial();
}

// EEPROM Functions
void loadEEPROMData() {
  if (EEPROM.read(0) == 0x01)  {
    BreakTime = EEPROM.read(1);
    MaBTime = EEPROM.read(2);
    RefreshRate = EEPROM.read(3);
    for (unsigned int i = 0; i < 508; i++) {
      user_config[i] = EEPROM.read(4+i);
    }
  } else {
    saveEEPROMData();
  }
}

unsigned char EEPROMupdate(unsigned int address, unsigned char value) {
  if (EEPROM.read(address) != value) {
    EEPROM.write(address, value);
    return 1;
  }
  return 0;
}

void saveEEPROMData() {
  unsigned char changed = 0;
  changed |= EEPROMupdate(0, 0x01);
  changed |= EEPROMupdate(1, BreakTime);
  changed |= EEPROMupdate(2, MaBTime);
  changed |= EEPROMupdate(3, RefreshRate);
  for (unsigned int i = 0; i < 508; i++) {
    changed |= EEPROMupdate(4+i, user_config[i]);
  }
  if (changed) {
    EEPROM.commit();
  }
  Serial.println("EEPROM Written");
}

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
      #ifdef DMX_ENABLE_RDM
      dmx_set_mode(DMX_PORT, DMX_MODE_WRITE);
      if (dataSize > 0) {
        sendDMX(dataSize, data_buffer);
      }
      #endif
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
      #ifdef DMX_ENABLE_RDM
      dmx_set_mode(DMX_PORT, DMX_MODE_WRITE);
      #endif
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
  calculateRDMDiscoverChecksum();
  // We will need to modify the esp_dmx library to support RDM Discover Messages

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
  if (data[0] == RDM_START_CODE) {
    #ifdef DMX_ENABLE_RDM
    if (rdm_queue_message) {
      rdm_queue_message = 0;
      memcpy(rdm_queued_message, data, length);
      // Zero rest of packet
      if (length < DMX_MAX_PACKET_SIZE) {
        memset(&rdm_queued_message[length], 0, DMX_MAX_PACKET_SIZE - length);
      }
      rdm_queued_message_ready = 1;
      return;
    }
    #endif
  }

  unsigned char dmx_packet[DMX_MAX_PACKET_SIZE];
  // Copy data into fixed length buffer
  memcpy(dmx_packet, data, length);
  if (length < DMX_MAX_PACKET_SIZE) {
    memset(&dmx_packet[length], 0, DMX_MAX_PACKET_SIZE - length);
  }

  if (data[0] == DMX_START_CODE) {
    // Copy dmx packet into buffer then switch the buffers
    memcpy(dmx_buffer_select ? dmx_buffer_a : dmx_buffer_b, data, length);
    dmx_buffer_select ^= 1;
  }

  // block until we are ready to send another packet
  dmx_wait_send_done(DMX_PORT, DMX_PACKET_TIMEOUT_TICK);
    
  dmx_write_packet(DMX_PORT, dmx_packet, DMX_MAX_PACKET_SIZE);
  dmx_send_packet(DMX_PORT, DMX_MAX_PACKET_SIZE);

  if (data[0] == DMX_START_CODE) {
    xTimerReset(DMXRefreshTimer, 10);
  }
  xTimerChangePeriod(ActivityLEDTimer, ACT_LED_DMX_OUT_TICKS, 10); // This also starts the timer
}

void DMXRefresh(TimerHandle_t pxTimer) {
  dmx_mode_t dmx_mode = DMX_MODE_READ;
  dmx_get_mode(DMX_PORT, &dmx_mode);
  if (dmx_mode == DMX_MODE_WRITE) {
    // block until we are ready to send another packet
    dmx_wait_send_done(DMX_PORT, DMX_PACKET_TIMEOUT_TICK);

    // Read from opposite buffer to the one we write to (prevents reading a partially updated buffer)
    dmx_write_packet(DMX_PORT, dmx_buffer_select ? dmx_buffer_b : dmx_buffer_a, DMX_MAX_PACKET_SIZE);
    dmx_send_packet(DMX_PORT, DMX_MAX_PACKET_SIZE);
  }
}

void ActivityLed(TimerHandle_t pxTimer) {
  TickType_t led_period = xTimerGetPeriod(pxTimer);
  // Turn LED On
  digitalWrite(ACTIVITY_LED, 1);
  vTaskDelay(led_period / 2); // Delay for 1/2 period
  // Turn LED Off
  digitalWrite(ACTIVITY_LED, 0);
}

void DMXRecvTask(void *parameter) {
  dmx_event_t event;
  while (1) {
    // if (xQueueReceive(dmx_queue, &event, DMX_PACKET_TIMEOUT_TICK)) {
    if (xQueueReceive(dmx_queue, &event, portMAX_DELAY)) {
      switch (event.status) {
        case DMX_OK: { // Create scope for local variables
            #ifdef DMX_RX_DEBUG
            printf("Received packet with start code: %02X and size: %i\n",
              event.start_code, event.size);
            #endif
            // data is ok - read the packet into our buffer
            // Prefix RX packet with a 0
            unsigned char dmx_rx_packet[DMX_MAX_PACKET_SIZE+1];
            dmx_rx_packet[0] = 0; // Set Receive Status Byte
            dmx_read_packet(DMX_PORT, &dmx_rx_packet[1], event.size);
            handleRecvDMXPacket(event.size, dmx_rx_packet); // I assume the start code is still in the packet?
            xTimerChangePeriod(ActivityLEDTimer, ACT_LED_DMX_IN_TICKS, 10); // This also starts the timer
          }
          break;

        case DMX_ERR_IMPROPER_SLOT:
          #ifdef DMX_RX_DEBUG
          printf("Received malformed byte at slot %i\n", event.size);
          #endif
          // a slot in the packet is malformed - possibly a glitch due to the
          //  XLR connector? will need some more investigation
          // data can be recovered up until event.size
          break;

        case DMX_ERR_PACKET_SIZE:
          #ifdef DMX_RX_DEBUG
          printf("Packet size %i is invalid\n", event.size);
          #endif
          // the host DMX device is sending a bigger packet than it should
          // data may be recoverable but something went very wrong to get here
          break;

        case DMX_ERR_BUFFER_SIZE:
          #ifdef DMX_RX_DEBUG
          printf("User DMX buffer is too small - received %i slots\n", 
            event.size);
          #endif
          // whoops - our buffer isn't big enough
          // this code will not run if buffer size is set to DMX_MAX_PACKET_SIZE
          break;

        case DMX_ERR_DATA_OVERFLOW:
          #ifdef DMX_RX_DEBUG
          printf("Data could not be processed in time\n");
          #endif
          // the UART FIFO overflowed
          // this could occur if the interrupt mask is misconfigured or if the
          //  DMX ISR is constantly preempted
          break;
      }
    } else {
      #ifdef DMX_RX_DEBUG
      printf("Lost DMX signal\n");
      #endif
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
    #ifdef DMX_ENABLE_RDM
    case RDM_START_CODE:
      if (data[1] == RDM_SUB_START_CODE) { // Check Sub START Code
        if (data[2] + 2 == dmx_data_length) { // Check Length
          if (RDMChecksumValid(data)) { // Check Checksum
            if (!RDMDestMatch(&data[3])) break; // Ignore messages not meant for us
            if (!HandleRDM(data)) break; // If HandleRDM returns 1, we forward to PC
          }
        }
      }
      sendResponse(DMX_PRO_RECV_PACKET, dmx_data_length+1, data);
      break;
    #endif
  }
}

// RDM Functions
unsigned char RDMDestMatch(unsigned char *dest_uid) {
  if (memcmp(dest_uid, rdm_uid, RDM_UID_LENGTH) == 0) return 1;
  if (memcmp(dest_uid, rdm_broadcast_all_uid, RDM_UID_LENGTH) == 0) return 1;
  if (memcmp(dest_uid, rdm_broadcast_manufacturer_uid, RDM_UID_LENGTH) == 0) return 1;
  return 0;
}

unsigned char RDMDestMatchUID(unsigned char *dest_uid) {
  return (memcmp(dest_uid, rdm_uid, RDM_UID_LENGTH) == 0);
}

unsigned char RDMChecksumValid(unsigned char *rdm_data) {
  unsigned char length = rdm_data[2];
  // We can skip a few steps here as we know that it must be prefixed with the start code and sub start code
  uint16_t calc_checksum = RDM_START_CODE + RDM_SUB_START_CODE;
  for (unsigned int i = 2; i < length; i++) {
    calc_checksum += (uint16_t)(rdm_data[i]);
  }
  return ((calc_checksum >> 8) & 0xff == rdm_data[length]) &&
    (calc_checksum & 0xff == rdm_data[length+1]);
}

unsigned char HandleRDM(unsigned char *rdm_data) {
  // Return 1 if we want to forward message to PC
  unsigned char command_class = rdm_data[20];
  uint16_t pid = (((uint16_t)rdm_data[21]) << 8) | ((uint16_t)rdm_data[22]);
  switch (command_class) {
    case RDM_CC_DISCOVER:
      {
        switch (pid) {
          case RDM_PID_DISC_UNIQUE_BRANCH:
            // This has been moved to DMX Driver since its not a normal request packet
            // Do Nothing
            return 0;
          case RDM_PID_DISC_MUTE:
            if (memcmp(&rdm_data[3], rdm_uid, RDM_UID_LENGTH) == 0) {
              SendRDMResponse(rdm_data, RDM_RESP_ACK, 2, rdm_disc_mute_control_field);
            }
            return 0;
          case RDM_PID_DISC_UNMUTE:
            if (memcmp(&rdm_data[3], rdm_uid, RDM_UID_LENGTH) == 0) {
              SendRDMResponse(rdm_data, RDM_RESP_ACK, 2, rdm_disc_mute_control_field);
            }
            return 0;
        }
      }
      break;
    case RDM_CC_GET_COMMAND:
      if (pid == RDM_PID_QUEUED_MESSAGE) {
        if (rdm_queued_message_ready) {
          // Send packet
          // block until we are ready to send another packet
          dmx_wait_send_done(DMX_PORT, DMX_PACKET_TIMEOUT_TICK);

          // Read from opposite buffer to the one we write to (prevents reading a partially updated buffer)
          dmx_write_packet(DMX_PORT, rdm_queued_message, DMX_MAX_PACKET_SIZE);
          dmx_send_packet(DMX_PORT, DMX_MAX_PACKET_SIZE);
          rdm_queued_message_ready = 0;
          xSemaphoreGive(RDMQueueMutex); // Allow next message to get queued
          return 0;
        }
        // If queued message is not ready, and we are not waiting for the PC, forward to PC
        if (rdm_queue_message) {
          SendRDMResponse(rdm_data, RDM_RESP_ACK_TIMER, 2, rdm_ack_timer_estimate);
          return 0;
        }
      }
      // Fallthrough
    case RDM_CC_SET_COMMAND:
      SendRDMResponse(rdm_data, RDM_RESP_ACK_TIMER, 2, rdm_ack_timer_estimate);

      // Take mutex for RDM Queue to avoid edge cases
      xSemaphoreTake(RDMQueueMutex, portMAX_DELAY);
      rdm_queue_message = 1;
      return 1;
  }
  return 1;
}

void SendRDMResponse(unsigned char *req_data,
    unsigned char resp_type, unsigned char resp_length, const unsigned char *resp_data) {
  unsigned char length = 24+resp_length;

  unsigned char rdm_packet[DMX_MAX_PACKET_SIZE];
  rdm_packet[0] = RDM_START_CODE;
  rdm_packet[1] = RDM_SUB_START_CODE;
  rdm_packet[2] = length;
  memcpy(&rdm_packet[3], &req_data[9], RDM_UID_LENGTH);
  memcpy(&rdm_packet[9], rdm_uid, RDM_UID_LENGTH);
  rdm_packet[15] = req_data[15];
  rdm_packet[16] = resp_type;
  rdm_packet[17] = rdm_queued_message_ready;
  rdm_packet[18] = req_data[18]; // Sub Device Upper
  rdm_packet[19] = req_data[19]; // Sub Device Lower
  rdm_packet[20] = req_data[20]; // CC
  rdm_packet[21] = req_data[21]; // PID Upper
  rdm_packet[22] = req_data[22]; // PID Lower
  rdm_packet[23] = resp_length;
  if (resp_length > 0) {
    memcpy(&rdm_packet[24], resp_data, resp_length);
  }
  uint16_t checksum = RDM_START_CODE + RDM_SUB_START_CODE;
  for (unsigned int i = 2; i < length; i++) {
    checksum += (uint16_t)rdm_packet[i];
  }
  rdm_packet[length] = (checksum >> 8) & 0xff;
  rdm_packet[length+1] = checksum & 0xff;
  if (length+2 < DMX_MAX_PACKET_SIZE) {
    memset(&rdm_packet[length+2], 0, DMX_MAX_PACKET_SIZE-(length+2));
  }

  dmx_set_mode(DMX_PORT, DMX_MODE_WRITE);

  // block until we are ready to send another packet
  dmx_wait_send_done(DMX_PORT, DMX_PACKET_TIMEOUT_TICK);

  // Read from opposite buffer to the one we write to (prevents reading a partially updated buffer)
  dmx_write_packet(DMX_PORT, rdm_packet, DMX_MAX_PACKET_SIZE);
  dmx_send_packet(DMX_PORT, DMX_MAX_PACKET_SIZE);

  dmx_set_mode(DMX_PORT, DMX_MODE_READ);
}

void calculateRDMDiscoverChecksum() {
  uint16_t checksum = 0;
  for (unsigned int i = 0; i < 12; i++) {
    checksum += (uint16_t)rdm_disc_packet[8+i];
  }
  rdm_disc_packet[20] = (checksum >> 8) & 0xAA;
  rdm_disc_packet[21] = (checksum >> 8) & 0x55;
  rdm_disc_packet[22] = checksum & 0xAA;
  rdm_disc_packet[23] = checksum & 0x55;
}
