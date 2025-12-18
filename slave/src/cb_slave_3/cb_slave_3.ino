// cb slave module
// (C) Jef Collin 2025

// notes
// there are 4 versions of the MCP4725, we use MCP4725A0T-E/CH with address 0x60


#include <Wire.h>
#include "Adafruit_MCP23X17.h"
#include <Adafruit_MCP4725.h>
#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// I2C IO expander
Adafruit_MCP23X17 mcp;

// since the S3 does not have a DAC we use an external one
Adafruit_MCP4725 dac;

// RS485 communication parameters
#define RS485_MASTER_ADDR 0x00
#define RS485_SLAVE_ADDR 0x01

// setup the RS485 port
HardwareSerial RS485(2);

// parameters write/read via mutex to ensure proper handover
SemaphoreHandle_t parameter_mutex;

// I2C devices
// 0x20 port expander
// 0x60 dac, see note

// pin definitions

// data to rs485
#define RS485_DI 1

// data re/de rs485
#define RS485_RE_DE 2

// data from rs485
#define RS485_RO 3

// power to display
#define PIN_DISPLAY_POWER 7

// squelch detect
#define PIN_SQ 8

// tx detect
#define PIN_TX 4

// s-meter analog
#define PIN_SMETER 13

// mcp pins
// PLL lock detect
#define PIN_LD 1

// Define the MCP23017 pins for PLL0 to PLL8
const uint8_t PIN_PLL[] = {8, 9, 10, 11, 12, 13, 14, 15, 0};

// Define the MCP23017 pins for channel switch 0-5
const uint8_t PIN_CHANNELSWITCH[] = {2, 3, 4, 5, 6, 7};

// original channel switch
uint16_t slave_switch_code = 0;

// master detection
boolean slave_master_active = false;
unsigned long slave_timer_master_detected = 0;
const unsigned long slave_master_detected_timeout = 3000;

// default calibrated tx power percentage
// 50 is around 1w
//uint8_t slave_tx_power_setting = 50;
// 4w
uint8_t slave_tx_power_setting = 71;



// power set by the master
uint8_t slave_remote_tx_power_setting = slave_tx_power_setting;

// pll lock detection
boolean slave_pll_locked = false;

// rs485 message parameters 4 bytes

uint16_t slave_in_message_pll_setting = 0;
uint8_t slave_in_message_power_setting = 0;
uint8_t slave_in_message_flags = 0;

uint8_t slave_out_message_pll_switch = 0;
uint16_t slave_out_message_s_meter = 0;
boolean slave_out_message_pll_locked = false;
boolean slave_out_message_squelch_status = false;
boolean slave_out_message_tx_on = false;
uint8_t slave_out_message_flags = 0;

// pll code including all extra bits as written to the pll
uint16_t slave_pll_code = 0;



void setup() {
  Serial.begin(115200);

  // setup I2C
  Wire.begin(9, 10);

  pinMode(RS485_DI, OUTPUT);
  pinMode(RS485_RE_DE, OUTPUT);
  digitalWrite(RS485_RE_DE, LOW);
  pinMode(RS485_RO, INPUT);

  pinMode(PIN_DISPLAY_POWER, OUTPUT);

  pinMode(PIN_SQ, INPUT);
  pinMode(PIN_TX, INPUT);
  pinMode(PIN_SMETER, INPUT);

  // setup io expander
  mcp.begin_I2C(0x20);

  mcp.pinMode(PIN_LD, INPUT);

  for (uint8_t i = 0; i < 6; i++) {
    mcp.pinMode(PIN_CHANNELSWITCH[i], INPUT);
  }

  for (uint8_t i = 0; i < 9; i++) {
    mcp.pinMode(PIN_PLL[i], OUTPUT);
  }

  // start the DAC
  dac.begin(0x60);

  // set default tx power
  set_tx_power(slave_tx_power_setting);

  // power on the display
  set_display_power(true);

  // setup adc for 12 bit 3.3v range
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // setup the rs485 serial port
  RS485.begin(115200, SERIAL_8N1, RS485_RO, RS485_DI);

  parameter_mutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(rs485_task, "rs485_task", 4096, NULL, 1, NULL, 0);

  xTaskCreatePinnedToCore(pll_monitor_task, "PLL Monitor", 2048, NULL, 1, NULL, 0);

}

void loop() {
  // we are allways preparing the outgoing packet since we need to respond after receiving a packet from the master

  // read original PLL switches
  if (read_channel_switch()) {
    // switch has changed
    if (!slave_master_active) {
      // change PLL setting in standalone mode
      pll_write(slave_switch_code, true);
    }
    // prepare frame data anyway
    xSemaphoreTake(parameter_mutex, portMAX_DELAY);
    slave_out_message_pll_switch = slave_switch_code;
    xSemaphoreGive(parameter_mutex);
  }

  // outgoing parameters
  uint16_t smeter_value = analogRead(PIN_SMETER);

  xSemaphoreTake(parameter_mutex, portMAX_DELAY);
  slave_out_message_s_meter = smeter_value;
  slave_out_message_pll_locked = slave_pll_locked;
  slave_out_message_squelch_status = digitalRead(PIN_SQ);
  slave_out_message_tx_on = !digitalRead(PIN_TX);
  slave_out_message_flags = 0;
  xSemaphoreGive(parameter_mutex);

  if (slave_master_active) {
    // incoming parameters
    xSemaphoreTake(parameter_mutex, portMAX_DELAY);
    uint16_t new_pll = slave_in_message_pll_setting;
    uint8_t new_power = slave_in_message_power_setting;
    xSemaphoreGive(parameter_mutex);
    if (new_pll != slave_pll_code) {
      pll_write(new_pll, false);
    }
    if (new_power != slave_remote_tx_power_setting) {
      slave_remote_tx_power_setting = new_power;
      set_tx_power(slave_remote_tx_power_setting);
    }
  }
  else {
    // standalone mode
    // this ensures tx power is reset to calibrated level when there is no connection to the master
    if (slave_remote_tx_power_setting != slave_tx_power_setting) {
      slave_remote_tx_power_setting = slave_tx_power_setting;
      // reset default power
      set_tx_power(slave_tx_power_setting);
    }
  }

  delay(1);

}

boolean read_channel_switch(void) {
  uint16_t allPins = mcp.readGPIOAB();
  uint16_t raw_state = (allPins >> 2) & 0x3F;
  unsigned long now = millis();

  static uint16_t last_raw_state = 0;
  static unsigned long last_change_time = 0;
  static bool is_debouncing = false;
  static uint16_t channel_switch_stable_state = 0;

  if (raw_state != last_raw_state) {
    last_raw_state = raw_state;
    last_change_time = now;
    is_debouncing = true;
    return false;
  }

  if (is_debouncing && (now - last_change_time) >= 25) {
    is_debouncing = false;
    if (raw_state != channel_switch_stable_state) {
      channel_switch_stable_state = raw_state;
      slave_switch_code = raw_state; // update external output only
      return true;
    }
  }

  return false;
}

// write to pll
void pll_write(uint16_t pll_code, boolean add_bits) {
  // add bits if original switch code is used
  if (add_bits) {
    pll_code |= (1 << 6);  // set bit6
    pll_code |= (1 << 7);  // set bit7
  }
  for (uint8_t i = 0; i < 9; i++) {
    uint8_t PLLbit = (pll_code >> i) & 1; // extract the bit at position i
    mcp.digitalWrite(PIN_PLL[i], PLLbit); // set the pin according to the bit value
  }
  slave_pll_code = pll_code;
}

// tx power in percentage 0-100
void set_tx_power(uint8_t percent) {
  const float Vref = 12.0;
  if (percent > 100) percent = 100;
  float Vout = (percent / 100.0) * Vref;
  float gain = 3.3 / Vref;
  int dacValue = (int)(Vout * gain * 4095 / 3.3 + 0.5);
  if (dacValue > 4095) dacValue = 4095;
  if (dacValue < 0) dacValue = 0;
  dac.setVoltage(dacValue, false);
}

// turn display power on or off
void set_display_power(boolean poweron) {
  digitalWrite(PIN_DISPLAY_POWER, poweron);
}



// virtual task for all rs485 communication
void rs485_task(void *pvParameters) {
  // frame parameters
  uint8_t src, dst, cmd; uint32_t data;
  for (;;) {
    // check for incoming message
    if (tryReceiveFrame(src, dst, cmd, data)) {
      if (dst == RS485_SLAVE_ADDR) {
        // incoming message
        if (cmd == 0x10) {
          decode_message(data);
          // any actions handled in main loop
          // a little delay in response to avoid quick ping pong
          vTaskDelay(15);
          // reply to master
          uint32_t returndata = encode_message();
          sendFrame(RS485_SLAVE_ADDR, RS485_MASTER_ADDR, 0x90, returndata);
        }
        if (!slave_master_active) {
          // first master detection
          set_display_power(false);
          slave_master_active = true;
          // reset to trigger new pll value in case it's the same
          slave_pll_code = 0;
        }
        slave_timer_master_detected = millis();
      }
    }

    // watchdog: we lost the master while in slave mode
    if (millis() - slave_timer_master_detected > slave_master_detected_timeout and slave_master_active) {
      // turn the display on
      set_display_power(true);
      // resume slave mode
      slave_master_active = false;
    }

    // rs485 message parameters 4 bytes
    //
    //    uint16_t slave_in_message_pll_setting = 0;
    //    uint8_t slave_in_message_power_setting = 0;
    //    uint8_t slave_in_message_flags = 0;
    //
    //    uint8_t slave_out_message_pll_switch = 0;
    //    uint16_t slave_out_message_s_meter = 0;
    //    boolean slave_out_message_pll_locked = false;
    //    boolean slave_out_message_squelch_status = false;
    //    boolean slave_out_message_tx_on = false;
    //    uint8_t slave_out_message_flags = 0;

    // to prevent watchdog error triggers
    vTaskDelay(1);
  }
}


//
//// decode incoming message into individual parameters
//void decode_message(uint32_t msgdata) {
//  master_in_message_pll_switch = (uint8_t) msgdata & 0x3F;
//  master_in_message_s_meter = (uint8_t) (msgdata >> 8) & 0xFF;
//  master_in_message_pll_locked = (msgdata >> 17) & 0x01;
//  master_in_message_squelch_open = (msgdata >> 18) & 0x01;
//  master_in_message_tx_on = (msgdata >> 19) & 0x01;
//  master_in_message_flags = (msgdata >> 24) & 0xFF;
//}
//
//// encode parameters into one message var
//uint32_t encode_message(void) {
//  uint32_t newdata = ((uint32_t) master_out_message_pll_setting & 0x01FF
//                      | (uint32_t) master_out_message_power_setting << 16
//                      | (uint32_t) master_out_message_flags << 24);
//  return newdata;
//}
//



// decode incoming message into individual parameter
void decode_message(uint32_t msgdata) {
  xSemaphoreTake(parameter_mutex, portMAX_DELAY);
  slave_in_message_pll_setting = (uint16_t) msgdata & 0x01FF;
  slave_in_message_power_setting = (uint8_t) (msgdata >> 16) & 0xFF;
  slave_in_message_flags = (uint8_t) (msgdata >> 24) & 0xFF;
  xSemaphoreGive(parameter_mutex);
}

// encode outgoing message
uint32_t encode_message(void) {
  xSemaphoreTake(parameter_mutex, portMAX_DELAY);
  uint32_t newdata = ((uint32_t) slave_out_message_pll_switch & 0x3F
                      | (uint32_t) slave_out_message_s_meter << 8
                      | (uint32_t) slave_out_message_pll_locked << 24
                      | (uint32_t) slave_out_message_squelch_status << 25
                      | (uint32_t) slave_out_message_tx_on << 26
                      | (uint32_t) slave_out_message_flags << 27);
  xSemaphoreGive(parameter_mutex);
  return newdata;
}

// 10 bytes packet structure
//[0] START (0xAA)
//[1] SRC (1 byte)
//[2] DST (1 byte)
//[3] CMD (1 byte)
//[4] DATA_L
//[5] DATA_x
//[6] DATA_x
//[7] DATA_H
//[8] CRC8
//[9] END (0x55)

// send frame to master
void sendFrame(uint8_t src, uint8_t dst, uint8_t cmd, uint32_t data) {
  uint8_t buf[10];
  // start byte
  buf[0] = 0xAA;
  // source number
  buf[1] = src;
  // destination number
  buf[2] = dst;
  // command
  buf[3] = cmd;
  // lsb first
  buf[4] = data & 0xFF;
  buf[5] = data >> 8;
  buf[6] = data >> 16;
  buf[7] = data >> 24;
  // checksum
  buf[8] = crc8(&buf[1], 7);
  // stop byte
  buf[9] = 0x55;

  digitalWrite(RS485_RE_DE, HIGH);
  delayMicroseconds(20);
  RS485.write(buf, 10);
  RS485.flush();
  delayMicroseconds(50);
  digitalWrite(RS485_RE_DE, LOW);
}

// check if we have a frame incoming and read it
bool tryReceiveFrame(uint8_t &src, uint8_t &dst, uint8_t &cmd, uint32_t &data) {
  static uint8_t buf[10];
  static uint8_t idx = 0;

  while (RS485.available()) {
    uint8_t b = RS485.read();
    if (idx == 0) {
      // check start byte
      if (b == 0xAA) buf[idx++] = b;
    } else {
      buf[idx++] = b;
      if (idx == 10) {
        idx = 0;
        if (buf[9] != 0x55) continue; // wrong end

        uint8_t crc = crc8(&buf[1], 7); // CRC covers bytes 1-7
        if (crc != buf[8]) continue; // CRC fail

        src = buf[1];
        dst = buf[2];
        cmd = buf[3];
        data = (uint32_t)buf[4] |
               ((uint32_t)buf[5] << 8) |
               ((uint32_t)buf[6] << 16) |
               ((uint32_t)buf[7] << 24);
        return true;
      }
    }
  }
  return false;
}

// CRC8 Dallas/Maxim (poly 0x31)
uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    uint8_t inbyte = *data++;
    for (uint8_t i = 8; i; --i) {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C; // reversed 0x31
      inbyte >>= 1;
    }
  }
  return crc;
}

// LD is high when PLL is locked, when unlocked pulses 100uS (10KHz) apart
void pll_monitor_task(void *parameter) {
  unsigned long ld_timer = 0;
  unsigned long continuous_high_time = 0;
  bool current_locked = false;
  bool new_locked = false;

  for (;;) {
    ld_timer = millis();
    bool low_detected = false;

    // measure during 10ms max
    while (millis() - ld_timer < 10) {
      if (!mcp.digitalRead(PIN_LD)) {
        low_detected = true;
        break;
      }
    }

    if (low_detected) {
      // not locked
      continuous_high_time = 0;
      new_locked = false;
    } else {
      // locked, update timer
      if (continuous_high_time == 0) {
        continuous_high_time = millis();
        new_locked = false;
      } else if (millis() - continuous_high_time >= 100) {
        // 100ms locked, final lock status
        new_locked = true;
      } else {
        new_locked = false;
      }
    }

    // mutex only is changed
    if (new_locked != current_locked) {
      xSemaphoreTake(parameter_mutex, portMAX_DELAY);
      slave_pll_locked = new_locked;
      xSemaphoreGive(parameter_mutex);
      current_locked = new_locked;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
