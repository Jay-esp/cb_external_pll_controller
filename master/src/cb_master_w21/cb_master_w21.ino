// CB master
// (C) Jef Collin
// 2025


// reminder: when lvgl library is updated, edit the config file!!!


// v 20 and up works with S3
// dont forget to short the in-out jumper pads
// set partition scheme to huge app or it will not fit

// make sure buffers are properly set in lv_conf otherwise either a hangup will happen when switching between settings screen and back or the webserver will not respond

// note that when connected to a pc with internal and external usb cable 5v will be low and this might trigger a brown out detection when wifi starts up
// for testing hold the button on boot until after the 2nd (re)boot due to brownout

// todo



#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <lwip/netdb.h>

#include "FS.h"
#include <SPI.h>
#include <Preferences.h>
#include <lvgl.h>
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <LovyanGFX.hpp>
#include <HardwareSerial.h>

#include "index_html.h"

extern lv_font_t dseg7_120;
extern lv_font_t dseg7_65;
extern lv_font_t dseg7_50;
extern lv_font_t dseg7_105;
extern lv_font_t dseg7_95;
extern lv_font_t dseg7_30;

// set to 1 for debug messages
// using the serial monitor seems to generated a lot of EMI on the CB radio

#define SHOW_DEBUG 0

SemaphoreHandle_t spi_mutex_lcd; // handle for the mutex lcd and sd

SemaphoreHandle_t responseSemaphore;


// change reference values if order changes in screen setup
#define TAB_MAIN_REF               0
#define TAB_SCAN_REF               1
#define TAB_MENU_REF               2
#define TAB_PRESETS_REF            3
#define TAB_SETTINGS_RANGE_REF     4
#define TAB_POWER_REF              5
#define TAB_DIRECTENTRYCHANNEL_REF 6
#define TAB_DIRECTENTRYFREQ_REF    7
#define TAB_WIFI_REF               8
#define TAB_WIFISETUP_REF          9
#define TAB_SETTINGS_REF           10
#define TAB_SETTINGS_S_PWR_REF     11

// parameters that change in vtask or isr best use a mutex
SemaphoreHandle_t parameter_mutex;

// pin definitions
#define RS485_DI    8
#define RS485_RO    14
#define RS485_RE_DE 9

HardwareSerial RS485(2);

#define MASTER_ADDR 0x00
#define SLAVE_ADDR 0x01

// lvgl
#define LV_USE_DEBUG = 0;
// 3.5" display
#define TFT_HOR_RES 480
#define TFT_VER_RES 320

#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))

// use alps or other decoder
#define Use_Alps_Encoder false

// rotary encode io pins
// reverse S1 and S2 for bourns
#define Encoder_1_Pin1 6
#define Encoder_1_Pin2 7
#define Encoder_2_Pin1 16
#define Encoder_2_Pin2 17
#define Encoder_1_Key 5
#define Encoder_2_Key 15

// rotary encoder
// No complete step yet.
#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20

#if (Use_Alps_Encoder)
// Alps EC11 encoder requires half step tables, others need full step
#define R_START 0x0
#define R_CCW_BEGIN 0x1
#define R_CW_BEGIN 0x2
#define R_START_M 0x3
#define R_CW_BEGIN_M 0x4
#define R_CCW_BEGIN_M 0x5

const unsigned char ttable[6][4] = {
  // R_START (00)
  {R_START_M,            R_CW_BEGIN,     R_CCW_BEGIN,  R_START},
  // R_CCW_BEGIN
  {R_START_M | DIR_CCW, R_START,        R_CCW_BEGIN,  R_START},
  // R_CW_BEGIN
  {R_START_M | DIR_CW,  R_CW_BEGIN,     R_START,      R_START},
  // R_START_M (11)
  {R_START_M,            R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},
  // R_CW_BEGIN_M
  {R_START_M,            R_START_M,      R_CW_BEGIN_M, R_START | DIR_CW},
  // R_CCW_BEGIN_M
  {R_START_M,            R_CCW_BEGIN_M,  R_START_M,    R_START | DIR_CCW},
};

#else
// Use the full-step state table (emits a code at 00 only)
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6
#define R_START 0x0

const unsigned char ttable[7][4] = {
  // R_START
  {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
  // R_CW_FINAL
  {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
  // R_CW_BEGIN
  {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
  // R_CW_NEXT
  {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
  // R_CCW_BEGIN
  {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
  // R_CCW_FINAL
  {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
  // R_CCW_NEXT
  {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};
#endif

unsigned char Encoder_1_State = R_START;
unsigned char Encoder_2_State = R_START;

boolean enc_key1 = false;
boolean enc_key2 = false;

// track rotary encoder changes
int EncoderCounter1 = 0;
int EncoderCounter2 = 0;

long unsigned timer_encoderbutton1;
long unsigned timer_encoderbutton2;

boolean Encoder_Key1_Long_Press = false;
boolean Encoder_Key2_Long_Press = false;

// general string buffer
char printbuf[20];

// scan state machine
enum ScanState {
  SCAN_IDLE,
  SCAN_START,
  SCAN_SCANNING,
  SCAN_HOLD,
  SCAN_STOP
};

ScanState scanState = SCAN_IDLE;

// WiFi States
enum WiFiState {
  WIFI_STATE_DISABLED,
  WIFI_STATE_ENABLED,
  WIFI_STATE_ENABLED_FAILED_LOGIN,
  WIFI_STATE_ENABLED_FAILED_CONNECT,
  WIFI_STATE_ENABLED_CONNECTED,
  WIFI_STATE_ENABLED_DISCONNECTED
};

// Global variables
WiFiState wifi_state = WIFI_STATE_DISABLED;

// graphics lib configuration
class LGFX : public lgfx::LGFX_Device
{
    lgfx::Panel_ILI9488     _panel_instance;
    lgfx::Touch_XPT2046     _touch_instance;
    lgfx::Bus_SPI           _bus_instance;
  public:
    LGFX(void)
    {
      {
        auto cfg = _bus_instance.config();
        // SPI
        cfg.spi_host = SPI2_HOST;
        cfg.spi_mode = 0;
        cfg.freq_write = 60000000;
        cfg.freq_read  = 16000000;
        cfg.spi_3wire  = true;
        cfg.use_lock   = true;
        cfg.dma_channel = SPI_DMA_CH_AUTO;
        cfg.pin_sclk = 12;
        cfg.pin_mosi = 11;
        cfg.pin_miso = 13;
        cfg.pin_dc   = 2;

        _bus_instance.config(cfg);
        _panel_instance.setBus(&_bus_instance);
      }
      {
        auto cfg = _panel_instance.config();
        cfg.pin_cs           =    10;
        cfg.pin_rst          =    -1;
        cfg.pin_busy         =    -1;
        cfg.panel_width      =   320;
        cfg.panel_height     =   480;
        cfg.offset_x         =     0;
        cfg.offset_y         =     0;
        cfg.offset_rotation  =     0;
        cfg.dummy_read_pixel =     8;
        cfg.dummy_read_bits  =     1;
        cfg.readable         =  true;
        cfg.invert           = false;
        cfg.rgb_order        = false;
        cfg.dlen_16bit       = false;
        cfg.bus_shared       = true;

        _panel_instance.config(cfg);
      }
      {
        // must be raw data point but not used if calibration is used
        auto cfg = _touch_instance.config();
        cfg.x_min      = 0;
        cfg.x_max      = 4000;
        cfg.y_min      = 0;
        cfg.y_max      = 4000;
        cfg.pin_int    = -1;
        cfg.bus_shared = true;
        cfg.offset_rotation = 0;

        // SPI
        cfg.spi_host = SPI2_HOST;
        cfg.freq = 1000000;
        cfg.pin_sclk = 12;
        cfg.pin_mosi = 11;
        cfg.pin_miso = 13;
        cfg.pin_cs = 42;
        _touch_instance.config(cfg);
        _panel_instance.setTouch(&_touch_instance);
      }
      setPanel(&_panel_instance);
    }
};

LGFX TFT;

LGFX_Sprite smeter_sprite(&TFT);

// callback routine for display
void my_disp_flush(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  lv_draw_sw_rgb565_swap(px_map, w * h);
  //TFT.startWrite();

  xSemaphoreTake(spi_mutex_lcd, portMAX_DELAY);
  TFT.waitDMA();
  if (TFT.getStartCount() == 0) {
    TFT.startWrite();
  }

  TFT.pushImageDMA(area->x1, area->y1, w, h, (uint16_t*)px_map);

  TFT.waitDMA();

  TFT.endWrite();

  xSemaphoreGive(spi_mutex_lcd);
  delay(1);

  lv_disp_flush_ready(disp);
}

// two buffer system
void *draw_buf_1;
void *draw_buf_2;

static lv_display_t *disp;

uint16_t setting_calibration_data[8];

Preferences preferences;

// timers
unsigned long LVGL_Timer;

static bool spinbox_lock = false;

uint8_t master_menu_mode = 0;

int8_t master_channel = 18;
int8_t master_previous_channel = 0;

uint16_t master_frequency = 27205;
uint16_t master_previous_frequency = 0;

boolean master_channel_is_alfa = false;

boolean master_channel_mode = true;

uint8_t master_stepsize = 10;

boolean master_screen_update_done = false;
uint8_t master_screen_update_counter = 0;

// default limits for -120 to 120, can be adjusted and stored
int8_t master_lower_channel = -120;
int8_t master_upper_channel = 120;
uint16_t master_lower_frequency = 25615;
uint16_t master_upper_frequency = 28305;

uint16_t master_minimum_frequency = master_lower_frequency;

// scan limits
int8_t master_scan_from_channel = 1;
int8_t master_scan_to_channel = 40;

int8_t master_scan_skip_channels[5];
uint8_t master_scan_skip_count = 0;

int8_t master_scan_current_channel = 1;

unsigned long master_scan_timer = 0;
unsigned long master_scan_hold_timer = 0;

// time to hold the scan after squelch is released
uint16_t master_scan_sq_hold_time = 3000;

uint16_t master_scan_interval = 150;

boolean master_scan_screen_mode_main = true;

uint8_t master_squelch_status = 0;

boolean master_squelch_ignore = false;

float master_scan_smeter_value = 0;

int8_t master_scan_hold_channel = 0;

boolean master_channel_frequency_refresh = true;

int8_t master_power = 73;

boolean master_pll_locked = false;

boolean master_tx_on = false;

boolean master_slave_is_connected = true;

boolean master_slave_is_connected_flag = false;

// dimensions smeter
const int16_t master_smeter_x = 58;
const int16_t master_smeter_y = 14;
const int16_t master_smeter_w = 362;
const int16_t master_smeter_h = 25;

// smeter labels
const char* master_smeter_labels[] = {
  "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "+10", "+20", "+30"
};
const int master_smeter_num_labels = sizeof(master_smeter_labels) / sizeof(master_smeter_labels[0]);

float master_smeter_value = 0;
float master_new_smeter_value = 0;

// power meter labels
const char* master_power_labels[] = {
  "0.5", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12+"
};

// power meter ticks
const int32_t master_power_positions[] = {68, 108, 141, 171, 192, 213, 228, 246, 258, 279, 306, 333, 360};

// button matrix maps for channel entry
static const char * master_direct_channel_btnm_map[] = {
  "1", "2", "3", "\n",
  "4", "5", "6", "\n",
  "7", "8", "9", "\n",
  "-", "0", LV_SYMBOL_LEFT, ""
};

static char master_direct_channel_input_buffer[10] = "";
static int master_direct_channel_input_index = 0;

// button matrix maps for frequency entry
static const char * master_direct_freq_btnm_map[] = {
  "1", "2", "3", "\n",
  "4", "5", "6", "\n",
  "7", "8", "9", "\n",
  "CLR", "0", LV_SYMBOL_LEFT, ""
};

static char master_direct_freq_input_buffer[10] = "";
static int master_direct_freq_input_index = 0;
static bool master_direct_freq_entered = false;

// rs485 message parameters 4 bytes
uint8_t master_in_message_pll_switch = 0;
uint16_t master_in_message_s_meter = 0;
boolean master_in_message_pll_locked = false;
boolean master_in_message_squelch_open = false;
boolean master_in_message_tx_on = false;
uint8_t master_in_message_flags = 0;

uint16_t master_out_message_pll_setting = 0;
uint8_t master_out_message_power_setting = 0;
uint8_t master_out_message_flags = 0;

// switch codes from slave translate to channel 1-40
uint8_t channel_array[40] = {
  63, 62, 61, 59, 58, 57, 56, 54, 53, 52, 51, 49, 48, 47, 46,
  44, 43, 42, 41, 39, 38, 37, 34, 36, 35, 33, 32, 31, 30, 29,
  28, 27, 26, 25, 24, 23, 22, 21, 20, 19
};

// presets
int8_t master_settings_default_channel = 20;
int8_t master_settings_lower_limit = -120;
int8_t master_settings_upper_limit = 120;
int8_t master_settings_preset1 = 20;
int8_t master_settings_preset2 = 20;
int8_t master_settings_preset3 = 20;
int8_t master_settings_preset4 = 20;
int8_t master_settings_power = 73;

String master_settings_wifi_ssid;
String master_settings_wifi_ssid_old;
String master_settings_wifi_password;
String master_settings_wifi_password_old;

int8_t master_settings_cal_s = 50;
int8_t master_settings_cal_pwr = 50;

int8_t master_settings_cal_s_previous = 50;
int8_t master_settings_cal_pwr_previous = 50;

bool chunkedTransferActive = false;
uint32_t lastChunkedTime = 0;

long unsigned wifi_timer;

// flag if we need to save the presets
boolean master_flag_save_presets = false;

char master_preset_texts[5][20] = {
  "STD ",   // Preset 0
  "P1",   // Preset 1
  "P2",  // Preset 2
  "P3",  // Preset 3
  "P4"   // Preset 4
};

AsyncWebServer webserver(80);
AsyncWebSocket ws("/ws");

unsigned long web_update = 0;

boolean master_allow_web = true;

int8_t master_web_channel = 18;
uint16_t master_web_frequency = 27205;
float master_web_smeter_value = 0;
boolean master_web_channel_is_alfa = false;
boolean master_web_tx_on = false;

// lvgl pointers
static lv_obj_t *tabview;

static lv_obj_t *tabmain;
static lv_obj_t *tabscan;
static lv_obj_t *tabmenu;
static lv_obj_t *tabpresets;
static lv_obj_t *tabpower;
static lv_obj_t *tabdirectentrychannel;
static lv_obj_t *tabdirectentryfreq;
static lv_obj_t *tabsettingsrange;
static lv_obj_t *tabwifi;
static lv_obj_t *tabwifisetup;
static lv_obj_t *tabsettings;
static lv_obj_t *tabsettings_s_pwr;

static lv_obj_t *btn_main_channel;
static lv_obj_t *lbl_main_channel;

static lv_obj_t *btn_main_freq;
static lv_obj_t *lbl_main_freq;

static lv_obj_t *btn_main_channel2;
static lv_obj_t *lbl_main_channel2;
static lv_obj_t *lbl_main_channel_alfa2;

static lv_obj_t *btn_main_freq2;
static lv_obj_t *lbl_main_freq2;

static lv_obj_t *led_main_nolock;
static lv_obj_t *lbl_main_noconnection;
static lv_obj_t *lbl_main_tx;
static lv_obj_t *lbl_main_wifistatus;
static lv_obj_t *btn_main_wifi;

static lv_obj_t *btn_directchannel_goto;
static lv_obj_t *btn_directchannel_cancel;

static lv_obj_t *btnmtrx_directchannel_keypad;

static lv_obj_t *lbl_directchannel_channel;

static lv_obj_t *btn_directfreq_goto;
static lv_obj_t *btn_directfreq_cancel;

static lv_obj_t *btnmtrx_directfreq_keypad;

static lv_obj_t *lbl_directfreq_frequency;

static lv_obj_t *btn_scan_channel;
static lv_obj_t *lbl_scan_channel;

static lv_obj_t *lbl_scan_squelch;

static lv_obj_t *bar_scan_progress;

static lv_obj_t * spinbox_scan_from;
static lv_obj_t * lbl_scan_from;

static lv_obj_t * spinbox_scan_to;
static lv_obj_t * lbl_scan_to;

static lv_obj_t * btn_scan_from_inc;
static lv_obj_t * btn_scan_from_dec;

static lv_obj_t * btn_scan_to_inc;
static lv_obj_t * btn_scan_to_dec;

static lv_obj_t *btn_scan_start;
static lv_obj_t *lbl_scan_start;

static lv_obj_t *btn_scan_skip;
static lv_obj_t *btn_scan_back;
static lv_obj_t *lbl_scan_back;

static lv_obj_t *btn_menu_scan;

static lv_obj_t *btn_menu_power;

static lv_obj_t *btn_menu_wifi;

static lv_obj_t *btn_menu_settings;

static lv_obj_t *btn_menu_back;

static lv_obj_t *slider_power;

static lv_obj_t *btn_power_save;
static lv_obj_t *btn_power_back;

static lv_obj_t *btn_settings_range_back;

static lv_obj_t * spinbox_settings_from;
static lv_obj_t * lbl_settings_from;

static lv_obj_t * spinbox_settings_to;
static lv_obj_t * lbl_settings_to;

static lv_obj_t * btn_settings_from_inc;
static lv_obj_t * btn_settings_from_dec;

static lv_obj_t * btn_settings_to_inc;
static lv_obj_t * btn_settings_to_dec;

static lv_obj_t *btn_settings_s_pwr_back;
static lv_obj_t *slider_settings_s_cal;
static lv_obj_t *slider_settings_power_cal;

static lv_obj_t *btn_settings_range;
static lv_obj_t *btn_settings_s_pwr_cal;
static lv_obj_t *btn_settings_back;

static lv_obj_t *lbl_wifi_ssid;

static lv_obj_t *lbl_wifi_ip;

static lv_obj_t *lbl_wifi_rssi;

static lv_obj_t *lbl_wifi_channel;

static lv_obj_t *lbl_wifi_wifistatus;

static lv_obj_t *btn_wifi_edit;

static lv_obj_t *btn_wifi_back;

static lv_obj_t *txt_wifi_ssid;

static lv_obj_t *txt_wifi_password;

static lv_obj_t *txt_wifi_keyboard;

static lv_obj_t *btn_presets_default;
static lv_obj_t *btn_presets_1;
static lv_obj_t *btn_presets_2;
static lv_obj_t *btn_presets_3;
static lv_obj_t *btn_presets_4;

static lv_obj_t *lbl_presets_default;
static lv_obj_t *lbl_presets_1;
static lv_obj_t *lbl_presets_2;
static lv_obj_t *lbl_presets_3;
static lv_obj_t *lbl_presets_4;

static lv_obj_t *btn_presets_save_default;
static lv_obj_t *btn_presets_save_1;
static lv_obj_t *btn_presets_save_2;
static lv_obj_t *btn_presets_save_3;
static lv_obj_t *btn_presets_save_4;

static lv_obj_t *btn_presets_back;


// touch screen callback
void my_touchpad_read(lv_indev_t * indev, lv_indev_data_t * data) {
  uint16_t touchX, touchY;
  data->state = LV_INDEV_STATE_REL;
  // Attempt to take mutex with timeout (e.g., 10ms)
  if (xSemaphoreTake(spi_mutex_lcd, pdMS_TO_TICKS(10)) == pdTRUE) {
    TFT.waitDMA();
    bool touched = TFT.getTouch(&touchX, &touchY);
    xSemaphoreGive(spi_mutex_lcd); // Release mutex immediately after hardware access
    if (touched) {
      data->state = LV_INDEV_STATE_PR;
      // Validate coordinates
      if (touchX <= 479 && touchY <= 319) {
        data->point.x = touchX;
        data->point.y = touchY;
      }
    }
  }
}


// encoder interrupts
void IRAM_ATTR isr1() {
  unsigned char pinstate = (digitalRead(Encoder_1_Pin2) << 1) | digitalRead(Encoder_1_Pin1);
  Encoder_1_State = ttable[Encoder_1_State & 0xf][pinstate];
  unsigned char result = Encoder_1_State & 0x30;
  if (result == DIR_CW) {
    if (EncoderCounter1 < 10) {
      xSemaphoreTake(parameter_mutex, portMAX_DELAY);
      EncoderCounter1++;
      xSemaphoreGive(parameter_mutex);
    }
  } else if (result == DIR_CCW) {
    if (EncoderCounter1 > -10) {
      xSemaphoreTake(parameter_mutex, portMAX_DELAY);
      EncoderCounter1--;
      xSemaphoreGive(parameter_mutex);
    }
  }
}

void IRAM_ATTR isr2() {
  unsigned char pinstate = (digitalRead(Encoder_2_Pin2) << 1) | digitalRead(Encoder_2_Pin1);
  Encoder_2_State = ttable[Encoder_2_State & 0xf][pinstate];
  unsigned char result = Encoder_2_State & 0x30;
  if (result == DIR_CW) {
    if (EncoderCounter2 < 10) {
      EncoderCounter2++;
    }
  } else if (result == DIR_CCW) {
    if (EncoderCounter2 > -10) {
      EncoderCounter2--;
    }
  }
}

// use Arduinos millis() as tick source
static uint32_t my_tick(void)
{
  return millis();
}

// wifi symbol
void show_wifi_status(void) {
  lv_color_t color;
  if (wifi_state == WIFI_STATE_ENABLED_CONNECTED) {
    color = lv_palette_main(LV_PALETTE_GREEN);
  } else {
    color = lv_palette_main(LV_PALETTE_RED);
  }
  lv_obj_set_style_text_color(lbl_main_wifistatus, color, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_wifi_wifistatus, color, LV_PART_MAIN);
}


void loop() {

  if (millis() - web_update > 100) {
    if (lv_tabview_get_tab_act(tabview) == TAB_MAIN_REF and wifi_state == WIFI_STATE_ENABLED_CONNECTED) {
      master_allow_web = true;
      if (master_web_channel != master_channel or master_web_frequency != master_frequency or master_web_smeter_value != master_smeter_value or master_web_channel_is_alfa != master_channel_is_alfa or master_web_tx_on != master_tx_on) {
        notifyClients();
      }
    }
    else {
      master_allow_web = false;
    }
    web_update = millis();
  }

  if (millis() - wifi_timer > 1000) {
    // check if we lost wifi
    if (wifi_state == WIFI_STATE_ENABLED_CONNECTED and WiFi.status() != WL_CONNECTED) {
      wifi_state = WIFI_STATE_DISABLED;
      show_wifi_status();
      update_wifi_parameters();
    }
    wifi_timer = millis();
  }

  check_actions_encoder1();
  check_actions_encoder2();
  check_actions_key1();
  check_actions_key2();

  check_refresh();

  // change in pll lock status
  if (master_pll_locked != master_in_message_pll_locked and lv_tabview_get_tab_act(tabview) == TAB_MAIN_REF) {
    master_pll_locked = master_in_message_pll_locked;
    show_lock_status(master_pll_locked, !master_slave_is_connected);
  }

  // change in tx/rx status
  if (master_tx_on != master_in_message_tx_on and (lv_tabview_get_tab_act(tabview) == TAB_MAIN_REF or lv_tabview_get_tab_act(tabview) == TAB_SETTINGS_S_PWR_REF)) {
    master_tx_on = master_in_message_tx_on;
    show_tx_status(master_tx_on, !master_slave_is_connected);
  }

  // change in slave connection status
  if (master_slave_is_connected != master_slave_is_connected_flag) {
    master_slave_is_connected = master_slave_is_connected_flag;
    if (lv_tabview_get_tab_act(tabview) == TAB_MAIN_REF) {
      show_connection_status(master_slave_is_connected);
      if (!master_slave_is_connected) {
        show_lock_status(false, true);
        show_tx_status(false, true);
      }
    }
    if (lv_tabview_get_tab_act(tabview) == TAB_SCAN_REF and !master_slave_is_connected and scanState != SCAN_IDLE) {
      scanState = SCAN_STOP;
    }
  }

  // change in smeter value
  if (master_smeter_value != master_new_smeter_value and (lv_tabview_get_tab_act(tabview) == TAB_MAIN_REF or lv_tabview_get_tab_act(tabview) == TAB_SCAN_REF or lv_tabview_get_tab_act(tabview) == TAB_SETTINGS_S_PWR_REF)) {
    master_smeter_value = master_new_smeter_value;
    if (lv_tabview_get_tab_act(tabview) == TAB_MAIN_REF or lv_tabview_get_tab_act(tabview) == TAB_SETTINGS_S_PWR_REF) {
      draw_s_meter_value(master_smeter_value);
    }
  }

  // scanner state machine
  switch (scanState) {
    case SCAN_IDLE:
      if (master_squelch_status != 0) {
        master_squelch_status = 0;
        show_squelch_status(master_squelch_status);
      }
      break;

    case SCAN_START:
      if (master_scan_to_channel - master_scan_from_channel > 20) {
        master_scan_current_channel = master_scan_from_channel;
        set_scan_channel(master_scan_current_channel);
        update_scan_channel(master_scan_current_channel);
        update_scan_progress(master_scan_current_channel);
        master_squelch_ignore = false;
        master_scan_timer = millis();
        scanState = SCAN_SCANNING;
        update_scan_buttons();
        master_scan_hold_channel = 0;
      }
      else {
        scanState = SCAN_IDLE;
      }
      break;

    case SCAN_SCANNING:
      // scanning stop conditions
      if (master_in_message_tx_on or !master_slave_is_connected) {
        scanState = SCAN_STOP;
        master_scan_hold_channel = 0;
      }
      else {
        if (master_in_message_squelch_open and is_channel_skipped(master_scan_current_channel)) {
          master_squelch_ignore = true;
        } else {
          master_squelch_ignore = false;
        }
        // squelch
        if (master_in_message_squelch_open and !master_squelch_ignore) {
          master_scan_hold_timer = millis();
          master_squelch_status = 1;
          show_squelch_status(master_squelch_status);
          scanState = SCAN_HOLD;
          update_scan_buttons();
          update_scan_progress(master_scan_current_channel);
          master_scan_smeter_value = master_smeter_value;
        }
        else {
          if (millis() - master_scan_timer > master_scan_interval) {
            next_scan_channel();
            set_scan_channel(master_scan_current_channel);
            update_scan_channel(master_scan_current_channel);
            update_scan_progress(master_scan_current_channel);
            master_scan_timer = millis();
          }
        }
      }
      break;

    case SCAN_HOLD:
      // hold due to squelch
      if (master_in_message_squelch_open) {
        // renew hold timer
        master_scan_hold_timer = millis();
        if (master_squelch_status != 1) {
          master_squelch_status = 1;
          show_squelch_status(master_squelch_status);
        }
        // redraw if smeter changed
        if (!master_scan_screen_mode_main and master_scan_smeter_value != master_smeter_value) {
          draw_scan_bar(250);
          master_scan_smeter_value = master_smeter_value;
        }
        master_scan_hold_channel = master_scan_current_channel;
      }
      else {
        if (master_squelch_status != 2) {
          master_squelch_status = 2;
          show_squelch_status(master_squelch_status);
        }
        if (millis() - master_scan_hold_timer >= master_scan_sq_hold_time) {
          // time elapsed, return to scanning
          master_squelch_status = 0;
          show_squelch_status(master_squelch_status);
          scanState = SCAN_SCANNING;
          update_scan_buttons();
        }
        master_scan_hold_channel = 0;
      }
      break;

    case SCAN_STOP:
      scanState = SCAN_IDLE;
      update_scan_buttons();
      break;
  }


  // update screen, let lvgl do its thing
  if (millis() >= LVGL_Timer + 5) {

    lv_timer_handler();

    // draw after screen is first drawn, smeter
    // we need some more time for the screen to rebuild so a counter is used
    if (!master_screen_update_done and (lv_tabview_get_tab_act(tabview) == TAB_MAIN_REF or lv_tabview_get_tab_act(tabview) == TAB_SETTINGS_S_PWR_REF) and master_screen_update_counter > 5) {

      draw_s_meter_scale();

      draw_s_meter_value(master_smeter_value);
      master_screen_update_done = true;

    }
    // no point increasing the counter further
    if (master_screen_update_counter < 10) {
      master_screen_update_counter++;
    }
    LVGL_Timer = millis();
  }
}

void check_actions_encoder1(void) {
  // check rotary encoders
  if (EncoderCounter1 != 0) {
    // int dir = (EncoderCounter1 > 0) ? 1 : -1;
    switch (lv_tabview_get_tab_act(tabview)) {
      case TAB_MAIN_REF:
        // main screen channel or frequency
        if (EncoderCounter1 != 0) {
          if (master_channel_mode) {
            // channel mode
            if (EncoderCounter1 > 0) {
              if (master_channel < master_upper_channel) {
                // intercept ch 0
                if (master_channel == -1) {
                  master_channel = 0;
                }
                master_channel++;
                adjust_encoder1(-1);
              } else if (master_channel == master_upper_channel) {
                master_channel = master_lower_channel;
                adjust_encoder1(-1);
              } else {
                adjust_encoder1(0);
              }
            }
            else {
              if (EncoderCounter1 < 0) {
                if (master_channel > master_lower_channel) {
                  // intercept ch 0
                  if (master_channel == 1) {
                    master_channel = 0;
                  }
                  master_channel--;
                  adjust_encoder1(1);
                } else if (master_channel == master_lower_channel) {
                  master_channel = master_upper_channel;
                  adjust_encoder1(1);
                } else {
                  adjust_encoder1(0);
                }
              }
            }
            if (master_channel != master_previous_channel) {
              master_channel_frequency_refresh = true;
            }
          }
          else {
            // frequency mode
            if (EncoderCounter1 > 0) {
              if (master_frequency < master_upper_frequency) {
                master_frequency = master_frequency + master_stepsize;
                adjust_encoder1(-1);
              } else if (master_frequency == master_upper_frequency) {
                master_frequency = master_lower_frequency;
                adjust_encoder1(-1);
              } else {
                adjust_encoder1(0);
              }
            }
            else {
              if (EncoderCounter1 < 0) {
                if (master_frequency > master_lower_frequency) {
                  master_frequency = master_frequency - master_stepsize;
                  adjust_encoder1(1);
                } else if (master_frequency == master_lower_frequency) {
                  master_frequency = master_upper_frequency;
                  adjust_encoder1(1);
                } else {
                  adjust_encoder1(0);
                }
              }
            }
            if (master_frequency != master_previous_frequency) {
              master_channel_frequency_refresh = true;
            }
          }
        }
        break;

      case TAB_SCAN_REF:
        // scan screen
        if (scanState == SCAN_IDLE) {
          if (EncoderCounter1 > 0) {
            my_spinbox_inc(spinbox_scan_from);
            adjust_encoder1(-1);
          }
          else {
            if (EncoderCounter1 < 0) {
              my_spinbox_dec(spinbox_scan_from);
              adjust_encoder1(1);
            }
          }
        }
        break;

      case TAB_POWER_REF:
        if (EncoderCounter1 > 0) {
          lv_slider_set_value(slider_power, lv_slider_get_value(slider_power) + 1, LV_ANIM_OFF);
          lv_obj_send_event(slider_power, LV_EVENT_VALUE_CHANGED, NULL);
          adjust_encoder1(-1);
        }
        else {
          if (EncoderCounter1 < 0) {
            lv_slider_set_value(slider_power, lv_slider_get_value(slider_power) - 1, LV_ANIM_OFF);
            lv_obj_send_event(slider_power, LV_EVENT_VALUE_CHANGED, NULL);
            adjust_encoder1(1);
          }
        }
        break;

      case TAB_SETTINGS_RANGE_REF:
        // settings screen
        if (EncoderCounter1 > 0) {
          my_spinbox_inc(spinbox_settings_from);
          adjust_encoder1(-1);
        }
        else {
          if (EncoderCounter1 < 0) {
            my_spinbox_dec(spinbox_settings_from);
            adjust_encoder1(1);
          }
        }
        break;

      case TAB_SETTINGS_S_PWR_REF:
        if (EncoderCounter1 > 0) {
          lv_slider_set_value(slider_settings_s_cal, lv_slider_get_value(slider_settings_s_cal) + 1, LV_ANIM_OFF);
          lv_obj_send_event(slider_settings_s_cal, LV_EVENT_VALUE_CHANGED, NULL);
          adjust_encoder1(-1);
        }
        else {
          if (EncoderCounter1 < 0) {
            lv_slider_set_value(slider_settings_s_cal, lv_slider_get_value(slider_settings_s_cal) - 1, LV_ANIM_OFF);
            lv_obj_send_event(slider_settings_s_cal, LV_EVENT_VALUE_CHANGED, NULL);
            adjust_encoder1(1);
          }
        }
        break;

      default:
        // clear if other screens
        adjust_encoder1(0);
        break;
    }
  }
}

void check_actions_encoder2(void) {
  if (EncoderCounter2 != 0) {
    int dir = (EncoderCounter2 > 0) ? 1 : -1;
    switch (lv_tabview_get_tab_act(tabview)) {
      case TAB_MAIN_REF:
        // main screen
        if (EncoderCounter2 > 0) {
          lv_slider_set_value(slider_power, lv_slider_get_value(slider_power) + 1, LV_ANIM_OFF);
          lv_obj_send_event(slider_power, LV_EVENT_VALUE_CHANGED, NULL);
          EncoderCounter2--;
        }
        else {
          if (EncoderCounter2 < 0) {
            lv_slider_set_value(slider_power, lv_slider_get_value(slider_power) - 1, LV_ANIM_OFF);
            lv_obj_send_event(slider_power, LV_EVENT_VALUE_CHANGED, NULL);
            EncoderCounter2++;
          }
        }
        break;

      case TAB_SCAN_REF:
        // scan screen
        if (scanState == SCAN_IDLE) {
          if (EncoderCounter2 > 0) {
            my_spinbox_inc(spinbox_scan_to);
            EncoderCounter2--;
          }
          else {
            if (EncoderCounter2 < 0) {
              my_spinbox_dec(spinbox_scan_to);
              EncoderCounter2++;
            }
          }
        }
        break;

      case TAB_SETTINGS_RANGE_REF:
        // settings screen
        if (EncoderCounter2 > 0) {
          my_spinbox_inc(spinbox_settings_to);
          EncoderCounter2--;
        }
        else {
          if (EncoderCounter2 < 0) {
            my_spinbox_dec(spinbox_settings_to);
            EncoderCounter2++;
          }
        }
        break;

      case TAB_SETTINGS_S_PWR_REF:
        if (EncoderCounter2 > 0) {
          lv_slider_set_value(slider_settings_power_cal, lv_slider_get_value(slider_settings_power_cal) + 1, LV_ANIM_OFF);
          lv_obj_send_event(slider_settings_power_cal, LV_EVENT_VALUE_CHANGED, NULL);
          EncoderCounter2--;
        }
        else {
          if (EncoderCounter2 < 0) {
            lv_slider_set_value(slider_settings_power_cal, lv_slider_get_value(slider_settings_power_cal) - 1, LV_ANIM_OFF);
            lv_obj_send_event(slider_settings_power_cal, LV_EVENT_VALUE_CHANGED, NULL);
            EncoderCounter2++;
          }
        }
        break;

      default:
        // clear if other screens
        EncoderCounter2 = 0;
        break;
    }
  }
}

// 0 = clear, +1 or -1
void adjust_encoder1(int8_t setmode) {
  xSemaphoreTake(parameter_mutex, portMAX_DELAY);
  if (setmode == 0) {
    EncoderCounter1 = 0;
  }
  else {
    EncoderCounter1 += setmode;
  }
  xSemaphoreGive(parameter_mutex);
}

void check_actions_key1(void) {
  if (digitalRead(Encoder_1_Key) == 0) {
    timer_encoderbutton1 = millis();
    Encoder_Key1_Long_Press = false;
    // wait until key is no longer pressed or time expired
    while (digitalRead(Encoder_1_Key) == 0) {
      if (millis() - timer_encoderbutton1 > 1000) {
        Encoder_Key1_Long_Press = true;
        break;
      }
    }
    if (Encoder_Key1_Long_Press) {
      // long press
      while (digitalRead(Encoder_1_Key) == 0) {}
      //little debounce
      delay(200);
    }
    else {
      // short press
      switch (lv_tabview_get_tab_act(tabview)) {
        case TAB_MAIN_REF:
          toggle_channel_freq();
          break;

        case TAB_SCAN_REF:
          if ((scanState == SCAN_IDLE or scanState == SCAN_HOLD) and master_slave_is_connected) {
            start_scan();
          } else if (scanState != SCAN_IDLE) {
            scanState = SCAN_STOP;
          }
          break;
      }
    }
  }
}

void check_actions_key2(void) {
  if (digitalRead(Encoder_2_Key) == 0) {
    timer_encoderbutton2 = millis();
    Encoder_Key2_Long_Press = false;
    // wait until key is no longer pressed or time expired
    while (digitalRead(Encoder_2_Key) == 0) {
      if (millis() - timer_encoderbutton2 > 1000) {
        Encoder_Key2_Long_Press = true;
        break;
      }
    }
    if (Encoder_Key2_Long_Press) {
      // long press
      while (digitalRead(Encoder_2_Key) == 0) {}
      //little debounce
      delay(200);
    }
    else {
      // short press
      switch (lv_tabview_get_tab_act(tabview)) {
        case TAB_MAIN_REF:
          lv_tabview_set_act(tabview, TAB_MENU_REF, LV_ANIM_OFF);
          break;

        case TAB_SCAN_REF:
          master_scan_screen_mode_main = !master_scan_screen_mode_main;
          set_scan_mode_screen(master_scan_screen_mode_main, true);
          break;

      }
    }
  }
}

void check_refresh(void) {
  if (master_channel_frequency_refresh) {
    if (master_channel_mode) {
      master_frequency = convert_channel_to_frequency(master_channel);
      master_previous_channel = master_channel;
      update_channel();
      update_frequency();
    }
    else {
      master_channel = convert_frequency_to_channel(master_frequency);
      master_previous_frequency = master_frequency;
      update_frequency();
      update_channel();
    }
    master_channel_frequency_refresh = false;
    // calculate new pll setting, this will be transferred to the slave automatically
    master_out_message_pll_setting = calculate_PLL(master_frequency);
  }
}

// splash screen
void display_splash_screen(void) {
  lv_obj_set_style_bg_color (lv_scr_act(), lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );

  LV_IMG_DECLARE(rystl_logo_2);
  lv_obj_t *  img_logo_main = lv_img_create(lv_scr_act());
  lv_img_set_src(img_logo_main, &rystl_logo_2);
  lv_obj_set_pos(img_logo_main, 139, 0);

  static lv_style_t label_style2;
  lv_style_set_text_color(&label_style2, lv_color_white());
  lv_style_set_text_font(&label_style2, &lv_font_montserrat_20);

  lv_obj_t *labelintro2 = lv_label_create(lv_scr_act());
  lv_obj_add_style(labelintro2, &label_style2, LV_PART_MAIN);
  lv_label_set_text(labelintro2, "At power on press:");
  lv_obj_align(labelintro2, LV_ALIGN_BOTTOM_MID, 0, -130);

  lv_obj_t *labelintro3 = lv_label_create(lv_scr_act());
  lv_obj_add_style(labelintro3, &label_style2, LV_PART_MAIN);
  lv_label_set_text(labelintro3, "- top encoder button to enable WiFi");
  lv_obj_align(labelintro3, LV_ALIGN_BOTTOM_MID, 0, -90);

  lv_obj_t *labelintro4 = lv_label_create(lv_scr_act());
  lv_obj_add_style(labelintro4, &label_style2, LV_PART_MAIN);
  lv_label_set_text(labelintro4, "- both encoder buttons to calibrate LCD");
  lv_obj_align(labelintro4, LV_ALIGN_BOTTOM_MID, 0, -50);

  static lv_style_t label_style3;
  lv_style_set_text_color(&label_style3, lv_color_white());
  lv_style_set_text_font(&label_style3, &lv_font_montserrat_28);
  lv_obj_t *labelintro10 = lv_label_create(lv_scr_act());
  lv_obj_add_style(labelintro10, &label_style3, LV_PART_MAIN);
  lv_label_set_text(labelintro10, "(C) Jef Collin 2025");
  lv_obj_align(labelintro10, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_refr_now(NULL);
}

void setup_screens(void) {
  // general reusable
  lv_obj_t * label;
  int32_t obj_height;

  // tabs
  tabview = lv_tabview_create(lv_scr_act());
  lv_tabview_set_tab_bar_size(tabview, 0);

  tabmain = lv_tabview_add_tab(tabview, "");
  tabscan = lv_tabview_add_tab(tabview, "");
  tabmenu = lv_tabview_add_tab(tabview, "");
  tabpresets = lv_tabview_add_tab(tabview, "");
  tabsettingsrange = lv_tabview_add_tab(tabview, "");
  tabpower = lv_tabview_add_tab(tabview, "");
  tabdirectentrychannel = lv_tabview_add_tab(tabview, "");
  tabdirectentryfreq = lv_tabview_add_tab(tabview, "");
  tabwifi = lv_tabview_add_tab(tabview, "");
  tabwifisetup = lv_tabview_add_tab(tabview, "");
  tabsettings = lv_tabview_add_tab(tabview, "");
  tabsettings_s_pwr = lv_tabview_add_tab(tabview, "");

  lv_obj_clear_flag(tabview, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_clear_flag(tabmain, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabscan, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabmenu, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabpresets, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabsettingsrange, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabpower, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabdirectentrychannel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabdirectentryfreq, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabwifi, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabwifisetup, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabsettings, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabsettings_s_pwr, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_clear_flag(lv_tabview_get_content(tabview), LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_set_style_bg_color (tabmain, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tabscan, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tabmenu, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tabpresets, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tabsettingsrange, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tabpower, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tabdirectentrychannel, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tabdirectentryfreq, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tabwifi, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tabwifisetup, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tabsettings, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tabsettings_s_pwr, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );

  lv_obj_set_style_bg_opa(tabmain, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabscan, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabmenu, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabpresets, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabsettingsrange, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabpower, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabdirectentrychannel, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabdirectentryfreq, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabwifi, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabwifisetup, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabsettings, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tabsettings_s_pwr, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);

  // remove 16 pix padding
  lv_obj_set_style_pad_top(tabscan, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabscan, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabscan, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabscan, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tabmain, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabmain, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabmain, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabmain, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tabmenu, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabmenu, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabmenu, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabmenu, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tabpresets, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabpresets, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabpresets, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabpresets, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tabsettingsrange, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabsettingsrange, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabsettingsrange, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabsettingsrange, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tabpower, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabpower, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabpower, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabpower, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tabdirectentrychannel, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabdirectentrychannel, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabdirectentrychannel, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabdirectentrychannel, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tabdirectentryfreq, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabdirectentryfreq, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabdirectentryfreq, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabdirectentryfreq, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tabwifi, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabwifi, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabwifi, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabwifi, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tabwifisetup, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabwifisetup, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabwifisetup, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabwifisetup, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tabsettings, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabsettings, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabsettings, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabsettings, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tabsettings_s_pwr, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tabsettings_s_pwr, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tabsettings_s_pwr, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tabsettings_s_pwr, 0, LV_PART_MAIN);

  // styles

  static lv_style_t label_style2;
  lv_style_set_text_color(&label_style2, lv_color_white());
  lv_style_set_text_font(&label_style2, &lv_font_montserrat_22);

  static lv_style_t label_style3;
  lv_style_set_text_color(&label_style3, lv_color_black());
  lv_style_set_text_font(&label_style3, &lv_font_montserrat_24);

  static lv_style_t label_style4;
  lv_style_set_text_color(&label_style4,  lv_color_white());
  lv_style_set_text_font(&label_style4, &lv_font_montserrat_28);

  static lv_style_t label_style5;
  lv_style_set_text_color(&label_style5, lv_color_white());
  lv_style_set_text_font(&label_style5, &lv_font_montserrat_24);

  static lv_style_t label_style6;
  lv_style_set_text_color(&label_style6,  lv_color_white());
  lv_style_set_text_font(&label_style6, &lv_font_montserrat_32);

  static lv_style_t label_style7;
  lv_style_set_text_color(&label_style7,  lv_color_black());
  lv_style_set_text_font(&label_style7, &dseg7_30);

  // button styles

  static lv_style_t btn_style2;
  lv_style_set_radius(&btn_style2, 3);
  lv_style_set_bg_color(&btn_style2, lv_color_white());
  lv_style_set_text_font(&btn_style2, &lv_font_montserrat_32);
  lv_style_set_text_color(&btn_style2, lv_palette_main(LV_PALETTE_GREEN));
  lv_style_set_size(&btn_style2, 75, 75);

  static lv_style_t btn_style3;
  lv_style_set_radius(&btn_style3, 3);
  lv_style_set_bg_color(&btn_style3, lv_color_white());
  lv_style_set_text_font(&btn_style3, &lv_font_montserrat_32);
  lv_style_set_text_color(&btn_style3, lv_palette_main(LV_PALETTE_RED));
  lv_style_set_size(&btn_style3, 75, 75);

  static lv_style_t btn_style4;
  lv_style_set_radius(&btn_style4, 3);
  lv_style_set_bg_color(&btn_style4, lv_color_white());
  lv_style_set_text_font(&btn_style4, &lv_font_montserrat_24);
  lv_style_set_text_color(&btn_style4, lv_color_black());

  static lv_style_t btn_style5;
  lv_style_set_bg_color(&btn_style5, lv_color_black());
  lv_style_set_border_width(&btn_style5, 0);

  static lv_style_t btn_style6;
  lv_style_set_radius(&btn_style6, 3);
  lv_style_set_bg_color(&btn_style6, lv_color_white());
  lv_style_set_text_font(&btn_style6, &lv_font_montserrat_32);
  lv_style_set_text_color(&btn_style6, lv_color_black());
  lv_style_set_size(&btn_style6, 75, 75);

  static lv_style_t btn_style7;
  lv_style_set_radius(&btn_style7, 3);
  lv_style_set_bg_color(&btn_style7, lv_color_white());
  lv_style_set_text_font(&btn_style7, &lv_font_montserrat_32);
  lv_style_set_text_color(&btn_style7, lv_color_black());
  lv_style_set_size(&btn_style7, 150, 100);

  static lv_style_t btn_style8;
  lv_style_set_radius(&btn_style8, 0);
  lv_style_set_text_font(&btn_style8,  &dseg7_120);
  lv_style_set_text_color(&btn_style8, lv_palette_main(LV_PALETTE_RED));
  lv_style_set_bg_color(&btn_style8, lv_color_black());
  lv_style_set_outline_opa(&btn_style8, LV_OPA_TRANSP);
  lv_style_set_shadow_opa(&btn_style8, LV_OPA_TRANSP);

  static lv_style_t btn_style9;
  lv_style_set_radius(&btn_style9, 0);
  lv_style_set_text_font(&btn_style9,  &dseg7_65);
  lv_style_set_text_color(&btn_style9, lv_color_white());
  lv_style_set_bg_color(&btn_style9, lv_color_black());
  lv_style_set_outline_opa(&btn_style9, LV_OPA_TRANSP);
  lv_style_set_shadow_opa(&btn_style9, LV_OPA_TRANSP);

  static lv_style_t btn_style10;
  lv_style_set_radius(&btn_style10, 0);
  lv_style_set_text_font(&btn_style10,  &dseg7_95);
  lv_style_set_text_color(&btn_style10, lv_palette_main(LV_PALETTE_RED));
  lv_style_set_bg_color(&btn_style10, lv_color_black());
  lv_style_set_outline_opa(&btn_style10, LV_OPA_TRANSP);
  lv_style_set_shadow_opa(&btn_style10, LV_OPA_TRANSP);

  static lv_style_t btn_style11;
  lv_style_set_radius(&btn_style11, 0);
  lv_style_set_text_font(&btn_style11,  &dseg7_105);
  lv_style_set_text_color(&btn_style11, lv_palette_main(LV_PALETTE_RED));
  lv_style_set_bg_color(&btn_style11, lv_color_black());
  lv_style_set_outline_opa(&btn_style11, LV_OPA_TRANSP);
  lv_style_set_shadow_opa(&btn_style11, LV_OPA_TRANSP);

  static lv_style_t btn_style12;
  lv_style_set_radius(&btn_style12, 3);
  lv_style_set_bg_color(&btn_style12, lv_color_white());
  lv_style_set_text_font(&btn_style12, &lv_font_montserrat_24);
  lv_style_set_text_color(&btn_style12, lv_color_black());
  lv_style_set_size(&btn_style12, 200, 50);

  static lv_style_t btn_style13;
  lv_style_set_radius(&btn_style13, 3);
  lv_style_set_bg_color(&btn_style13, lv_color_white());
  lv_style_set_text_font(&btn_style13, &lv_font_montserrat_28);
  lv_style_set_text_color(&btn_style13, lv_color_black());
  lv_style_set_size(&btn_style13, 50, 50);

  static lv_style_t btn_style14;
  lv_style_init(&btn_style14);
  lv_style_set_radius(&btn_style14, 0);
  lv_style_set_bg_opa(&btn_style14, LV_OPA_TRANSP);
  lv_style_set_outline_opa(&btn_style14, LV_OPA_TRANSP);
  lv_style_set_shadow_opa(&btn_style14, LV_OPA_TRANSP);
  lv_style_set_pad_all(&btn_style14, 0);
  lv_style_set_border_width(&btn_style14, 0);

  // slider styles
  static const lv_style_prop_t props[] = {LV_STYLE_BG_COLOR, 0};
  static lv_style_transition_dsc_t transition_dsc;
  lv_style_transition_dsc_init(&transition_dsc, props, lv_anim_path_linear, 300, 0, NULL);

  static lv_style_t slider_style1_main;
  static lv_style_t slider_style1_indicator;
  static lv_style_t slider_style1_knob;
  static lv_style_t slider_style1_pressed_color;
  lv_style_init(&slider_style1_main);
  lv_style_set_bg_opa(&slider_style1_main, LV_OPA_COVER);
  lv_style_set_bg_color(&slider_style1_main, lv_color_hex3(0xbbb));
  lv_style_set_radius(&slider_style1_main, LV_RADIUS_CIRCLE);
  lv_style_set_pad_ver(&slider_style1_main, -2); /*Makes the indicator larger*/

  lv_style_init(&slider_style1_indicator);
  lv_style_set_bg_opa(&slider_style1_indicator, LV_OPA_COVER);
  lv_style_set_bg_color(&slider_style1_indicator, lv_palette_main(LV_PALETTE_LIGHT_BLUE));
  lv_style_set_radius(&slider_style1_indicator, LV_RADIUS_CIRCLE);
  lv_style_set_transition(&slider_style1_indicator, &transition_dsc);

  lv_style_init(&slider_style1_knob);
  lv_style_set_bg_opa(&slider_style1_knob, LV_OPA_COVER);
  lv_style_set_bg_color(&slider_style1_knob, lv_palette_main(LV_PALETTE_LIGHT_BLUE));
  lv_style_set_border_color(&slider_style1_knob, lv_palette_main(LV_PALETTE_LIGHT_BLUE));
  lv_style_set_border_width(&slider_style1_knob, 2);
  lv_style_set_radius(&slider_style1_knob, LV_RADIUS_CIRCLE);
  lv_style_set_pad_all(&slider_style1_knob, 6); /*Makes the knob larger*/
  lv_style_set_transition(&slider_style1_knob, &transition_dsc);
  lv_style_init(&slider_style1_pressed_color);
  lv_style_set_bg_color(&slider_style1_pressed_color, lv_palette_main(LV_PALETTE_LIGHT_BLUE));

  static lv_style_t spinbox_style1;
  lv_style_set_bg_color(&spinbox_style1, lv_color_white());
  lv_style_set_text_font(&spinbox_style1, &lv_font_montserrat_20);
  lv_style_set_text_color(&spinbox_style1, lv_color_black());

  // main screen

  btn_main_wifi = lv_btn_create(tabmain);
  lv_obj_add_event_cb(btn_main_wifi, event_wifiscr, LV_EVENT_ALL, NULL);
  lv_obj_set_size(btn_main_wifi, 32, 32);                     // exact raakvlak
  lv_obj_align(btn_main_wifi, LV_ALIGN_TOP_RIGHT, 0, 0);      // knop rechtsboven
  lv_obj_add_style(btn_main_wifi, &btn_style14, 0);

  lbl_main_wifistatus = lv_label_create(btn_main_wifi);
  lv_label_set_text(lbl_main_wifistatus, LV_SYMBOL_WIFI);
  lv_obj_set_style_text_color(lbl_main_wifistatus, lv_palette_main(LV_PALETTE_RED), 0);
  lv_obj_set_style_text_font(lbl_main_wifistatus, &lv_font_montserrat_24, 0);
  lv_obj_align(lbl_main_wifistatus, LV_ALIGN_TOP_RIGHT, 0, 0);

  btn_main_channel = lv_btn_create(tabmain);
  lv_obj_add_event_cb(btn_main_channel, event_channel_touch, LV_EVENT_ALL, NULL);
  lv_obj_set_width(btn_main_channel, 423);
  lv_obj_set_height(btn_main_channel, 122);
  lv_obj_add_style(btn_main_channel, &btn_style8, LV_PART_MAIN);

  lbl_main_channel = lv_label_create(btn_main_channel);
  lv_label_set_text(lbl_main_channel, "");
  lv_obj_set_align(lbl_main_channel,  LV_ALIGN_RIGHT_MID);
  lv_obj_set_style_margin_right(lbl_main_channel, 0, 0);
  lv_obj_set_style_pad_right(lbl_main_channel, 0, 0);
  lv_obj_align(btn_main_channel, LV_ALIGN_CENTER, 0, 0);

  btn_main_freq = lv_btn_create(tabmain);
  lv_obj_add_event_cb(btn_main_freq, event_toggle_channel_freq, LV_EVENT_CLICKED, NULL);
  lv_obj_set_width(btn_main_freq, 350);
  lv_obj_set_height(btn_main_freq, 67);
  lv_obj_add_style(btn_main_freq, &btn_style9, LV_PART_MAIN);

  lbl_main_freq = lv_label_create(btn_main_freq);
  lv_label_set_text(lbl_main_freq, "27.125");
  lv_obj_set_align(lbl_main_freq,  LV_ALIGN_RIGHT_MID);
  lv_obj_set_style_margin_right(lbl_main_freq, 0, 0);
  lv_obj_set_style_pad_right(lbl_main_freq, 0, 0);
  lv_obj_align(btn_main_freq, LV_ALIGN_TOP_LEFT, 101, 250);

  btn_main_freq2 = lv_btn_create(tabmain);
  lv_obj_add_event_cb(btn_main_freq2, event_frequency_touch, LV_EVENT_ALL, NULL);
  lv_obj_set_width(btn_main_freq2, 423);
  lv_obj_set_height(btn_main_freq2, 122);
  lv_obj_add_style(btn_main_freq2, &btn_style10, LV_PART_MAIN);

  lbl_main_freq2 = lv_label_create(btn_main_freq2);
  lv_label_set_text(lbl_main_freq2, "27.125");
  lv_obj_set_align(lbl_main_freq2,  LV_ALIGN_RIGHT_MID);
  lv_obj_set_style_margin_right(lbl_main_freq2, 0, 0);
  lv_obj_set_style_pad_right(lbl_main_freq2, 0, 0);
  lv_obj_align(btn_main_freq2, LV_ALIGN_CENTER, 0, 0);

  btn_main_channel2 = lv_btn_create(tabmain);
  lv_obj_add_event_cb(btn_main_channel2, event_toggle_channel_freq, LV_EVENT_CLICKED, NULL);
  lv_obj_set_width(btn_main_channel2, 350);
  lv_obj_set_height(btn_main_channel2, 67);
  lv_obj_add_style(btn_main_channel2, &btn_style9, LV_PART_MAIN);

  lbl_main_channel2 = lv_label_create(btn_main_channel2);
  lv_label_set_text(lbl_main_channel2, "");
  lv_obj_set_align(lbl_main_channel2,  LV_ALIGN_RIGHT_MID);
  lv_obj_set_style_margin_right(lbl_main_channel2, 0, 0);
  lv_obj_set_style_pad_right(lbl_main_channel2, 0, 0);
  lv_obj_align(btn_main_channel2, LV_ALIGN_TOP_LEFT, 101, 250);

  lbl_main_channel_alfa2 = lv_label_create(tabmain);
  lv_label_set_text(lbl_main_channel_alfa2, "A");
  lv_obj_add_style(lbl_main_channel_alfa2, &label_style6, LV_PART_MAIN);
  lv_obj_align(lbl_main_channel_alfa2,  LV_ALIGN_TOP_LEFT, 443, 283);

  lv_obj_add_flag(btn_main_channel2, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(lbl_main_channel2, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(lbl_main_channel_alfa2, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(btn_main_freq2, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(lbl_main_freq2, LV_OBJ_FLAG_HIDDEN);

  lbl_main_noconnection = lv_label_create(tabmain);
  lv_label_set_text(lbl_main_noconnection, LV_SYMBOL_LOOP);
  lv_obj_align(lbl_main_noconnection, LV_ALIGN_TOP_LEFT, 5, 215);
  lv_obj_set_style_text_color(lbl_main_noconnection, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_main_noconnection, &lv_font_montserrat_24, LV_PART_MAIN);

  lbl_main_tx = lv_label_create(tabmain);
  lv_label_set_text(lbl_main_tx, "TX");
  lv_obj_align(lbl_main_tx, LV_ALIGN_TOP_LEFT, 5, 255);
  lv_obj_set_style_text_color(lbl_main_tx, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_main_tx, &lv_font_montserrat_24, LV_PART_MAIN);

  led_main_nolock = lv_led_create(tabmain);
  lv_obj_align(led_main_nolock, LV_ALIGN_TOP_LEFT, 15, 295);
  lv_obj_set_size(led_main_nolock, 10, 10);
  lv_led_set_brightness(led_main_nolock, 150);
  lv_led_set_color(led_main_nolock, lv_palette_main(LV_PALETTE_RED));
  lv_led_on(led_main_nolock);

  // direct entry channel

  static lv_style_t display_style;
  lv_style_init(&display_style);
  lv_style_set_text_font(&display_style, &dseg7_50);
  lv_style_set_text_color(&display_style, lv_color_white());

  btnmtrx_directchannel_keypad = lv_btnmatrix_create(tabdirectentrychannel);
  lv_btnmatrix_set_map(btnmtrx_directchannel_keypad, master_direct_channel_btnm_map);
  lv_obj_set_size(btnmtrx_directchannel_keypad, 390, 260);
  lv_obj_align(btnmtrx_directchannel_keypad, LV_ALIGN_TOP_LEFT, 0, 60);
  lv_obj_add_style(btnmtrx_directchannel_keypad, &btn_style4, LV_PART_ITEMS);
  lv_obj_add_style(btnmtrx_directchannel_keypad, &btn_style5, LV_PART_MAIN);
  lv_obj_add_event_cb(btnmtrx_directchannel_keypad, direct_channel_btnm_event_handler, LV_EVENT_VALUE_CHANGED, NULL);

  // channel display
  lbl_directchannel_channel = lv_label_create(tabdirectentrychannel);
  lv_obj_set_size(lbl_directchannel_channel, 390, 50);
  lv_obj_align(lbl_directchannel_channel, LV_ALIGN_TOP_LEFT, 0, 10);
  lv_label_set_text(lbl_directchannel_channel, ".");
  lv_obj_add_style(lbl_directchannel_channel, &display_style, 0);
  lv_obj_set_style_text_align(lbl_directchannel_channel, LV_TEXT_ALIGN_CENTER, 0);

  btn_directchannel_goto = lv_btn_create(tabdirectentrychannel);
  lv_obj_add_style(btn_directchannel_goto, &btn_style2, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_directchannel_goto, event_gotochannel, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_directchannel_goto);
  lv_label_set_text(label, LV_SYMBOL_OK);
  lv_obj_center(label);
  lv_obj_align(btn_directchannel_goto, LV_ALIGN_TOP_RIGHT, 0, 134);

  btn_directchannel_cancel = lv_btn_create(tabdirectentrychannel);
  lv_obj_add_style(btn_directchannel_cancel, &btn_style3, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_directchannel_cancel, event_mainscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_directchannel_cancel);
  lv_label_set_text(label, LV_SYMBOL_CLOSE);
  lv_obj_center(label);
  lv_obj_align(btn_directchannel_cancel, LV_ALIGN_TOP_RIGHT, 0, 229);

  // direct frequency screen

  btnmtrx_directfreq_keypad = lv_btnmatrix_create(tabdirectentryfreq);
  lv_btnmatrix_set_map(btnmtrx_directfreq_keypad, master_direct_freq_btnm_map);
  lv_obj_set_size(btnmtrx_directfreq_keypad, 390, 260);
  lv_obj_align(btnmtrx_directfreq_keypad, LV_ALIGN_TOP_LEFT, 0, 60);
  lv_obj_add_style(btnmtrx_directfreq_keypad, &btn_style4, LV_PART_ITEMS);
  lv_obj_add_style(btnmtrx_directfreq_keypad, &btn_style5, LV_PART_MAIN);
  lv_obj_add_event_cb(btnmtrx_directfreq_keypad, direct_freq_btnm_event_handler, LV_EVENT_VALUE_CHANGED, NULL);

  // frequency display
  lbl_directfreq_frequency = lv_label_create(tabdirectentryfreq);
  lv_obj_set_size(lbl_directfreq_frequency, 390, 50);
  lv_obj_align(lbl_directfreq_frequency, LV_ALIGN_TOP_LEFT, 0, 10);
  lv_label_set_text(lbl_directfreq_frequency, "2-.--5");
  lv_obj_add_style(lbl_directfreq_frequency, &display_style, 0);
  lv_obj_set_style_text_align(lbl_directfreq_frequency, LV_TEXT_ALIGN_CENTER, 0);

  btn_directfreq_goto = lv_btn_create(tabdirectentryfreq);
  lv_obj_add_style(btn_directfreq_goto, &btn_style2, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_directfreq_goto, event_gotofreq, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_directfreq_goto);
  lv_label_set_text(label, LV_SYMBOL_OK);
  lv_obj_center(label);
  lv_obj_align(btn_directfreq_goto, LV_ALIGN_TOP_RIGHT, 0, 134);

  btn_directfreq_cancel = lv_btn_create(tabdirectentryfreq);
  lv_obj_add_style(btn_directfreq_cancel, &btn_style3, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_directfreq_cancel, event_mainscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_directfreq_cancel);
  lv_label_set_text(label, LV_SYMBOL_CLOSE);
  lv_obj_center(label);
  lv_obj_align(btn_directfreq_cancel, LV_ALIGN_TOP_RIGHT, 0, 229);

  // scan screen

  btn_scan_channel = lv_btn_create(tabscan);
  lv_obj_add_event_cb(btn_scan_channel, event_scan_toggle, LV_EVENT_CLICKED, NULL);
  lv_obj_set_width(btn_scan_channel, 370);
  lv_obj_set_height(btn_scan_channel, 122);
  lv_obj_add_style(btn_scan_channel, &btn_style11, LV_PART_MAIN);

  lbl_scan_channel = lv_label_create(btn_scan_channel);
  lv_label_set_text(lbl_scan_channel, "----");
  lv_obj_set_align(lbl_scan_channel,  LV_ALIGN_RIGHT_MID);
  lv_obj_set_style_margin_right(lbl_scan_channel, 0, 0);
  lv_obj_set_style_pad_right(lbl_scan_channel, 0, 0);
  lv_obj_align(btn_scan_channel, LV_ALIGN_TOP_LEFT, 0, 5);

  lbl_scan_squelch = lv_label_create(tabscan);
  lv_label_set_text(lbl_scan_squelch, "S");
  lv_obj_align(lbl_scan_squelch, LV_ALIGN_TOP_LEFT, 370, 20);
  lv_obj_set_style_text_color(lbl_scan_squelch, lv_palette_main(LV_PALETTE_GREEN), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_scan_squelch, &lv_font_montserrat_24, LV_PART_MAIN);

  spinbox_scan_from = lv_spinbox_create(tabscan);
  lv_spinbox_set_range(spinbox_scan_from, master_lower_channel, master_upper_channel - 20);
  lv_spinbox_set_digit_format(spinbox_scan_from, 3, 0);
  lv_spinbox_set_step(spinbox_scan_from, 1);
  lv_obj_set_width(spinbox_scan_from, 75);
  lv_obj_set_height(spinbox_scan_from, 46);
  lv_obj_align(spinbox_scan_from, LV_ALIGN_TOP_LEFT, 51, 243);
  lv_obj_add_style(spinbox_scan_from, &spinbox_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(spinbox_scan_from, event_scan_from_change, LV_EVENT_ALL,  NULL);

  obj_height = lv_obj_get_height(spinbox_scan_from);

  btn_scan_from_inc = lv_button_create(tabscan);
  lv_obj_set_size(btn_scan_from_inc, obj_height, obj_height);
  lv_obj_align_to(btn_scan_from_inc, spinbox_scan_from, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_scan_from_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_scan_from_inc, increment_scan_from_event_cb, LV_EVENT_ALL,  NULL);

  btn_scan_from_dec = lv_button_create(tabscan);
  lv_obj_set_size(btn_scan_from_dec, obj_height, obj_height);
  lv_obj_align_to(btn_scan_from_dec, spinbox_scan_from, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_scan_from_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_scan_from_dec, decrement_scan_from_event_cb, LV_EVENT_ALL, NULL);

  lbl_scan_from = lv_label_create(tabscan);
  lv_label_set_text(lbl_scan_from, LV_SYMBOL_PLAY);
  lv_obj_add_style(lbl_scan_from, &label_style2, LV_PART_MAIN);
  lv_obj_align_to(lbl_scan_from, spinbox_scan_from, LV_ALIGN_OUT_TOP_MID, 0, -5);

  spinbox_scan_to = lv_spinbox_create(tabscan);
  lv_spinbox_set_range(spinbox_scan_to, master_lower_channel + 20, master_upper_channel);
  lv_spinbox_set_digit_format(spinbox_scan_to, 3, 0);
  lv_spinbox_set_step(spinbox_scan_to, 1);
  lv_obj_set_width(spinbox_scan_to, 75);
  lv_obj_set_height(spinbox_scan_to, 46);
  lv_obj_align(spinbox_scan_to, LV_ALIGN_TOP_LEFT, 233, 243);
  lv_obj_add_style(spinbox_scan_to, &spinbox_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(spinbox_scan_to, event_scan_to_change, LV_EVENT_ALL,  NULL);

  obj_height = lv_obj_get_height(spinbox_scan_to);

  btn_scan_to_inc = lv_button_create(tabscan);
  lv_obj_set_size(btn_scan_to_inc, obj_height, obj_height);
  lv_obj_align_to(btn_scan_to_inc, spinbox_scan_to, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_scan_to_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_scan_to_inc, increment_scan_to_event_cb, LV_EVENT_ALL,  NULL);

  btn_scan_to_dec = lv_button_create(tabscan);
  lv_obj_set_size(btn_scan_to_dec, obj_height, obj_height);
  lv_obj_align_to(btn_scan_to_dec, spinbox_scan_to, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_scan_to_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_scan_to_dec, decrement_scan_to_event_cb, LV_EVENT_ALL, NULL);

  lbl_scan_to = lv_label_create(tabscan);

  lv_label_set_text(lbl_scan_to, LV_SYMBOL_STOP);

  lv_obj_add_style(lbl_scan_to, &label_style2, LV_PART_MAIN);
  lv_obj_align_to(lbl_scan_to, spinbox_scan_to, LV_ALIGN_OUT_TOP_MID, 0, -5);

  btn_scan_start = lv_btn_create(tabscan);
  lv_obj_add_style(btn_scan_start, &btn_style6, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_scan_start, event_scanstart, LV_EVENT_CLICKED, NULL);
  lbl_scan_start = lv_label_create(btn_scan_start);
  lv_label_set_text(lbl_scan_start, LV_SYMBOL_PLAY);
  lv_obj_center(lbl_scan_start);
  lv_obj_align(btn_scan_start, LV_ALIGN_TOP_RIGHT, 0, 59);

  btn_scan_skip = lv_btn_create(tabscan);
  lv_obj_add_style(btn_scan_skip, &btn_style6, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_scan_skip, event_scan_skip, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_scan_skip);
  lv_label_set_text(label, LV_SYMBOL_MINUS);
  lv_obj_center(label);
  lv_obj_align(btn_scan_skip, LV_ALIGN_TOP_RIGHT, 0, 149);

  btn_scan_back = lv_btn_create(tabscan);
  lv_obj_add_style(btn_scan_back, &btn_style6, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_scan_back, event_scan_back, LV_EVENT_CLICKED, NULL);
  lbl_scan_back = lv_label_create(btn_scan_back);
  lv_label_set_text(lbl_scan_back, LV_SYMBOL_HOME);
  lv_obj_center(lbl_scan_back);
  lv_obj_align(btn_scan_back, LV_ALIGN_TOP_RIGHT, 0, 239);

  static lv_style_t style_bg;
  static lv_style_t style_indic;

  lv_style_init(&style_bg);
  lv_style_set_border_color(&style_bg, lv_color_white());

  lv_style_set_border_width(&style_bg, 2);
  lv_style_set_pad_all(&style_bg, 6);
  lv_style_set_radius(&style_bg, 6);
  lv_style_set_anim_duration(&style_bg, 1000);

  lv_style_init(&style_indic);
  lv_style_set_bg_opa(&style_indic, LV_OPA_COVER);
  lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_BLUE));
  lv_style_set_radius(&style_indic, 3);

  bar_scan_progress = lv_bar_create(tabscan);
  lv_obj_remove_style_all(bar_scan_progress);
  lv_obj_add_style(bar_scan_progress, &style_bg, 0);
  lv_obj_add_style(bar_scan_progress, &style_indic, LV_PART_INDICATOR);
  lv_obj_set_size(bar_scan_progress, 370, 20);
  lv_bar_set_value(bar_scan_progress, 0, LV_ANIM_ON);
  lv_obj_align(bar_scan_progress, LV_ALIGN_TOP_LEFT, 0, 149);

  // menu screen

  btn_menu_scan = lv_btn_create(tabmenu);
  lv_obj_add_style(btn_menu_scan, &btn_style7, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_menu_scan, event_scanscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_menu_scan);
  lv_label_set_text(label, "Scan");
  lv_obj_center(label);
  lv_obj_align(btn_menu_scan, LV_ALIGN_TOP_LEFT, 0, 0);

  btn_menu_power = lv_btn_create(tabmenu);
  lv_obj_add_style(btn_menu_power, &btn_style7, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_menu_power, event_powerscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_menu_power);
  lv_label_set_text(label, "Power");
  lv_obj_center(label);
  lv_obj_align(btn_menu_power, LV_ALIGN_TOP_LEFT, 0, 110);

  btn_menu_wifi = lv_btn_create(tabmenu);
  lv_obj_add_style(btn_menu_wifi, &btn_style7, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_menu_wifi, event_wifiscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_menu_wifi);
  lv_label_set_text(label, "WiFi");
  lv_obj_center(label);
  lv_obj_align(btn_menu_wifi, LV_ALIGN_TOP_RIGHT, 0, 0);

  btn_menu_settings = lv_btn_create(tabmenu);
  lv_obj_add_style(btn_menu_settings, &btn_style7, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_menu_settings, event_settingsscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_menu_settings);
  lv_label_set_text(label, "Settings");
  lv_obj_center(label);
  lv_obj_align(btn_menu_settings, LV_ALIGN_TOP_RIGHT, 0, 110);

  btn_menu_back = lv_btn_create(tabmenu);
  lv_obj_add_style(btn_menu_back, &btn_style7, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_menu_back, event_mainscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_menu_back);
  lv_label_set_text(label, LV_SYMBOL_HOME);
  lv_obj_center(label);
  lv_obj_align(btn_menu_back, LV_ALIGN_TOP_RIGHT, 0, 220);

  // power screen

  slider_power = lv_slider_create(tabpower);
  lv_obj_remove_style_all(slider_power);
  lv_obj_add_style(slider_power, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_power, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_power, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_power, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_power, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_power, 15, 200);
  lv_slider_set_range(slider_power, 30, 100);
  lv_obj_align(slider_power, LV_ALIGN_TOP_LEFT, 165, 60);
  lv_obj_add_event_cb(slider_power, event_power_change, LV_EVENT_ALL, NULL);

  lv_obj_t * slider_label = lv_label_create(tabpower);
  lv_label_set_text(slider_label, "Power");
  lv_obj_add_style(slider_label, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(slider_label, slider_power, LV_ALIGN_OUT_TOP_MID, 0, -25);

  btn_power_save = lv_btn_create(tabpower);
  lv_obj_add_style(btn_power_save, &btn_style6, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_power_save, event_power_save, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_power_save);
  lv_label_set_text(label, LV_SYMBOL_SAVE);
  lv_obj_center(label);
  lv_obj_align(btn_power_save, LV_ALIGN_TOP_RIGHT, 0, 149);

  btn_power_back = lv_btn_create(tabpower);
  lv_obj_add_style(btn_power_back, &btn_style6, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_power_back, event_mainscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_power_back);
  lv_label_set_text(label, LV_SYMBOL_HOME);
  lv_obj_center(label);
  lv_obj_align(btn_power_back, LV_ALIGN_TOP_RIGHT, 0, 239);


  // settings screen

  btn_settings_range = lv_btn_create(tabsettings);
  lv_obj_add_style(btn_settings_range, &btn_style7, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_settings_range, event_settingsrangescr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_settings_range);
  lv_label_set_text(label, "Range");
  lv_obj_center(label);
  lv_obj_align(btn_settings_range, LV_ALIGN_TOP_LEFT, 0, 0);

  btn_settings_s_pwr_cal = lv_btn_create(tabsettings);
  lv_obj_add_style(btn_settings_s_pwr_cal, &btn_style7, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_settings_s_pwr_cal, event_settings_s_pwr_scr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_settings_s_pwr_cal);
  lv_label_set_text(label, "Calibrate");
  lv_obj_center(label);
  lv_obj_align(btn_settings_s_pwr_cal, LV_ALIGN_TOP_LEFT, 0, 110);

  btn_settings_back = lv_btn_create(tabsettings);
  lv_obj_add_style(btn_settings_back, &btn_style7, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_settings_back, event_settings_back, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_settings_back);
  lv_label_set_text(label, LV_SYMBOL_HOME);
  lv_obj_center(label);
  lv_obj_align(btn_settings_back, LV_ALIGN_TOP_RIGHT, 0, 239);

  // settings range screen

  label = lv_label_create(tabsettingsrange);
  lv_label_set_text(label, "Channel range");
  lv_obj_add_style(label, &label_style5, LV_PART_MAIN);
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 88, 20);

  spinbox_settings_from = lv_spinbox_create(tabsettingsrange);
  lv_spinbox_set_range(spinbox_settings_from, -120, 1);
  lv_spinbox_set_digit_format(spinbox_settings_from, 3, 0);
  lv_spinbox_set_step(spinbox_settings_from, 1);
  lv_obj_set_width(spinbox_settings_from, 75);
  lv_obj_set_height(spinbox_settings_from, 46);
  lv_obj_align(spinbox_settings_from, LV_ALIGN_TOP_LEFT, 51, 95);
  lv_obj_add_style(spinbox_settings_from, &spinbox_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(spinbox_settings_from, event_settings_from_change, LV_EVENT_ALL,  NULL);

  obj_height = lv_obj_get_height(spinbox_settings_from);

  btn_settings_from_inc = lv_button_create(tabsettingsrange);
  lv_obj_set_size(btn_settings_from_inc, obj_height, obj_height);
  lv_obj_align_to(btn_settings_from_inc, spinbox_settings_from, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_settings_from_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_settings_from_inc, increment_settings_from_event_cb, LV_EVENT_ALL,  NULL);

  btn_settings_from_dec = lv_button_create(tabsettingsrange);
  lv_obj_set_size(btn_settings_from_dec, obj_height, obj_height);
  lv_obj_align_to(btn_settings_from_dec, spinbox_settings_from, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_settings_from_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_settings_from_dec, decrement_settings_from_event_cb, LV_EVENT_ALL, NULL);

  lbl_settings_from = lv_label_create(tabsettingsrange);

  lv_label_set_text(lbl_settings_from, LV_SYMBOL_PLAY);

  lv_obj_add_style(lbl_settings_from, &label_style2, LV_PART_MAIN);
  lv_obj_align_to(lbl_settings_from, spinbox_settings_from, LV_ALIGN_OUT_TOP_MID, 0, -5);

  spinbox_settings_to = lv_spinbox_create(tabsettingsrange);
  lv_spinbox_set_range(spinbox_settings_to, 22, 120);
  lv_spinbox_set_digit_format(spinbox_settings_to, 3, 0);
  lv_spinbox_set_step(spinbox_settings_to, 1);
  lv_obj_set_width(spinbox_settings_to, 75);
  lv_obj_set_height(spinbox_settings_to, 46);
  lv_obj_align(spinbox_settings_to, LV_ALIGN_TOP_LEFT, 233, 95);
  lv_obj_add_style(spinbox_settings_to, &spinbox_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(spinbox_settings_to, event_settings_to_change, LV_EVENT_ALL,  NULL);

  obj_height = lv_obj_get_height(spinbox_settings_to);

  btn_settings_to_inc = lv_button_create(tabsettingsrange);
  lv_obj_set_size(btn_settings_to_inc, obj_height, obj_height);
  lv_obj_align_to(btn_settings_to_inc, spinbox_settings_to, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_settings_to_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_settings_to_inc, increment_settings_to_event_cb, LV_EVENT_ALL,  NULL);

  btn_settings_to_dec = lv_button_create(tabsettingsrange);
  lv_obj_set_size(btn_settings_to_dec, obj_height, obj_height);
  lv_obj_align_to(btn_settings_to_dec, spinbox_settings_to, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_settings_to_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_settings_to_dec, decrement_settings_to_event_cb, LV_EVENT_ALL, NULL);

  lbl_settings_to = lv_label_create(tabsettingsrange);

  lv_label_set_text(lbl_settings_to, LV_SYMBOL_STOP);

  lv_obj_add_style(lbl_settings_to, &label_style2, LV_PART_MAIN);
  lv_obj_align_to(lbl_settings_to, spinbox_settings_to, LV_ALIGN_OUT_TOP_MID, 0, -5);

  btn_settings_range_back = lv_btn_create(tabsettingsrange);
  lv_obj_add_style(btn_settings_range_back, &btn_style6, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_settings_range_back, event_settings_range_back, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_settings_range_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_settings_range_back, LV_ALIGN_TOP_RIGHT, 0, 239);

  // settings calibrate vu and power meter

  slider_settings_s_cal = lv_slider_create(tabsettings_s_pwr);
  lv_obj_remove_style_all(slider_settings_s_cal);
  lv_obj_add_style(slider_settings_s_cal, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_settings_s_cal, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_settings_s_cal, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_settings_s_cal, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_settings_s_cal, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_settings_s_cal, 15, 150);
  lv_slider_set_range(slider_settings_s_cal, 0, 100);
  lv_obj_align(slider_settings_s_cal, LV_ALIGN_TOP_LEFT, 75, 125);
  lv_obj_add_event_cb(slider_settings_s_cal, event_cal_s_change, LV_EVENT_ALL, NULL);

  lv_obj_t * slider_label2 = lv_label_create(tabsettings_s_pwr);
  lv_label_set_text(slider_label2, "S");
  lv_obj_add_style(slider_label2, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(slider_label2, slider_settings_s_cal, LV_ALIGN_OUT_TOP_MID, 0, -25);

  slider_settings_power_cal = lv_slider_create(tabsettings_s_pwr);
  lv_obj_remove_style_all(slider_settings_power_cal);
  lv_obj_add_style(slider_settings_power_cal, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_settings_power_cal, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_settings_power_cal, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_settings_power_cal, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_settings_power_cal, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_settings_power_cal, 15, 150);
  lv_slider_set_range(slider_settings_power_cal, 0, 100);
  lv_obj_align(slider_settings_power_cal, LV_ALIGN_TOP_LEFT, 250, 125);
  lv_obj_add_event_cb(slider_settings_power_cal, event_cal_pwr_change, LV_EVENT_ALL, NULL);

  lv_obj_t * slider_label3 = lv_label_create(tabsettings_s_pwr);
  lv_label_set_text(slider_label3, "PWR");
  lv_obj_add_style(slider_label3, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(slider_label3, slider_settings_power_cal, LV_ALIGN_OUT_TOP_MID, 0, -25);

  btn_settings_s_pwr_back = lv_btn_create(tabsettings_s_pwr);
  lv_obj_add_style(btn_settings_s_pwr_back, &btn_style6, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_settings_s_pwr_back, event_settings_calibrate_back, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_settings_s_pwr_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_settings_s_pwr_back, LV_ALIGN_TOP_RIGHT, 0, 239);

  // presets screen

  btn_presets_default = lv_btn_create(tabpresets);
  lv_obj_add_style(btn_presets_default, &btn_style12, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presets_default, event_presets_goto_default, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_presets_default);
  lv_label_set_text(label, "STD");
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_obj_align(label, LV_ALIGN_LEFT_MID, 5, 0);

  lbl_presets_default = lv_label_create(btn_presets_default);
  lv_label_set_text(lbl_presets_default, "18");
  lv_obj_add_style(lbl_presets_default, &label_style7, LV_PART_MAIN);
  lv_obj_align(lbl_presets_default, LV_ALIGN_RIGHT_MID, -5, 0);
  lv_obj_align(btn_presets_default, LV_ALIGN_TOP_LEFT, 20, 5);

  btn_presets_save_default = lv_btn_create(tabpresets);
  lv_obj_add_style(btn_presets_save_default, &btn_style13, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presets_save_default, event_presets_save_default, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_presets_save_default);
  lv_label_set_text(label, LV_SYMBOL_SAVE);
  lv_obj_center(label);
  lv_obj_align(btn_presets_save_default, LV_ALIGN_TOP_LEFT, 250, 5);

  btn_presets_1 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn_presets_1, &btn_style12, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presets_1, event_presets_goto_preset1, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_presets_1);
  lv_label_set_text(label, "P1");
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_obj_align(label, LV_ALIGN_LEFT_MID, 5, 0);

  lbl_presets_1 = lv_label_create(btn_presets_1);
  lv_label_set_text(lbl_presets_1, "18");
  lv_obj_add_style(lbl_presets_1, &label_style7, LV_PART_MAIN);
  lv_obj_align(lbl_presets_1, LV_ALIGN_RIGHT_MID, -5, 0);
  lv_obj_align(btn_presets_1, LV_ALIGN_TOP_LEFT, 20, 70);

  btn_presets_save_1 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn_presets_save_1, &btn_style13, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presets_save_1, event_presets_save_1, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_presets_save_1);
  lv_label_set_text(label, LV_SYMBOL_SAVE);
  lv_obj_center(label);
  lv_obj_align(btn_presets_save_1, LV_ALIGN_TOP_LEFT, 250, 70);

  btn_presets_2 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn_presets_2, &btn_style12, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presets_2, event_presets_goto_preset2, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_presets_2);
  lv_label_set_text(label, "P2");
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_obj_align(label, LV_ALIGN_LEFT_MID, 5, 0);

  lbl_presets_2 = lv_label_create(btn_presets_2);
  lv_label_set_text(lbl_presets_2, "18");
  lv_obj_add_style(lbl_presets_2, &label_style7, LV_PART_MAIN);
  lv_obj_align(lbl_presets_2, LV_ALIGN_RIGHT_MID, -5, 0);
  lv_obj_align(btn_presets_2, LV_ALIGN_TOP_LEFT, 20, 135);

  btn_presets_save_2 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn_presets_save_2, &btn_style13, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presets_save_2, event_presets_save_2, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_presets_save_2);
  lv_label_set_text(label, LV_SYMBOL_SAVE);
  lv_obj_center(label);
  lv_obj_align(btn_presets_save_2, LV_ALIGN_TOP_LEFT, 250, 135);

  btn_presets_3 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn_presets_3, &btn_style12, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presets_3, event_presets_goto_preset3, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_presets_3);
  lv_label_set_text(label, "P3");
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_obj_align(label, LV_ALIGN_LEFT_MID, 5, 0);

  lbl_presets_3 = lv_label_create(btn_presets_3);
  lv_label_set_text(lbl_presets_3, "18");
  lv_obj_add_style(lbl_presets_3, &label_style7, LV_PART_MAIN);
  lv_obj_align(lbl_presets_3, LV_ALIGN_RIGHT_MID, -5, 0);
  lv_obj_align(btn_presets_3, LV_ALIGN_TOP_LEFT, 20, 200);

  btn_presets_save_3 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn_presets_save_3, &btn_style13, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presets_save_3, event_presets_save_3, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_presets_save_3);
  lv_label_set_text(label, LV_SYMBOL_SAVE);
  lv_obj_center(label);
  lv_obj_align(btn_presets_save_3, LV_ALIGN_TOP_LEFT, 250, 200);

  btn_presets_4 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn_presets_4, &btn_style12, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presets_4, event_presets_goto_preset4, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_presets_4);
  lv_label_set_text(label, "P4");
  lv_obj_add_style(label, &label_style3, LV_PART_MAIN);
  lv_obj_align(label, LV_ALIGN_LEFT_MID, 5, 0);

  lbl_presets_4 = lv_label_create(btn_presets_4);
  lv_label_set_text(lbl_presets_4, "18");
  lv_obj_add_style(lbl_presets_4, &label_style7, LV_PART_MAIN);
  lv_obj_align(lbl_presets_4, LV_ALIGN_RIGHT_MID, -5, 0);
  lv_obj_align(btn_presets_4, LV_ALIGN_TOP_LEFT, 20, 265);

  btn_presets_save_4 = lv_btn_create(tabpresets);
  lv_obj_add_style(btn_presets_save_4, &btn_style13, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presets_save_4, event_presets_save_4, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_presets_save_4);
  lv_label_set_text(label, LV_SYMBOL_SAVE);
  lv_obj_center(label);
  lv_obj_align(btn_presets_save_4, LV_ALIGN_TOP_LEFT, 250, 265);

  btn_presets_back = lv_btn_create(tabpresets);
  lv_obj_add_style(btn_presets_back, &btn_style6, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presets_back, event_presets_back, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_presets_back);
  lv_label_set_text(label, LV_SYMBOL_HOME);
  lv_obj_center(label);
  lv_obj_align(btn_presets_back, LV_ALIGN_TOP_RIGHT, 0, 239);

  // wifi screen

  lbl_wifi_wifistatus = lv_label_create(tabwifi);
  lv_label_set_text(lbl_wifi_wifistatus, LV_SYMBOL_WIFI);
  lv_obj_align(lbl_wifi_wifistatus, LV_ALIGN_TOP_RIGHT, 0, 0);
  lv_obj_set_style_text_color(lbl_wifi_wifistatus, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_wifi_wifistatus, &lv_font_montserrat_24, LV_PART_MAIN);

  label = lv_label_create(tabwifi);
  lv_label_set_text(label, "SSID");
  lv_obj_add_style(label, &label_style5, LV_PART_MAIN);
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 10);

  lbl_wifi_ssid = lv_label_create(tabwifi);
  lv_obj_add_style(lbl_wifi_ssid, &label_style5, LV_PART_MAIN);
  lv_obj_align(lbl_wifi_ssid, LV_ALIGN_TOP_LEFT, 120, 10);
  lv_label_set_text(lbl_wifi_ssid, "---");

  label = lv_label_create(tabwifi);
  lv_label_set_text(label, "IP");
  lv_obj_add_style(label, &label_style5, LV_PART_MAIN);
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 70);

  lbl_wifi_ip = lv_label_create(tabwifi);
  lv_obj_add_style(lbl_wifi_ip, &label_style5, LV_PART_MAIN);
  lv_obj_align(lbl_wifi_ip, LV_ALIGN_TOP_LEFT, 120, 70);
  lv_label_set_text(lbl_wifi_ip, "---");

  label = lv_label_create(tabwifi);
  lv_label_set_text(label, "RSSI");
  lv_obj_add_style(label, &label_style5, LV_PART_MAIN);
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 130);

  lbl_wifi_rssi = lv_label_create(tabwifi);
  lv_obj_add_style(lbl_wifi_rssi, &label_style5, LV_PART_MAIN);
  lv_obj_align(lbl_wifi_rssi, LV_ALIGN_TOP_LEFT, 120, 130);
  lv_label_set_text(lbl_wifi_rssi, "---");

  label = lv_label_create(tabwifi);
  lv_label_set_text(label, "CH");
  lv_obj_add_style(label, &label_style5, LV_PART_MAIN);
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 200);

  lbl_wifi_channel = lv_label_create(tabwifi);
  lv_obj_add_style(lbl_wifi_channel, &label_style5, LV_PART_MAIN);
  lv_obj_align(lbl_wifi_channel, LV_ALIGN_TOP_LEFT, 120, 200);
  lv_label_set_text(lbl_wifi_channel, "---");

  btn_wifi_edit = lv_btn_create(tabwifi);
  lv_obj_add_style(btn_wifi_edit, &btn_style6, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_wifi_edit, event_wifisetupscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_wifi_edit);
  lv_label_set_text(label, LV_SYMBOL_EDIT);
  lv_obj_center(label);
  lv_obj_align(btn_wifi_edit, LV_ALIGN_TOP_RIGHT, 0, 149);

  btn_wifi_back = lv_btn_create(tabwifi);
  lv_obj_add_style(btn_wifi_back, &btn_style6, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_wifi_back, event_settings_back, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_wifi_back);
  lv_label_set_text(label, LV_SYMBOL_HOME);
  lv_obj_center(label);
  lv_obj_align(btn_wifi_back, LV_ALIGN_TOP_RIGHT, 0, 239);

  // wifi setup screen

  label = lv_label_create(tabwifisetup);
  lv_label_set_text(label, "SSID");
  lv_obj_add_style(label, &label_style5, LV_PART_MAIN);
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 12);

  label = lv_label_create(tabwifisetup);
  lv_label_set_text(label, "PW");
  lv_obj_add_style(label, &label_style5, LV_PART_MAIN);
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 82);

  txt_wifi_ssid = lv_textarea_create(tabwifisetup);
  lv_obj_set_width(txt_wifi_ssid, 250);
  lv_obj_set_height(txt_wifi_ssid, 46);
  lv_obj_align(txt_wifi_ssid, LV_ALIGN_TOP_LEFT, 120, 0);
  lv_textarea_set_placeholder_text(txt_wifi_ssid, "WIFI NAME");
  lv_textarea_set_max_length(txt_wifi_ssid, 100);
  lv_textarea_set_one_line(txt_wifi_ssid, true);

  lv_obj_add_style(txt_wifi_ssid, &label_style3, LV_PART_MAIN);

  txt_wifi_password = lv_textarea_create(tabwifisetup);
  lv_obj_set_width(txt_wifi_password, 250);
  lv_obj_set_height(txt_wifi_password, 46);
  lv_obj_align(txt_wifi_password, LV_ALIGN_TOP_LEFT, 120, 70);
  lv_textarea_set_placeholder_text(txt_wifi_password, "PASSWORD");
  lv_textarea_set_max_length(txt_wifi_password, 100);
  lv_textarea_set_one_line(txt_wifi_password, true);
  lv_textarea_set_password_mode(txt_wifi_password, true);

  lv_obj_add_style(txt_wifi_password, &label_style3, LV_PART_MAIN);

  txt_wifi_keyboard = lv_keyboard_create(tabwifisetup);
  lv_obj_set_size(txt_wifi_keyboard,  LV_HOR_RES, LV_VER_RES / 2);
  lv_obj_align(txt_wifi_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);

  lv_obj_add_event_cb(txt_wifi_ssid, ta_event_cb, LV_EVENT_ALL, NULL);
  lv_obj_add_event_cb(txt_wifi_password, ta_event_cb, LV_EVENT_ALL, NULL);
  lv_obj_add_event_cb(txt_wifi_keyboard, kb_event_cb, LV_EVENT_ALL, NULL);

}


void kb_event_cb(lv_event_t * e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * kb = (lv_obj_t *) lv_event_get_target(e);
  if (code == LV_EVENT_READY) {
    master_settings_wifi_ssid = lv_textarea_get_text(txt_wifi_ssid);
    master_settings_wifi_password = lv_textarea_get_text(txt_wifi_password);
    if (master_settings_wifi_ssid != master_settings_wifi_ssid_old or master_settings_wifi_password != master_settings_wifi_password_old) {
      // disconnect if login changed
      if (wifi_state == WIFI_STATE_ENABLED_CONNECTED and WiFi.status() == WL_CONNECTED) {
        WiFi.disconnect(true, true); // true = wifi_off, true = clear_credentials
        WiFi.mode(WIFI_OFF);
        wifi_state = WIFI_STATE_DISABLED;
        show_wifi_status();
        update_wifi_parameters();
      }
      save_presets();
    }
    lv_tabview_set_act(tabview, TAB_WIFI_REF, LV_ANIM_OFF);
  }
  else if (code == LV_EVENT_CANCEL) {
    lv_tabview_set_act(tabview, TAB_WIFI_REF, LV_ANIM_OFF);
  }
}

// callback to connect textareas tp keyboard
void ta_event_cb(lv_event_t * e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * ta = (lv_obj_t *) lv_event_get_target(e);
  if (code == LV_EVENT_FOCUSED) {
    // connect keyboard to textarea
    lv_keyboard_set_textarea(txt_wifi_keyboard, ta);
    lv_obj_clear_flag(txt_wifi_keyboard, LV_OBJ_FLAG_HIDDEN);  // toon keyboard
  }
}

void update_wifi_parameters(void) {
  if (wifi_state == WIFI_STATE_ENABLED_CONNECTED) {
    // SSID
    lv_label_set_text(lbl_wifi_ssid, WiFi.SSID().c_str());

    // IP
    lv_label_set_text(lbl_wifi_ip, WiFi.localIP().toString().c_str());

    // RSSI (signaalsterkte)
    {
      char buf_rssi[32];
      snprintf(buf_rssi, sizeof(buf_rssi), "%d dBm", WiFi.RSSI());
      lv_label_set_text(lbl_wifi_rssi, buf_rssi);
    }

    // Kanaal
    {
      char buf_channel[32];
      snprintf(buf_channel, sizeof(buf_channel), "%d", WiFi.channel());
      lv_label_set_text(lbl_wifi_channel, buf_channel);
    }
  }
  else {
    lv_label_set_text(lbl_wifi_ssid, "---");
    lv_label_set_text(lbl_wifi_ip, "---");
    lv_label_set_text(lbl_wifi_rssi, "---");
    lv_label_set_text(lbl_wifi_channel, "---");
  }
}

static void event_mainscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    cleanup();
  }
}

static void event_menuscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_MENU_REF, LV_ANIM_OFF);
  }
}

static void event_scanscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    master_scan_screen_mode_main = true;
    set_scan_mode_screen(master_scan_screen_mode_main, false);
    lv_tabview_set_act(tabview, TAB_SCAN_REF, LV_ANIM_OFF);

    lv_spinbox_set_value(spinbox_scan_from, master_scan_from_channel);
    lv_spinbox_set_value(spinbox_scan_to, master_scan_to_channel);
    clear_skip_channels();
    master_scan_current_channel = master_scan_from_channel;
    update_scan_channel(0);
    update_scan_progress(0);
  }
}

static void event_powerscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_POWER_REF, LV_ANIM_OFF);
  }
}

static void event_settingsscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_SETTINGS_REF, LV_ANIM_OFF);
  }
}

static void event_settingsrangescr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_SETTINGS_RANGE_REF, LV_ANIM_OFF);
    lv_spinbox_set_value(spinbox_settings_from, master_settings_lower_limit);
    lv_spinbox_set_value(spinbox_settings_to, master_settings_upper_limit);
  }
}

static void event_settings_s_pwr_scr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_SETTINGS_S_PWR_REF, LV_ANIM_OFF);
    // force redraw
    master_screen_update_done = false;
    master_screen_update_counter = 0;
    master_settings_cal_s_previous = master_settings_cal_s;
    master_settings_cal_pwr_previous = master_settings_cal_pwr;
  }
}


static void event_wifiscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_WIFI_REF, LV_ANIM_OFF);
    update_wifi_parameters();
  }
}

static void event_wifisetupscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_textarea_set_text(txt_wifi_ssid, master_settings_wifi_ssid.c_str());
    lv_textarea_set_text(txt_wifi_password, master_settings_wifi_password.c_str());
    master_settings_wifi_ssid_old = master_settings_wifi_ssid;
    master_settings_wifi_password_old = master_settings_wifi_password;
    lv_tabview_set_act(tabview, TAB_WIFISETUP_REF, LV_ANIM_OFF);
  }
}

void cleanup(void) {

  show_connection_status(master_slave_is_connected);
  show_lock_status(master_pll_locked, !master_slave_is_connected);
  show_tx_status(master_tx_on, !master_slave_is_connected);
  master_channel_frequency_refresh = true;
  switch (lv_tabview_get_tab_act(tabview)) {
    case TAB_SCAN_REF:
      scanState = SCAN_STOP;

      if (master_scan_hold_channel != 0) {
        master_channel = master_scan_hold_channel;
        master_frequency = convert_channel_to_frequency(master_channel);
      }
      break;

  }

  if (master_flag_save_presets) {
    save_presets();
    master_flag_save_presets = false;
  }

  lv_tabview_set_act(tabview, TAB_MAIN_REF, LV_ANIM_OFF);
  // force redraw
  master_screen_update_done = false;
  master_screen_update_counter = 0;

  //  Serial.print("free total: "); Serial.println(esp_get_free_heap_size());
  //  Serial.print("largest total: "); Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
  //  Serial.print("free internal: "); Serial.println(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  //  Serial.print("largest internal: "); Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
  //  Serial.print("free spiram: "); Serial.println(heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  //  Serial.print("largest spiram: "); Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
  //  Serial.print("free dma: "); Serial.println(heap_caps_get_free_size(MALLOC_CAP_DMA));
  //  Serial.print("largest dma: "); Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
  //  Serial.println("---");
  //
  //  Serial.print("LV stack highwater: ");
  //  Serial.println(uxTaskGetStackHighWaterMark(NULL));


}

static void event_toggle_channel_freq(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    toggle_channel_freq();
  }
}

void toggle_channel_freq(void) {
  master_channel_mode = !master_channel_mode;
  set_channel_frequency_mode(master_channel_mode);
  // clear alfa channel but no further action, lower normal channel is used if switched from freq to channel
  master_channel_is_alfa = false;
  master_channel_frequency_refresh = true;
  notifyClients();
}

// switch display of channel or frequency as primary
void set_channel_frequency_mode(boolean c_mode) {
  if (c_mode) {
    lv_obj_add_flag(btn_main_channel2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(lbl_main_channel2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(lbl_main_channel_alfa2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(btn_main_freq2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(lbl_main_freq2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(btn_main_channel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(lbl_main_channel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(btn_main_freq, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(lbl_main_freq, LV_OBJ_FLAG_HIDDEN);
  }
  else {
    lv_obj_add_flag(btn_main_channel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(lbl_main_channel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(btn_main_freq, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(lbl_main_freq, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(btn_main_channel2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(lbl_main_channel2, LV_OBJ_FLAG_HIDDEN);
    if (master_channel_is_alfa) {
      lv_obj_clear_flag(lbl_main_channel_alfa2, LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_clear_flag(btn_main_freq2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(lbl_main_freq2, LV_OBJ_FLAG_HIDDEN);
  }
}

void show_connection_status(boolean isconnected) {
  if (isconnected) {
    lv_obj_set_style_text_color(lbl_main_noconnection, lv_palette_main(LV_PALETTE_GREEN), LV_PART_MAIN);
  }
  else {
    lv_obj_set_style_text_color(lbl_main_noconnection, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);
  }
}

void show_tx_status(boolean istx, boolean noshow) {
  if (!istx or noshow) {
    lv_obj_add_flag(lbl_main_tx, LV_OBJ_FLAG_HIDDEN);
  }
  else {
    lv_obj_clear_flag(lbl_main_tx, LV_OBJ_FLAG_HIDDEN);
  }
}

void show_lock_status(boolean islocked, boolean noshow) {
  if (islocked or noshow) {
    lv_obj_add_flag(led_main_nolock, LV_OBJ_FLAG_HIDDEN);
  }
  else {
    lv_obj_clear_flag(led_main_nolock, LV_OBJ_FLAG_HIDDEN);
  }
}

static void update_channel(void) {
  // always minimum 4 positions wide, padded left with spaces
  snprintf(printbuf, sizeof(printbuf), "%4d", master_channel);
  lv_label_set_text(lbl_main_channel, printbuf);
  lv_label_set_text(lbl_main_channel2, printbuf);
  if (!master_channel_mode) {
    if (master_channel_is_alfa) {
      lv_obj_clear_flag(lbl_main_channel_alfa2, LV_OBJ_FLAG_HIDDEN);
    }
    else {
      lv_obj_add_flag(lbl_main_channel_alfa2, LV_OBJ_FLAG_HIDDEN);
    }
  }
}

static void update_frequency(void) {
  // split MHz and kHz
  int mhz = master_frequency / 1000;
  int khz = master_frequency % 1000;
  snprintf(printbuf, sizeof(printbuf), "%2d.%03d", mhz, khz);
  lv_label_set_text(lbl_main_freq, printbuf);
  lv_label_set_text(lbl_main_freq2, printbuf);
}

// convert frequency to channel including alfa
int16_t convert_frequency_to_channel(uint16_t input_freq) {
  int16_t return_channel = 0;
  uint8_t FrequencyBand;
  uint8_t Lookup;
  // 45 values per band, 40 plus alpha channels
  // tables based on Jumbo 3
  // channel array positive channels
  int ChannelArrayPos[45] = {1, 2, 3, 3, 4, 5, 6, 7, 7, 8, 9, 10, 11, 11, 12, 13, 14, 15, 15, 16, 17, 18, 19, 19, 20, 21, 22, 24, 25, 23, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40};
  // alfa channel array positive channels
  boolean AlfaArrayPos[45] = {false, false, false, true, false, false, false, false, true, false, false, false, false, true, false, false, false, false, true, false, false, false, false, true, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
  // channel array negative channels
  int ChannelArrayNeg[45] = { -40, -39, -38, -37, -36, -35, -34, -33, -32, -31, -30, -29, -28, -27, -26, -23, -25, -24, -22, -21, -20, -19, -19, -18, -17, -16, -15, -15, -14, -13, -12, -11, -11, -10, -9, -8, -7, -7, -6, -5, -4, -3, -3, -2, -1};
  // alfa channel array negative channels
  boolean AlfaArrayNeg[45] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, true, false, false, false, false, true, false, false, false, false, true, false, false, false, false, true, false, false, false, false, true, false, false, false};

  // calculate band, 45 per band 10khz per step
  FrequencyBand = (input_freq - master_minimum_frequency) / 450;
  // calculate lookup parameter
  Lookup = ((input_freq - master_minimum_frequency) - (FrequencyBand * 450)) / 10;

  if (FrequencyBand < 3) {
    return_channel = ChannelArrayNeg[Lookup];
    master_channel_is_alfa = AlfaArrayNeg[Lookup];
  }
  else {
    return_channel = ChannelArrayPos[Lookup];
    master_channel_is_alfa = AlfaArrayPos[Lookup];
  }
  switch (FrequencyBand) {
    case 0:
      // -81 -120
      return_channel = return_channel - 80;
      break;

    case 1:
      // -41 -80
      return_channel = return_channel - 40;
      break;

    case 2:
      // -1 -40
      break;

    case 3:
      // 1 40
      break;

    case 4:
      // 41 80
      return_channel = return_channel + 40;
      break;

    case 5:
      // 81 120
      return_channel = return_channel + 80;
      break;
  }
  return return_channel;
}

// convert channel number to frequency
uint16_t convert_channel_to_frequency(int16_t input_channel) {
  uint16_t return_frequency = 0;
  // freq multiplier array positive channels
  int ChannelArrayPos[45] = {0, 1, 2, 4, 5, 6, 7, 9, 10, 11, 12, 14, 15, 16, 17, 19, 20, 21, 22, 24, 25, 26, 29, 27, 28, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44 };
  // freq multiplier array negative channels
  int ChannelArrayNeg[45] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 15, 18, 19, 20, 22, 23, 24, 25, 27, 28, 29, 30, 32, 33, 34, 35, 37, 38, 39, 40, 42, 43, 44};

  uint8_t FrequencyBand = 3;
  uint8_t Lookup = 0;

  if (input_channel >= -120 and input_channel <= -81) {
    FrequencyBand = 0;
    Lookup = 0;
    Lookup = 120 + input_channel;
  }
  else {
    if (input_channel >= -80 and input_channel <= -41) {
      FrequencyBand = 1;
      Lookup = 80 + input_channel;
    }
    else {
      if (input_channel >= -40 and input_channel <= -1) {
        FrequencyBand = 2;
        Lookup = 40 + input_channel;
      }
      else {
        if (input_channel >= 1 and input_channel <= 40) {
          FrequencyBand = 3;
          Lookup = input_channel - 1;
        }
        else {
          if (input_channel >= 41 and input_channel <= 80) {
            FrequencyBand = 4;
            Lookup = input_channel - 41;
          }
          else {
            if (input_channel >= 81 and input_channel <= 120) {
              FrequencyBand = 5;
              Lookup = input_channel - 81;
            }
          }
        }
      }
    }
  }
  if (FrequencyBand < 3) {
    return_frequency = 26515 + (ChannelArrayNeg[Lookup] * 10) - ((2 - FrequencyBand) * 450);
  }
  else {
    return_frequency = 26965 + (ChannelArrayPos[Lookup] * 10) + ((FrequencyBand - 3) * 450);
  }
  return return_frequency;
}

static void event_channel_touch(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);

  static bool channel_long_press_handled = false;

  // Reset flag at the start of a new press sequence
  if (event == LV_EVENT_PRESSED) {
    channel_long_press_handled = false;
  }
  if (event == LV_EVENT_LONG_PRESSED) {
    channel_long_press_handled = true;
    lv_tabview_set_act(tabview, TAB_PRESETS_REF, LV_ANIM_OFF);
  } else if (event == LV_EVENT_CLICKED) {
    if (channel_long_press_handled) {
      return;
    }
    direct_channel_clear_input();
    direct_channel_update_display();
    refresh_direct_channel_button();
    lv_tabview_set_act(tabview, TAB_DIRECTENTRYCHANNEL_REF, LV_ANIM_OFF);
  }
}

static void event_frequency_touch(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);

  static bool freq_long_press_handled = false;

  // Reset flag at the start of a new press sequence
  if (event == LV_EVENT_PRESSED) {
    freq_long_press_handled = false;
  }
  if (event == LV_EVENT_LONG_PRESSED) {
    freq_long_press_handled = true;
    lv_tabview_set_act(tabview, TAB_PRESETS_REF, LV_ANIM_OFF);
  } else if (event == LV_EVENT_CLICKED) {
    if (freq_long_press_handled) {
      return;
    }
    direct_freq_clear_input();
    direct_freq_update_display();
    refresh_direct_freq_button();
    lv_tabview_set_act(tabview, TAB_DIRECTENTRYFREQ_REF, LV_ANIM_OFF);
  }
}

static void event_gotochannel(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (master_direct_channel_input_index != 0) {
      int16_t channel = atoi(master_direct_channel_input_buffer);
      // validate channel range
      if (channel >= master_lower_channel and channel <= master_upper_channel and channel != 0) {
        master_channel = channel;
        master_channel_frequency_refresh = true;
      }
    }
    lv_tabview_set_act(tabview, TAB_MAIN_REF, LV_ANIM_OFF);
    // force redraw
    master_screen_update_done = false;
    master_screen_update_counter = 0;
  }
}

static void event_gotofreq(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (master_direct_freq_input_index == 3 and strlen(master_direct_freq_input_buffer) == 3 and validate_frequency(master_direct_freq_input_buffer)) {
      // build full frequency number (2 + mid_digits + 5)
      char full_freq[6];
      snprintf(full_freq, sizeof(full_freq), "2%s5", master_direct_freq_input_buffer);
      int frequency = atoi(full_freq);
      master_frequency = frequency;
      master_channel_frequency_refresh = true;
    }
    lv_tabview_set_act(tabview, TAB_MAIN_REF, LV_ANIM_OFF);
    // force redraw
    master_screen_update_done = false;
    master_screen_update_counter = 0;
  }
}

// clear the input buffer
static void direct_channel_clear_input(void) {
  master_direct_channel_input_index = 0;
  master_direct_channel_input_buffer[0] = '\0';
}

// button matrix event handler
static void direct_channel_btnm_event_handler(lv_event_t * e) {
  lv_obj_t * obj = (lv_obj_t *)lv_event_get_target(e);
  uint16_t btn_id = lv_btnmatrix_get_selected_btn(obj);
  if (btn_id == LV_BTNMATRIX_BTN_NONE) return;
  const char * txt = lv_btnmatrix_get_btn_text(obj, btn_id);
  if (strcmp(txt, LV_SYMBOL_LEFT) == 0) {
    direct_channel_handle_backspace();
  } else if (strcmp(txt, "") != 0) {
    direct_channel_add_character(txt);
  }
  refresh_direct_channel_button();
  direct_channel_update_display();
}

// update the display label
static void direct_channel_update_display(void) {
  if (master_direct_channel_input_index == 0) {
    lv_label_set_text(lbl_directchannel_channel, ".");
  } else {
    char display_text[20];
    snprintf(display_text, sizeof(display_text), "%s", master_direct_channel_input_buffer);
    lv_label_set_text(lbl_directchannel_channel, display_text);
  }
}

// handle backspace
static void direct_channel_handle_backspace(void) {
  if (master_direct_channel_input_index > 0) {
    master_direct_channel_input_index--;
    master_direct_channel_input_buffer[master_direct_channel_input_index] = '\0';
  }
}

// add character to input buffer
static void direct_channel_add_character(const char * c) {
  // check if we can add more characters
  if (master_direct_channel_input_index >= sizeof(master_direct_channel_input_buffer) - 1) return;
  // special handling for minus sign
  if (strcmp(c, "-") == 0) {
    // minus can only be first character
    if (master_direct_channel_input_index == 0) {
      master_direct_channel_input_buffer[master_direct_channel_input_index++] = '-';
      master_direct_channel_input_buffer[master_direct_channel_input_index] = '\0';
    }
    return;
  }

  // check if adding this digit would exceed maximum digits
  // maximum value is 120, so maximum digits is 3 (plus minus sign)
  int current_len = strlen(master_direct_channel_input_buffer);
  if (current_len >= (master_direct_channel_input_buffer[0] == '-' ? 4 : 3)) return;

  // add the digit
  master_direct_channel_input_buffer[master_direct_channel_input_index++] = c[0];
  master_direct_channel_input_buffer[master_direct_channel_input_index] = '\0';

  // check if the current value would be valid
  int value = atoi(master_direct_channel_input_buffer);
  if (value < master_lower_channel || value > master_upper_channel || value == 0) {
    // if not, remove the last character
    master_direct_channel_input_index--;
    master_direct_channel_input_buffer[master_direct_channel_input_index] = '\0';
  }
}

void refresh_direct_channel_button(void) {
  if (validate_channel()) {
    lv_obj_clear_state(btn_directchannel_goto, LV_STATE_DISABLED);
  }
  else {
    lv_obj_add_state(btn_directchannel_goto, LV_STATE_DISABLED);
  }
}

// validate channel
boolean validate_channel(void) {
  if (master_direct_channel_input_index != 0) {
    int16_t channel = atoi(master_direct_channel_input_buffer);
    // validate channel range
    if (channel >= master_lower_channel and channel <= master_upper_channel and channel != 0) {
      return true;
    }
  }
  return false;
}

// clear the input buffer
static void direct_freq_clear_input(void) {
  master_direct_freq_input_index = 0;
  master_direct_freq_input_buffer[0] = '\0';
  master_direct_freq_entered = false;
}

// button matrix event handler
static void direct_freq_btnm_event_handler(lv_event_t * e) {
  lv_obj_t * obj = (lv_obj_t *)lv_event_get_target(e);
  uint16_t btn_id = lv_btnmatrix_get_selected_btn(obj);
  if (btn_id == LV_BTNMATRIX_BTN_NONE) return;
  const char * txt = lv_btnmatrix_get_btn_text(obj, btn_id);
  if (strcmp(txt, LV_SYMBOL_LEFT) == 0) {
    direct_freq_handle_backspace();
  } else if (strcmp(txt, "CLR") == 0) {
    direct_freq_clear_input();
  } else if (strcmp(txt, "") != 0) {
    direct_freq_add_digit(txt);
  }
  refresh_direct_freq_button();
  direct_freq_update_display();
}

// update the display label
static void direct_freq_update_display(void) {
  if (master_direct_freq_input_index == 0) {
    lv_label_set_text(lbl_directfreq_frequency, "2-.--5");
  } else {
    char display_text[20];
    char padded[4] = "000";
    // copy input and pad with zeros if needed
    strncpy(padded, master_direct_freq_input_buffer, master_direct_freq_input_index);
    // build the frequency string: 2 + first digit + . + next two digits + 5
    snprintf(display_text, sizeof(display_text), "2%c.%c%c5",
             padded[0],
             master_direct_freq_input_index > 1 ? padded[1] : '-',
             master_direct_freq_input_index > 2 ? padded[2] : '-');
    lv_label_set_text(lbl_directfreq_frequency, display_text);
  }
}

// handle backspace
static void direct_freq_handle_backspace(void) {
  if (master_direct_freq_input_index > 0) {
    master_direct_freq_input_index--;
    master_direct_freq_input_buffer[master_direct_freq_input_index] = '\0';
    master_direct_freq_entered = (master_direct_freq_input_index == 3);
  }
}

// add digit to input buffer
static void direct_freq_add_digit(const char * digit) {
  // only allow 3 digits
  if (master_direct_freq_input_index >= 3) return;
  // if we already have a complete frequency, clear it first
  if (master_direct_freq_entered) {
    direct_freq_clear_input();
  }
  // add the digit
  master_direct_freq_input_buffer[master_direct_freq_input_index++] = digit[0];
  master_direct_freq_input_buffer[master_direct_freq_input_index] = '\0';
  // if we have 3 digits, it's a complete frequency
  if (master_direct_freq_input_index == 3) {
    master_direct_freq_entered = true;
  }
}

void refresh_direct_freq_button(void) {
  if (validate_frequency(master_direct_freq_input_buffer)) {
    lv_obj_clear_state(btn_directfreq_goto, LV_STATE_DISABLED);
  }
  else {
    lv_obj_add_state(btn_directfreq_goto, LV_STATE_DISABLED);
  }
}

// validate frequency from middle digits
boolean validate_frequency(const char * mid_digits) {
  if (strlen(mid_digits) != 3) return false;
  // build full frequency number (2 + mid_digits + 5)
  char full_freq[6];
  snprintf(full_freq, sizeof(full_freq), "2%s5", mid_digits);
  int frequency = atoi(full_freq);
  // check range
  return (frequency >= master_lower_frequency && frequency <= master_upper_frequency);
}

void my_spinbox_inc(lv_obj_t * spinbox) {
  lv_spinbox_increment(spinbox);
  if (lv_spinbox_get_value(spinbox) == 0) {
    lv_spinbox_set_value(spinbox, 1);
  }
}

void my_spinbox_dec(lv_obj_t * spinbox) {
  lv_spinbox_decrement(spinbox);
  if (lv_spinbox_get_value(spinbox) == 0) {
    lv_spinbox_set_value(spinbox, -1);
  }
}

// find which channel belongs to the pll code we received from the slave in order to check movement and direction
int find_channel(uint8_t pll_code) {
  // the channel array holds the pll codes for channels 1-40
  for (int i = 0; i < 40; i++) {
    if (channel_array[i] == pll_code) {
      return i + 1; // index = channelnumber-1
    }
  }
  return -1; // invalid
}

// calculate difference between channels and direction
int channel_difference(uint8_t old_pll, uint8_t new_pll) {
  int old_ch = find_channel(old_pll);
  int new_ch = find_channel(new_pll);
  if (old_ch == -1 || new_ch == -1) {
    return 0; // invalid
  }
  int diff = new_ch - old_ch;
  // take shortest direction
  if (diff > 20) {
    diff -= 40;
  } else if (diff < -20) {
    diff += 40;
  }
  constrain(diff, -40, +40);
  return diff;
  // positive = turned clockwise
  // negative = turned counterclockwise
}

// calculate PLL code based on frequency and return pll setting
uint16_t calculate_PLL(uint16_t FrequencyIn) {
  // define the range of frequency and PLL
  // this is based on cybernet PLL02A platform
  const uint16_t FreqMin = 24405;
  const uint16_t FreqMax = 29505;
  const uint16_t PLLMin = 1;
  const uint16_t PLLMax = 511;
  // ensure frequency is within the valid range for channel -120 to 120
  if (FrequencyIn < 25615) {
    FrequencyIn = 25615;
  } else if (FrequencyIn > 28305) {
    FrequencyIn = 28305;
  }
  // Perform inverse linear mapping
  uint16_t PLLOut = map(FrequencyIn, FreqMin, FreqMax, PLLMax, PLLMin);
  return PLLOut;
}

// draw the smeter scale
void draw_s_meter_scale() {
  xSemaphoreTake(spi_mutex_lcd, portMAX_DELAY);
  TFT.waitDMA();
  TFT.setTextSize(1);
  TFT.drawRect(master_smeter_x, master_smeter_y, master_smeter_w, master_smeter_h, TFT_WHITE);
  int step = master_smeter_w / (master_smeter_num_labels - 1); // afstand tussen labels
  int base_y = master_smeter_y + master_smeter_h + 12;
  TFT.setTextDatum(top_center);
  for (int i = 0; i < master_smeter_num_labels; i++) {
    int px = master_smeter_x + i * step;
    uint16_t c = (i <= 9) ? TFT_GREEN : TFT_RED;
    uint16_t o = (i <= 9) ? 1 : 0;
    TFT.fillRect(px, master_smeter_y + master_smeter_h,
                 2, 6, c);  // breedte 2, hoogte 6 pixels
    TFT.setTextColor(c);
    TFT.drawString(master_smeter_labels[i], px + o, base_y);
  }
  TFT.setTextColor(TFT_RED);
  for (int i = 0; i <= 12; i++) {
    TFT.fillRect(master_smeter_x + master_power_positions[i], master_smeter_y - 6, 2, 6, TFT_RED);
    TFT.drawString(master_power_labels[i], master_smeter_x + 1 + master_power_positions[i] , 0);
  }
  TFT.setTextColor(TFT_WHITE);
  TFT.drawString("P", master_smeter_x - 30 , 0);
  TFT.drawString("S", master_smeter_x - 30 , base_y);
  xSemaphoreGive(spi_mutex_lcd);
}

float read_cb_smeter_linear(uint16_t adc_raw) {
  // convert to voltage at ADC pin (0-3.3V)
  float v_adc = (adc_raw / 4095.0f) * 3.3f;

  // calibrate with slider values
  float cal_ = 0;
  if (master_tx_on) {
    cal_ = -((float)master_settings_cal_pwr - 50.0f) * 0.05f;
  }
  else {
    cal_ = -((float)master_settings_cal_s - 50.0f) * 0.05f;
  }

  // convert to original meter voltage (divide by opamp gain 12.5x)
  float v_meter = v_adc / (12.5f + cal_);

  // convert mV to S-value based on measured points
  float s_value;

  if (v_meter <= 0.014f) {
    // below S1
    s_value = (v_meter / 0.014f); // 0 to S1
  }
  else if (v_meter <= 0.042f) {
    // S1 to S3: linear 14mV to 42mV
    s_value = 1.0f + ((v_meter - 0.014f) / 0.014f); // 1.0 to 3.0
  }
  else if (v_meter <= 0.147f) {
    // S3 to S9: linear 42mV to 147mV
    s_value = 3.0f + ((v_meter - 0.042f) / 0.0175f); // 3.0 to 9.0
  }
  else if (v_meter <= 0.200f) {
    // S9 to S9+30: linear 147mV to 200mV
    s_value = 9.0f + ((v_meter - 0.147f) / 0.01767f); // 9.0 to 12.0
  }
  else {
    // Above full scale
    s_value = 12.0f;
  }

  // Round to nearest 0.1
  s_value = roundf(s_value * 10.0f) / 10.0f;

  // Ensure within 0-12 range
  if (s_value < 0.0f) s_value = 0.0f;
  if (s_value > 12.0f) s_value = 12.0f;

  return s_value;
}

// draw the actual smeter using a sprite
void draw_s_meter_value(float s_value) {
  if (s_value < 0) s_value = 0;
  if (s_value > 12) s_value = 12;
  // use FLOAT division to get exact step size
  float step = (float)(master_smeter_w - 2) / (float)(master_smeter_num_labels - 1);
  int fill_px = (int)(s_value * step + 0.5);
  // use sprite for drawing
  smeter_sprite.fillScreen(TFT_BLACK);
  if (fill_px > 0) {
    int green_limit = 9 * step;
    int green_w = min(fill_px, green_limit);
    int red_w = max(0, fill_px - green_limit);
    if (green_w > 0) {
      smeter_sprite.fillRect(0, 0, green_w, master_smeter_h - 2, master_tx_on ? TFT_RED : TFT_GREEN);
    }
    if (red_w > 0) {
      smeter_sprite.fillRect(0 + green_limit, 0, red_w, master_smeter_h - 2, TFT_RED);
    }
  }
  smeter_sprite.pushSprite(&TFT, master_smeter_x + 1, master_smeter_y + 1);
}

// scan screen

// change in spinbox values
static void event_scan_from_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    if (lv_spinbox_get_value(spinbox_scan_from) > lv_spinbox_get_value(spinbox_scan_to) - 20) {
      if (lv_spinbox_get_value(spinbox_scan_from) + 20 == 0) {
        lv_spinbox_set_value(spinbox_scan_to, lv_spinbox_get_value(spinbox_scan_from) + 21);
      }
      else {
        lv_spinbox_set_value(spinbox_scan_to, lv_spinbox_get_value(spinbox_scan_from) + 20);
      }
    }
    master_scan_from_channel = lv_spinbox_get_value(spinbox_scan_from);
  }
}

// change in spinbox values
static void event_scan_to_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    if (lv_spinbox_get_value(spinbox_scan_to) < lv_spinbox_get_value(spinbox_scan_from) + 20) {
      if (lv_spinbox_get_value(spinbox_scan_to) - 20 == 0) {
        lv_spinbox_set_value(spinbox_scan_from, lv_spinbox_get_value(spinbox_scan_to) - 21);
      }
      else {
        lv_spinbox_set_value(spinbox_scan_from, lv_spinbox_get_value(spinbox_scan_to) - 20);
      }
    }
    master_scan_to_channel = lv_spinbox_get_value(spinbox_scan_to);
  }
}

void increment_scan_from_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    my_spinbox_inc(spinbox_scan_from);
  }
}

static void decrement_scan_from_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    my_spinbox_dec(spinbox_scan_from);
  }
}

void increment_scan_to_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    my_spinbox_inc(spinbox_scan_to);
  }
}

static void decrement_scan_to_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    my_spinbox_dec(spinbox_scan_to);
  }
}

// toggle entry&progress or smeter chart
static void event_scan_toggle(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    master_scan_screen_mode_main = !master_scan_screen_mode_main;
    set_scan_mode_screen(master_scan_screen_mode_main, true);
  }
}

void set_scan_mode_screen(boolean screenmodemain, boolean do_clear) {
  if (screenmodemain) {
    // clear smeter area
    if (do_clear) {
      xSemaphoreTake(spi_mutex_lcd, portMAX_DELAY);
      TFT.waitDMA();
      TFT.fillRect(0, 250 - 48, 400, 48, TFT_BLACK);
      xSemaphoreGive(spi_mutex_lcd);
    }
    lv_obj_clear_flag(bar_scan_progress, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(spinbox_scan_from, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(spinbox_scan_to, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(btn_scan_from_inc, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(btn_scan_from_dec, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(btn_scan_to_inc, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(btn_scan_to_dec, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(lbl_scan_from, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(lbl_scan_to, LV_OBJ_FLAG_HIDDEN);
  }   else {
    lv_obj_add_flag(bar_scan_progress, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(spinbox_scan_from, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(spinbox_scan_to, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(btn_scan_from_inc, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(btn_scan_from_dec, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(btn_scan_to_inc, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(btn_scan_to_dec, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(lbl_scan_from, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(lbl_scan_to, LV_OBJ_FLAG_HIDDEN);
  }
}

void update_scan_channel(int8_t scan_channel) {
  if (scan_channel == 0) {
    lv_label_set_text(lbl_scan_channel, "----");
  }
  else {
    snprintf(printbuf, sizeof(printbuf), "%4d", scan_channel);
    lv_label_set_text(lbl_scan_channel, printbuf);
  }
}

void set_scan_channel(int8_t scan_channel) {
  uint16_t scan_frequency = convert_channel_to_frequency(scan_channel);
  // calculate new pll setting, this will be transferred to the slave automatically
  master_out_message_pll_setting = calculate_PLL(scan_frequency);
}

static void event_scanstart(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    start_scan();
  }
}

void start_scan(void) {
  if (scanState == SCAN_IDLE and master_slave_is_connected) {
    scanState = SCAN_START;
  }
  else if (scanState == SCAN_HOLD and master_slave_is_connected) {
    next_scan_channel();
    master_squelch_status = 0;
    show_squelch_status(master_squelch_status);
    scanState = SCAN_SCANNING;
    update_scan_buttons();
    update_scan_progress(master_scan_current_channel);
    update_scan_channel(master_scan_current_channel);
    set_scan_channel(master_scan_current_channel);
    master_scan_timer = millis();
  }
}

void event_scan_skip(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    add_skip_channel(master_scan_current_channel);
    next_scan_channel();
    master_squelch_status = 0;
    show_squelch_status(master_squelch_status);
    scanState = SCAN_SCANNING;
    update_scan_buttons();
    update_scan_progress(master_scan_current_channel);
    update_scan_channel(master_scan_current_channel);
    set_scan_channel(master_scan_current_channel);
    master_scan_timer = millis();
  }
}

static void event_scan_back(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (scanState == SCAN_IDLE) {
      cleanup();
    }
    else {
      scanState = SCAN_STOP;
    }
  }
}

void show_squelch_status(uint8_t showmode) {
  if (showmode == 0) {
    lv_obj_add_flag(lbl_scan_squelch, LV_OBJ_FLAG_HIDDEN);
  }
  else {
    if (showmode == 1) {
      lv_obj_set_style_text_color(lbl_scan_squelch, lv_palette_main(LV_PALETTE_GREEN), LV_PART_MAIN);
    }
    else {
      lv_obj_set_style_text_color(lbl_scan_squelch, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_MAIN);
    }
    lv_obj_clear_flag(lbl_scan_squelch, LV_OBJ_FLAG_HIDDEN);
  }
}

void update_scan_progress(int16_t currentChannel) {
  if (master_scan_screen_mode_main) {
    if (currentChannel == 0) {
      lv_bar_set_value(bar_scan_progress, 0, LV_ANIM_OFF);
    }
    else {
      int16_t fromCh = master_scan_from_channel;
      int16_t toCh   = master_scan_to_channel;
      int16_t minCh  = min(fromCh, toCh);
      int16_t maxCh  = max(fromCh, toCh);

      int16_t totalSteps = (maxCh - minCh) + 1;
      if (totalSteps < 2) totalSteps = 2; // protectie tegen deling door nul

      // index in range
      int16_t index = currentChannel - minCh;
      if (index < 0) index = 0;
      if (index >= totalSteps) index = totalSteps - 1;

      // calc percentage
      int progress = (index * 100) / (totalSteps - 1);

      // update progress bar
      lv_bar_set_value(bar_scan_progress, progress, LV_ANIM_OFF);
    }
  }
  else {
    draw_scan_bar(250);
  }
}

void draw_scan_bar(int base_y ) {
  int area_width = 400;
  int chan_count = master_scan_to_channel - master_scan_from_channel + 1;
  // skip channel 0
  if (master_scan_from_channel <= 0 && master_scan_to_channel >= 0) {
    chan_count--;
  }
  if (chan_count <= 0) return;

  // calc bar width and centering
  int bar_width = area_width / chan_count;
  if (bar_width < 1) bar_width = 1;
  int total_width = bar_width * chan_count;
  int x_offset = (area_width - total_width) / 2;

  // index of channel
  int idx = master_scan_current_channel - master_scan_from_channel;
  if (master_scan_current_channel > 0 && master_scan_from_channel <= 0) {
    idx--;
  }
  if (master_scan_current_channel == 0) return;

  if (idx < 0 || idx >= chan_count) return;

  // s-meter value (0..12.0 → 0..48 px)
  float val = master_smeter_value;
  if (val < 0.0f) val = 0.0f;
  if (val > 12.0f) val = 12.0f;
  int h = (int)(val * 4.0f);  // 4 px per S-unit

  // S9 border = 9.0 → 36 px
  int h_green = (h > 36) ? 36 : h;
  int h_red   = (h > 36) ? (h - 36) : 0;

  // x-position
  int x = x_offset + idx * bar_width;
  xSemaphoreTake(spi_mutex_lcd, portMAX_DELAY);
  TFT.waitDMA();
  TFT.startWrite();

  // erase column
  TFT.fillRect(x, base_y - 48, bar_width, 48, TFT_BLACK);
  // green part (0 .. S9)
  if (h_green > 0) {
    TFT.fillRect(x, base_y - h_green, bar_width, h_green, TFT_GREEN);
  }
  // red part (> S9)
  if (h_red > 0) {
    TFT.fillRect(x, base_y - h_green - h_red, bar_width, h_red, TFT_RED);
  }
  TFT.endWrite();
  xSemaphoreGive(spi_mutex_lcd);

}


// new version, really skip tagged channels instead of ignoring squelch open because release time triggers the next channel squelch open

void next_scan_channel(void) {
  // max number of channel skips to try, avoid infinite loop
  uint8_t attempts = (master_scan_to_channel - master_scan_from_channel + 1);

  do {
    master_scan_current_channel++;

    // skip channel 0
    if (master_scan_current_channel == 0) {
      master_scan_current_channel++;
    }

    // wrap-around
    if (master_scan_current_channel > master_scan_to_channel) {
      master_scan_current_channel = master_scan_from_channel;
    }

    // doen if not a skipped channel
    if (!is_channel_skipped(master_scan_current_channel)) return;

    attempts--;

  } while (attempts > 0);

  // set to default start if all skipped
  master_scan_current_channel = master_scan_from_channel;
}

void clear_skip_channels(void) {
  master_scan_skip_count = 0;
}

bool add_skip_channel(int8_t ch) {
  if (master_scan_skip_count >= 5) return false;
  // vermijd dubbel toevoegen
  for (uint8_t i = 0; i < master_scan_skip_count; ++i)
    if (master_scan_skip_channels[i] == ch) return false;
  master_scan_skip_channels[master_scan_skip_count++] = ch;
  return true;
}

bool is_channel_skipped(int8_t ch) {
  for (uint8_t i = 0; i < master_scan_skip_count; i++) {
    if (master_scan_skip_channels[i] == ch) return true;
  }
  return false;
}

void update_scan_buttons(void) {
  if (scanState == SCAN_IDLE) {
    lv_obj_clear_state(spinbox_scan_from, LV_STATE_DISABLED);
    lv_obj_clear_state(spinbox_scan_to, LV_STATE_DISABLED);
    lv_obj_clear_state(btn_scan_from_inc, LV_STATE_DISABLED);
    lv_obj_clear_state(btn_scan_from_dec, LV_STATE_DISABLED);
    lv_obj_clear_state(btn_scan_to_inc, LV_STATE_DISABLED);
    lv_obj_clear_state(btn_scan_to_dec, LV_STATE_DISABLED);
    lv_label_set_text(lbl_scan_back, LV_SYMBOL_HOME);
  }
  else {
    lv_obj_add_state(spinbox_scan_from, LV_STATE_DISABLED);
    lv_obj_add_state(spinbox_scan_to, LV_STATE_DISABLED);
    lv_obj_add_state(btn_scan_from_inc, LV_STATE_DISABLED);
    lv_obj_add_state(btn_scan_from_dec, LV_STATE_DISABLED);
    lv_obj_add_state(btn_scan_to_inc, LV_STATE_DISABLED);
    lv_obj_add_state(btn_scan_to_dec, LV_STATE_DISABLED);
    lv_label_set_text(lbl_scan_back, LV_SYMBOL_STOP);
  }
  if (scanState == SCAN_HOLD) {
    lv_obj_clear_state(btn_scan_skip, LV_STATE_DISABLED);
  }
  else {
    lv_obj_add_state(btn_scan_skip, LV_STATE_DISABLED);
  }
  if (scanState == SCAN_IDLE or scanState == SCAN_HOLD) {
    lv_obj_clear_state(btn_scan_start, LV_STATE_DISABLED);
  }
  else {
    lv_obj_add_state(btn_scan_start, LV_STATE_DISABLED);
  }
}

// power screen

static void event_power_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    master_power = lv_slider_get_value(obj);
    set_power(master_power);
  }
}

void set_power(int8_t newpower) {
  master_out_message_power_setting = newpower;
}

static void event_power_save(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    master_settings_power = master_power;
    master_flag_save_presets = true;
  }
}

// settings screen

static void event_settings_from_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    if (lv_spinbox_get_value(spinbox_settings_from) > lv_spinbox_get_value(spinbox_settings_to) - 20) {
      if (lv_spinbox_get_value(spinbox_settings_from) + 20 == 0) {
        lv_spinbox_set_value(spinbox_settings_to, lv_spinbox_get_value(spinbox_settings_from) + 21);
      }
      else {
        lv_spinbox_set_value(spinbox_settings_to, lv_spinbox_get_value(spinbox_settings_from) + 20);
      }
    }
    master_settings_lower_limit = lv_spinbox_get_value(spinbox_settings_from);
    master_flag_save_presets = true;
  }
}

static void event_settings_to_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    if (lv_spinbox_get_value(spinbox_settings_to) < lv_spinbox_get_value(spinbox_settings_from) + 20) {
      if (lv_spinbox_get_value(spinbox_settings_to) - 20 == 0) {
        lv_spinbox_set_value(spinbox_settings_from, lv_spinbox_get_value(spinbox_settings_to) - 21);
      }
      else {
        lv_spinbox_set_value(spinbox_settings_from, lv_spinbox_get_value(spinbox_settings_to) - 20);
      }
    }
    master_settings_upper_limit = lv_spinbox_get_value(spinbox_settings_to);
    master_flag_save_presets = true;
  }
}

void increment_settings_from_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    my_spinbox_inc(spinbox_settings_from);
  }
}

static void decrement_settings_from_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    my_spinbox_dec(spinbox_settings_from);
  }
}

void increment_settings_to_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    my_spinbox_inc(spinbox_settings_to);
  }
}

static void decrement_settings_to_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    my_spinbox_dec(spinbox_settings_to);
  }
}

static void event_settings_back(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    cleanup();
  }
}

static void event_settings_range_back(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    // check if range has changed
    if (master_lower_channel != master_settings_lower_limit or master_upper_channel != master_settings_upper_limit) {
      master_lower_channel = master_settings_lower_limit;
      master_upper_channel = master_settings_upper_limit;
      master_lower_frequency = convert_channel_to_frequency(master_lower_channel);
      master_upper_frequency = convert_channel_to_frequency(master_upper_channel);
      goto_preset(master_settings_default_channel, false);
    }
    lv_tabview_set_act(tabview, TAB_SETTINGS_REF, LV_ANIM_OFF);
  }
}

static void event_settings_calibrate_back(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (master_settings_cal_s != master_settings_cal_s_previous or master_settings_cal_pwr != master_settings_cal_pwr_previous) {
      master_flag_save_presets = true;
    }
    lv_tabview_set_act(tabview, TAB_SETTINGS_REF, LV_ANIM_OFF);
  }
}

static void event_cal_s_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    master_settings_cal_s  = lv_slider_get_value(obj);
  }
}

static void event_cal_pwr_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    master_settings_cal_pwr  = lv_slider_get_value(obj);
  }
}

// presets

void set_preset_labels(void) {
  snprintf(printbuf, sizeof(printbuf), "%4d", master_settings_default_channel);
  lv_label_set_text(lbl_presets_default, printbuf);

  snprintf(printbuf, sizeof(printbuf), "%4d", master_settings_preset1);
  lv_label_set_text(lbl_presets_1, printbuf);

  snprintf(printbuf, sizeof(printbuf), "%4d", master_settings_preset2);
  lv_label_set_text(lbl_presets_2, printbuf);

  snprintf(printbuf, sizeof(printbuf), "%4d", master_settings_preset3);
  lv_label_set_text(lbl_presets_3, printbuf);

  snprintf(printbuf, sizeof(printbuf), "%4d", master_settings_preset4);
  lv_label_set_text(lbl_presets_4, printbuf);

  snprintf(master_preset_texts[0], sizeof(master_preset_texts[0]), "STD %4d", master_settings_default_channel);
  snprintf(master_preset_texts[1], sizeof(master_preset_texts[1]), "P1 %4d", master_settings_preset1);
  snprintf(master_preset_texts[2], sizeof(master_preset_texts[2]), "P2 %4d", master_settings_preset2);
  snprintf(master_preset_texts[3], sizeof(master_preset_texts[3]), "P3 %4d", master_settings_preset3);
  snprintf(master_preset_texts[4], sizeof(master_preset_texts[4]), "P4 %4d", master_settings_preset4);
}

static void event_presets_save_default(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    master_settings_default_channel = master_channel;
    set_preset_labels();
    master_flag_save_presets = true;
  }
}

static void event_presets_save_1(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    master_settings_preset1 = master_channel;
    set_preset_labels();
    master_flag_save_presets = true;
  }
}

static void event_presets_save_2(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    master_settings_preset2 = master_channel;
    set_preset_labels();
    master_flag_save_presets = true;
  }
}

static void event_presets_save_3(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    master_settings_preset3 = master_channel;
    set_preset_labels();
    master_flag_save_presets = true;
  }
}

static void event_presets_save_4(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    master_settings_preset4 = master_channel;
    set_preset_labels();
    master_flag_save_presets = true;
  }
}

static void event_presets_goto_default(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    goto_preset(master_settings_default_channel, true);
  }
}

static void event_presets_goto_preset1(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    goto_preset(master_settings_preset1, true);
  }
}

static void event_presets_goto_preset2(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    goto_preset(master_settings_preset2, true);
  }
}

static void event_presets_goto_preset3(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    goto_preset(master_settings_preset3, true);
  }
}

static void event_presets_goto_preset4(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    goto_preset(master_settings_preset4, true);
  }
}

void goto_preset(int8_t newchannel, boolean do_cleanup) {
  // only if within set limits
  if (newchannel >= master_settings_lower_limit and newchannel <= master_settings_upper_limit) {
  }
  else {
    newchannel = 1;
  }
  master_channel = newchannel;
  master_frequency = convert_channel_to_frequency(master_channel);
  master_previous_channel = master_channel;
  master_previous_frequency = master_frequency;
  update_channel();
  update_frequency();
  // calculate new pll setting, this will be transferred to the slave automatically
  master_out_message_pll_setting = calculate_PLL(master_frequency);
  if (do_cleanup) {
    cleanup();
  }
}

static void event_presets_back(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    cleanup();
  }
}

// save preferences
void save_presets(void) {
  preferences.begin("presets", false);
  preferences.putUChar("version", 1); // Mark this set as valid

  preferences.putChar("def_chan", master_settings_default_channel);
  preferences.putChar("lim_low", master_settings_lower_limit);
  preferences.putChar("lim_high", master_settings_upper_limit);
  preferences.putChar("prst_1", master_settings_preset1);
  preferences.putChar("prst_2", master_settings_preset2);
  preferences.putChar("prst_3", master_settings_preset3);
  preferences.putChar("prst_4", master_settings_preset4);
  preferences.putChar("power", master_settings_power);

  preferences.putChar("cal_s", master_settings_cal_s);
  preferences.putChar("cal_pwr", master_settings_cal_pwr);

  preferences.putString("ssid", master_settings_wifi_ssid);
  preferences.putString("pass", master_settings_wifi_password);

  preferences.end();
}

// load prefs from set 0-5
bool load_presets(void) {
  preferences.begin("presets", true);
  uint8_t version = preferences.getUChar("version", 0);
  if (version != 1) {
    preferences.end();
    return false;
  }

  master_settings_default_channel = preferences.getChar("def_chan", 18);
  master_settings_lower_limit = preferences.getChar("lim_low", -120);
  master_settings_upper_limit = preferences.getChar("lim_high", 120);
  master_settings_preset1 = preferences.getChar("prst_1", 18);
  master_settings_preset2 = preferences.getChar("prst_2", 18);
  master_settings_preset3 = preferences.getChar("prst_3", 18);
  master_settings_preset4 = preferences.getChar("prst_4", 18);
  master_settings_power = preferences.getChar("power", 73);

  master_settings_cal_s = preferences.getChar("cal_s", 50);
  master_settings_cal_pwr = preferences.getChar("cal_pwr", 50);

  master_settings_wifi_ssid = preferences.getString("ssid", "---");
  master_settings_wifi_password = preferences.getString("pass", "---");

  preferences.end();

  return true;
}

// set all relevant objects from preferences
void apply_presets(void) {
  if (SHOW_DEBUG) {
    Serial.print("Lower limit set at ");
    Serial.println(master_settings_lower_limit);
    Serial.print("Upper limit set at ");
    Serial.println(master_settings_upper_limit);
    Serial.print("Default channel set at ");
    Serial.println(master_settings_default_channel);
    Serial.print("Power set at ");
    Serial.println(master_settings_power);
  }

  master_lower_channel = master_settings_lower_limit;
  master_upper_channel = master_settings_upper_limit;
  master_lower_frequency = convert_channel_to_frequency(master_lower_channel);
  master_upper_frequency = convert_channel_to_frequency(master_upper_channel);

  goto_preset(master_settings_default_channel, true);

  set_preset_labels();

  lv_slider_set_value(slider_power, master_settings_power, LV_ANIM_OFF);
  if (master_out_message_power_setting != master_settings_power) {
    master_power = master_settings_power;
    set_power(master_power);
  }

  lv_slider_set_value(slider_settings_s_cal, master_settings_cal_s, LV_ANIM_OFF);

  lv_slider_set_value(slider_settings_power_cal, master_settings_cal_pwr, LV_ANIM_OFF);

}

static void event_calibrate(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    calibrate_screen();
    lv_tabview_set_act(tabview, TAB_MAIN_REF, LV_ANIM_OFF);
    // force redraw
    master_screen_update_done = false;
    master_screen_update_counter = 0;
  }
}

void calibrate_screen() {
  TFT.fillScreen(TFT_BLACK);
  // clear this namespace
  preferences.begin("calibration", false);
  preferences.clear();
  preferences.end();
  TFT.calibrateTouch(setting_calibration_data, TFT_WHITE, TFT_RED, 14);
  save_calibration();
  TFT.setTouchCalibrate(setting_calibration_data);
}

void save_calibration() {
  preferences.begin("calibration", false);
  preferences.putBytes("calib", setting_calibration_data, sizeof(setting_calibration_data));
  preferences.end();
}

bool load_calibration() {
  preferences.begin("calibration", true);
  size_t len = preferences.getBytes("calib", setting_calibration_data, sizeof(setting_calibration_data));
  preferences.end();
  if (len != sizeof(setting_calibration_data)) {
    if (SHOW_DEBUG) {
      Serial.println("No valid calibration data found.");
    }
    return false;
  }
  // check if all 0
  bool allzero = true;
  for (int i = 0; i < 8; i++) {
    if (setting_calibration_data[i] != 0) {
      allzero = false;
      break;
    }
  }
  if (allzero) {
    if (SHOW_DEBUG) {
      Serial.println("Calibration data empty.");
    }
    return false;
  }
  if (SHOW_DEBUG) {
    Serial.println("Calibration data loaded:");
    for (int i = 0; i < 8; i++) {
      Serial.printf("  %d: %d\n", i, setting_calibration_data[i]);
    }
  }
  return true;
}

// reset all calibration settings and save
void reset_calibration() {
  preferences.begin("calibration", false);
  preferences.clear();
  clear_calibration();
  save_calibration();
  preferences.end();
}

// reset all calibration settings to default
void clear_calibration() {
  for (int i = 0; i < 8; i++) {
    setting_calibration_data[i] = 0;
  }
}


// rs485

// all rs485 communication is done in this task
// master sends out a frame to the slave and waits for a reply, if no reply within the defined period, repeat
// since we need a constant data flow for the S-meter all parameters are sent every time so the code has to check for differences and act
void rs485_task(void *pvParameters) {
  // frame parameters, 4 bytes of data just in case
  // total frame time ca 1 msec
  uint8_t src, dst, cmd; uint32_t data;
  // trigger next transmit by timer
  unsigned long transmit_timer = millis();
  // trigger next transmit by slave
  boolean trigger_next_round = true;

  unsigned long slave_detected = millis();

  for (;;) {
    // sending a frame to the slave will trigger a response from the slave after a short delay, make sure we dont send a new frame within that period
    if (trigger_next_round or millis() - transmit_timer > 30) {
      // prepare frame
      uint32_t masterdata = encode_message();
      rs485_send_frame(MASTER_ADDR, SLAVE_ADDR, 0x10, masterdata);
      // turn off trigger
      trigger_next_round = false;
      // make sure the frame is gone
      vTaskDelay(5);
      // start a new round
      transmit_timer = millis();
    }

    // see if we have received a frame from the slave
    if (rs485_try_receive_frame(src, dst, cmd, data)) {
      // keep old pll code
      uint8_t previous_pll = master_in_message_pll_switch;
      if (src == SLAVE_ADDR && dst == MASTER_ADDR) {
        // incoming message
        if (cmd == 0x90) {
          master_slave_is_connected_flag = true;
          slave_detected = millis();
          // split data
          decode_message(data);
          // actions
          if (previous_pll != 0 and previous_pll != master_in_message_pll_switch and lv_tabview_get_tab_act(tabview) == TAB_MAIN_REF) {
            // check if we have movement
            int pll_dif = channel_difference(previous_pll, master_in_message_pll_switch);
            // this will work in channel and frequency mode
            if (pll_dif != 0) {
              xSemaphoreTake(parameter_mutex, portMAX_DELAY);
              EncoderCounter1 += pll_dif;
              xSemaphoreGive(parameter_mutex);
            }
          }
          master_new_smeter_value = read_cb_smeter_linear(master_in_message_s_meter);

          vTaskDelay(4);
          // send a new frame
          trigger_next_round = true;
        }
      }
    }
    // we lost connection to the slave
    if (millis() - slave_detected > 500 and master_slave_is_connected_flag) {
      master_in_message_pll_locked = false;
      master_in_message_s_meter = 0;
      master_in_message_tx_on = false;
      master_in_message_squelch_open = false;
      master_slave_is_connected_flag = false;
    }
    // prevent watchdog error triggers
    vTaskDelay(1);
  }
}

// decode incoming message into individual parameters
void decode_message(uint32_t msgdata) {
  master_in_message_pll_switch = (uint8_t) msgdata & 0x3F;
  master_in_message_s_meter = (uint16_t) (msgdata >> 8) & 0x0FFF;
  master_in_message_pll_locked = (msgdata >> 24) & 0x01;
  master_in_message_squelch_open = (msgdata >> 25) & 0x01;
  master_in_message_tx_on = (msgdata >> 26) & 0x01;
  master_in_message_flags = (msgdata >> 27) & 0xFF;
}

// encode parameters into one message var
uint32_t encode_message(void) {
  uint32_t newdata = ((uint32_t) master_out_message_pll_setting & 0x01FF
                      | (uint32_t) master_out_message_power_setting << 16
                      | (uint32_t) master_out_message_flags << 24);
  return newdata;
}

// setup rs485 port
void rs485_setup() {
  pinMode(RS485_RE_DE, OUTPUT);
  digitalWrite(RS485_RE_DE, LOW);
  RS485.begin(115200, SERIAL_8N1, RS485_RO, RS485_DI);
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

// send rs485 frame to slave
void rs485_send_frame(uint8_t src, uint8_t dst, uint8_t cmd, uint32_t data) {
  uint8_t buf[10];
  buf[0] = 0xAA;
  buf[1] = src;
  buf[2] = dst;
  buf[3] = cmd;
  // lsb first
  buf[4] = data & 0xFF;
  buf[5] = data >> 8;
  buf[6] = data >> 16;
  buf[7] = data >> 24;
  buf[8] = crc8(&buf[1], 7);
  buf[9] = 0x55;
  digitalWrite(RS485_RE_DE, HIGH);
  delayMicroseconds(20);
  RS485.write(buf, 10);
  RS485.flush();
  delayMicroseconds(50);
  digitalWrite(RS485_RE_DE, LOW);
}

// check if we are receiving data and process frame, non blocking
bool rs485_try_receive_frame(uint8_t &src, uint8_t &dst, uint8_t &cmd, uint32_t &data) {
  static uint8_t buf[10];
  static uint8_t idx = 0;
  while (RS485.available()) {
    uint8_t b = RS485.read();
    if (idx == 0) {
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

// send websocket response
void notifyClients() {
  if (master_allow_web) {
    master_web_channel = master_channel;
    master_web_frequency = master_frequency;
    master_web_smeter_value = master_smeter_value;
    master_web_channel_is_alfa = master_channel_is_alfa;
    master_web_tx_on = master_tx_on;

    // bounds check
    if (master_web_channel < -120) master_web_channel = -120;
    if (master_web_channel > 120) master_web_channel = 120;

    char buffer[512];
    char channel_str[12];
    char freq_str[8];

    // format channel
    if (master_web_channel_is_alfa) {
      // alfa channel
      snprintf(channel_str, sizeof(channel_str), "%dA", master_web_channel);
    } else {
      snprintf(channel_str, sizeof(channel_str), "%d", master_web_channel);
    }

    // format frequency
    snprintf(freq_str, sizeof(freq_str), "%.3f", master_web_frequency / 1000.0);

    const char* string_upper;
    const char* string_lower;

    if (master_channel_mode == true) {
      // true = channel up, freq below
      string_upper = channel_str;
      string_lower = freq_str;
    } else {
      // false = freq up, channel below
      string_upper = freq_str;
      string_lower = channel_str;
    }

    float temp_smeter_value = master_web_smeter_value;

    if (master_web_tx_on) {
      temp_smeter_value = 0;
    }

    snprintf(buffer, sizeof(buffer),
             "{\"s\":%.1f,\"boven\":\"%s\",\"onder\":\"%s\",\"presets\":[\"%s\",\"%s\",\"%s\",\"%s\",\"%s\"]}",
             temp_smeter_value,
             string_upper,
             string_lower,
             master_preset_texts[0], // Preset 0 tekst
             master_preset_texts[1], // Preset 1 tekst
             master_preset_texts[2], // Preset 2 tekst
             master_preset_texts[3], // Preset 3 tekst
             master_preset_texts[4]  // Preset 4 tekst
            );
    ws.textAll(buffer);
  }
}

// websocket activity
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (SHOW_DEBUG) {
    switch (type) {
      case WS_EVT_CONNECT:
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        break;
      case WS_EVT_DISCONNECT:
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
        break;
      case WS_EVT_ERROR:
        Serial.printf("WebSocket client #%u error\n", client->id());
        break;
    }
  }
  if (master_allow_web) {
    if (SHOW_DEBUG) {
      Serial.printf("WebSocket event: ");
      if (type == WS_EVT_CONNECT) {
        Serial.printf("CLIENT CONNECTED - Client #%u from %s\n", client->id(), client->remoteIP().toString().c_str());
      }
      else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("CLIENT DISCONNECTED - Client #%u\n", client->id());
      }
      else if (type == WS_EVT_DATA) {
        Serial.printf("DATA RECEIVED - Client #%u, Length: %d bytes\n", client->id(), len);
      }
      else if (type == WS_EVT_PONG) {
        Serial.printf("PONG RECEIVED - Client #%u\n", client->id());
      }
      else if (type == WS_EVT_ERROR) {
        Serial.printf("ERROR - Client #%u: %s\n", client->id(), (char*)arg);
      }
      else {
        Serial.printf("UNKNOWN EVENT TYPE: %d\n", type);
      }
    }
    if (type == WS_EVT_DISCONNECT) {
      if (SHOW_DEBUG) {
        Serial.printf("🔌 WebSocket client #%u disconnected\n", client->id());
      }
      delay(100);
      ws.cleanupClients();
    }

    if (type == WS_EVT_CONNECT) {
      if (SHOW_DEBUG) {
        Serial.println("WebSocket connected");
      }
      notifyClients();
    }
    // ignore if transmitting
    else if (type == WS_EVT_DATA and !master_tx_on) {
      AwsFrameInfo *info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        String msg = String((char*)data).substring(0, len);
        if (SHOW_DEBUG) {
          Serial.println("received: " + msg);
        }
        if (msg == "CHAN_UP") {
          if (master_channel_mode) {
            // channel mode
            if (master_channel < master_upper_channel) {
              // intercept ch 0
              if (master_channel == -1) {
                master_channel = 0;
              }
              master_channel++;
            } else if (master_channel == master_upper_channel) {
              master_channel = master_lower_channel;
            }
          }
          else {
            // frequency mode
            if (master_frequency < master_upper_frequency) {
              master_frequency = master_frequency + master_stepsize;
            } else if (master_frequency == master_upper_frequency) {
              master_frequency = master_lower_frequency;
            }
          }
          master_channel_frequency_refresh = true;
        }
        else if (msg == "CHAN_DOWN") {
          if (master_channel_mode) {
            // channel mode
            if (master_channel > master_lower_channel) {
              // intercept ch 0
              if (master_channel == 1) {
                master_channel = 0;
              }
              master_channel--;
            } else if (master_channel == master_lower_channel) {
              master_channel = master_upper_channel;
            }
          }
          else {
            // frequency mode
            if (master_frequency > master_lower_frequency) {
              master_frequency = master_frequency - master_stepsize;
            } else if (master_frequency == master_lower_frequency) {
              master_frequency = master_upper_frequency;
            }
          }
          master_channel_frequency_refresh = true;
        }
        else if (msg == "PRESET_0") {
          goto_preset(master_settings_default_channel, false);
        }
        else if (msg == "PRESET_1") {
          goto_preset(master_settings_preset1, false);
        }
        else if (msg == "PRESET_2") {
          goto_preset(master_settings_preset2, false);
        }
        else if (msg == "PRESET_3") {
          goto_preset(master_settings_preset3, false);
        }
        else if (msg == "PRESET_4") {
          goto_preset(master_settings_preset4, false);
        }
        else if (msg == "TOGGLE_MODE") {
          toggle_channel_freq();
        }
        notifyClients();
      }
    }
  }
}


// debug routine in case of problems
// alternative enable monitoring in lvgl
void print_memory_info_to_serial() {
  // Gather memory info
  size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
  size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
  size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  size_t total_psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
  size_t stack_free = uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t);

  // Print memory information to the serial monitor
  Serial.println("=== Memory Info ===");
  Serial.printf("Heap: %u / %u bytes free\n", (unsigned int)free_heap, (unsigned int)total_heap);

  // If PSRAM is available, display its info
  if (total_psram > 0) {
    Serial.printf("PSRAM: %u / %u bytes free\n", (unsigned int)free_psram, (unsigned int)total_psram);
  } else {
    Serial.println("PSRAM: Not available");
  }

  // Display stack memory info
  Serial.printf("Stack: %u bytes free\n", (unsigned int)stack_free);
  Serial.println("====================");

  // Free DRAM (internal RAM where `dram0_0_seg` is located)
  size_t free_dram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  size_t largest_free_block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  size_t total_dram = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);

  Serial.printf("Total DRAM: %d bytes\n", total_dram);
  Serial.printf("Free DRAM: %d bytes\n", free_dram);
  Serial.printf("Largest Free Block in DRAM: %d bytes\n", largest_free_block);

  lv_mem_monitor_t mon;
  lv_mem_monitor(&mon);

  Serial.printf("mon.total_size: %d \n", mon.total_size);
  Serial.printf("mon.free_cnt: %d \n", mon.free_cnt);
  Serial.printf("mon.free_size: %d \n", mon.free_size);
  Serial.printf("mon.free_biggest_size: %d \n", mon.free_biggest_size);
  Serial.printf("mon.used_cnt: %d \n", mon.used_cnt);
  Serial.printf("mon.max_used: %d \n", mon.max_used);
  Serial.printf("mon.used_pct: %d \n", mon.used_pct);
  Serial.printf("mon.frag_pct: %d \n", mon.frag_pct);

}


// setup is moved here because index page messes up the compilation
void setup() {

  //  if (SHOW_DEBUG) {
  Serial.begin(115200);
  // }
  pinMode(RS485_DI, OUTPUT);
  pinMode(RS485_RO, INPUT);
  pinMode(RS485_RE_DE, OUTPUT);

  pinMode(Encoder_1_Pin1, INPUT);
  pinMode(Encoder_1_Pin2, INPUT);
  pinMode(Encoder_2_Pin1, INPUT);
  pinMode(Encoder_2_Pin2, INPUT);
  pinMode(Encoder_1_Key, INPUT);
  pinMode(Encoder_2_Key, INPUT);

  parameter_mutex = xSemaphoreCreateMutex();

  // get current state to start otherwise encoder might not react to first click
  unsigned char temppinstate1 = (digitalRead(Encoder_1_Pin2) << 1) | digitalRead(Encoder_1_Pin1);
  Encoder_1_State = ttable[Encoder_1_State & 0xf][temppinstate1];

  unsigned char temppinstate2 = (digitalRead(Encoder_2_Pin2) << 1) | digitalRead(Encoder_2_Pin1);
  Encoder_2_State = ttable[Encoder_2_State & 0xf][temppinstate2];

  attachInterrupt(Encoder_1_Pin1, isr1, CHANGE);
  attachInterrupt(Encoder_1_Pin2, isr1, CHANGE);
  attachInterrupt(Encoder_2_Pin1, isr2, CHANGE);
  attachInterrupt(Encoder_2_Pin2, isr2, CHANGE);

  TFT.init();
  TFT.setRotation(3);
  TFT.fillScreen(TFT_BLACK);

  lv_init();

  // set a tick source so that LVGL will know how much time elapsed
  lv_tick_set_cb(my_tick);

  // setup the two buffers
  draw_buf_1 = heap_caps_malloc(DRAW_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  draw_buf_2 = heap_caps_malloc(DRAW_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

  disp = lv_display_create(TFT_HOR_RES, TFT_VER_RES);

  lv_display_set_flush_cb(disp, my_disp_flush);
  lv_display_set_buffers(disp, draw_buf_1, draw_buf_2, DRAW_BUF_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);

  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, my_touchpad_read);

  // mutex for i2s display
  spi_mutex_lcd = xSemaphoreCreateMutex();

  responseSemaphore = xSemaphoreCreateMutex();

  clear_calibration();

  display_splash_screen();

  boolean forcecalibrate = false;
  // check if both buttons is pressed during boot
  if ((digitalRead(Encoder_1_Key) + digitalRead(Encoder_2_Key)) == 0) {
    // make sure its not a power on glitch due to capacitor charging on the encoder module
    delay(500);
    if ((digitalRead(Encoder_1_Key) + digitalRead(Encoder_2_Key)) == 0) {
      // call the calibration screen if one and only one of the encoder switches is pressed during power on
      forcecalibrate = true;
    }
  }
  if (!load_calibration() or forcecalibrate) {
    calibrate_screen();
  } else {
    TFT.setTouchCalibrate(setting_calibration_data);
  }

  // check if top encoder pressed during boot - enable wifi
  if ((digitalRead(Encoder_1_Key) == 0 and digitalRead(Encoder_2_Key)) == 1) {
    // make sure its not a power on glitch due to capacitor charging on the encoder module
    delay(500);
    if ((digitalRead(Encoder_1_Key) == 0 and digitalRead(Encoder_2_Key)) == 1) {
      wifi_state = WIFI_STATE_ENABLED;
    }
  }

  delay(2000);

  if (!load_presets()) {
    save_presets();
    load_presets();
  }

  if (wifi_state == WIFI_STATE_ENABLED) {
    // check if we have a login
    if (master_settings_wifi_ssid.length() == 0 || master_settings_wifi_password.length() == 0) {
      wifi_state = WIFI_STATE_ENABLED_FAILED_LOGIN;
      TFT.fillScreen(TFT_BLACK);
      TFT.setTextColor(TFT_RED, TFT_BLACK);
      TFT.setTextDatum(MC_DATUM);
      TFT.setTextSize(3);
      TFT.drawString("WiFi login invalid", TFT.width() / 2, TFT.height() / 2);
      delay(3000);
      wifi_state = WIFI_STATE_DISABLED;
    }
    else {
      TFT.fillScreen(TFT_BLACK);
      TFT.setTextColor(TFT_GREEN, TFT_BLACK);
      TFT.setTextDatum(MC_DATUM);
      TFT.setTextSize(3);
      TFT.drawString("Connecting to WiFi...", TFT.width() / 2, TFT.height() / 2);
      WiFi.mode(WIFI_STA);
      WiFi.setSleep(false);
      delay(100);
      WiFi.begin(master_settings_wifi_ssid.c_str(), master_settings_wifi_password.c_str());

      unsigned long connect_timer = millis();
      unsigned long connect_wait_timer = millis();

      while (WiFi.status() != WL_CONNECTED and millis() - connect_timer < 20000) {
        if (millis() - connect_wait_timer > 500) {
          if (SHOW_DEBUG) {
            Serial.print(".");
          }
          connect_wait_timer = millis();
        }
      }

      if (WiFi.status() == WL_CONNECTED) {
        if (SHOW_DEBUG) {
          Serial.println("\n✅ WiFi connected!");
          Serial.println("IP: " + WiFi.localIP().toString());
        }
        // Start webserver
        ws.onEvent(onWsEvent);
        webserver.addHandler(&ws);

        webserver.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
          if (master_allow_web) {
            if (xSemaphoreTake(responseSemaphore, portMAX_DELAY) == pdTRUE) {
              request->send_P(200, "text/html", index_html);
              xSemaphoreGive(responseSemaphore);
            }
          }
          else {
            if (xSemaphoreTake(responseSemaphore, portMAX_DELAY) == pdTRUE) {
              request->send_P(200, "text/plain", "Go to main screen to enable web contents.");
              xSemaphoreGive(responseSemaphore);
            }
          }
        });

        webserver.on("/test", HTTP_GET, [](AsyncWebServerRequest * request) {
          if (xSemaphoreTake(responseSemaphore, portMAX_DELAY) == pdTRUE) {
            if (SHOW_DEBUG) {
              //              Serial.println("/test handler called");
              //              Serial.printf("Free heap before send: %d\n", ESP.getFreeHeap());
              //              print_memory_info_to_serial();
            }
            request->send_P(200, "text/plain", "Test OK");
            if (SHOW_DEBUG) {
              Serial.println("ok sent");
            }
            xSemaphoreGive(responseSemaphore);
          }
        });

        // Start server
        webserver.begin();
        if (SHOW_DEBUG) {
          Serial.println("Webserver started on port 80");
          Serial.println("\n✅ WiFi verbonden!");
          Serial.print("📡 IP Address: ");
          Serial.println(WiFi.localIP());
          Serial.print("📶 Signal Strength (RSSI): ");
          Serial.println(WiFi.RSSI());
          Serial.print("🌐 Subnet Mask: ");
          Serial.println(WiFi.subnetMask());
          Serial.print("🔌 Gateway: ");
          Serial.println(WiFi.gatewayIP());
          Serial.println(ESP.getFreeHeap());
        }
        wifi_state = WIFI_STATE_ENABLED_CONNECTED;
      }
      else {
        wifi_state = WIFI_STATE_ENABLED_FAILED_CONNECT;
        TFT.fillScreen(TFT_BLACK);
        TFT.setTextColor(TFT_RED, TFT_BLACK);
        TFT.setTextDatum(MC_DATUM);
        TFT.setTextSize(3);
        TFT.drawString("WiFi login failed", TFT.width() / 2, TFT.height() / 2);
        delay(3000);
        wifi_state = WIFI_STATE_DISABLED;
      }
    }
  }

  setup_screens();
  set_channel_frequency_mode(master_channel_mode);
  show_squelch_status(0);
  apply_presets();
  smeter_sprite.setColorDepth(TFT.getColorDepth());
  smeter_sprite.createSprite(master_smeter_w - 2, master_smeter_h - 2);
  // set here because the setup will trigger the filling of the parameters from the spinboxes
  lv_spinbox_set_value(spinbox_scan_from, 1);
  lv_spinbox_set_value(spinbox_scan_to, 40);
  rs485_setup();
  // all rs485 tasks are handled in this vtask
  xTaskCreatePinnedToCore(rs485_task, "rs485_task", 4096, NULL, 1, NULL, 0);
  LVGL_Timer = millis() - 200;
  wifi_timer = millis();
  if (SHOW_DEBUG) {
    print_memory_info_to_serial();
  }
  show_wifi_status();
}
