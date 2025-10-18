#include <lvgl.h>
#include "esp_lcd_touch_axs5106l.h"
#include <Wire.h>

// #include <examples/lv_examples.h>
#include <demos/lv_demos.h>

// #define DIRECT_RENDER_MODE

#include <Arduino_GFX_Library.h>

#define ROTATION 0
// #define ROTATION 1
// #define ROTATION 2
// #define ROTATION 3

#define GFX_BL 46

#define Touch_I2C_SDA 42
#define Touch_I2C_SCL 41
#define Touch_RST     47
#define Touch_INT     48

#define MPS_I2C_SDA 9
#define MPS_I2C_SCL 8
#define MPS_I2C_ADDR 0x40

#define LEDC_FREQ             5000
#define LEDC_TIMER_10_BIT     10

void lvgl_power_control_ui_init(lv_obj_t *parent);

TwoWire mpsWire = TwoWire(0);

Arduino_DataBus *bus = new Arduino_ESP32SPI(45, 21, 38, 39);

Arduino_GFX *gfx = new Arduino_ST7789(
  bus, 40, 0, false,
  172, 320,
  34, 0,
  34, 0);

const float voltage_presets[] = {1.8, 3.3, 5.0, 12.0};
const char* voltage_labels[] = {"1.8V", "3.3V", "5.0V", "12.0V"};
const int preset_count = 4;

float current_voltage = 1.8;
float current_current = 0.0;
int current_preset = 0;

void init_i2c();
bool write_register(uint8_t addr, uint8_t reg, uint16_t value);
bool read_register(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t length);
bool set_output_voltage(float voltage);
float read_output_voltage();
float read_output_current();

void lcd_reg_init(void) {
  static const uint8_t init_operations[] = {
    BEGIN_WRITE,
    WRITE_COMMAND_8, 0x11,
    END_WRITE,
    DELAY, 120,

    BEGIN_WRITE,
    WRITE_C8_D16, 0xDF, 0x98, 0x53,
    WRITE_C8_D8, 0xB2, 0x23, 

    WRITE_COMMAND_8, 0xB7,
    WRITE_BYTES, 4,
    0x00, 0x47, 0x00, 0x6F,

    WRITE_COMMAND_8, 0xBB,
    WRITE_BYTES, 6,
    0x1C, 0x1A, 0x55, 0x73, 0x63, 0xF0,

    WRITE_C8_D16, 0xC0, 0x44, 0xA4,
    WRITE_C8_D8, 0xC1, 0x16, 

    WRITE_COMMAND_8, 0xC3,
    WRITE_BYTES, 8,
    0x7D, 0x07, 0x14, 0x06, 0xCF, 0x71, 0x72, 0x77,

    WRITE_COMMAND_8, 0xC4,
    WRITE_BYTES, 12,
    0x00, 0x00, 0xA0, 0x79, 0x0B, 0x0A, 0x16, 0x79, 0x0B, 0x0A, 0x16, 0x82,

    WRITE_COMMAND_8, 0xC8,
    WRITE_BYTES, 32,
    0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00, 0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00,

    WRITE_COMMAND_8, 0xD0,
    WRITE_BYTES, 5,
    0x04, 0x06, 0x6B, 0x0F, 0x00,

    WRITE_C8_D16, 0xD7, 0x00, 0x30,
    WRITE_C8_D8, 0xE6, 0x14, 
    WRITE_C8_D8, 0xDE, 0x01, 

    WRITE_COMMAND_8, 0xB7,
    WRITE_BYTES, 5,
    0x03, 0x13, 0xEF, 0x35, 0x35,

    WRITE_COMMAND_8, 0xC1,
    WRITE_BYTES, 3,
    0x14, 0x15, 0xC0,

    WRITE_C8_D16, 0xC2, 0x06, 0x3A,
    WRITE_C8_D16, 0xC4, 0x72, 0x12,
    WRITE_C8_D8, 0xBE, 0x00, 
    WRITE_C8_D8, 0xDE, 0x02, 

    WRITE_COMMAND_8, 0xE5,
    WRITE_BYTES, 3,
    0x00, 0x02, 0x00,

    WRITE_COMMAND_8, 0xE5,
    WRITE_BYTES, 3,
    0x01, 0x02, 0x00,

    WRITE_C8_D8, 0xDE, 0x00, 
    WRITE_C8_D8, 0x35, 0x00, 
    WRITE_C8_D8, 0x3A, 0x05, 

    WRITE_COMMAND_8, 0x2A,
    WRITE_BYTES, 4,
    0x00, 0x22, 0x00, 0xCD,

    WRITE_COMMAND_8, 0x2B,
    WRITE_BYTES, 4,
    0x00, 0x00, 0x01, 0x3F,

    WRITE_C8_D8, 0xDE, 0x02, 

    WRITE_COMMAND_8, 0xE5,
    WRITE_BYTES, 3,
    0x00, 0x02, 0x00,
    
    WRITE_C8_D8, 0xDE, 0x00, 
    WRITE_C8_D8, 0x36, 0x00,
    WRITE_COMMAND_8, 0x21,
    END_WRITE,
    
    DELAY, 10,

    BEGIN_WRITE,
    WRITE_COMMAND_8, 0x29,
    END_WRITE
  };
  bus->batchOperation(init_operations, sizeof(init_operations));
}

uint32_t screenWidth;
uint32_t screenHeight;
uint32_t bufSize;
lv_disp_draw_buf_t draw_buf;
lv_color_t *disp_draw_buf;
lv_disp_drv_t disp_drv;

#if LV_USE_LOG != 0
void my_print(const char *buf)
{
  Serial.printf(buf);
  Serial.flush();
}
#endif

void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
#ifndef DIRECT_RENDER_MODE
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif
#endif

  lv_disp_flush_ready(disp_drv);
}

void touchpad_read_cb(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
  touch_data_t touch_data;
  uint8_t touchpad_cnt = 0;

  bsp_touch_read();
  bool touchpad_pressed = bsp_touch_get_coordinates(&touch_data);

  if (touchpad_pressed) {
    data->point.x = touch_data.coords[0].x;
    data->point.y = touch_data.coords[0].y;
    data->state = LV_INDEV_STATE_PRESSED;
    Serial.printf("Touch detected: x=%d, y=%d\n", data->point.x, data->point.y);
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

void init_i2c() {
  mpsWire.begin(MPS_I2C_SDA, MPS_I2C_SCL);
  mpsWire.setClock(100000);
  Serial.println("MPS I2C initialized with separate bus");
}

bool write_register(uint8_t addr, uint8_t reg, uint16_t value) {
  mpsWire.beginTransmission(addr);
  mpsWire.write(reg);
  mpsWire.write((value >> 8) & 0xFF);
  mpsWire.write(value & 0xFF);
  return (mpsWire.endTransmission() == 0);
}

bool read_register(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t length) {
  mpsWire.beginTransmission(addr);
  mpsWire.write(reg);
  if (mpsWire.endTransmission(false) != 0) {
    return false;
  }
  
  mpsWire.requestFrom(addr, length);
  if (mpsWire.available() == length) {
    for (int i = 0; i < length; i++) {
      data[i] = mpsWire.read();
    }
    return true;
  }
  return false;
}

bool set_output_voltage(float voltage) {
  uint16_t voltage_code = (uint16_t)(voltage * 1000 / 1.25);
  
  bool success = write_register(MPS_I2C_ADDR, 0x21, voltage_code);
  if (success) {
    current_voltage = voltage;
    Serial.printf("Real: Set voltage to %.1fV\n", voltage);
  } else {
    Serial.println("Failed to set voltage via I2C");
  }
  return success;
}

float read_output_voltage() {
  uint8_t data[2];
  if (read_register(MPS_I2C_ADDR, 0x8B, data, 2)) {
    uint16_t voltage_code = (data[0] << 8) | data[1];
    return (voltage_code * 1.25) / 1000.0;
  }
  return 0.0;
}

float read_output_current() {
  uint8_t data[2];
  if (read_register(MPS_I2C_ADDR, 0x8C, data, 2)) {
    uint16_t current_code = (data[0] << 8) | data[1];
    return (current_code * 1.0) / 1000.0;
  }
  return 0.0;
}

void setup()
{
#ifdef DEV_DEVICE_INIT
  DEV_DEVICE_INIT();
#endif

  Serial.begin(115200);
  Serial.println("MPS Power Control");
  String LVGL_Arduino = String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
  Serial.println(LVGL_Arduino);

  init_i2c();

  set_output_voltage(current_voltage);

  if (!gfx->begin())
  {
    Serial.println("gfx->begin() failed!");
  }
  lcd_reg_init();
  gfx->setRotation(ROTATION);
  gfx->fillScreen(RGB565_BLACK);

#ifdef GFX_BL
  ledcAttach(GFX_BL , LEDC_FREQ, LEDC_TIMER_10_BIT);
  ledcWrite(GFX_BL , (1 << LEDC_TIMER_10_BIT) / 100 * 80);
#endif

  Wire.begin(Touch_I2C_SDA, Touch_I2C_SCL);
  bsp_touch_init(&Wire, Touch_RST, Touch_INT, gfx->getRotation(), gfx->width(), gfx->height());
  lv_init();

#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print);
#endif

  screenWidth = gfx->width();
  screenHeight = gfx->height();

#ifdef DIRECT_RENDER_MODE
  bufSize = screenWidth * screenHeight;
#else
  bufSize = screenWidth * 40;
#endif

#ifdef ESP32
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!disp_draw_buf)
  {
    disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);
  }
#endif

  if (!disp_draw_buf)
  {
    Serial.println("LVGL disp_draw_buf allocate failed!");
  }
  else
  {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, bufSize);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
#ifdef DIRECT_RENDER_MODE
    disp_drv.direct_mode = true;
#endif
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_read_cb;
    lv_indev_drv_register(&indev_drv);
    
    lvgl_power_control_ui_init(lv_scr_act());
  }

  Serial.println("Real I2C Control Setup done");
}

void loop()
{
  lv_timer_handler();

  static unsigned long last_update = 0;
  if (millis() - last_update > 500) {
    current_voltage = read_output_voltage();
    current_current = read_output_current();
    last_update = millis();
  }

#ifdef DIRECT_RENDER_MODE
#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#else
  gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#endif
#endif

  delay(5);
}

lv_obj_t *label_voltage;
lv_obj_t *label_preset;

void slider_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_VALUE_CHANGED)
    {
        lv_obj_t *slider = lv_event_get_target(e);
        int preset_index = lv_slider_get_value(slider);
        
        if (preset_index >= 0 && preset_index < preset_count) {
            float new_voltage = voltage_presets[preset_index];
            
            if (set_output_voltage(new_voltage)) {
                current_preset = preset_index;
                lv_label_set_text_fmt(label_preset, "Preset: %s", voltage_labels[preset_index]);
                Serial.printf("Real I2C: Set voltage to %.1fV\n", new_voltage);
            }
        }
        
        lv_event_stop_bubbling(e);
    }
}

void update_display_cb(lv_timer_t *timer) {
    char voltage_str[20];
    snprintf(voltage_str, sizeof(voltage_str), "Voltage: %.2f V", current_voltage);
    lv_label_set_text(label_voltage, voltage_str);
}

void lvgl_power_control_ui_init(lv_obj_t *parent)
{
    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_set_size(obj, lv_pct(90), lv_pct(60));
    lv_obj_align(obj, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_border_width(obj, 2, 0);
    lv_obj_set_style_border_color(obj, lv_color_hex(0x0099FF), 0);

    lv_obj_t *label_title = lv_label_create(obj);
    lv_label_set_text(label_title, "MPS Power Control");
    lv_obj_set_style_text_font(label_title, &lv_font_montserrat_14, 0);
    lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 10);

    lv_obj_t *slider = lv_slider_create(obj);
    lv_slider_set_range(slider, 0, preset_count - 1);
    lv_slider_set_value(slider, current_preset, LV_ANIM_OFF);
    lv_obj_set_size(slider, lv_pct(80), 20);
    lv_obj_align(slider, LV_ALIGN_CENTER, 0, -20);

    lv_obj_set_style_bg_color(slider, lv_color_hex(0x0099FF), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(slider, lv_color_hex(0x00FF00), LV_PART_KNOB);

    label_preset = lv_label_create(obj);
    lv_label_set_text_fmt(label_preset, "Preset: %s", voltage_labels[current_preset]);
    lv_obj_set_style_text_font(label_preset, &lv_font_montserrat_14, 0);
    lv_obj_align(label_preset, LV_ALIGN_CENTER, 0, 10);

    label_voltage = lv_label_create(obj);
    lv_label_set_text_fmt(label_voltage, "Voltage: %.2f V", current_voltage);
    lv_obj_set_style_text_font(label_voltage, &lv_font_montserrat_14, 0);
    lv_obj_align(label_voltage, LV_ALIGN_CENTER, 0, 40);

    lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_timer_create(update_display_cb, 500, NULL);

    lv_obj_clear_flag(obj, LV_OBJ_FLAG_GESTURE_BUBBLE);
}