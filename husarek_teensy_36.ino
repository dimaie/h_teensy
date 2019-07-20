#include <Bounce.h>

#include <SI570.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>


#define LCD_ADDR                        0x27
#define ENC_1                           4
#define ENC_2                           5
#define BUTTON_1                        24
#define SETTINGS_MENU_SIZE              1
#define FREQ_STEP_IDX                   0

typedef void* (*encoder_handler)(int32_t delta, int8_t direction, uint16_t active_steps, void* data);

struct MenuItem {
  // prompt to display
  char prompt[10];
  // current value
  int32_t value;
  // if is_array allowed values are in array of valid_values,
  // otherwise non_array_settings is used
  bool is_array;
  union {
    // is_array is false
    struct {
      // increment step,
      uint16_t step;
      // min value
      int32_t min_value;
      // max value
      int32_t max_value;
    } non_array_settings;
    // is_array is true
    int32_t valid_values[10];
  } item_settings;
};

MenuItem settings[SETTINGS_MENU_SIZE];
const uint8_t BUTTON_HANDLER_EXIT       = 1;
const uint8_t MENU_PROCESS_COMPLETED    = 2;
const uint32_t starting_frequency       = 10000000;
uint32_t _frequency                     = starting_frequency;
uint8_t frequency_step                  = 50;
uint8_t pulses_interval                 = 20;
const int button_pin_1                  = 32;
uint8_t settings_index                  = 0;

Bounce button_1 = Bounce(button_pin_1, 5);
Encoder encoder(ENC_1, ENC_2);
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
SI570 si570;

typedef void* (*button_handler)(Bounce* bounce, void* data);

void init_menu() {
  int32_t _array[10] = {10, 50, 100, 1000, 10000, 100000, 1000000};
  strcpy(settings[FREQ_STEP_IDX].prompt, "Step:");
  memcpy(settings[FREQ_STEP_IDX].item_settings.valid_values, _array, sizeof(settings[FREQ_STEP_IDX].item_settings.valid_values));
  settings[FREQ_STEP_IDX].value = settings[FREQ_STEP_IDX].item_settings.valid_values[1];
  settings[FREQ_STEP_IDX].is_array = true;
}

void display_frequency(uint32_t frequency) {
  char str[16];
  uint8_t frequency_mhz = (uint8_t)(frequency / 1000000);
  uint16_t frequency_khz = frequency / 1000;
  sprintf(str, "% 2u'%03u.%02u", frequency_mhz, frequency_khz % 1000, (frequency % 1000) / 10);
  lcd.setCursor(0, 0);
  lcd.print(str);
}

void* process_menu_item(Bounce* bounce, void* data) {
  Serial.println("process_menu_item");
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print(settings[settings_index].prompt);
  char str[6];
  sprintf(str, "%d", settings[settings_index].value);
  lcd.setCursor(9, 0);
  lcd.print(str);
  Serial.println("exit process_menu_item");
}

void* process_settings_menu(Bounce* bounce, void* data) {
  Serial.println("process settings");
  process_menu_item(bounce, data);
  while (handle_button(bounce, process_menu_item, data) != &BUTTON_HANDLER_EXIT) {
    ++settings_index;
    if (settings_index == SETTINGS_MENU_SIZE) {
      settings_index = 0;
    }
  }
  lcd.setCursor(0, 0);
  lcd.print("                ");  
  Serial.println("exit process_settings_menu");
  return &MENU_PROCESS_COMPLETED;
}

bool is_long_press(Bounce* bounce) {
  return get_button_press_interval(bounce) > 600;
}

/*
 * returns time interval for the pressed button
 */
uint32_t get_button_press_interval(Bounce* bounce) {
  uint32_t interval = millis();
  do {
    bounce -> update();
    delay(20);
  } while (bounce -> read() == LOW);
  return millis() - interval;
}

/*
 * button handler
 */
void* handle_button(Bounce* bounce, button_handler handler, void* data) {
  void* result;
  bounce -> update();
  if (bounce -> read() == LOW) {
    if (is_long_press(bounce)) {
      return &BUTTON_HANDLER_EXIT;
    }
    if (handler) {
      Serial.println("handler");
      result = handler(bounce, data);
      Serial.println("exit handle_button");
    }
    delay(500);
  }
  return result;
}

void* set_frequency(int32_t delta, int8_t direction, uint16_t active_steps, void* data) {
  if (delta) {
    _frequency += (direction * active_steps * frequency_step);
  }
  display_frequency(_frequency);
  si570.set_frequency(_frequency);
  return NULL;
}

void* handle_encoder(void* data, encoder_handler handler) {
  int16_t delta = encoder.read();
  if (delta) {
    // multiplier for frequency steps
    uint16_t active_steps = (uint16_t)(abs(delta) / pulses_interval);
    if (active_steps) {
      encoder.write(0);
      int8_t direction = delta > 0 ? 1 : (delta == 0 ? 0 : -1);
      if (handler) {
        return handler(delta, direction, active_steps, data);
      }
    }
  }
  return NULL;
}

void setup() {
  Serial.begin(9600);

  pinMode(button_pin_1, INPUT_PULLUP);

  // init lcd
  lcd.init();                           // initialize the lcd
  lcd.backlight();

  si570.init();

  encoder.write(0);

  set_frequency(0, 0, 0, NULL);
  init_menu();
}

void loop() {
  if (handle_button(&button_1, process_settings_menu, NULL) == &MENU_PROCESS_COMPLETED) {
    set_frequency(0, 0, 0, NULL);
  }
  handle_encoder(NULL, set_frequency);
}
