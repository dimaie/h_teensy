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
#define SETTINGS_MENU_SIZE              2
#define FREQ_STEP_IDX                   0
#define OSC_SHIFT_IDX                   1

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
    struct {
      int32_t valid_values[10];
      int8_t length;
    } array_settings;
  } item_settings;
};

MenuItem settings[SETTINGS_MENU_SIZE];
const uint8_t MENU_PROCESS_COMPLETED          = 2;
const uint8_t MENU_ITEM_PROCESS_COMPLETED     = 3;
const uint8_t BUTTON_LONG_PRESSED             = 4;
const uint8_t BUTTON_SHORT_PRESSED            = 5;
const uint8_t NO_BUTTON_PRESSED               = 6;
const uint32_t starting_frequency             = 10000000;
uint32_t _frequency                           = starting_frequency;
uint8_t pulses_interval                       = 20;
const int button_pin_1                        = 32;
uint8_t settings_index                        = 0;

Bounce button_1 = Bounce(button_pin_1, 5);
Encoder encoder(ENC_1, ENC_2);
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
SI570 si570;

typedef void* (*button_handler)(Bounce* bounce, void* data);

void init_menu() {
  // frequency step
  int32_t _array[10] = {10, 50, 100, 1000, 10000, 100000, 1000000};
  strcpy(settings[FREQ_STEP_IDX].prompt, "Step:");
  settings[FREQ_STEP_IDX].item_settings.array_settings.length = 7;
  memcpy(settings[FREQ_STEP_IDX].item_settings.array_settings.valid_values, _array, sizeof(settings[FREQ_STEP_IDX].item_settings.array_settings.valid_values));
  settings[FREQ_STEP_IDX].value = 1;
  settings[FREQ_STEP_IDX].is_array = true;
  // oscillator tx shift
  strcpy(settings[OSC_SHIFT_IDX].prompt, "TX Shift:");
  settings[OSC_SHIFT_IDX].value = 700;
  settings[OSC_SHIFT_IDX].is_array = false;
  settings[OSC_SHIFT_IDX].item_settings.non_array_settings.min_value = -1000;
  settings[OSC_SHIFT_IDX].item_settings.non_array_settings.max_value = 1000;
  settings[OSC_SHIFT_IDX].item_settings.non_array_settings.step = 100;
}

void display_frequency(uint32_t frequency) {
  char str[16];
  uint8_t frequency_mhz = (uint8_t)(frequency / 1000000);
  uint16_t frequency_khz = frequency / 1000;
  sprintf(str, "% 2u'%03u.%02u", frequency_mhz, frequency_khz % 1000, (frequency % 1000) / 10);
  lcd.setCursor(0, 0);
  lcd.print(str);
}

void display_menu_item() {
  MenuItem* current_menu_item = settings + settings_index;
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print(current_menu_item -> prompt);
  char str[6];
  sprintf(str, "%d", current_menu_item -> is_array ? current_menu_item -> item_settings.array_settings.valid_values[current_menu_item -> value] : current_menu_item -> value);
  lcd.setCursor(9, 0);
  lcd.print(str);
}

void* set_menu_item_value(int32_t delta, int8_t direction, uint16_t active_steps, void* data) {
  MenuItem* current_menu_item = settings + settings_index;
  if (current_menu_item -> is_array) {
    // increment array index
    current_menu_item -> value += direction;
    if (current_menu_item -> value > (current_menu_item -> item_settings.array_settings.length - 1)) {
      current_menu_item -> value = 0;
    } else if (current_menu_item -> value < 0) {
      current_menu_item -> value = current_menu_item -> item_settings.array_settings.length - 1;
    }
  } else {
    // increment value by step
    current_menu_item -> value += (direction * current_menu_item -> item_settings.non_array_settings.step);
    if (current_menu_item -> value > current_menu_item -> item_settings.non_array_settings.max_value) {
      current_menu_item -> value = current_menu_item -> item_settings.non_array_settings.max_value;
    } else if (current_menu_item -> value < current_menu_item -> item_settings.non_array_settings.min_value) {
      current_menu_item -> value = current_menu_item -> item_settings.non_array_settings.min_value;
    }
  }
  return NULL;
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
  void* result = &NO_BUTTON_PRESSED;
  bounce -> update();
  if (bounce -> read() == LOW) {
    if (is_long_press(bounce)) {
      Serial.println("long press");
      return &BUTTON_LONG_PRESSED;
    }
    Serial.println("short press");
    result = &BUTTON_SHORT_PRESSED;
    if (handler) {
      Serial.println("handler_button");
      result = handler(bounce, data);
      Serial.println("exit handle_button");
    }
    delay(100);
  }
  return result;
}

void* process_settings_menu(Bounce* bounce, void* data) {
  display_menu_item();
  // change item index until long press
  do {
    data = handle_button(bounce, NULL, data);
    if (data == &BUTTON_SHORT_PRESSED) {
      // increment menu index and redraw menu item     
      ++settings_index;
      if (settings_index == SETTINGS_MENU_SIZE) {
        settings_index = 0;
      }      
      display_menu_item();
    }
    // 
    MenuItem* current_menu_item = settings + settings_index;
    int32_t value = current_menu_item -> value;
    handle_encoder(NULL, set_menu_item_value);
    if (current_menu_item -> value != value) {
      display_menu_item();
    }
    delay(50);
  } while (data != &BUTTON_LONG_PRESSED);
  
  // clear screen
  lcd.setCursor(0, 0);
  lcd.print("                ");  
  return &MENU_PROCESS_COMPLETED;
}


void* set_frequency(int32_t delta, int8_t direction, uint16_t active_steps, void* data) {
  int32_t frequency_step = (settings + FREQ_STEP_IDX) -> item_settings.array_settings.valid_values[(settings + FREQ_STEP_IDX) -> value];
  if (delta) {
    _frequency += (direction * active_steps *  frequency_step);
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
    // process_menu_settings clears screen, so update frequency 
    // after it has been completed
    set_frequency(0, 0, 0, NULL);
  }
  handle_encoder(NULL, set_frequency);
}
