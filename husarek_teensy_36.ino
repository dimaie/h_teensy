#include <Audio.h>

#include <EEPROM.h>
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
#include "filters.h"
#include "PCF8574.h"


#define LCD_ADDR                        0x27
#define P0                              0
#define P1                              1
#define P2                              2
#define P3                              3
#define P4                              4
#define P5                              5
#define P6                              6
#define P7                              7
#define ENC_1                           4
#define ENC_2                           5
#define BUTTON_1                        24
#define SETTINGS_MENU_SIZE              7
#define FREQ_STEP_IDX                   0
#define ATT_IDX                         1
#define AMP_1_IDX                       2
#define AMP_2_IDX                       3
#define OSC_SHIFT_IDX                   4
#define OSC_MULT_IDX                    5
#define INPUT_L_SENS_IDX                6
#define ROUTE_RX                        0     // used for all recieve modes
#define INITIAL_VOLUME                  .5   // 0-1.0 output volume on startup
#define RX_LEVEL_I                      1.0   // 0-1.0 adjust for RX I/Q balance
#define RX_LEVEL_Q                      1.0   // 0-1.0 adjust for RX I/Q balance
#define IF_FREQ                         11000 // IF Oscillator frequency
#define TONE_TYPE_SINE                  0
#define TONE_TYPE_SAWTOOTH              1
#define TONE_TYPE_SQUARE                2
#define TONE_TYPE_TRIANGLE              3
// radio operation mode defines used for filter selections etc
#define SSB_USB                         0
#define SSB_LSB                         1
#define CW                              2
#define CWR                             3
// offset of main module switches
#define R_T_OFF                         0
#define ATT_OFF                         1
#define AMP_1_OFF                       2
#define AMP_2_OFF                       3
#define CW_OFF                          4
#define RX_OFF                          5
#define TX_OFF                          6

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
const uint8_t CONF_VERSION                    = 2;
const uint8_t EEPROM_OFFSET                   = 0;
const uint32_t starting_frequency             = 10000000;
const uint32_t min_frequency                  = 1500000;
const uint32_t max_frequency                  = 57000000;
uint32_t _frequency                           = starting_frequency;
uint8_t pulses_interval                       = 20;
const uint8_t button_pin_1                    = 32;
const uint8_t button_pin_2                    = 30;
uint8_t settings_index                        = 0;
const int input_rx                            = AUDIO_INPUT_LINEIN;
uint8_t radio_board_config                    = 0;
float volume                                  = INITIAL_VOLUME;

Bounce menu_button = Bounce(button_pin_1, 5);
Bounce function_button = Bounce(button_pin_2, 5);

Encoder encoder(ENC_1, ENC_2);
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
SI570 si570;

typedef void* (*button_handler)(Bounce* bounce, void* data);
AudioInputI2S       audio_input;           // Audio Shield: mic or line-in
// Create the Audio components.  These should be created in the
// order data flows, inputs/sources -> processing -> outputs
//
// FIR filters
AudioFilterFIR      hilbert_45_i;
AudioFilterFIR      hilbert_45_q;
AudioFilterFIR      fir_bpf;
AudioFilterFIR      post_fir;

AudioMixer4         summer;         // summer (add inputs)
AudioSynthWaveform  if_osc;         // Local Oscillator
AudioEffectMultiply mixer;          // mixer (multiply inputs)
AudioAnalyzePeak    smeter;         // Measure Audio Peak for S meter
AudioMixer4         audio_selector_i;   // summer used for AGC and audio switch
AudioMixer4         audio_selector_q;   // summer used as audio selector
AudioAnalyzePeak    agc_peak;       // Measure Audio Peak for AGC use
AudioOutputI2S      audio_output;   // Audio Shield: headphones & line-out

AudioControlSGTL5000 audio_shield;  // Create an object to control the audio shield.

//---------------------------------------------------------------------------------------------------------
// Create Audio connections to build a software defined Radio Receiver
//
AudioConnection c1(audio_input, 0, hilbert_45_i, 0);// Audio inputs to +/- 45 degree filters
AudioConnection c2(audio_input, 1, hilbert_45_q, 0);

AudioConnection c3(hilbert_45_i, 0, summer, 0);      // Sum the shifted filter outputs to supress the image
AudioConnection c4(hilbert_45_q, 0, summer, 1);
//
AudioConnection c11(summer, 0, fir_bpf, 0);         // 2.4 kHz USB or LSB filter centred at either 12.5 or 9.5 kHz
//                                                  // ( local oscillator zero beat is at 11 kHz, see NCO )
AudioConnection c12(fir_bpf, 0, mixer, 0);          // IF from BPF to mixer
AudioConnection c13(if_osc, 0, mixer, 1);           // Local Oscillator to mixer (11 kHz)
//
AudioConnection c20(mixer, 0, post_fir, 0);          // 2700Hz Low Pass filter or 200 Hz wide CW filter at 700Hz on audio output
AudioConnection c30(post_fir, 0, smeter, 0);         // RX signal S-Meter measurement point
//
// RX is mono output , but for TX we need I and Q audio channel output
// two summers (I and Q) on the output used to select different audio paths for different RX and TX modes
//
AudioConnection c31(post_fir, 0, audio_selector_i, ROUTE_RX);           // mono RX audio and AGC Gain loop adjust
AudioConnection c32(post_fir, 0, audio_selector_q, ROUTE_RX);           // mono RX audio to 2nd channel
AudioConnection c40(audio_selector_i, 0, agc_peak, 0);                  // AGC Gain loop measure
AudioConnection c41(audio_selector_i, 0, audio_output, 0);              // Output the sum on both channels
AudioConnection c42(audio_selector_q, 0, audio_output, 1);

PCF8574 PCF_39(0x39);
/*
   returns true if configuration was read,
   or false otherwise
*/
bool read_conf() {
  byte conf_version = EEPROM.read(EEPROM_OFFSET);
  if (conf_version != CONF_VERSION) {
    return false;
  }
  byte size_int = sizeof(int32_t);
  byte int_array[size_int];
  byte offset = EEPROM_OFFSET + 1;
  for (uint8_t i = offset; i < (offset + SETTINGS_MENU_SIZE); ++i) {
    for (uint8_t k = 0; k < size_int; ++k) {
      int_array[size_int - 1 - k] = EEPROM.read(i * size_int + k);
      String s = (String("") + k) + "=" + int_array[k];
    }
    memcpy(&(settings[i - offset].value), int_array, size_int);
  }
  return true;
}

void set_radio_board_config(uint8_t value, uint8_t offset) {
  radio_board_config = value ? (radio_board_config | (1 << offset)) : (radio_board_config & ~(1 << offset));
}

void apply_settings() {
  uint8_t _radio_board_config = radio_board_config;
  set_radio_board_config(settings[AMP_1_IDX].value, P2);
  set_radio_board_config(settings[AMP_2_IDX].value, P3);
  set_radio_board_config(settings[ATT_IDX].value, P1);
  // update frequency, in case if multiplier has been changed
  update_frequency();
  // update line sensitivity
  uint8_t line_in_sens = settings[INPUT_L_SENS_IDX].value;
  audio_shield.lineInLevel(line_in_sens, line_in_sens);
  // update radio board configuration, in case if it has been changed
  if (_radio_board_config != radio_board_config) {
    PCF_39.write8(radio_board_config);
  }
}
void write_conf() {
  EEPROM.write(EEPROM_OFFSET, CONF_VERSION);

  byte offset = EEPROM_OFFSET + 1;
  byte size_int = sizeof(int32_t);
  for (uint8_t i = offset; i < offset + SETTINGS_MENU_SIZE; ++i) {
    for (uint8_t k = 0; k < size_int; ++k) {
      EEPROM.write(i * size_int + k, (settings[i - offset].value >> ((size_int - (k + 1)) * 8)) & 0xFF);
    }
  }
}
void init_menu() {
  // frequency step
  int32_t _array[10] = {10, 50, 100, 1000, 10000, 100000, 1000000};
  strcpy(settings[FREQ_STEP_IDX].prompt, "Step:");
  settings[FREQ_STEP_IDX].item_settings.array_settings.length = 7;
  memcpy(settings[FREQ_STEP_IDX].item_settings.array_settings.valid_values, _array, sizeof(settings[FREQ_STEP_IDX].item_settings.array_settings.valid_values));
  settings[FREQ_STEP_IDX].value = 1;
  settings[FREQ_STEP_IDX].is_array = true;
  // attenuator shift
  strcpy(settings[ATT_IDX].prompt, "Atten:");
  settings[ATT_IDX].value = 0;
  settings[ATT_IDX].is_array = false;
  settings[ATT_IDX].item_settings.non_array_settings.min_value = 0;
  settings[ATT_IDX].item_settings.non_array_settings.max_value = 1;
  settings[ATT_IDX].item_settings.non_array_settings.step = 1;
  // amplifier 1 shift
  strcpy(settings[AMP_1_IDX].prompt, "Amp 1:");
  settings[AMP_1_IDX].value = 1;
  settings[AMP_1_IDX].is_array = false;
  settings[AMP_1_IDX].item_settings.non_array_settings.min_value = 0;
  settings[AMP_1_IDX].item_settings.non_array_settings.max_value = 1;
  settings[AMP_1_IDX].item_settings.non_array_settings.step = 1;
  // amplifier 2 shift
  strcpy(settings[AMP_2_IDX].prompt, "Amp 2:");
  settings[AMP_2_IDX].value = 1;
  settings[AMP_2_IDX].is_array = false;
  settings[AMP_2_IDX].item_settings.non_array_settings.min_value = 0;
  settings[AMP_2_IDX].item_settings.non_array_settings.max_value = 1;
  settings[AMP_2_IDX].item_settings.non_array_settings.step = 1;
  // oscillator tx shift
  strcpy(settings[OSC_SHIFT_IDX].prompt, "TX Shift:");
  settings[OSC_SHIFT_IDX].value = 700;
  settings[OSC_SHIFT_IDX].is_array = false;
  settings[OSC_SHIFT_IDX].item_settings.non_array_settings.min_value = -1000;
  settings[OSC_SHIFT_IDX].item_settings.non_array_settings.max_value = 1000;
  settings[OSC_SHIFT_IDX].item_settings.non_array_settings.step = 100;
  // oscillator multiplier
  int32_t _array1[10] = {1, 2, 4};
  strcpy(settings[OSC_MULT_IDX].prompt, "OscMul:");
  settings[OSC_MULT_IDX].item_settings.array_settings.length = 3;
  memcpy(settings[OSC_MULT_IDX].item_settings.array_settings.valid_values, _array1, sizeof(settings[OSC_MULT_IDX].item_settings.array_settings.valid_values));
  settings[OSC_MULT_IDX].value = 0;
  settings[OSC_MULT_IDX].is_array = true;
  // input line sensitivity shift
  strcpy(settings[INPUT_L_SENS_IDX].prompt, "InSens:");
  settings[INPUT_L_SENS_IDX].value = 7;
  settings[INPUT_L_SENS_IDX].is_array = false;
  settings[INPUT_L_SENS_IDX].item_settings.non_array_settings.min_value = 0;
  settings[INPUT_L_SENS_IDX].item_settings.non_array_settings.max_value = 15;
  settings[INPUT_L_SENS_IDX].item_settings.non_array_settings.step = 1;
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
   returns time interval for the pressed button
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
   button handler
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
  write_conf();
  return &MENU_PROCESS_COMPLETED;
}

void* set_volume(int32_t delta, int8_t direction, uint16_t active_steps, void* data) {
  float max_volume = 1;
  float min_volume = 0;
  volume += (direction * .1);
  if (volume > max_volume) {
    volume = max_volume;
  } else if (volume < min_volume) {
    volume = min_volume;
  }
  return NULL;
}

void display_volume() {
  char ch_volume[17];
  strcpy(ch_volume, "                \0");
  float i = 0;
  float step = 1.0 / 16;
  for (int8_t k = 0; k < 17; ++k, i += step) {
    ch_volume[k] = i <= volume ? 255 : ' ';
  }
  lcd.setCursor(0, 1);
  lcd.print(ch_volume);
}

void* process_volume(Bounce* bounce, void* data) {
  lcd.setCursor(0, 1);
  lcd.print("                ");
  display_volume();
  do {
    data = handle_button(bounce, NULL, data);
    if (data == &BUTTON_SHORT_PRESSED) {
      break;
    }
    //
    float _volume = volume;
    handle_encoder(NULL, set_volume);
    if (volume != _volume) {
      display_volume();
      audio_shield.volume(volume);    
    }
    delay(50);
  } while (true);

  // clear screen
  lcd.setCursor(0, 1);
  lcd.print("                ");
  return &MENU_PROCESS_COMPLETED;
}

void* set_frequency(int32_t delta, int8_t direction, uint16_t active_steps, void* data) {
  int32_t frequency_step = (settings + FREQ_STEP_IDX) -> item_settings.array_settings.valid_values[(settings + FREQ_STEP_IDX) -> value];
  int32_t osc_mult = (settings + OSC_MULT_IDX) -> item_settings.array_settings.valid_values[(settings + OSC_MULT_IDX) -> value];
  if (delta) {
    _frequency += (direction * active_steps *  frequency_step);
  }
  if (_frequency < min_frequency) {
    _frequency = min_frequency;
  } else if (_frequency > max_frequency) {
    _frequency = max_frequency;
  }
  // apply multiplier and tx shift
  uint32_t applied_frequency = _frequency * osc_mult;

  display_frequency(_frequency);
  si570.set_frequency(applied_frequency);
  return NULL;
}

void update_frequency() {
  set_frequency(0, 0, 0, NULL);
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

float audiolevels_I[1][4] = {
  RX_LEVEL_I, 0, 0, 0,        // RX mode channel levels
};

float audiolevels_Q[1][4] = {
  RX_LEVEL_Q, 0, 0, 0,        // RX mode channel levels
};

void audio_channel_setup(int route) {
  for (int i = 0; i < 4 ; ++i) {
    audio_selector_i.gain(i, audiolevels_I[route][i]); // set gains on audioselector channels
    audio_selector_q.gain(i, audiolevels_Q[route][i]);
  }
}
// set up radio for RX modes - USB, LSB etc
void setup_RX(int mode) {
  AudioNoInterrupts();   // Disable Audio while reconfiguring filters

  audio_shield.inputSelect(input_rx); // RX mode uses line ins
  audio_channel_setup(ROUTE_RX);   // switch audio path to RX processing chain
  audio_shield.lineInLevel(7);

  if_osc.begin(1.0, IF_FREQ, TONE_TYPE_SINE);

  // Initialize the wideband +/-45 degree Hilbert filters
  hilbert_45_i.begin(RX_hilbertm45, HILBERT_COEFFS);
  hilbert_45_q.begin(RX_hilbert45, HILBERT_COEFFS);

  if ((mode == SSB_LSB) || (mode == CWR))             // LSB modes
    fir_bpf.begin(firbpf_lsb, BPF_COEFFS);      // 2.4kHz LSB filter
  else fir_bpf.begin(firbpf_usb, BPF_COEFFS);      // 2.4kHz USB filter

  switch (mode) {
    case CWR:
      post_fir.begin(postfir_700, COEFF_700);    // 700 Hz LSB filter
      break;
    case SSB_LSB:
      post_fir.begin(postfir_lpf, COEFF_LPF);    // 2.4kHz LSB filter
      break;
    case CW:
      post_fir.begin(postfir_700, COEFF_700);    // 700 Hz LSB filter
      break;
    case SSB_USB:
      post_fir.begin(postfir_lpf, COEFF_LPF);    // 2.4kHz LSB filter
      break;
  }
  AudioInterrupts();
}

void setup() {
  Serial.begin(9600);

  pinMode(button_pin_1, INPUT_PULLUP);
  pinMode(button_pin_2, INPUT_PULLUP);

  // init lcd
  lcd.init();                           // initialize the lcd
  lcd.backlight();
  
  si570.init();

  encoder.write(0);

  set_frequency(0, 0, 0, NULL);
  init_menu();

  if (!read_conf()) {
    write_conf();
  }

  AudioMemory(16);

  // Enable the audio shield and set the output volume.
  audio_shield.enable();
  audio_shield.volume(volume);
  audio_shield.unmuteLineout();

  setup_RX(CW);
}

void loop() {
  if (handle_button(&menu_button, process_settings_menu, NULL) == &MENU_PROCESS_COMPLETED) {
    // process_menu_settings clears screen, so update frequency
    // after it has been completed
    update_frequency();
  }
  if (handle_button(&function_button, process_volume, NULL) == &MENU_PROCESS_COMPLETED) {
    // process_volume clears screen, so update frequency
    // after it has been completed
    update_frequency();
  }
  apply_settings();
  handle_encoder(NULL, set_frequency);
}
