#include "buttons_handler.h"
#include <Arduino.h>

extern Encoder encoder;
uint8_t pulses_interval                       = 20;

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
void* button_handle(Bounce* bounce, button_handler handler, void* data) {
  void* result = NO_BUTTON_PRESSED;
  bounce -> update();
  if (bounce -> read() == LOW) {
    if (is_long_press(bounce)) {
      Serial.println("long press");
      return BUTTON_LONG_PRESSED;
    }
    Serial.println("short press");
    result = BUTTON_SHORT_PRESSED;
    if (handler) {
      Serial.println("handler_button");
      result = handler(bounce, data);
      Serial.println("exit handle_button");
    }
    delay(100);
  }
  return result;
}

/*
   button handler with handle functions for short and long clicks 
*/
void* slbutton_handle(Bounce* bounce, button_handler handler_short, button_handler handler_long, void* data) {
  void* result = &NO_BUTTON_PRESSED;
  bounce -> update();
  if (bounce -> read() == LOW) {
    if (is_long_press(bounce)) {
      result = &BUTTON_LONG_PRESSED;
      if (handler_long) {
        result = handler_long(bounce, data);
      } 
    } else {
      result = &BUTTON_SHORT_PRESSED;
      if (handler_short) {
        result = handler_short(bounce, data);
      } 
    }
    delay(100);
  }
  return result;
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
