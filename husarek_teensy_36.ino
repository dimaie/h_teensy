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

typedef void* (*encoder_handler)(int32_t delta, int8_t direction, uint16_t active_steps, void* data);

int led                                 = 13;
const unsigned long starting_frequency  = 10000000UL;
unsigned long _frequency                = starting_frequency;
unsigned int frequency_step             = 50;
unsigned int pulses_interval            = 20;
const int buttonPin                     = 32;
int8_t previousState                    = 1;


Bounce button_1 = Bounce(buttonPin, 5); 
Encoder encoder(ENC_1, ENC_2);
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
SI570 si570;
 
void display_frequency(unsigned long frequency) {
    char str[16];
    unsigned int frequency_mhz = frequency / 1000000;
    unsigned int frequency_khz = frequency / 1000;
    sprintf(str,"% 2u'%03u.%02u", frequency_mhz, frequency_khz % 1000, (frequency % 1000) / 10);
    lcd.setCursor(0, 0);
    lcd.print(str); 
}

void handle_button_1() {
    button_1.update();
    if (button_1.read() == LOW) {
      if (previousState < 0) {
        lcd.noCursor();
      } else {
        lcd.cursor();
      }
      previousState = -previousState;
      Serial.println(previousState);
      delay(500);
    }
}


void* set_frequency(int32_t delta, int8_t direction, uint16_t active_steps, void* data) {
  if (delta) {
    _frequency += direction * active_steps * frequency_step;
  }
  display_frequency(_frequency);
  si570.set_frequency(_frequency);
  return NULL;
}

void* handle_encoder(void* data, encoder_handler handler) {
  long delta = encoder.read();
  if (delta) {

    // multiplier for frequency steps
    uint16_t active_steps = abs(delta) / pulses_interval;
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
  
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);    
  
  pinMode(buttonPin, INPUT_PULLUP);
  
  // init lcd 
  lcd.init();                           // initialize the lcd 
  lcd.backlight();                     

  si570.init();

  encoder.write(0);
  
  set_frequency(0, 0, 0, NULL);
  Serial.println("init");
}

void loop() {
  handle_button_1();
  handle_encoder(NULL, set_frequency);
} 
