#ifndef _BUTTONS_HANDLER_H
#define _BUTTONS_HANDLER_H

#include <Encoder.h>
#include <Bounce.h>
#include <stdint.h>

const uint8_t BUTTON_LONG_PRESSED             = 4;
const uint8_t BUTTON_SHORT_PRESSED            = 5;
const uint8_t NO_BUTTON_PRESSED               = 6;

typedef void* (*button_handler)(Bounce* bounce, void* data);
typedef void* (*encoder_handler)(int32_t delta, int8_t direction, uint16_t active_steps, void* data);

#ifdef __cplusplus
extern "C" {
#endif

/*
   button handler
*/
void* button_handle(Bounce* bounce, button_handler handler, void* data);

/*
   button handler with handle functions for short and long clicks 
*/
void* slbutton_handle(Bounce* bounce, button_handler handler_short, button_handler handler_long, void* data);

/*
   returns time interval for the pressed button
*/
uint32_t get_button_press_interval(Bounce* bounce);

void* handle_encoder(void* data, encoder_handler handler);

#ifdef __cplusplus
}
#endif

#endif
