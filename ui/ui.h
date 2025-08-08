#ifndef UI_H
#define UI_H
#include <xc.h>
#include <stdint.h>


#include "../display_driver/display.h"
#include "../clock/clock.h"
#include "../uart_driver/uart_protocol.h"



typedef enum {
    MENU_STATE_HELLO,
    MENU_STATE_SHOW_ADDRESS,
    MENU_STATE_WAIT_ADDRESS,
    MENU_STATE_SHOW_NAME,
    MENU_STATE_WAIT_NAME,
    MENU_STATE_NOMENU,
    MENU_STATE_NOMENU_SHOW,
    MENU_STATE_SHOW_1,
    MENU_STATE_VIEW_1,
    MENU_STATE_SHOW_2,
    MENU_STATE_VIEW_2,
    MENU_STATE_SHOW_3,
    MENU_STATE_VIEW_3,
    MENU_STATE_SHOW_4,
    MENU_STATE_VIEW_4,
    MENU_STATE_IDLE,
    MENU_WAIT,


    MENU_STATE_ERROR
} MenuState;

void ui_init(void);
void ui_update(uint32_t time, narval_msg_t* narval_msg, uint8_t got_address, uint8_t own_address,uint8_t* change_menu);



typedef struct {
    const char* text;
} narval_matrix_t;

#endif // UI_H