#include "ui.h"


static MenuState ui_state;
Display display;
static uint32_t state_entry_time = 0;
//narval_msg_t narval_msg = {0, {0}, 1}; // Initialize narval_msg with default values

static MenuState last_ui_state = MENU_STATE_HELLO; // Track the last state for transitions

extern volatile uint8_t one_second_flag;

static uint8_t clear_display_flag = 0;
static uint32_t clear_display_time = 0;

static uint8_t display_update_flag = 0;


const narval_matrix_t matrix[] = {
    {"Muvezeto"},
    {"Kontroll"},
    {"Diszpecser"},
    {"Karbantarto"},
    {"IPC labor"},
    {"Uveg elok.2"},
    {"Anyagfogado"},
    {"Kulso folyoso"},
    {"Anyagf.bemero"},
    {"Bemero"},
    {"Fecsi elokesz"},
    {"Fecsi tolto"},
    {"Steril kiszedo"},
    {"Elokeszito"},
    {"Mosogato"},
    {"Kis formulalo"},
    {"Steril szuro"},
    {"Lio be/ki"},
    {"Uvegtolto"},
    {"Uveg elokesz"},
    {"Kupakzaro"},
    {"Lio tech"},
    {"Formulalo"},
    {"Formulalo tech"},
    {"Mezzanine"},
    {"Steril foly."},
    {"MP3 bejatszas"},
    {"Korfolyoso"},
    {"Hormon form."},
    {"Line3 elok."},
    {"Line3 tolto"},
    {"Formulalo 2"},
    {"Elokszito 2"},
    {"Mosogato 2"},
    {"Autoklav bead."},
    {"Leszedo"}
    
};

void ui_init(void) {
    Display_Init(&display);
    ui_state = MENU_STATE_HELLO;
    // Display_Printf(&display, 0, "NARVAL DSPIC TEST");
}



static void handle_call_command(narval_msg_t* narval_msg) 
{
    if(narval_msg->ext_or_int == CMD_EXT) {
        Display_Printf(&display, 1, "\x7E%d Hivja:  %d   ", 
                      narval_msg->data[0]+1, narval_msg->data[1]+1);
    } else {
        Display_Printf(&display, 1, "\x7F%d Hivja:  %d   ", 
                      narval_msg->data[0]+1, narval_msg->data[1]+1);
    }
    clear_display_flag = 0;
    display_update_flag = 1;
}

static void handle_disconnect_command(narval_msg_t* narval_msg) 
{
    if(narval_msg->ext_or_int == CMD_EXT) {
        Display_Printf(&display, 1, "\x7E%d Bontja: %d   ", 
                      narval_msg->data[0]+1, narval_msg->data[1]+1);
                      display_update_flag = 1;
    } else {
        Display_Printf(&display, 1, "\x7F%d Bontja: %d   ", 
                      narval_msg->data[0]+1, narval_msg->data[1]+1);
    }
    // Set flag to clear display after 3 seconds
    clear_display_flag = 1;
    clear_display_time = 0; // Will be set in ui_update
    display_update_flag = 1;
}

static void handle_call_command_string(narval_msg_t* narval_msg, uint8_t own_address) 
{
    if(narval_msg->ext_or_int == CMD_INT)
        {
            // Internal command, process accordingly
            if(narval_msg->data[1] < sizeof(matrix)/sizeof(matrix[0]))
            {
                Display_Printf(&display, 1, "Ki:%-13s",matrix[narval_msg->data[1]].text);
            }
            else
            {
                Display_Printf(&display, 1, "Ki: %d", narval_msg->data[1]);
            }
        }
    clear_display_flag = 0;
    display_update_flag = 1;
}

static void handle_disconnect_command_string(narval_msg_t* narval_msg, uint8_t own_address) //!!!!!!!!!!!!
{
    if(narval_msg->ext_or_int == CMD_INT)
        {
            Display_Printf(&display, 1, "Bontva          ");
        }
    else if((narval_msg->ext_or_int == CMD_EXT)&&(narval_msg->data[1] == own_address)) 
        {
            Display_Printf(&display, 1, "Bontva          ");
        }
    // Set flag to clear display after 3 seconds
    clear_display_flag = 1;
    clear_display_time = 0; // Will be set in ui_update
    display_update_flag = 1;
}

static void handle_own_call_command(narval_msg_t* narval_msg, uint8_t own_address) 
{
    if(narval_msg->ext_or_int == CMD_INT) {
        Display_Printf(&display, 1, "Hivom:  %d      ", narval_msg->data[1]+1);
    }
    else if((narval_msg->ext_or_int == CMD_EXT)&&(narval_msg->data[1] == own_address)) {
        Display_Printf(&display, 1, "Hiv:  %d        ", narval_msg->data[0]+1);
    }
    clear_display_flag = 0;
    display_update_flag = 1;
}

static void handle_own_disconnect_command(narval_msg_t* narval_msg, uint8_t own_address) 
{
    if(narval_msg->ext_or_int == CMD_INT) {
        Display_Printf(&display, 1, "Bontom: %d      ", narval_msg->data[1]+1);
        
    }
    else if((narval_msg->ext_or_int == CMD_EXT)&&(narval_msg->data[1] == own_address)) {
        Display_Printf(&display, 1, "Bont:  %d       ", narval_msg->data[0]+1);
    }
    // Set flag to clear display after 3 seconds
    clear_display_flag = 1;
    clear_display_time = 0; // Will be set in ui_update
    display_update_flag = 1;
}







void ui_update(uint32_t time, narval_msg_t* narval_msg, uint8_t got_address, uint8_t own_address,uint8_t* change_menu)
{
    
    switch (ui_state) 
    {
        case MENU_STATE_ERROR:
        {
            Display_Printf(&display, 1, "ERROR STATE");
            break;
        }
        case MENU_STATE_HELLO:
        {
            Display_Printf(&display, 1, " NARVAL SYSTEMS ");
            state_entry_time = time;  // Record the time when entering this state
            ui_state = MENU_STATE_SHOW_ADDRESS;
            break;
        }
        case MENU_STATE_SHOW_ADDRESS:
        {
            if (got_address) 
            {
                Display_Printf(&display, 1, "Sajat cim:%4d   ", own_address+1);
                state_entry_time = time;
                ui_state = MENU_STATE_WAIT_ADDRESS;
            } 
            else if(time - state_entry_time >= 15000) 
            {
                Display_Printf(&display, 1, "Sajat cim:????   ");
                state_entry_time = time;
                ui_state = MENU_STATE_WAIT_ADDRESS;
            }
            break;
        }
        case MENU_STATE_WAIT_ADDRESS:
        {
            if(time - state_entry_time >= 2000) 
            {
                state_entry_time = time;
                if(got_address)
                {
                    ui_state = MENU_STATE_SHOW_NAME;
                }
                else
                {
                    ui_state = MENU_STATE_WAIT_NAME;
                }
            }
            break;
        }
        case MENU_STATE_SHOW_NAME:
        {
            if(own_address <= sizeof(matrix) / sizeof(matrix[0]))
            {
                Display_Printf(&display, 1, "%-16s", matrix[own_address].text);
            }
            else
            {
                Display_Printf(&display, 1, "Sajat nev: %d", own_address + 1);
            }
            
            state_entry_time = time;  // Record the time when entering this state
            ui_state = MENU_STATE_WAIT_NAME;
            break;
        }
        case MENU_STATE_WAIT_NAME:
        {
            if(time - state_entry_time >= 6000) 
            {
                Display_Printf(&display, 1, "                ");
                state_entry_time = time;  // Record the time when entering this state
                ui_state = MENU_STATE_NOMENU_SHOW;
            }
            break;
        }
        case MENU_STATE_NOMENU_SHOW:
        {
            Display_Printf(&display, 1, "1: Nincs kijelz.");
            state_entry_time = time;  // Record the time when entering this state
            ui_state = MENU_STATE_NOMENU;
            break;
        }
        case MENU_STATE_NOMENU:
        {
            if(time - state_entry_time >= 2000) 
            {
                Display_Printf(&display, 1, "                ");
                state_entry_time = time;  // Record the time when entering this state
                ui_state = MENU_STATE_IDLE;
            }
            break;
        }
        case MENU_STATE_IDLE:
        {
            if(*change_menu)
            {
               ui_state = MENU_STATE_SHOW_1;
               *change_menu = 0; // Reset change_menu flag
               Display_Printf(&display, 1, "2:Minden utasit.");
               state_entry_time = time;  // Record state entry time
            }
            break;
        }
        case MENU_STATE_SHOW_1:
        {
            if(narval_msg->new_data_available)
            {
                switch(narval_msg->len)
                {
                    case 3:
                    {
                        switch(narval_msg->data[2])
                        {
                            case 0x1:
                            {
                                handle_call_command(narval_msg);
                                break;
                            }
                            case 0x2:
                            {
                                handle_disconnect_command(narval_msg);
                                clear_display_time = time; // Set the time when disconnect was received
                                break;
                            }
                            default:
                                break;
                        }
                        break;
                    }
                    default:
                        break;
                }
            }
            // Check if we need to clear the display after disconnect
            if(clear_display_flag && (time - clear_display_time >= 3000)) // Clear after 3 seconds
            {
                Display_Clear(&display, 1); // Clear line 1
                clear_display_flag = 0;
            }
            if(*change_menu)
            {   
                ui_state = MENU_STATE_SHOW_2;
                *change_menu = 0; // Reset change_menu flag
                Display_Printf(&display, 1, "3: Sajat utasit.");
                state_entry_time = time;  // Record state entry time
            }
            break;
        }
        case MENU_STATE_SHOW_2:
        {
            if(narval_msg->new_data_available)
                {
                    switch(narval_msg->len)
                    {
                        case 3:
                        {
                            switch(narval_msg->data[2])
                            {
                                case 0x1:
                                {
                                    handle_own_call_command(narval_msg, own_address);
                                    break;
                                }
                                case 0x2:
                                {
                                    handle_own_disconnect_command(narval_msg, own_address);
                                    clear_display_time = time; // Set the time when disconnect was received
                                    break;
                                }
                                default:
                                    break;
                            }
                            break;
                        }
                        default:
                            break;
                    }

                }
            // Check if we need to clear the display after disconnect
            if(clear_display_flag && (time - clear_display_time >= 3000)) // Clear after 3 seconds
            {
                Display_Clear(&display, 1); // Clear line 1
                clear_display_flag = 0;
            }
            if(*change_menu)
            {   
                ui_state = MENU_STATE_SHOW_3;
                *change_menu = 0; // Reset change_menu flag
                Display_Printf(&display, 1, "4:Nev kijelzesek");
                state_entry_time = time;  // Record state entry time
            }
            break;
        }
        case MENU_STATE_SHOW_3:
        {
            if(narval_msg->new_data_available)
            {
                switch(narval_msg->len)
                {
                    case 3:
                    {
                        switch(narval_msg->data[2])
                        {
                            case 0x1:
                            {
                                if(narval_msg->ext_or_int == CMD_INT)
                                {
                                    handle_call_command_string(narval_msg, own_address);
                                }
                                break;
                            }
                            case 0x2:
                            {
                                if(narval_msg->ext_or_int == CMD_EXT)
                                {
                                    handle_disconnect_command_string(narval_msg, own_address);
                                    clear_display_time = time; // Set the time when disconnect was received
                                }
                                break;
                            }
                            default:
                                break;
                        }
                        break;
                    }
                    default:
                        break;
                }
            }
            // Check if we need to clear the display after disconnect
            if(clear_display_flag && (time - clear_display_time >= 3000)) // Clear after 3 seconds
            {
                Display_Clear(&display, 1); // Clear line 1
                clear_display_flag = 0;
            }
            if(*change_menu)
            {   
                ui_state = MENU_STATE_NOMENU_SHOW;
                *change_menu = 0; // Reset change_menu flag
                state_entry_time = time;  // Record state entry time
            }
            break;
        }
    }






    if(one_second_flag || ui_state != last_ui_state)
    {
        one_second_flag = 0;
        clock_time_t current_time = clock_get_time();
        clock_date_t current_date = clock_get_date();
        Display_Printf(&display, 0, "'%02d.%02d.%02d. %2d:%02d",
                        current_date.years % 100,
                        current_date.months,
                        current_date.days,
                        current_time.hours, 
                        current_time.minutes);
        Display_Send(&display);
        last_ui_state = ui_state;  // Update last_ui_state after sending display
    }
}
