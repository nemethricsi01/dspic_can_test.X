#include "ui.h"


static MenuState ui_state;
Display display;
static uint32_t state_entry_time = 0;
//narval_msg_t narval_msg = {0, {0}, 1}; // Initialize narval_msg with default values

static MenuState last_ui_state = MENU_STATE_HELLO; // Track the last state for transitions

extern volatile uint8_t one_second_flag;

static uint8_t clear_display_flag = 0;
static uint64_t clear_display_time = 0;

static uint8_t display_update_flag = 0;
static uint64_t last_update_time = 0;


const narval_matrix_t matrix[] = {
    {"Muvezeto"},//1
    {"Kontroll"},//2
    {"Diszpecser"},//3
    {"Karbantarto"},//4
    {"IPC labor"},//5
    {"Uveg elok.2"},//6
    {"Anyagfogado"},//7
    {"Kulso folyoso"},//8
    {"Anyagf.bemero"},//9
    {"Bemero"},//10
    {"Fecsi elokesz"},//11
    {"Fecsi tolto"},//12
    {"Steril kiszed"},//13
    {"Elokeszito"},//14
    {"Mosogato"},//15
    {"Kis formulalo"},//16
    {"Steril szuro"},//17
    {"Lio be/ki"},//18
    {"Uvegtolto"},//19
    {"Uveg elokesz"},//20
    {"Kupakzaro"},//21
    {"Lio tech"},//22
    {"Formulalo"},//23
    {"Formulal tech"},//24
    {"Mezzanine"},//25
    {"Steril foly."},//26
    {"MP3 bejatszas"},//27
    {"Korfolyoso"},//28
    {"Hormon form."},//29
    {"Line3 elok."},//30
    {"Line3 tolto"},//31
    {"Formulalo 2"},//32
    {"Elokeszito 2"},//33
    {"Mosogato 2"},//34
    {"Autoklav bead"},//35
    {"Leszedo"}//36
};

void ui_init(void) {
    Display_Init(&display);
    ui_state = MENU_STATE_HELLO;
    last_update_time = 0; // Initialize last update time
    state_entry_time = 0; // Initialize state entry time
    clear_display_flag = 0; // Initialize clear display flag
    display_update_flag = 1; // Initialize display update flag, clock will be displayed
    last_ui_state = MENU_STATE_HELLO; // Initialize last UI state
    // Display_Printf(&display, 0, "NARVAL DSPIC TEST");
}
/*
NARVAL MESSAGE FORMAT:
DATA[0] = FROM WHO
DATA[1] = TO WHO
DATA[2] = TYPE: 0x01 call, 0x02 disconnect
*/


static void handle_call_command(narval_msg_t* narval_msg) 
{
    if(narval_msg->data[1] == 249)
    {
        return; // Ignore calls to 249
    }
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
    if(narval_msg->data[1] == 249)
    {
        return; // Ignore calls to 249
    }
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

static void handle_own_call_command(narval_msg_t* narval_msg, uint8_t own_address) 
{
    if(narval_msg->data[1] == 249)
    {
        return; // Ignore calls to 249
    }
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
    if(narval_msg->data[1] == 249)
    {
        return; // Ignore calls to 249
    }
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

static void handle_call_command_string(narval_msg_t* narval_msg, uint8_t own_address) 
{
    if(narval_msg->data[1] == 249)
    {
        return; // Ignore calls to 249
    }
    /*
    NARVAL MESSAGE FORMAT:
    DATA[0] = FROM WHO
    DATA[1] = TO WHO
    DATA[2] = TYPE: 0x01 call, 0x02 disconnect
    */
    if(narval_msg->ext_or_int == CMD_INT)
        {
            // Internal command, process accordingly
            if(narval_msg->data[1] < sizeof(matrix)/sizeof(matrix[0]))
            {
                Display_Printf(&display, 1, "Ki:%-13s",matrix[narval_msg->data[1]].text);
            }
            else
            {
                Display_Printf(&display, 1, "Ki: %d          ", narval_msg->data[1]);
            }
        }
        else if((narval_msg->ext_or_int == CMD_EXT)&&(narval_msg->data[1] == own_address)) 
            {
                if(narval_msg->data[0] < sizeof(matrix)/sizeof(matrix[0]))
                {
                    Display_Printf(&display, 1, "Be:%-13s",matrix[narval_msg->data[0]].text);
                }
                else
                {
                    Display_Printf(&display, 1, "Be: %d          ", narval_msg->data[0]);
                }
            }
    clear_display_flag = 0;
    display_update_flag = 1;
}

static void handle_disconnect_command_string(narval_msg_t* narval_msg, uint8_t own_address) //!!!!!!!!!!!!
{
    if(narval_msg->data[1] == 249)
    {
        return; // Ignore calls to 249
    }
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
                ui_state = MENU_STATE_SHOW_3;
                Display_Printf(&display, 1, "4:Nev kijelzesek");
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
            else if(*change_menu)
            {
                ui_state = MENU_STATE_SHOW_1;
                *change_menu = 0; // Reset change_menu flag
                Display_Printf(&display, 1, "2:Minden utasit.");
                state_entry_time = time;  // Record state entry time
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
        case MENU_STATE_SHOW_3://4:Nev kijelzesek
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
                                    handle_call_command_string(narval_msg, own_address);
                                break;
                            }
                            case 0x2:
                            {
                                    handle_disconnect_command_string(narval_msg, own_address);
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
                display_update_flag = 1; // Ensure we send the updated buffer
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




    // Always refresh the top line once per second
    if (one_second_flag)
    {
        clock_time_t current_time = clock_get_time();
        clock_date_t current_date = clock_get_date();
        Display_Printf(&display, 0, "'%02d.%02d.%02d. %2d:%02d",
                    current_date.years % 100,
                    current_date.months,
                    current_date.days,
                    current_time.hours,
                    current_time.minutes);
        one_second_flag = 0;
        display_update_flag = 1; // ensure we send the updated buffer
    }

    if((ui_state != last_ui_state)||(display_update_flag && (time - last_update_time >= 300)))
    {
        display_update_flag = 0; // Reset display update flag
        Display_Send(&display);
        last_ui_state = ui_state;  // Update last_ui_state after sending display
        last_update_time = time;    // Update last_update_time after sending display
    }
}
