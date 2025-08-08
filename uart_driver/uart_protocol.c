#include "uart_protocol.h"

#include "uart.h"


uint8_t uart_state = UART_STATE_IDLE;
static uint8_t packet_buffer[8];  // Small buffer for current packet only
static uint8_t packet_index = 0;
static uint8_t expected_length = 0;
narval_msg_t narval_msg = {0, {0}, 1};

void uart_process(uint8_t received_byte)
{
    switch (uart_state) 
    {
        case UART_STATE_IDLE:
            if ((received_byte == UART_SYNC_BYTE_EXT)||(received_byte == UART_SYNC_BYTE_INT)) 
            {
                packet_index = 0;
                if(received_byte == UART_SYNC_BYTE_EXT) 
                {
                    narval_msg.ext_or_int = CMD_EXT; // External message
                } 
                else 
                {
                    narval_msg.ext_or_int = CMD_INT; // Internal message
                }
                uart_state = UART_STATE_READLEN;
            }
            break;

        case UART_STATE_READLEN:
            if(received_byte <= sizeof(packet_buffer)) 
            {
                expected_length = received_byte;
                uart_state = (expected_length == 0) ? UART_STATE_READEND : UART_STATE_READDATA;
            }
            else 
            {
                uart_state = UART_STATE_IDLE;
            }
            break;

        case UART_STATE_READDATA:
            if(packet_index < sizeof(packet_buffer)) 
            {
                packet_buffer[packet_index++] = received_byte;
                
                if(packet_index >= expected_length) 
                {
                    uart_state = UART_STATE_READEND;
                }
            }
            else 
            {
                uart_state = UART_STATE_IDLE;
            }
            break;

        case UART_STATE_READEND:
            if(received_byte == UART_END_BYTE) 
            {
                // Copy to final destination
                narval_msg.len = packet_index;
                for(uint8_t i = 0; i < packet_index; i++) {
                    narval_msg.data[i] = packet_buffer[i];
                }
                narval_msg.new_data_available = 1; // Indicate that a new message is available
            }
            uart_state = UART_STATE_IDLE;
            break;

        default:
            uart_state = UART_STATE_IDLE;
            break;
    }
}