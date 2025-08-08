/* 
 * File:   uart_protocol.h
 * Author: nemet
 *
 * Created on 2025. j�lius 24., 13:39
 */

#ifndef UART_PROTOCOL_H
#define	UART_PROTOCOL_H
#include <stdint.h>
#include <xc.h>

#define UART_SYNC_BYTE_EXT 0x55 // sync byte for UART communication device received
#define UART_SYNC_BYTE_INT 0xBB // sync byte for UART communication device sent
#define UART_END_BYTE 0xAA // end byte for UART communication
#define CMD_EXT 0x01 // command from outside device
#define CMD_INT 0x02 // command from inside device



enum {
    UART_STATE_IDLE = 0,
    UART_STATE_READLEN,
    UART_STATE_READDATA,
    UART_STATE_READEND
};

typedef struct {
    uint8_t len; // Length of the data
    uint8_t data[8]; // Data buffer
    uint8_t ext_or_int; // 1 for external, 2 for internal
    uint8_t new_data_available;
} narval_msg_t;
extern narval_msg_t narval_msg;

void uart_process(uint8_t received_byte);

#endif	/* UART_PROTOCOL_H */

