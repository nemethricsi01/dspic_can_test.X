#ifndef CAN_H
#define CAN_H

#include <stdint.h>
#include <xc.h>

/* Macro used to write filter/mask ID to Register CiRXMxSID and
CiRXFxSID. For example to setup the filter to accept a value of
0x123, the macro when called as CAN_FILTERMASK2REG_SID(0x123) will
write the register space to accept message with ID 0x123
USE FOR STANDARD MESSAGES ONLY */
#define CAN_FILTERMASK2REG_SID(x) ((x & 0x07FF)<< 5)
/* the Macro will set the "MIDE" bit in CiRXMxSID */
#define CAN_SETMIDE(sid) (sid | 0x0008)
/* the macro will set the EXIDE bit in the CiRXFxSID to
accept extended messages only */
#define CAN_FILTERXTD(sid) (sid | 0x0008)
/* the macro will clear the EXIDE bit in the CiRXFxSID to
accept standard messages only */
#define CAN_FILTERSTD(sid) (sid & 0xFFF7)

extern unsigned int canTxBuff[4][8]__attribute__((aligned(4 * 16)));
extern unsigned int canRxBuff[32][8]__attribute__((aligned(32 * 16)));



void can_init(void);
void can_send(uint16_t id, uint8_t *data, uint8_t dlc);
void can_get_data_frombuff(uint8_t *data, uint8_t *dlc);


#endif // CAN_H