#ifndef BSP_RC_H
#define BSP_RC_H
#include "struct_typedef.h"

extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);

extern void Assitant_Init(uint8_t *RX1_buf, uint8_t *RX2_buf, uint16_t dma1_buf_num);
#endif
