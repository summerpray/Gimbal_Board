#ifndef BSP_SPI_H
#define BSP_SPI_H
#include "struct_typedef.h"

extern void SPI1_DMA_init(unsigned int tx_buf, unsigned int rx_buf, uint16_t num);
extern void SPI1_DMA_enable(unsigned int tx_buf, unsigned int rx_buf, uint16_t ndtr);

#endif
