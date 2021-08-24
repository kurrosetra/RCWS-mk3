#ifndef RING_BUFFER_CONFIG_H_
#define RING_BUFFER_CONFIG_H_

#include "main.h"

/*
 * example buffer usage

 //global variables
 uint8_t rxDBuffer[RING_BUFFER_RX_SIZE];
 uint8_t txDBuffer[RING_BUFFER_TX_SIZE];
 TSerial debug;

 //initialization
 serial_init(&debug, (char*) &rxDBuffer, sizeof(rxDBuffer), (char*) &txDBuffer,sizeof(txDBuffer), &hlpuart1);

 //irq handler
 void UART4_IRQHandler(void)
 void USART2_IRQHandler(void)
 void LPUART1_IRQHandler(void)
 {
 uint8_t ret = USARTx_IRQHandler(&debug);
 }

 *
 */

#define RING_BUFFER_RX_SIZE		256
#define RING_BUFFER_TX_SIZE		256

#endif /* RING_BUFFER_CONFIG_H_ */
