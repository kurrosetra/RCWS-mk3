#include "stm_hal_serial.h"

static void buffer_init(TSerial *serial, char *rxBuffer, const uint16_t rxBufsize, char *txBuffer,
		const uint16_t txBufsize)
{
	ring_buffer_init(&serial->TBufferRx, (uint8_t*) rxBuffer, rxBufsize);
	ring_buffer_init(&serial->TBufferTx, (uint8_t*) txBuffer, txBufsize);
}

void serial_init(TSerial *serial, char *rxBuffer, const uint16_t rxBufsize, char *txBuffer,
		const uint16_t txBufsize, UART_HandleTypeDef *huart)
{
	buffer_init(serial, rxBuffer, rxBufsize, txBuffer, txBufsize);
	serial->huart = huart;

	__HAL_UART_ENABLE_IT(serial->huart, UART_IT_RXNE);
}

void serial_start_transmitting(TSerial *serial)
{
	/* Enable the Transmit Data Register Empty interrupt */
	__HAL_UART_ENABLE_IT(serial->huart, UART_IT_TXE);
}

void serial_write_flush(TSerial *serial)
{
	while (__HAL_UART_GET_IT_SOURCE(serial->huart, UART_IT_TXE))
		;
}

uint8_t USARTx_IRQHandler(TSerial *serial)
{
#if defined(USART_ISR_RXNE) && defined(USART_ISR_TXE) && defined(USART_ISR_TC)
	uint32_t isrflags = READ_REG(serial->huart->Instance->ISR);
#endif	//if defined(USART_ISR_RXNE) && defined(USART_ISR_TXE) && defined(USART_ISR_TC)

#if defined(USART_SR_RXNE) && defined(USART_SR_TXE) && defined(USART_SR_TC)
	uint32_t isrflags = READ_REG(serial->huart->Instance->SR);
#endif	//if defined(USART_SR_RXNE) && defined(USART_SR_TXE) && defined(USART_SR_TC)

	char c;
	uint8_t ret = 0;

	/* UART in mode Receiver ---------------------------------------------------*/
#if defined(USART_ISR_RXNE)
	if ((isrflags & USART_ISR_RXNE) != 0U) {
#endif	//if defined(USART_ISR_RXNE)
#if defined(USART_SR_RXNE)
	if ((isrflags & USART_SR_RXNE) != 0U) {
#endif	//if defined(USART_SR_RXNE)

#if defined(USART_RDR_RDR)
		c = (char) (serial->huart->Instance->RDR & 0xFF);
#endif	//if defined(USART_RDR_RDR)
#if defined(USART_DR_DR)
		c = (char) (serial->huart->Instance->DR & 0xFF);
#endif	//if defined(USART_DR_DR)

		ring_buffer_write(&serial->TBufferRx, (uint8_t*) &c, 1);

		return HAL_UART_RETURN_RX;
	}

	/* UART in mode Transmitter ------------------------------------------------*/
#if defined(USART_ISR_TXE)
	if ((isrflags & USART_ISR_TXE) != 0U) {
#endif	//if defined(USART_ISR_TXE)
#if defined(USART_SR_TXE)
	if ((isrflags & USART_SR_TXE) != 0U) {
#endif	//if defined(USART_SR_TXE)
		if (ring_buffer_available(&serial->TBufferTx) == 0) {

			//no more data available
			/* Disable the UART Transmit Data Register Empty Interrupt */
			__HAL_UART_DISABLE_IT(serial->huart, UART_IT_TXE);

			/* Enable the UART Transmit Complete Interrupt */
			__HAL_UART_ENABLE_IT(serial->huart, UART_IT_TC);

		}
		else {
			ring_buffer_read(&serial->TBufferTx, (uint8_t*) &c, 1);
#if defined(USART_TDR_TDR)
			serial->huart->Instance->TDR = (uint8_t) c;
#endif	//if defined(USART_TDR_TDR)
#if defined(USART_DR_DR)
			serial->huart->Instance->DR = (uint8_t) c;
#endif	//if defined(USART_DR_DR)

			return HAL_UART_RETURN_TX_INGOING;
		}
	}

	/* UART in mode Transmitter end --------------------------------------------*/
#if defined(USART_ISR_TC)
	if ((isrflags & USART_ISR_TC) != 0U) {
#endif	//if defined(USART_ISR_TC)
#if defined(USART_SR_TC)
	if ((isrflags & USART_SR_TC) != 0U) {
#endif	//if defined(USART_SR_TC)

		/* Disable the UART Transmit Complete Interrupt */
		__HAL_UART_DISABLE_IT(serial->huart, UART_IT_TC);

		return HAL_UART_RETURN_TX_DONE;
	}

	return ret;
}

uint16_t serial_available(TSerial *serial)
{
	return ring_buffer_available(&serial->TBufferRx);
}

uint16_t serial_read_free(TSerial *serial)
{
	return ring_buffer_free(&serial->TBufferRx);
}

uint16_t serial_write_free(TSerial *serial)
{
	return ring_buffer_free(&serial->TBufferTx);
}

char serial_peek(TSerial *serial)
{
	uint8_t u08 = 0;

	if (ring_buffer_peek(&serial->TBufferRx, &u08, 0) == HAL_OK)
		return (char) u08;

	return 0;
}

char serial_peekn(TSerial *serial, const uint16_t pos)
{
	uint8_t u08 = 0;

	if (ring_buffer_peek(&serial->TBufferRx, &u08, pos) == HAL_OK)
		return (char) u08;

	return 0;
}

char serial_read(TSerial *serial)
{
	char ch;

	if (ring_buffer_read(&serial->TBufferRx, (uint8_t*) &ch, 1) != HAL_OK)
		return 0;

	return ch;
}

HAL_StatusTypeDef serial_read_str(TSerial *serial, char *str)
{
	uint16_t byteAvailable = ring_buffer_available(&serial->TBufferRx);
	if (byteAvailable > 0)
		return ring_buffer_read(&serial->TBufferRx, (uint8_t*) str, byteAvailable);
	else
		return HAL_ERROR;
}

uint16_t serial_read_until(TSerial *serial, char *str, const char stopChar)
{
	uint16_t byteAvailable = ring_buffer_available(&serial->TBufferRx);
	if (byteAvailable > 0)
		return ring_buffer_read_until(&serial->TBufferRx, (uint8_t*) str, stopChar);
	else
		return 0;
}

void serial_read_flush(TSerial *serial)
{
	ring_buffer_flush(&serial->TBufferRx);
}

void serial_write(TSerial *serial, const char c)
{
	ring_buffer_write(&serial->TBufferTx, (uint8_t*) &c, 1);
	serial_start_transmitting(serial);
}

void serial_write_str(TSerial *serial, const char *str, const uint16_t len)
{
	ring_buffer_write(&serial->TBufferTx, (uint8_t*) str, len);

	serial_start_transmitting(serial);
}

