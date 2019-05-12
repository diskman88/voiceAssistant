#ifndef _UART1_H_
#define _UART1_H_

#define UART_NUMBER UART_NUM_1
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

void uartx_init(void);
void uart_send_package(uint8_t device_id, uint8_t function, char *buff, uint16_t buff_len);

#endif

