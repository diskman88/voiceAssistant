#ifndef UART_PACKAGE_PARSE_H
#define UART_PACKAGE_PARSE_H

#define DEVICE_BTN 		  0x01
#define DEVICE_RMT 		  0x02
#define DEVICE_BUZZER     0x03
#define DEVICE_RGB_LED    0x04
#define DEVICE_MOTOR      0x05
#define DEVICE_BLUE_SPP   0x06
#define DEVICE_MQTT       0x07

typedef enum{
	SRC_UART,
    SRC_MQTT,
}data_src;

typedef union{
	uint8_t byte[2];
	uint16_t intData;
}byte_int;

typedef struct{
	byte_int data_len;
	uint8_t device_id;
	uint8_t function;
	char buff[2000];
}data_msg;

extern void parse_uart_package_task(void *pvParameters);

#endif //end UART_PACKAGE_PARSE_H
