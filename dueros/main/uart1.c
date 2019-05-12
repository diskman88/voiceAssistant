/* UART 
*/
#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <stdlib.h>
#include "uart_package_parse.h"
#include "uart1.h"

#define GPIO_PIN_UART1_TX  GPIO_NUM_33
#define GPIO_PIN_UART1_RX  GPIO_NUM_32

/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */
static const char *TAG = "uart_events";
//QueueHandle_t queue_uart_rx;
static QueueHandle_t uartx_queue;
bool newPackage = true;
uint16_t readCnt = 0;
uint16_t indexOffset = 0;
data_msg dataMsg;
SemaphoreHandle_t xSemaphore_uart_package = NULL;

/*数据包格式：
    0xff  0x55  len_L len_H   device_id function data0 ... datan checksum
        包头     数据包长         设备号     功能号       数据         校验=(device_id+function+data0+...datan)&0xff
                data0-dataN
*/
static void uartx_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
	uint8_t tmp[120];

    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uartx_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
		    ESP_LOGI(TAG, "uart[%d] event:", event.size);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    //ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    //uart_read_bytes(UART_NUMBER, dtmp, event.size, 100 / portTICK_RATE_MS);
                    //uart_write_bytes(UART_NUMBER, (const char*) dtmp, event.size);
                    if(newPackage) //新包
                    {
						uart_read_bytes(UART_NUMBER, tmp, event.size, 100 / portTICK_RATE_MS);
						if(tmp[0] == 0xff && tmp[1] == 0x55) //包头判断
						{
							ESP_LOGI(TAG, "Get package..........");
						    dataMsg.data_len.byte[0] = tmp[2];
						    dataMsg.data_len.byte[1] = tmp[3];
                            dataMsg.device_id = tmp[4];
                            dataMsg.function = tmp[5];
                            memcpy(dataMsg.buff,&tmp[6], event.size - 6); 
                            if(event.size < 120) //数据包长度小于120，可一次接收完
                            {
                                //ESP_LOGI(TAG, "send queue_uart_rx message.");
                                //发送收到数据包消息(内含包长信息)
                                xSemaphoreGive( xSemaphore_uart_package ); 
                                newPackage = true; 
                                break;                                                              
                            }					
                            else 
                            {   //刚好收完一个数据包
                                if(dataMsg.data_len.intData == event.size - 7) 
                                {
                                    //ESP_LOGI(TAG, "send queue_uart_rx message.");
                                    //发送收到数据包消息(内含包长信息)
                                    xSemaphoreGive( xSemaphore_uart_package ); 
                                    newPackage = true; 
                                    break;     
                                }
                                //数据包总长大于120
                                indexOffset = event.size - 6;
                                readCnt = 0; 
                                newPackage = false;
                            }
						}
						else  //不在包中,且收到的不是包头，清环形缓存和队列
						{
							uart_flush_input(UART_NUMBER);
							xQueueReset(uartx_queue);
                            indexOffset = 0;
                            readCnt = 0;
						}
                    }
					if(!newPackage) //接收数据包剩余数据
					{
                        if(event.size < 120) //数据包结束
						{
							uart_read_bytes(UART_NUMBER, (uint8_t *)(dataMsg.buff + indexOffset + readCnt*120), event.size, 100 / portTICK_RATE_MS);
                            ESP_LOGI(TAG, "send queue_uart_rx message.");
                            //发送收到数据包消息(内含包长信息)
                            readCnt = 0; 
                            indexOffset = 0;
                            xSemaphoreGive( xSemaphore_uart_package );
                            //xQueueSend(queue_uart_rx, (void *)&dataMsg, 100 / portTICK_RATE_MS); 
                            newPackage = true; 
						}	
						else //=120
						{
						   ESP_LOGI(TAG, "package len: %d", readCnt);
							uart_read_bytes(UART_NUMBER, (uint8_t *)(dataMsg.buff + indexOffset + readCnt*120), 120, 100 / portTICK_RATE_MS);
							readCnt++;
                            if((readCnt*120 + indexOffset) == dataMsg.data_len.intData+1)
                            {
                                //ESP_LOGI(TAG, "send queue_uart_rx message.");
                                //发送收到数据包消息(内含包长信息)
                                readCnt = 0; 
                                xSemaphoreGive( xSemaphore_uart_package );
                                //xQueueSend(queue_uart_rx, (void *)&dataMsg, 100 / portTICK_RATE_MS); 
                                newPackage = true;                                
                            }
						}
					}
					break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    //ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUMBER);
                    xQueueReset(uartx_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    //ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUMBER);
                    xQueueReset(uartx_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    //ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    //ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    //ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
					/*
                    uart_get_buffered_data_len(UART_NUMBER, &buffered_size);
                    int pos = uart_pattern_pop_pos(UART_NUMBER);
                    //ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(UART_NUMBER);
                    } else {
                        uart_read_bytes(UART_NUMBER, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(UART_NUMBER, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        //ESP_LOGI(TAG, "read data: %s", dtmp);
                        //ESP_LOGI(TAG, "read pat : %s", pat);
                    }*/
                    break;
                //Others
                default:
                    //ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
	}
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void uartx_init(void)
{
	//queue_uart_rx = xQueueCreate(1, sizeof(data_msg));
    vSemaphoreCreateBinary( xSemaphore_uart_package );	

   // esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUMBER, &uart_config);
    //Set UART log level
   // esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
	uart_set_pin(UART_NUMBER, GPIO_PIN_UART1_TX, GPIO_PIN_UART1_RX, -1, -1);
    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUMBER, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uartx_queue, 0);

    //Set uart pattern detect function.
    //uart_enable_pattern_det_intr(UART_NUMBER, '+', PATTERN_CHR_NUM, 10000, 10, 10);
    //Reset the pattern queue length to record at most 20 pattern positions.
    //uart_pattern_queue_reset(UART_NUMBER, 20);

    //Create a task to handler UART event from ISR
    xTaskCreate(uartx_event_task, "uart1_event_task", 2048, NULL, 12, NULL);
    // xTaskCreate(parse_uart_package_task, "user_parse_package", 4096, NULL, 4, NULL);
}

void uart_send_package(uint8_t device_id, uint8_t function, char *buff, uint16_t buff_len)
{
	uint8_t package_buff[1024] = {0};
	uint8_t len_L = (uint8_t)(buff_len & 0xff);
	uint8_t len_H = (uint8_t)(buff_len >> 8);
	uint32_t checksum = len_L + len_H + device_id + function;

	package_buff[0] = 0xff;
	package_buff[1] = 0x55;
	package_buff[2] = len_L;
	package_buff[3] = len_H;
	package_buff[4] = device_id;
    package_buff[5] = function;
	for(uint16_t i = 0; i < buff_len; i++)
	{
		package_buff[i + 6] = *(buff + i);
		checksum += *(buff + i);
	}
	package_buff[buff_len + 6] = (uint8_t)(checksum & 0xff);
	uart_write_bytes(UART_NUMBER, (const char*)package_buff, (buff_len + 7));
}
