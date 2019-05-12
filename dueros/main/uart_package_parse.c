#include <stdio.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
//#include "freertos/semphr.h"
#include "esp_log.h"
#include "uart_package_parse.h"
#include "cJSON.h"
#include <esp_spi_flash.h>
#include "driver/uart.h"
#include "uart1.h"
#include "driver/gpio.h"
#include "lightduer_connagent.h"
#include "dueros_app.h"

static const char *TAG = "uart_package";
char duerProfile[DUER_PROFILE_SIZE];
char *pDuerprofile = NULL;
uint16_t dataIndex;
uint8_t packIndex = 0;
extern QueueHandle_t queue_uart_rx;
extern data_msg dataMsg;
extern SemaphoreHandle_t xSemaphore_uart_package;
extern char wifi_ssid[32];
extern char wifi_password[64];

static void parse_uart_package(data_msg *dataMsg);

void parse_uart_package_task(void *pvParameters)
{
	uint16_t checkSum = 0;
	uint16_t i;
	//ESP_LOGI(TAG,"uart_package_parse task init");
	while(1)
	{	
		//等待串口接收队更数据
		//if(xQueueReceive(queue_uart_rx, (void *)&msg, (portTickType)portMAX_DELAY))  
		if( xSemaphoreTake( xSemaphore_uart_package, ( TickType_t )portMAX_DELAY ) == pdTRUE )
		{
			ESP_LOGI(TAG,"msg.package_len.intData: %d", dataMsg.data_len.intData);		
			// 做下数据包校验
			checkSum = dataMsg.data_len.byte[0] + dataMsg.data_len.byte[1] + dataMsg.device_id + dataMsg.function;
			for(i = 0; i < dataMsg.data_len.intData; i++)
			{
				checkSum += dataMsg.buff[i];
			}
			ESP_LOGI(TAG,"Caculater checkSum = : %d", checkSum);
			ESP_LOGI(TAG,"package checkSum = : %d", dataMsg.buff[dataMsg.data_len.intData]);
			if((uint8_t)(checkSum & 0xff) == dataMsg.buff[dataMsg.data_len.intData])
				parse_uart_package(&dataMsg);
		} 
		//vTaskDelay(500 / portTICK_RATE_MS);
	}
	ESP_LOGI(TAG,"uart_package_parse task deleted.");
    vTaskDelete(NULL);
}

/*数据包格式：
    0xff  0x55  len_L len_H   device_id function data0 ... datan checksum
        包头     数据包长         设备号     功能号       数据         校验=(device_id+function+data0+...datan)&0xff
                data0-dataN
*/
static void parse_uart_package(data_msg *dataMsg)
{
	uint8_t tmp[2];

	ESP_LOGI(TAG,"Device id is [  %d ]", dataMsg-> device_id);
	ESP_LOGI(TAG,"funtion is [  %d ]", dataMsg-> function);
	switch(dataMsg-> device_id)
	{
		case 1: // wifi config
			tmp[0] = 0xff;
			tmp[1] = 0x55;
			memcpy(wifi_ssid, dataMsg ->buff, 32);
			memcpy(wifi_password, dataMsg ->buff + 32, 64);
			ESP_LOGI(TAG,"wifi_ssid: %s", wifi_ssid);
			ESP_LOGI(TAG,"wifi_password: %s", wifi_password);        
			ESP_LOGI(TAG, "write param to flash........");
			ESP_ERROR_CHECK(spi_flash_erase_sector(WIFI_PARAM_SECTOR_ADDR));
			ESP_ERROR_CHECK(spi_flash_write(SPI_WIFI_PARAM_ADDR, tmp, 2)); 
			ESP_ERROR_CHECK(spi_flash_write(SPI_WIFI_PARAM_ADDR + 2, dataMsg ->buff, dataMsg ->data_len.intData)); //读出DuerProfile
		break;
		case 2: // duerProfie config
			if(dataMsg-> function == 1){    //uno板RAM不够，分三个数据包发送
				if(packIndex == 0)
				{
					dataIndex = 0;
					pDuerprofile = NULL;
					memset(duerProfile, 0, DUER_PROFILE_SIZE);
				}
				memcpy(&duerProfile[dataIndex], dataMsg ->buff, dataMsg ->data_len.intData);
				dataIndex += dataMsg ->data_len.intData;
				packIndex++;
				if(packIndex == 3)
				{
					packIndex = 0;
					pDuerprofile = duerProfile;
					ESP_ERROR_CHECK(spi_flash_erase_sector(DUER_PROFILE_SECTOR_ADDR));
					ESP_ERROR_CHECK(spi_flash_write(DUER_PROFILE_ADDR, duerProfile, DUER_PROFILE_SIZE));
					ESP_LOGI(TAG, "duer_start, len:%d\n%s", strlen(duerProfile), duerProfile);
					duer_start(duerProfile, strlen(duerProfile));
				}
			}
			else if(dataMsg-> function == 2){ //micro python不存在这个问题
				pDuerprofile = duerProfile;
				memset(duerProfile, 0, DUER_PROFILE_SIZE);
				memcpy(duerProfile, dataMsg ->buff, dataMsg ->data_len.intData);
				ESP_ERROR_CHECK(spi_flash_erase_sector(DUER_PROFILE_SECTOR_ADDR));
				ESP_ERROR_CHECK(spi_flash_write(DUER_PROFILE_ADDR, duerProfile, DUER_PROFILE_SIZE));
				ESP_LOGI(TAG, "duer_start, len:%d\n%s", strlen(duerProfile), duerProfile);
				duer_start(duerProfile, strlen(duerProfile));
			}
		break;
		default:
		break;
	}
}

