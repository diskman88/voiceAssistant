#include "act.h"
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include "esp_log.h"
#include "lightduer_connagent.h"
#include "lightduer_dcs.h"
#include "baidu_json.h"
#include "driver/uart.h"
#include "cJSON.h"

#define LINGO_ID 0x08
#define CMD_SET_ACTOR 0x01
static const char *TAG = "bits resource";
#define SUCCESS     1
#define FAIE        -1

#define ON          1
#define OFF         0

static int lamp_on_off = OFF;
static int lamp_color = 4;
static int lamp_brightness = 0;

static  void sendPackage(uint8_t lingoId, uint8_t command, uint8_t* sourceBuff, uint8_t length);
static  int duerMsgProccess(duer_msg_t *msg, duer_addr_t *addr, char *deviceName, char *valueName, int *value);

static duer_status_t bits_set_lamp_brightness(duer_context ctx, duer_msg_t *msg, duer_addr_t *addr)
{
	duerMsgProccess(msg, addr, "lamp", "brightness", &lamp_brightness);
    return DUER_OK;
}

static duer_status_t bits_set_lamp_OnOff(duer_context ctx, duer_msg_t *msg, duer_addr_t *addr)
{
	duerMsgProccess(msg, addr, "lamp", "on_off", &lamp_on_off);
    return DUER_OK;	
}

static duer_status_t bits_set_lamp_color(duer_context ctx, duer_msg_t *msg, duer_addr_t *addr)
{
	duerMsgProccess(msg, addr, "lamp", "color", &lamp_color);
    return DUER_OK;	
}

void bits_regist_resource(void){
	duer_res_t res[] = {
		  {DUER_RES_MODE_DYNAMIC, DUER_RES_OP_PUT | DUER_RES_OP_GET, "bits_lampBrightness",.res.f_res = bits_set_lamp_brightness},
		  {DUER_RES_MODE_DYNAMIC, DUER_RES_OP_PUT | DUER_RES_OP_GET, "bits_lampOnOff",.res.f_res = bits_set_lamp_OnOff},
		  {DUER_RES_MODE_DYNAMIC, DUER_RES_OP_PUT | DUER_RES_OP_GET, "bits_lampColor",.res.f_res = bits_set_lamp_color},
	};

	duer_add_resources(res, sizeof(res) / sizeof(res[0]));
}

/* sendPackage()
	制作并发送数据
	输入：
	uint8_t lingoId： lingoId
	uint8_t command：命令
	uint8_t* sourceBuff：源数据
	uint8_t* destinationBuff：制作包缓存
	uint8_t length：源数据长
	返回值：无
*/
static void sendPackage(uint8_t lingoId, uint8_t command, uint8_t* sourceBuff, uint8_t length)
{
	uint8_t i; 
	uint16_t sum;
	uint8_t buffer[64];
	sum = 0;
	buffer[0] = 0xff;
	buffer[1] = 0x55;
	buffer[2] = length + 2;
	buffer[3] = lingoId;
	buffer[4] = command;
	for (i = 0; i < length; i++)
	{
		buffer[i+5] = *(sourceBuff + i);
		sum += *(sourceBuff + i);
	}
	sum += length + 2;
	sum += lingoId;
	sum += command;
	buffer[5 + length] = (uint8_t)(sum & 0xff); //??length + lingoId + command + data
	uart_write_bytes(UART_NUM_1, (const char*) buffer, length+6);
}

static  int duerMsgProccess(duer_msg_t *msg, duer_addr_t *addr, char *deviceName, char *valueName, int *value)
{
	int ret = 1;
	int msg_code = DUER_MSG_RSP_CHANGED;
	cJSON *cmd_json = NULL;
	cJSON *value_json = NULL;
	cJSON *set_cmd_json = NULL;
	cJSON *deviceName_json = NULL;
	cJSON *valueName_json = NULL;
	cJSON *vale_json = NULL;
	char *obj_json_char = NULL;

    if (msg == NULL || addr == NULL) {  
        ESP_LOGI(TAG, "Argument Error");
        duer_response(msg, msg_code, NULL, 0);
    }

    if (msg->msg_code == DUER_MSG_REQ_PUT) {   //下发控制命令
		ESP_LOGI(TAG, 	"msg->payload: %s	", msg->payload);
		cmd_json = cJSON_Parse((char *)msg->payload);
	   if (cmd_json != NULL) {  //获得了设置数据
			value_json = cJSON_GetObjectItem((char *)cmd_json, "value");
			if (value_json != NULL) {
				set_cmd_json = cJSON_CreateObject();
				cJSON_AddStringToObject(set_cmd_json, "deviceName", deviceName);
				cJSON_AddStringToObject(set_cmd_json, "valueName", valueName);
				cJSON_AddStringToObject(set_cmd_json, "value", value_json->valuestring);
				obj_json_char = baidu_json_Print(set_cmd_json);
				//ESP_LOGI(TAG, "%s", obj_json_char);
				uart_write_bytes(UART_NUM_1, (const char*)obj_json_char, strlen(obj_json_char));
				cJSON_free((void *)obj_json_char);
				cJSON_free((void *)set_cmd_json);

				ret = 0;
            }
			else //无键值
			{
            	ESP_LOGI(TAG, "Get value keywork failed");
			}
	   	}
		cJSON_Delete(cmd_json);
    }

    if (msg->msg_code == DUER_MSG_REQ_GET) {  //获取设备状态值
        ESP_LOGI(TAG, "Get switch status");
        msg_code = DUER_MSG_RSP_CONTENT;
		duer_response(msg, msg_code, NULL, 0);
        // if (on_flag > 0) {
        //     duer_response(msg, msg_code, "true", 4);
        // } else {
        //     duer_response(msg, msg_code, "false", 5);
        // }
    }

	return ret;
}

/* static  int duerMsgProccess(duer_msg_t *msg, duer_addr_t *addr, char *deviceName, char *valueName, int *value)
{
	int ret = 1;
	int msg_code = DUER_MSG_RSP_CHANGED;
	baidu_json *cmd_json = NULL;
	baidu_json *value_json = NULL;
	baidu_json *set_cmd_json = NULL;
	baidu_json *deviceName_json = NULL;
	baidu_json *valueName_json = NULL;
	baidu_json *vale_json = NULL;
	char *obj_json_char = NULL;

    if (msg == NULL || addr == NULL) {  
        ESP_LOGI(TAG, "Argument Error");
        duer_response(msg, msg_code, NULL, 0);
    }

    if (msg->msg_code == DUER_MSG_REQ_PUT) {   //下发控制命令
		ESP_LOGI(TAG, 	"msg->payload: %s	", msg->payload);
		cmd_json = baidu_json_Parse((char *)msg->payload);
	   if (cmd_json != NULL) {  //获得了设置数据
			value_json = baidu_json_GetObjectItem((char *)cmd_json, "value");
			if (value_json != NULL) {
				set_cmd_json = baidu_json_CreateObject();
				deviceName_json = baidu_json_CreateString("deviceName");
				valueName_json = baidu_json_CreateString("valueName");
				vale_json = baidu_json_CreateString("value");
				baidu_json_AddItemToObject(set_cmd_json, deviceName, deviceName_json);
				baidu_json_AddItemToObject(set_cmd_json, valueName, valueName_json);
				baidu_json_AddItemToObject(set_cmd_json, value_json->valuestring, valueName_json);
				obj_json_char = baidu_json_Print(set_cmd_json);
				//ESP_LOGI(TAG, "%s", obj_json_char);
				//uart_write_bytes(UART_NUM_1, (const char*)obj_json_char, strlen(obj_json_char));
				baidu_json_release((void *)obj_json_char);
				baidu_json_release((void *)set_cmd_json);
				baidu_json_release((void *)deviceName_json);
				baidu_json_release((void *)valueName_json);
				baidu_json_release((void *)vale_json);

				ret = 0;
            }
			else //无键值
			{
            	ESP_LOGI(TAG, "Get value keywork failed");
			}
	   	}
		baidu_json_Delete(cmd_json);
    }

    if (msg->msg_code == DUER_MSG_REQ_GET) {  //获取设备状态值
        ESP_LOGI(TAG, "Get switch status");
        msg_code = DUER_MSG_RSP_CONTENT;
        // if (on_flag > 0) {
        //     duer_response(msg, msg_code, "true", 4);
        // } else {
        //     duer_response(msg, msg_code, "false", 5);
        // }
    }

	return ret;
} */




