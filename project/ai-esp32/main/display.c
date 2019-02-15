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

static const char *TAG  = "DUER screen";

void duer_dcs_screen_init(void)
{
	ESP_LOGE(TAG, "display init .........................................");
	//if you want use screen, init it here;
}

duer_status_t duer_dcs_render_card_handler(baidu_json *payload)
{
	ESP_LOGE(TAG, "display content .........................................");
	char *duerrsp = cJSON_Print((const cJSON *)payload);
	ESP_LOGE(TAG, "%s", duerrsp);

	return DUER_OK;
}









