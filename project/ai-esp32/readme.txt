ai_esp32基于doeros例程做修改：

2018.10.09下载的版本似乎components文件夹下audio_pipeline有改动，录音时会出错resample错误，把旧版本的文件夹拷过来。

1、用自已注册的profile

2、录音实现有问题，暂做以下修改：
   1) 录音开始，制作wav文件头。
        在dueros_task()的while循环中加入以下代码：
        ...
        else if (duer_msg.type == DUER_CMD_START) {
                        ...
                        wav_header_t *wav_info = (wav_header_t *) audio_malloc(sizeof(wav_header_t));
                        wav_head_init(wav_info, 16000, 16, 1);
                        wav_head_size(wav_info, 0x7A120);
                        memcpy(voiceData, wav_info, sizeof(wav_header_t));
                        if(file)
                            fwrite(voiceData, 1,  sizeof(wav_header_t), file);
                        duer_voice_send(voiceData, sizeof(wav_header_t));
                        audio_free(wav_info);
                        while (1) {
        ...
        包含头文件: #include "wav_head.h"
        作用：添加wav文件头，云端似乎需要一个wav文件，而不是单纯音频流。因不知音频流结束时间，文件的大小无法确定，设为0x7a120(500k).太大了有问题。
   2) 不使用adf的录音引擎，只创建一录音管道，暂不使用VAD。
   3) 使用一个定时器录音，时间3秒。录音按键触发。
3、语音回放有问题，暂未解决。

2、设备资源注册，实现对设备操控
    加入act.c act.h  labplus_uart.c labplus_uart.h
    doeros_task.c的 
    duer_event_hook()中加入 bits_regist_resource()注册设备资源。

4、串口0用作调试，增加串口1做为模块交互接口：
    app_main()中加入串口初始化代码：void uart1_init(void);

    为防止引脚冲突，修改task.c 中button外设初始化代码，去掉TOUCH8\9的引脚初始化。
    .touch_mask = TOUCH_PAD_SEL4 | TOUCH_PAD_SEL7   //| TOUCH_PAD_SEL8 | TOUCH_PAD_SEL9,

5、串口数据包实现。
     

6、加大初始化的音量设置。
    修改duer_audio_wrapper.c中setup_play()中默认音量设置，由45改为70
    esp_audio_vol_set(player, 70);

注：使用SD卡调试调试音频时，sd卡的mount需要时间，必须等mount成功后，才能进行文件相关操作，因此，把dueros_task()下的这段代码
        #if RECORD_DEBUG
            file = fopen("/sdcard/rec_debug_1.wav", "w+");
            if (NULL == file) {
                ESP_AUDIO_LOGW(TAG, "open rec_debug_1.wav failed,[%d]", __LINE__);
            }
        #endif
    移到duer_service_create()的sd卡mount代码后。同时把file高为全局变量。把ESP_AUDIO_LOGW改为ESP_LOGI，把文件名修改为recDebug.wav，原
    来的文件名不知是太长还是有下划线，有问题。

编译：
   make menuconfig里设置好串口和wifi，同时把分区表设置为用户指定的分区表，分区表设为用户定义，在项目录下：partitions_singleapp.csv
   make flash monitor编译

添加wm8978驱动：
1、创建驱动文件 wm8978.c wm8978.h
2、修改esp-adf/components/audio_hal/下audio_hal.c文件
     在static struct audio_hal audio_hal_codecs_default[] 中加入wm8978的函数列表，注意添加#include "wm8978.h"
3、在audio_hal.h中加入：
    #define AUDIO_HAL_WS8978_DEFAULT(){                     \
        .adc_input  = AUDIO_HAL_ADC_INPUT_LINE1,        \
        .dac_output = AUDIO_HAL_DAC_OUTPUT_ALL,         \
        .codec_mode = AUDIO_HAL_CODEC_MODE_BOTH,        \
        .i2s_iface = {                                  \
            .mode = AUDIO_HAL_MODE_SLAVE,               \
            .fmt = AUDIO_HAL_I2S_NORMAL,                \
            .samples = AUDIO_HAL_48K_SAMPLES,           \
            .bits = AUDIO_HAL_BIT_LENGTH_16BITS,        \
        },                                              \
};

4 修改esp-adf\components\audio_hal下的component.mk文件，加入：
   ./driver/wm8978 编译时会编译wm8978文件夹下的文件
5、修改dueros_task.c中


