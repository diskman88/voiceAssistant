/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * 改动：
 *  1. es8388_start() ES8388_DACCONTROL16 由09改为00
 *  2. es8388_init() 中mic gain ES8388_ADCCONTROL1 bb改为44
 */

#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "es8388.h"
#include "board.h"

static const char *ES_TAG = "ES8388_DRIVER";

#define ES_ASSERT(a, format, b, ...) \
    if ((a) != 0) { \
        ESP_LOGE(ES_TAG, format, ##__VA_ARGS__); \
        return b;\
    }

static const i2c_config_t es_i2c_cfg = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = IIC_DATA,
    .scl_io_num = IIC_CLK,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000
};

static int es_write_reg(uint8_t slave_add, uint8_t reg_add, uint8_t data)
{
    int res = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    res |= i2c_master_start(cmd);
    res |= i2c_master_write_byte(cmd, slave_add, 1 /*ACK_CHECK_EN*/);
    res |= i2c_master_write_byte(cmd, reg_add, 1 /*ACK_CHECK_EN*/);
    res |= i2c_master_write_byte(cmd, data, 1 /*ACK_CHECK_EN*/);
    res |= i2c_master_stop(cmd);
    res |= i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    ES_ASSERT(res, "es_write_reg error", -1);
    return res;
}

static int es_read_reg(uint8_t reg_add, uint8_t *pData)
{
    uint8_t data;
    int res = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    res |= i2c_master_start(cmd);
    res |= i2c_master_write_byte(cmd, ES8388_ADDR, 1 /*ACK_CHECK_EN*/);
    res |= i2c_master_write_byte(cmd, reg_add, 1 /*ACK_CHECK_EN*/);
    res |= i2c_master_stop(cmd);
    res |= i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    res |= i2c_master_start(cmd);
    res |= i2c_master_write_byte(cmd, ES8388_ADDR | 0x01, 1 /*ACK_CHECK_EN*/);
    res |= i2c_master_read_byte(cmd, &data, 0x01/*NACK_VAL*/);
    res |= i2c_master_stop(cmd);
    res |= i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    ES_ASSERT(res, "es_read_reg error", -1);
    *pData = data;
    return res;
}

static int i2c_init()
{
    int res;
    res = i2c_param_config(I2C_NUM_0, &es_i2c_cfg);
    res |= i2c_driver_install(I2C_NUM_0, es_i2c_cfg.mode, 0, 0, 0);
    ES_ASSERT(res, "i2c_init error", -1);
    return res;
}

void es8388_read_all()
{
    for (int i = 0; i < 50; i++) {
        uint8_t reg = 0;
        es_read_reg(i, &reg);
        ets_printf("%x: %x\n", i, reg);
    }
}

int es8388_write_reg(uint8_t reg_add, uint8_t data)
{
    return es_write_reg(ES8388_ADDR, reg_add, data);
}

/**
 * @brief Configure ES8388 ADC and DAC volume. Basicly you can consider this as ADC and DAC gain
 *
 * @param mode:             set ADC or DAC or all
 * @param volume:           -96 ~ 0              for example Es8388SetAdcDacVolume(ES_MODULE_ADC, 30, 6); means set ADC volume -30.5db
 * @param dot:              whether include 0.5. for example Es8388SetAdcDacVolume(ES_MODULE_ADC, 30, 4); means set ADC volume -30db
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
static int es8388_set_adc_dac_volume(int mode, int volume, int dot)
{
    int res = 0;
    if ( volume < -96 || volume > 0 ) {
        ESP_LOGW(ES_TAG, "Warning: volume < -96! or > 0!\n");
        if (volume < -96)
            volume = -96;
        else
            volume = 0;
    }
    dot = (dot >= 5 ? 1 : 0);
    volume = (-volume << 1) + dot;
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL8, volume);
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL9, volume);  //ADC Right Volume=0db
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL5, volume);
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL4, volume);
    }
    return res;
}


/**
 * @brief Power Management
 *
 * @param mod:      if ES_POWER_CHIP, the whole chip including ADC and DAC is enabled
 * @param enable:   false to disable true to enable
 *
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
int es8388_start(es_module_t mode)
{
    int res = 0;
    // uint8_t prev_data = 0, data = 0;
    // es_read_reg(ES8388_DACCONTROL21, &prev_data);
    // if (mode == ES_MODULE_LINE) {
    //     res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x09); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2 by pass enable
    //     res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x50); // left DAC to left mixer enable  and  LIN signal to left mixer enable 0db  : bupass enable
    //     res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x50); // right DAC to right mixer enable  and  LIN signal to right mixer enable 0db : bupass enable
    //     res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0xC0); //enable adc
    // } else {
    //     res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80);   //enable dac
    // }
    // es_read_reg(ES8388_DACCONTROL21, &data);
    // if (prev_data != data) {
    //     res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xF0);   //start state machine
    //     // res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x16);
    //     // res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
    //     res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00);   //start state machine
    // }
    // if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
    //     res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x00);   //power up adc and line in
    // }
    // if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
    //     res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3c);   //power up dac and line out
    //     res |= es8388_set_voice_mute(false);
    //     ESP_LOGD(ES_TAG, "es8388_start default is mode:%d", mode);
    // }

    return res;
}

/**
 * @brief Power Management
 *
 * @param mod:      if ES_POWER_CHIP, the whole chip including ADC and DAC is enabled
 * @param enable:   false to disable true to enable
 *
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
int es8388_stop(es_module_t mode)
{
    int res = 0;

    //     if (mode == ES_MODULE_LINE) {
    //         res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80); //enable dac
    //         res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x00); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2
    //         res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x90); // only left DAC to left mixer enable 0db
    //         res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x90); // only right DAC to right mixer enable 0db
    //         return res;
    //     }
    //     if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
    //         res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x00);
    //         res |= es8388_set_voice_mute(true); //res |= Es8388SetAdcDacVolume(ES_MODULE_DAC, -96, 5);      // 0db
    //         //res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0xC0);  //power down dac and line out
    //     }
    //     if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
    //         //res |= Es8388SetAdcDacVolume(ES_MODULE_ADC, -96, 5);      // 0db
    //         res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0xFF);  //power down adc and line in
    //     }
    //     if (mode == ES_MODULE_ADC_DAC) {
    //         res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x9C);  //disable mclk
    // //        res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x00);
    // //        res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x58);
    // //        res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xF3);  //stop state machine
    //     }

    return res;
}


/**
 * @brief Config I2s clock in MSATER mode
 *
 * @param cfg.sclkDiv:      generate SCLK by dividing MCLK in MSATER mode
 * @param cfg.lclkDiv:      generate LCLK by dividing MCLK in MSATER mode
 *
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
int es8388_i2s_config_clock(es_i2s_clock_t cfg)
{
    int res = 0;
    res |= es_write_reg(ES8388_ADDR, ES8388_MASTERMODE, cfg.sclk_div);
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, cfg.lclk_div);  //ADCFsMode,singel SPEED,RATIO=256
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL2, cfg.lclk_div);  //ADCFsMode,singel SPEED,RATIO=256
    return res;
}

esp_err_t es8388_deinit(void)
{
    int res = 0;
    res = es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xFF);  //reset and stop es8388
    return res;
}

/**
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
esp_err_t es8388_init(audio_hal_codec_config_t *cfg)
{
    int res = 0;
    int chipMode;
#ifdef CONFIG_ESP_LYRAT_V4_3_BOARD
    #include "headphone_detect.h"
    headphone_detect_init();
#endif
    
    res = i2c_init(); // ESP32 in master mode
    chipMode = cfg -> codec_mode;
     // res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x86);        
    /* 1 芯片通用设置 须按顺序一步一步设置 */
    res |= es_write_reg(ES8388_ADDR, ES8388_MASTERMODE, 0x00); //cfg->i2s_iface.mode); //Set Chip to Master/Slave Mode Reg 0x08 = 0x00 (Slave Mode) 0x80(master mode)
    res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xF3); //Power down DEM and STM Reg 0x02 = 0xF3 
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80); //Set same LRCK Reg 0x2B = 0x80   
    res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x05); //Set Chip to Play&Record Mode Reg 0x00 = 0x05 
    res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x40); //Power Up Analog and Ibias Reg 0x01= 0x40 
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x00);   
    /* 2 按需求开启相应功能模块电源 */
    if(chipMode == AUDIO_HAL_CODEC_MODE_ENCODE || chipMode == AUDIO_HAL_CODEC_MODE_BOTH)
    {
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x00); //Power up ADC / Analog Input / Micbias for Record Reg 0x03 = 0x00 
        ESP_LOGE(ES_TAG, "1.0 ADC power.........");
    }            
    if(chipMode == AUDIO_HAL_CODEC_MODE_DECODE || chipMode == AUDIO_HAL_CODEC_MODE_BOTH)
    {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3C); //Power up DAC and Enable LOUT/ROUIT Reg 0x04 = 0x3C 
        ESP_LOGE(ES_TAG, "1.1 DAC power.........");
    }
       
    if(chipMode == AUDIO_HAL_CODEC_MODE_LINE_IN) //bypass mode
    {
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x3F);//Power down ADC, Power up Analog Input for Bypass Reg 0x03 = 0x3F 
        res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0xFC);//Power down DAC, Power up Analog Output for Bypass Reg 0x04 = 0xFC 
        ESP_LOGE(ES_TAG, "1.2 analog in/out power.........");
    }
  
    /* 3 adc 设置 codec 和 adc模式下须设置以下项目*/
    if(chipMode == AUDIO_HAL_CODEC_MODE_ENCODE || chipMode == AUDIO_HAL_CODEC_MODE_BOTH)
    {
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL2, 0x00); //Select Analog input channel for ADC Reg 0x0A = 0x00:L/Rin1 0x50:L/Rin2 0xf0 = diff
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, 0x88); //Select Analog Input PGA Gain for ADC Reg 0x09 = 0x55 (+15 dB)0x88 (24dB max)
        //res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL3, 0x04); //diff select: 0x80:R2-L2 0x00:R1-L1
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, 0x0C); //Set SFI for ADC Reg 0x0c = 0x0F (DSP/PCM 16Bit) 0x0C(I2S 16bit) 0x00(I2S 24Bit)
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, 0x02); //Select MCLK / LRCK ratio for ADC Reg 0x0D = 0x02 (256)
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL8, 0x00); //Set ADC Digital Volume Reg 0x10 = 0x00 (0dB)
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL9, 0x00); //Set ADC Digital Volume Reg 0x11 = 0x00 (0dB)
        if(chipMode == AUDIO_HAL_CODEC_MODE_ENCODE)
            res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL7, 0x30);//UnMute ADC Reg 0x0F = 0x30 (ADC unmute)
        //adc alc set. Select ALC for ADC Record Reg 0x12 to Reg 0x16 Refer toALC description
        // res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL10, 0xe2); // Reg 0x12 = 0xe2 (ALC enable, PGA Max. Gain=23.5dB, Min. Gain=0dB) 
        // res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL11, 0xa0); // Reg 0x13 = 0xc0 (ALC Target=-4.5dB, ALC Hold time =0 mS) 
        // res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL12, 0x12); // Reg 0x14 = 0x12 (Decay time =820uS , Attack time = 416 uS) 
        // res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL13, 0x06); // Reg 0x15 = 0x06 (ALC mode) 
        // res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL14, 0xc3); // Reg 0x16 = 0xc3 (nose gate = -40.5dB, NGG = 0x01(mute ADC)) 
        ESP_LOGE(ES_TAG, "2.0 adc set finish .........");
    }
    if(chipMode == AUDIO_HAL_CODEC_MODE_LINE_IN)
    {
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL2, 0x50); //Select Analog input channel for ADC Reg 0x0A = 0x00:L/Rin1 0x50:L/Rin2 0xf0 = diff
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, 0x55); //Select Analog Input PGA Gain for ADC Reg 0x09 = 0x55 (+15 dB)0x88 (24dB max)
        ESP_LOGE(ES_TAG, "2.1 analog set for bypass mode finish .........");
    }
 
    /* 4 dac 设置 codec 和 dac模式下须设置以下项目*/
    if(chipMode == AUDIO_HAL_CODEC_MODE_DECODE || chipMode == AUDIO_HAL_CODEC_MODE_BOTH)
    {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, 0x18); //Set SFI for DAC Reg 0x17 = 0x1E: (DSP/PCM 16Bit) 0x18:(I2S 16bit)
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL2, 0x02); //Select MCLK / LRCK ratio for DAC Reg 0x18 = 0x02 (256)
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL4, 0x00); //Set DAC Digital Volume Reg 0x1A = 0x00 (0dB max) 
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL5, 0x00); //Set DAC Digital Volume Reg 0x1B = 0x00 (0dB max)
        if(chipMode == AUDIO_HAL_CODEC_MODE_DECODE)
            res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x32);//UnMute DAC Reg 0x19 = 0x32 (DAC unmute)
        ESP_LOGE(ES_TAG, "3 dac set finish.........");
    }
    /* 5 output 设置 */
    if(chipMode == AUDIO_HAL_CODEC_MODE_DECODE || chipMode == AUDIO_HAL_CODEC_MODE_BOTH)
    {
        //res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x09); //Select LMIXSEL src. Reg 0x26 = 0x00: L1/R1  0x09: L2/R2 to mixer
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0xB8); //left mixer src selct and gain set  Reg 0x27 = 0xB8(codec dac mode) 0x50(bypass mode)
        // res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL18, 0x38); //Reg 0x28 = 0x38 
        // res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL19, 0x38); //Reg 0x29 = 0x38
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0xB8); //right mixer src selct and gain set Reg 0x2A = 0xB8(codec dac mode) 0x50(bypass mode)
        ESP_LOGE(ES_TAG, "4 output mixer src select dac finish.........");
    }
    if(chipMode == AUDIO_HAL_CODEC_MODE_LINE_IN)
    {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x09); //Select LMIXSEL src. Reg 0x26 = 0x00: L1/R1  0x09: L2/R2 to mixer
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x50); //left mixer src selct and gain set  Reg 0x27 = 0xB8(codec dac mode) 0x50(bypass mode)
        // res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL18, 0x38); //Reg 0x28 = 0x38 
        // res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL19, 0x38); //Reg 0x29 = 0x38
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x50); //right mixer src selct and gain set Reg 0x2A = 0xB8(codec dac mode) 0x50(bypass mode)
        ESP_LOGE(ES_TAG, "4 output mixer select L/R finish.........");
    }
    /* 6 output volume set */
    if(chipMode != AUDIO_HAL_CODEC_MODE_ENCODE)
    {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL24, 0x1E); //Set LOUT/ROUT Volume LOUR1:Reg 0x2E = 0x1E (0dB)   
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL25, 0x1E); //Set LOUT/ROUT Volume ROUT1:Reg 0x2F = 0x1E (0dB)
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL26, 0x1E); //Set LOUT/ROUT Volume LOUT2;Reg 0x30 = 0x1E (0dB)
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL27, 0x1E); //Set LOUT/ROUT Volume ROUT2:Reg 0x31 = 0x1E (0dB)
        ESP_LOGE(ES_TAG, "5 output volume set finish.........");
    }

    /* 7 根据芯片模式选择相应功能电源开 */
    if(chipMode == AUDIO_HAL_CODEC_MODE_BOTH)
    {
        res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00); //Power up DEM and STMReg 0x02 = 0x00
        ESP_LOGE(ES_TAG, "6.0 all power open .........");
    }
    if(chipMode == AUDIO_HAL_CODEC_MODE_ENCODE)
    {
        res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x55); //Power up DEM and STMReg 0x02 = 0x00
        ESP_LOGE(ES_TAG, "6.1 adc power open .........");
    }
    if(chipMode == AUDIO_HAL_CODEC_MODE_DECODE)
    {
        res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xAA); //Power up DEM and STMReg 0x02 = 0x00
        ESP_LOGE(ES_TAG, "6.2 dac power open .........");
    }
    if(chipMode == AUDIO_HAL_CODEC_MODE_LINE_IN)
    {
        res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xF0); //Power up DEM and STMReg 0x02 = 0x00
         ESP_LOGE(ES_TAG, "6.3 bypass power open .........");      
    }
    //ESP_LOGI(ES_TAG, "init,out:%02x, in:%02x", cfg->dac_output, cfg->adc_input);
    es8388_read_all();
    return res;
}

/**
 * @brief Configure ES8388 I2S format
 *
 * @param mode:           set ADC or DAC or all
 * @param bitPerSample:   see Es8388I2sFmt
 *
 * @return
 *     - (-1) Error
 *     - (0)  Success
 */
int es8388_config_fmt(es_module_t mode, es_i2s_fmt_t fmt)
{
    int res = 0;
    uint8_t reg = 0;
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        res = es_read_reg(ES8388_ADCCONTROL4, &reg);
        reg = reg & 0xfc;
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, reg | fmt);
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res = es_read_reg(ES8388_DACCONTROL1, &reg);
        reg = reg & 0xf9;
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, reg | (fmt << 1));
    }
    return res;
}

/**
 * @param volume: 0 ~ 100
 *
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
int es8388_set_voice_volume(int volume)
{
    int res;
    if (volume < 0)
        volume = 0;
    else if (volume > 100)
        volume = 100;
    volume /= 3;
    res = es_write_reg(ES8388_ADDR, ES8388_DACCONTROL24, volume);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL25, volume);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL26, 0);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL27, 0);
    return res;
}
/**
 *
 * @return
 *           volume
 */
int es8388_get_voice_volume(int *volume)
{
    int res;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACCONTROL24, &reg);
    if (res == ESP_FAIL) {
        *volume = 0;
    } else {
        *volume = reg;
        *volume *= 3;
        if (*volume == 99)
            *volume = 100;
    }
    return res;
}

/**
 * @brief Configure ES8388 data sample bits
 *
 * @param mode:             set ADC or DAC or all
 * @param bitPerSample:   see BitsLength
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
int es8388_set_bits_per_sample(es_module_t mode, es_bits_length_t bits_length)
{
    int res = 0;
    uint8_t reg = 0;
    int bits = (int)bits_length;

    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        res = es_read_reg(ES8388_ADCCONTROL4, &reg);
        reg = reg & 0xe3;
        res |=  es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, reg | (bits << 2));
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res = es_read_reg(ES8388_DACCONTROL1, &reg);
        reg = reg & 0xc7;
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, reg | (bits << 3));
    }
    return res;
}

/**
 * @brief Configure ES8388 DAC mute or not. Basicly you can use this function to mute the output or don't
 *
 * @param enable: enable or disable
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
int es8388_set_voice_mute(int enable)
{
    int res;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACCONTROL3, &reg);
    reg = reg & 0xFB;
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, reg | (((int)enable) << 2));
    return res;
}

int es8388_get_voice_mute(void)
{
    int res = -1;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACCONTROL3, &reg);
    if (res == ESP_OK) {
        reg = (reg & 0x04) >> 2;
    }
    return res == ESP_OK ? reg : res;
}

/**
 * @param gain: Config DAC Output
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
int es8388_config_dac_output(int output)
{
    int res;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACPOWER, &reg);
    reg = reg & 0xc3;
    res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, reg | output);
    return res;
}

/**
 * @param gain: Config ADC input
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
int es8388_config_adc_input(es_adc_input_t input)
{
    int res;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_ADCCONTROL2, &reg);
    reg = reg & 0x0f;
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL2, reg | input);
    return res;
}

/**
 * @param gain: see es_mic_gain_t
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
int es8388_set_mic_gain(es_mic_gain_t gain)
{
    int res, gain_n;
    gain_n = (int)gain / 3;
    res = es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, gain_n); //MIC PGA
    return res;
}

int es8388_ctrl_state(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state)
{
    int res = 0;
    int es_mode_t = 0;
    switch (mode) {
        case AUDIO_HAL_CODEC_MODE_ENCODE:
            es_mode_t  = ES_MODULE_ADC;
            break;
        case AUDIO_HAL_CODEC_MODE_LINE_IN:
            es_mode_t  = ES_MODULE_LINE;
            break;
        case AUDIO_HAL_CODEC_MODE_DECODE:
            es_mode_t  = ES_MODULE_DAC;
            break;
        case AUDIO_HAL_CODEC_MODE_BOTH:
            es_mode_t  = ES_MODULE_ADC_DAC;
            break;
        default:
            es_mode_t = ES_MODULE_DAC;
            ESP_LOGW(ES_TAG, "Codec mode not support, default is decode mode");
            break;
    }
    if (AUDIO_HAL_CTRL_STOP == ctrl_state) {
        res = es8388_stop(es_mode_t);
    } else {
        res = es8388_start(es_mode_t);
        ESP_LOGD(ES_TAG, "start default is decode mode:%d", es_mode_t);
    }
    return res;
}

int es8388_config_i2s(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t *iface)
{
    int res = 0;
    int tmp = 0;
    res |= es8388_config_fmt(ES_MODULE_ADC_DAC, iface->fmt);
    if (iface->bits == AUDIO_HAL_BIT_LENGTH_16BITS) {
        tmp = BIT_LENGTH_16BITS;
    } else if (iface->bits == AUDIO_HAL_BIT_LENGTH_24BITS) {
        tmp = BIT_LENGTH_24BITS;
    } else {
        tmp = BIT_LENGTH_32BITS;
    }
    res |= es8388_set_bits_per_sample(ES_MODULE_ADC_DAC, tmp);
    return res;
}

void es8388_pa_power(bool enable)
{
    gpio_config_t  io_conf;
    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = BIT(GPIO_PA_EN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    if (enable) {
        gpio_set_level(GPIO_PA_EN, 1);
    } else {
        gpio_set_level(GPIO_PA_EN, 0);
    }
}
