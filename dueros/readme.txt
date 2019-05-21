2019.05.11
用当前的esp-adf构造项目
adf提交：925393609f1b0da207feea28b8470e449241cd7e

测试固件
1、复制example下的dueros例程到项目文件夹
2、修改dueros的Makefile文件，加入esp-adf路径变量
3、修改adf下dueros_service下的duer_profile文件，改成自注册的profile
4、make menuconfig
   修改wifi ssid password
5、make flash monitor....

代码修改：
对esp-adf库的修改：
注：已经从乐鑫fork adf库，可以修改并保存之。
1、修改dueros_service.c中的duer_login()函数，加入对从串口中传过来的profile的处理。

－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－
1、修改按键接口
   去掉不用的触摸按键，增加录音触摸按键
    periph_touch_cfg_t touch_cfg = {
    .touch_mask = TOUCH_PAD_SEL4 | TOUCH_PAD_SEL7 | TOUCH_PAD_SEL3, //| TOUCH_PAD_SEL8 | TOUCH_PAD_SEL9,
    .tap_threshold_percent = 70,
   };

   注释掉实体按键初始化代码

2、去掉sd卡初始化代码
3、修改外设回调函数中相关响应代码，去掉不用的按键，把录音按键改成触控响应。
   periph_callback(audio_event_iface_msg_t *event, void *context){}
4、修改指示灯代码
5、加入串口协议，实现wifi密码、duerprotfile、语音控制数据的传输。
   串口1初始化
   数据包收发（配置参数、语音指令）
   dueros_app.c中app_init()加入：
        uartx_init();
        xTaskCreate(parse_uart_package_task, "user_parse_package", 4096, NULL, 4, NULL);

6、wifi帐号密码及duer_profile保存到flash
  1）flash中开辟一个区域，用来保存数据
    在dueros_app.h中定义
    #define WIFI_PARAM_SECTOR_ADDR      998
    #define SPI_WIFI_PARAM_ADDR         (WIFI_PARAM_SECTOR_ADDR * SPI_FLASH_SEC_SIZE) 
    #define DUER_PROFILE_SECTOR_ADDR   999
    #define DUER_PROFILE_ADDR           (DUER_PROFILE_SECTOR_ADDR * SPI_FLASH_SEC_SIZE)
    #define WIFI_PARAM_SIZE             96
    #define DUER_PROFILE_SIZE           2000

    定义几个全局变量
  2）创建一个函数，用来读flash中参数
    dueros_app.c中创建：
    read_flash_user_param(void)
    在wifi连接前调用本函数，先获取参数
  3）上串口通信中收到的参数会保存到flash中。

7、语音控制点实现
   加入act.c/h，在dueros_serviceduer_event_hook()中加入：
   bits_regist_resource(); //注册资源

8、加大初始化的音量设置。
    修改duer_audio_wrapper.c中setup_play()中默认音量设置，由45改为70
    esp_audio_vol_set(player, 70);





