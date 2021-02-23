#### GD32F450工程移植注意事项

>* 在system_gd32f4xx.c 中根据有无晶振，晶振频率选择系统时钟，目前使用8M外部晶振，故选用 __SYSTEM_CLOCK_200M_PLL_8M_HXTAL； 
>* GD工程库头文件包含在gd32f4xx_libopt.h中；
>* 在gd32f4xx.h中#if ! define (GD32F450)....下面取消GD32F450的注释；
>* 初始化时调用StstemInit();用于配置时钟，调用systick_config();打开滴答定时器；'注意systick_config 与 Systick_Config(xxxx)是不同的，注意大小写'；



