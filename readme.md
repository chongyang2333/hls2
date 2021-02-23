#### GD32F450工程移植注意事项

1. 在`system_gd32f4xx.c` 中根据有无晶振，晶振频率选择系统时钟，目前使用8M外部晶振，故选用 `__SYSTEM_CLOCK_200M_PLL_8M_HXTAL`； 
2. GD工程库头文件包含在`gd32f4xx_libopt.h`中；
3. `gd32f4xx.h`中`#if ! define (GD32F450)....`下面取消`GD32F450`的注释；
4. 初始化时调用`StstemInit()`;用于配置时钟，调用`systick_config()`;打开滴答定时器； 注意`systick_config `与 `Systick_Config(xxxx)`是**不同的，注意大小写**；



