#### GD32开发环境搭建
1. 安装`GD32F4xx_AddOn_V2.0.4`,根据所安装KEIL版本选择安装，路径：[共享文件夹/固件库/GD32F4系列固件库/GD32F4XX_AddOn_V2.0.4/keil](https://pan.baidu.com/s/1nuifedz#list/path=%2F%E5%85%B1%E4%BA%AB%E6%96%87%E4%BB%B6%E5%A4%B9%2F%E5%9B%BA%E4%BB%B6%E5%BA%93%2FGD32F4%E7%B3%BB%E5%88%97%E5%9B%BA%E4%BB%B6%E5%BA%93); 
2. 安装`GD_Link_Program`,解压完毕，修改`GD_Link_CLI.ex_`后缀为`GD_Link_CLI.exe`。修改`GD-Link Programmer.ex_`后缀为`GD-Link Programmer.exe`。打开`GD-Link Programmer.exe`,用法与ST_Link相似。路径：[共享文件夹/软件开发工具/GD相关软件/GD_Link上位机](https://pan.baidu.com/s/1nuifedz#list/path=%2F%E5%85%B1%E4%BA%AB%E6%96%87%E4%BB%B6%E5%A4%B9%2F%E8%BD%AF%E4%BB%B6%E5%BC%80%E5%8F%91%E5%B7%A5%E5%85%B7%2FGD%E7%9B%B8%E5%85%B3%E8%BD%AF%E4%BB%B6%2FGD_Link%E4%B8%8A%E4%BD%8D%E6%9C%BA&parentPath=%2F)；
3. 打开keil新建工程选择设备`GD32F450VG`,可以在`Manage Run-Time Environment`中选择使用的外设库，也可以自己添加，目前是自己添加外设库;
4. 使用开发板上`gd-link`调试时，在`Options for Target`窗口的Debug中选择`CMSIS-DAP Debugger`
5. 底盘固件移植时需要将`Post Universe Chasis.bat`和`CalculFirmwareCRC.exe`，并且修改`Post Universe Chasis.bat`中的axf路径


#### GD32F450工程移植注意事项

1. 在`system_gd32f4xx.c` 中根据有无晶振，晶振频率选择系统时钟，目前使用8M外部晶振，选用 `__SYSTEM_CLOCK_200M_PLL_8M_HXTAL`，**注意修改晶振值，`HXTAL_VALUE = 8000000`**； 
2. GD工程库头文件包含在`gd32f4xx_libopt.h`中；
3. `gd32f4xx.h`中`#if ! define (GD32F450)....`下面取消`GD32F450`的注释；
4. 调用`systick_config()`;打开滴答定时器时； 注意`systick_config `与 `Systick_Config(xxxx)`是**不同的，注意大小写**；

#### 应用层移植注意事项

1. 通用GPIO配置请根据应用层模块一一对应配置；
2. 外设驱动配置时，`HAL_xxx_MspInit()`完全被注释了，需要重新配置对应外设的GPIO引脚复用函数，并且在该外设初始化前调用该函数；
3. 与底层寄存器/HAL库相关的函数接口：[语雀--硬件组---嵌软固件模块---底盘板卡MCU选型](https://pudutech.yuque.com/rdheib/umbwrq/ib5sql#KeTZw)



