#include "stm32f7xx_hal.h"
#include "battery_manage.h"
#include "delay.h"

static battery_t sBattery;

/**
 * @brief BQ34Z100 字节转UINT16
 * @param UINT8* data 字节指针
 * @return NONE
 */
static inline UINT16 bytes_to_integer(UINT8* data)
{
	UINT16 tmp;

	tmp = ((data[1] << 8) & 0xFF00);
	return ((UINT16)(tmp + data[0]) & 0x0000FFFF);
}


/**
 * @brief BQ34Z100 I2C起始信号
 * @param NONE
 * @return NONE
 */
void bq34z100_i2c_start(void)
{
	SDA_1();
	I2CDELAY();
	SCL_1();
	I2CDELAY();
	SDA_0();
	I2CDELAY();
	SCL_0();
	I2CDELAY();
}

/**
 * @brief BQ34Z100 I2C停止信号
 * @param NONE
 * @return NONE
 */
void bq34z100_i2c_stop(void)
{
	SDA_0();
	I2CDELAY();
	SCL_1();
	I2CDELAY();
	SDA_1();
	I2CDELAY();
}

/**
 * @brief BQ34Z100 I2C发送字节
 * @param UINT8 data 发送数据
 * @return ACK
 */
UINT8 bq34z100_i2c_send_byte(UINT8 data)
{
	UINT8 bits, temp, ack;
	UINT16 wait_cnt;
	SCL_0();
	temp = data;
	bits = 0x08;
	while (bits != 0x00) {
		if (temp & 0x80)
			SDA_1();
		else
			SDA_0();
		I2CDELAY();
		SCL_1();
		wait_cnt = 0;
		while ((BATTERY_I2C_SCL_GPIO->IDR & BATTERY_I2C_SCL_PIN) == 0) {
			wait_cnt++;
			if (wait_cnt > 20000) {
				bq34z100_i2c_stop();
				return (0);
			}
		}
		I2CDELAY();
		temp = (temp << 1);
		SCL_0();
		bits = (bits - 1);
	}
	I2CDELAY();
	SDA_1();
	SCL_1();
	wait_cnt = 0;
	while ((BATTERY_I2C_SCL_GPIO->IDR & BATTERY_I2C_SCL_PIN) == 0) {
		wait_cnt++;
		if (wait_cnt > 20000) {
			bq34z100_i2c_stop();
			return (0);
		}
	}
	I2CDELAY();
	ack = (((BATTERY_I2C_SDA_GPIO->IDR & BATTERY_I2C_SDA_PIN) == 0) ? 0 : 1);
	SCL_0();
	if (ack)
		return (1);
	else
		return (0);
}

/**
 * @brief BQ34Z100 I2C接收字节
 * @param UINT8 ack 是否响应ACK
 * @return UINT8 data 接收数据
 */
UINT8 bq34z100_i2c_rev_byte(UINT8 ack)
{
	UINT8 bits, data = 0;

	SDA_1();
	bits = 0x08;
	while (bits > 0) {
		SCL_1();
		while ((BATTERY_I2C_SCL_GPIO->IDR & BATTERY_I2C_SCL_PIN) == 0)
			I2CDELAY();
		data = (data << 1);
		if (BATTERY_I2C_SDA_GPIO->IDR & BATTERY_I2C_SDA_PIN)
			data = (data + 1);
		SCL_0();
		I2CDELAY();
		bits = (bits - 1);
	}
	if (ack)
		SDA_0();
	else
		SDA_1();
	SCL_1();
	I2CDELAY();
	SCL_0();
	SDA_1();

	return (data);
}

/**
 * @brief BQ34Z100 写入数据块
 * @param UINT8 SlaveAddress  设备地址
 * @param UINT16 numBytes 读取字节
 * @param void* rx_data 数据指针
 * @param unsigned char multi 是否多数据帧
 * @return NONE
 */
void bq34z100_i2c_write_block(UINT8 SlaveAddress,
                              UINT16 numBytes, UINT8 multi,
                              void* TxData)
{
	UINT16  i;
	UINT8* temp;

	temp = (UINT8*)TxData;
	bq34z100_i2c_start();
	bq34z100_i2c_send_byte(SlaveAddress + 0);
	for (i = 0; i < numBytes; i++) {
		bq34z100_i2c_send_byte(*(temp));
		temp++;
	}
	if (multi == 0) {
		bq34z100_i2c_stop();
	}
	I2CDELAY();
}

/**
 * @brief BQ34Z100 读取数据块
 * @param UINT8 SlaveAddress  设备地址
 * @param UINT16 numBytes 读取字节
 * @param void* rx_data 数据指针
 * @return NONE
 */
void bq34z100_i2c_read_block(UINT8 SlaveAddress,
                             UINT16 numBytes, void* rx_data)
{
	UINT16  i;
	UINT8* temp;

	temp = (UINT8*)rx_data;
	bq34z100_i2c_start();
	bq34z100_i2c_send_byte(SlaveAddress + 1);
	for (i = 0; i < numBytes; i++) {
		if (i == (numBytes - 1))
			*(temp) = bq34z100_i2c_rev_byte(BQ34Z100_NACK);
		else
			*(temp) = bq34z100_i2c_rev_byte(BQ34Z100_ACK);
		temp++;
	}
	bq34z100_i2c_stop();
}

/**
 * @brief BQ34Z100 i2c初始化
 * @param NONE
 * @return NONE
 */
void bq34z100_i2c_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.Pin = BATTERY_I2C_SDA_PIN;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(BATTERY_I2C_SDA_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = BATTERY_I2C_SCL_PIN;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(BATTERY_I2C_SCL_GPIO, &GPIO_InitStructure);
}

/**
 * @brief BQ34Z100寄存器读取
 * @param UINT8 port_switch 端口选择 1：电池1,2：电池2
 * @param UINT16 bytes 读取字节
 * @param void* rx_data 数据指针
 * @return NONE
 */
void bq34z100_read_reg(UINT8 cmd, UINT16 bytes, void* rx_data)
{
	UINT8 tx[1];
	tx[0] = cmd;   

	bq34z100_i2c_write_block(BQ34Z100_ADDR, 1, 1, tx);
	bq34z100_i2c_read_block(BQ34Z100_ADDR, bytes, rx_data);
}


/**
 * @brief BQ34Z100信息读取
 * @param UINT8 battery_num 电池号
 * @return NONE
 */
void BQ34Z100_get_charge_state(void)
{
	UINT8 tmp[2];
	static UINT8 step = 0;
    
    battery_t* battery = &sBattery;
    
	switch (step) {
	case 0:
		// 温度 K
		bq34z100_read_reg(BQ34Z100_TEMPERATURE_LSB, 2, tmp);
		battery->temperature = bytes_to_integer(tmp);
		break;

	case 1:
		// 电量百分比 0-100%
		bq34z100_read_reg(BQ34Z100_STATE_OF_CHARGE, 1, tmp);
		battery->charge_state = tmp[0];
		break;

	case 2:
		// 电池容量 mAh (充满电之后校准)
		bq34z100_read_reg(BQ34Z100_FULL_CHAGRE_CAP_LSB, 2, tmp);
		battery->full_charge_cap = bytes_to_integer(tmp);
		break;

	case 3:
		// 剩余电池容量 * 1 mAh
		bq34z100_read_reg(BQ34Z100_REMAIN_CAP_LSB, 2, tmp);
		battery->remain_cap = bytes_to_integer(tmp);
		break;

	case 4:
		// 电压 mV
		bq34z100_read_reg(BQ34Z100_VOLTAGE_LSB, 2, tmp);
		battery->voltage = bytes_to_integer(tmp);
		break;

	case 5:
		// 电流 mA
		bq34z100_read_reg(BQ34Z100_CURRENT_LSB, 2, tmp);
		battery->current = bytes_to_integer(tmp);
		break;

	case 6:
		// 运行标志
		bq34z100_read_reg(BQ34Z100_FLAGS_LSB, 2, tmp);
		battery->flag = bytes_to_integer(tmp);
		break;

	default:

		break;
	}
    
	(step >= 1) ? (step = 0) : step++;
	
}
