/**
 * @file LeadAcidBMS.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-02-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "LeadAcidBMS.h"
#include "param.h"
#include "PowerManager.h"
#include "Eeprom.h"
#include "UartApp.h"
#include "logger.h"
#include "hardApi.h"
#include "CanApp.h"
#include "delay.h"
#include "gpio.h"

#define BATTERY_FULL_CAPACITY       15000   //mAh
#define DISCHARGE_CURVE_CALIBRATION_EN      1       //使用放电特性曲线对电量校准

typedef struct 
{
    /*********************** 有浮充时的第一段拟合 ************************/
    float a1;                   //y=a1*x + b1
    float b1;   

    /*********************** 正常放电时，放点量与电压的拟合曲线 ********************/
    float a2;                   //y=a2*x^2 + b2*x + c2
    float b2;
    float c2;

    float fFullVoltage;         //满电电压, V
    float fCurve2Right;           //第二段曲线右值
    float fCurve2Left;           //第二段曲线定义域左值
    float fSOC;

    /*********************** 电池cycle与容量的拟合曲线，第一段 **********************/
    float ca1;
    float cb1;
    float cc1;
    
    /*********************** 电池cycle与容量的拟合曲线，第二段 **********************/
    float ca2;
    float cb2;
    float cc2;

    UINT16 hCycleDivide;        //cycle曲线第一段与第二段的分界点;
}BatteryCharacteristic_t;

typedef  struct
{
	float filterValue;  //k-1时刻的滤波值，即是k-1时刻的值
	float kalmanGain;   //   Kalamn增益
	float A;   // x(n)=A*x(n-1)+u(n),u(n)~N(0,Q)
	float H;   // z(n)=H*x(n)+w(n),w(n)~N(0,R)
	float Q;   //预测过程噪声偏差的方差
	float R;   //测量噪声偏差，(系统搭建好以后，通过测量统计实验获得)
	float P;   //估计误差协方差
}  KalmanInfo_t;

typedef struct
{
    UINT16 cnt;                  //计次
    UINT32 lKalmanCnt;          //
    UINT16 hSaveCnt;            //用于计算执行存储到eeprom的时间
    UINT8 hIsPowerOn;          //开机标志，用于初始化初始电量数据
    UINT16 hPowerOning;
    float fCurrentSum;          //电流累计
    float fVoltageSum;          //电压累计
    float fCurCurrent;          //当前电流(平均),A
    float fCurVoltage;          //当前电压(平均),V
    float fPoweroffVoltage;     //电源开关关闭后，充电时的电池电压

    UINT16 ChargeCnt;                  //计次
    UINT16 FullChargeCnt;
    UINT32 lChgTime;                   //充电时长
    float fChargeCurrentSum;          //电流累计
    float fChargeVoltageSum;          //电压累计
    float fChargeCurCurrent;          //当前电流(平均),A
    float fChargeCurVoltage;          //当前电压(平均),V
    float fChargeFullCurrent;           //充电满后的电流, A
    float fChargeFUllCurrentC;          //充电满后的电流, C

    float fSOC;          //电池剩余电量,mA.H
    float fRealSOC;     //真实剩余电量,mA.H
    float fMapRatio;    //真实电量与显示电量的比值 ratio = Soc/RealSoc
    float fFC;                  //电池实际容量
    
    BatteryCharacteristic_t tChar;
    KalmanInfo_t tKalman;
}BatteryListen_t;


PRIVATE BatteryListen_t gs_tBatListen = {
    .hIsPowerOn = 1,
};

extern struct PowerManagerStruct sPowerManager; /* 需要优化，屏蔽全局 */

PRIVATE void BatteryDischargeHandle(void);
PRIVATE void BatteryChargeHandle(void);

/**
 * @brief 铅酸电池事件任务
 * @param[in] event 
 * @return  
 */
PUBLIC void LeadAcideEventTask(LeadAcidEvent_t event)
{
    switch (event)
    {
        case LEAD_ACID_EVENT_NONE:break;
        case LEAD_ACID_EVENT_INSERT_IN:
        {
            gs_tBatListen.ChargeCnt = 0;
            gs_tBatListen.fChargeCurrentSum = 0;
            gs_tBatListen.fChargeVoltageSum = 0;  
        }break;
        case LEAD_ACID_EVENT_CHARGING:
        {
            g_tBatterySaveInfo.isCharge = 1;
            g_tBatterySaveInfo.hSaveInfo = 1;
        }break;
        case LEAD_ACID_EVENT_PULL_OUT:
        {
            gs_tBatListen.hSaveCnt = 0;         //拔出充电器后，等2min后再写电池信息，这样电池电压避免虚高
        }break;
        default:break;
    }
}

/**
 * @brief 获取铅蓄电池的信息
 * @return 
 */
PUBLIC void lead_acid_battery_info(void)
{
    static UINT32 ReadStartTime = 0;

    static UINT8 step = 0;
    UINT32 tmp = 0;
    UINT8 i = 0;
	UINT8	ret = 0;
    UINT8   ret1 = 0;
    UINT8 start_index = 0x40;

	//10s刷新一次电池SOH、容量、cycle等信息
	if (sPowerManager.uTick - ReadStartTime <= 400)
		return; 
    

    switch(step)
	{
		case 0:
            tmp = sPowerManager.sBatteryInfo.BatteryFullCapacity;
            ret = 1;
			break;

		case 1:
            tmp = sPowerManager.sBatteryInfo.BatteryRemaingCapacity;
			ret = 1;
			break;

		case 2:
			/*battery voltage iic*/
            tmp = sPowerManager.sBatteryInfo.BatteryVoltage;
			ret = 1;     
			break;

		case 3:
			/*state of health*/
            tmp = g_tBatterySaveInfo.hCycleCount - g_tBatterySaveInfo.hCycle;
            tmp = tmp * 100 /  g_tBatterySaveInfo.hCycleCount;
            sPowerManager.sBatteryInfo.SOH = tmp;
			ret = 1;
			break;

		case 4:
			/*cycle count*/
            sPowerManager.sBatteryInfo.CycleCnt = g_tBatterySaveInfo.hCycle;
            tmp = sPowerManager.sBatteryInfo.CycleCnt;
			ret = 1;
			break;  

		case 5:
			/*Safety Alert*/
			break; 
            
		case 6:
			/*Safety Status*/
			break; 

		case 7:
			/*PF Alert*/
			break;   

		case 8:
			/*PF Status*/
			break; 
            
		case 9:
			/*Operation Status*/
			break;   

		case 10:
			/*Charging Status*/
			break;
            
        case 11:
            /*serial number*/
            break;
            
        case 12:
            /*battery type*/
            tmp = sPowerManager.sBatteryInfo.BMS_icType;
            ret = 1;
            break;
        
		case 13:
			/*cell voltage*/
			break;        
            
		default:
			break;
	}

    /*upload normal status*/
	if (ret)
	{
		CanSendBatteryInfo(step, tmp);
        delay_us(2000);
	}
    
    /*upload cell voltage*/
    if (ret1)
    {
        for (i=0; i<7; i++)
        {
            CanSendBatteryInfo(start_index+i, *(sPowerManager.sBatteryInfo.CellVoltage+i));
            delay_us(2000);
        }
    }
    
	(step >= 14) ? (step = 0) : step++;
}


/**
 * @brief 铅蓄电池参数更新，后期优化为read函数
 * 
 * @return 
 */
PUBLIC void LeadAcidInfoUpdate(void)
{
#ifdef USER_LOGGER
    static UINT8 cnt = 0;
#endif

    BatteryDischargeHandle();
    BatteryChargeHandle();

    gSensorData.BatteryLevel0x400D = gs_tBatListen.fSOC * 100 / gs_tBatListen.fFC;

    //这个IO设置为推挽输出，不知道所有机器能否完成读取IO操作
    sPowerManager.sChargeInfo.eChargeMosState = ReadChargeMosState();
    sPowerManager.sChargeInfo.ChargeCurrent = 1000.0f * gs_tBatListen.fChargeCurCurrent;
    sPowerManager.sChargeInfo.ChargeVoltage = 1000.0f * gs_tBatListen.fChargeCurVoltage;
    sPowerManager.sBatteryInfo.BatteryCurrent = 1000.0f * gs_tBatListen.fCurCurrent;
    sPowerManager.sBatteryInfo.BatteryVoltage = 1000.0f * gs_tBatListen.fCurVoltage;
    sPowerManager.sBatteryInfo.BatteryLevelRaw = gSensorData.BatteryLevel0x400D;
    sPowerManager.sBatteryInfo.BatteryTemp = gSensorData.BatteryTemp0x4010; //温度值

#ifdef USER_LOGGER
    cnt++;
    if(cnt >= 40)
    {
        cnt = 0;
        Log("", "charge:V:%d, C:%d discharge:V:%d, C:%d, soc:%d, real:%d", sPowerManager.sChargeInfo.ChargeVoltage, \
            sPowerManager.sChargeInfo.ChargeCurrent, sPowerManager.sBatteryInfo.BatteryVoltage, \
            sPowerManager.sBatteryInfo.BatteryCurrent, sPowerManager.sBatteryInfo.BatteryRemaingCapacity, \
            (UINT16)gs_tBatListen.fRealSOC);
    }   
#endif
}


/**
 * @brief init 方法
 * 
 * @return  
 */
PUBLIC void LeadAcidBatteryInit(void)
{
    sPowerManager.sBatteryInfo.BatteryFullCapacity = BATTERY_FULL_CAPACITY;     //赋值电池标称容量mA.h

    g_tBatterySaveInfo.hCycleCount = 600;           //深度循环总次数，600次

    gs_tBatListen.tChar.a1 = -0.89423077f;
    gs_tBatListen.tChar.b1 = 24.23365385f;
    gs_tBatListen.tChar.a2 = -2.3325062f;
    gs_tBatListen.tChar.b2 = 84.62729529f;
    gs_tBatListen.tChar.c2 = -620.33548624f;

    gs_tBatListen.tChar.ca2 = -0.0002067f;
    gs_tBatListen.tChar.cb2 = 0.04669856f;
    gs_tBatListen.tChar.cc2 = 106.392576f;
    gs_tBatListen.tChar.ca1 = -0.00064f;
    gs_tBatListen.tChar.cb1 = 0.152f;
    gs_tBatListen.tChar.cc1 = 100;

    gs_tBatListen.tChar.hCycleDivide = 125;
    gs_tBatListen.tChar.fFullVoltage = 27.1f;
    gs_tBatListen.tChar.fCurve2Right = 26.06f;
    gs_tBatListen.tChar.fCurve2Left = 22.6425f;
    gs_tBatListen.fPoweroffVoltage = 10.0f;
    gs_tBatListen.fMapRatio = 1.0f;

    gs_tBatListen.fChargeFUllCurrentC = 0.03f;          //铅酸电池充电电流小于0.03C后，认为电池充满
    gs_tBatListen.fChargeFullCurrent = gs_tBatListen.fChargeFUllCurrentC * sPowerManager.sBatteryInfo.BatteryFullCapacity / 1000;
    g_tBatterySaveInfo.hCapacity = BATTERY_FULL_CAPACITY;
}

PUBLIC INT16 leadAcidConnCurrentThrd(void)
{
    return (INT16)(gs_tBatListen.fChargeFullCurrent * 1000 - 100);
}

/**
 * @brief Get the Battery Soc object
 * 
 * @return 
 */
PUBLIC float GetBatterySoc(void)
{
    return gs_tBatListen.fSOC; //单位,mA.h
}

/*********************************************************************************
* @brief Init_KalmanInfo   初始化滤波器的初始值
* @param info  滤波器指针
* @param Q 预测噪声方差 由系统外部测定给定
* @param R 测量噪声方差 由系统外部测定给定
*********************************************************************************/
void InitKalmanInfo(KalmanInfo_t* info, float Q, float R, float initValue)
{
	info->A = 1;  //标量卡尔曼
	info->H = 1;  //
	info->P = 10;  //后验状态估计值误差的方差的初始值（不要为0问题不大）
	info->Q = Q;    //预测（过程）噪声方差 影响收敛速率，可以根据实际需求给出
	info->R = R;    //测量（观测）噪声方差 可以通过实验手段获得
	info->filterValue = initValue;// 测量的初始值
}

float KalmanFilter(KalmanInfo_t* kalman, float lastMeasurement)
{
	//预测下一时刻的值
	float predictValue = kalman->A * kalman->filterValue;   //x的先验估计由上一个时间点的后验估计值和输入信息给出，此处需要根据基站高度做一个修改
	
	//求协方差
	kalman->P = kalman->A * kalman->A * kalman->P + kalman->Q;  //计算先验均方差 p(n|n-1)=A^2*p(n-1|n-1)+q
	// float preValue = kalman->filterValue;                   //记录上次实际坐标的值
 
	//计算kalman增益
	kalman->kalmanGain = kalman->P * kalman->H / (kalman->P * kalman->H * kalman->H + kalman->R);  //Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + R)
	//修正结果，即计算滤波值
	kalman->filterValue = predictValue + (lastMeasurement - predictValue) * kalman->kalmanGain;  //利用残余的信息改善对x(t)的估计，给出后验估计，这个值也就是输出  X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1))
	//更新后验估计
	kalman->P = (1 - kalman->kalmanGain * kalman->H) * kalman->P;           //计算后验均方差  P[n|n]=(1-K[n]*H)*P[n|n-1]
 
	return  kalman->filterValue;
}

/*********************************************************************
 * @brief 根据循环次数，校准电池实际满电容量
 * @param pChar 电池特性相关信息
 * @param cycle 循环次数
 * @param nominalCapacity   满电标称容量, mAh
 * @return 电池实际容量
 *********************************************************************/
PRIVATE float BatteryCapacityCheck(BatteryCharacteristic_t *pChar, UINT16 cycle, float nominalCapacity)
{
    float tmp;

    if(cycle < pChar->hCycleDivide)
    {
        tmp = pChar->ca1 * cycle * cycle + pChar->cb1 * cycle + pChar->cc1;
    }
    else
    {
        tmp = pChar->ca2 * cycle * cycle + pChar->cb2 * cycle + pChar->cc2;
    }

    tmp = tmp / 100 * nominalCapacity;

    return tmp;
}

/*********************************************************************
 * @brief 电池电量校准
 * @param pChar 电池特性相关信息
 * @param voltage 电池电压
 * @return 电池实际SOC
 *********************************************************************/
PRIVATE float BatterySOCCheck(BatteryCharacteristic_t *pChar, float voltage)
{
    float tmp;

    if(voltage > pChar->fFullVoltage)
    {
        tmp = 1.0f;
    }
    else if(voltage >= pChar->fCurve2Right)
    {
        tmp = pChar->a1 * voltage + pChar->b1;
        tmp = 100 - tmp;
    }
    else if(voltage >= pChar->fCurve2Left)
    {
        tmp = pChar->a2 * voltage * voltage + pChar->b2 * voltage + pChar->c2;
        tmp = 100 - tmp;
        if(tmp < 0)
        {
            tmp = 0;
        }
    }
    else
    {
        tmp = 0;
    }
    tmp = gs_tBatListen.fFC / 100.0f * tmp;

    return tmp;
}

PUBLIC void LeadAcidBatteryDischargeListen(void)
{
    gs_tBatListen.fCurrentSum += GetBatteryCurrent();
    gs_tBatListen.fVoltageSum += GetBatteryVoltage();
    gs_tBatListen.cnt++;
}

void BatterySaveInfoCalibration(void)
{
    if((gs_tBatListen.fChargeCurVoltage > sPowerManager.sChargeInfo.ChargerConnectedVoltageThreshold) && \
        (gs_tBatListen.fCurVoltage <= gs_tBatListen.fPoweroffVoltage))        //充电器插入但电源开关没开
    {
        gs_tBatListen.fSOC = 0;
    }
    else
    {
        gs_tBatListen.fSOC = BatterySOCCheck(&gs_tBatListen.tChar, gs_tBatListen.fCurVoltage);
    }
    g_tBatterySaveInfo.hSoc = gs_tBatListen.fSOC;
    gs_tBatListen.fRealSOC = gs_tBatListen.fSOC;
    InitKalmanInfo(&gs_tBatListen.tKalman, 0.00247875f, 0.36787944f, gs_tBatListen.fSOC);
}


void BatteryPowerOnCalibration(void)
{
#if DISCHARGE_CURVE_CALIBRATION_EN
    float fTmp = g_tBatterySaveInfo.hSoc;

    if((gs_tBatListen.fChargeCurVoltage > sPowerManager.sChargeInfo.ChargerConnectedVoltageThreshold) && \
        (gs_tBatListen.fCurVoltage <= gs_tBatListen.fPoweroffVoltage))        //充电器插入但电源开关没开
    {
        gs_tBatListen.fSOC = fTmp;           //只能使用上次记录的信息
    }
    else
    {
        gs_tBatListen.fSOC = BatterySOCCheck(&gs_tBatListen.tChar, gs_tBatListen.fCurVoltage);
        gs_tBatListen.fRealSOC = gs_tBatListen.fSOC;
        InitKalmanInfo(&gs_tBatListen.tKalman, 0.00247875f, 0.36787944f, gs_tBatListen.fSOC);
        if(gs_tBatListen.fCurVoltage < 23.0f)        //防止过放，过放保护
        {
            ;
        }
        else if((g_tBatterySaveInfo.isCharge == 1) && (gs_tBatListen.fSOC > fTmp))      //前一次关机时是充电状态，电压可能存在虚高, 需要综合内存信息判断
        {
            gs_tBatListen.fSOC = fTmp;           //只能使用上次记录的信息
        }
        else if((fTmp - gs_tBatListen.fSOC) > (gs_tBatListen.fFC / 4))  //保留该判定逻辑
        {
            /* 以下用于粗略判定是否出现长期放置不使用的情况, 由于铅酸电池电压波动大，
             * 故判定以SOC误差超过预期10%为条件，相当于在25℃环境下放置5~6个月 */
            ;       //不用赋值, 直接等于计算结果即可
        }
        else
        {
            gs_tBatListen.fSOC = fTmp;           //其他情况，一律按照内存记录的信息
        }

        g_tBatterySaveInfo.isCharge = 0;
    }
#else
    gs_tBatListen.fSOC = g_tBatterySaveInfo.hSoc;
    gs_tBatListen.fRealSOC = gs_tBatListen.fSOC;
#endif
}

PRIVATE void BatteryDischargeHandle(void)
{
    float fTmp, fPreSoc;
    gs_tBatListen.fCurCurrent = gs_tBatListen.fCurrentSum / gs_tBatListen.cnt;
    gs_tBatListen.fCurVoltage = gs_tBatListen.fVoltageSum / gs_tBatListen.cnt;
    gs_tBatListen.cnt = 0;
    gs_tBatListen.fCurrentSum = 0;
    gs_tBatListen.fVoltageSum = 0;

    if(sPowerManager.sBatteryInfo.BMS_icType != LEAD_ACID_BAT)
        return;


    g_tBatterySaveInfo.isCharge = ReadChargeMosState();

    if(gs_tBatListen.hIsPowerOn)        //如果时刚开机，需要矫正数据
    {
        gs_tBatListen.hIsPowerOn = 0;          //mA.h
        gs_tBatListen.fChargeCurVoltage = gs_tBatListen.fChargeVoltageSum / gs_tBatListen.ChargeCnt;
        if(g_tBatterySaveInfo.CrcState != 0)            //eeprom数据出错
        {
            gs_tBatListen.fFC = sPowerManager.sBatteryInfo.BatteryFullCapacity;
            g_tBatterySaveInfo.hCycle = 0;
            g_tBatterySaveInfo.lDischargeSum = 0;
            g_tBatterySaveInfo.CrcState = 0;
            g_tBatterySaveInfo.lNonFullChg = 0;
            g_tBatterySaveInfo.lChgSum = 0;
            BatterySaveInfoCalibration();
            g_tBatterySaveInfo.hSaveInfo = 1;
        }
        else
        {
            gs_tBatListen.fFC = BatteryCapacityCheck(&gs_tBatListen.tChar, g_tBatterySaveInfo.hCycle, sPowerManager.sBatteryInfo.BatteryFullCapacity); 
            BatteryPowerOnCalibration();
        }
        gs_tBatListen.lKalmanCnt = 24000;
        gs_tBatListen.hPowerOning = 200;
        // Log("", "g_tBatterySaveInfo.hSoc=%d", g_tBatterySaveInfo.hSoc);
    }
    else
    {
        if(sPowerManager.sChargeInfo.eChargerConnectState == DISCONNECT)
        {
            if(gs_tBatListen.hPowerOning)
            {
                gs_tBatListen.hPowerOning--;
                gs_tBatListen.fRealSOC = KalmanFilter(&gs_tBatListen.tKalman, BatterySOCCheck(&gs_tBatListen.tChar, gs_tBatListen.fCurVoltage));
                gs_tBatListen.fMapRatio = gs_tBatListen.fSOC / gs_tBatListen.fRealSOC;
            }
            else
            {
                if((gs_tBatListen.fCurCurrent > 0.55f) && (gs_tBatListen.fCurCurrent < 1.0f))
                {
                    gs_tBatListen.lKalmanCnt++;
                    if(gs_tBatListen.lKalmanCnt == 24000)
                    {
                        gs_tBatListen.tKalman.filterValue = BatterySOCCheck(&gs_tBatListen.tChar, gs_tBatListen.fCurVoltage);
                    }
                    else if(gs_tBatListen.lKalmanCnt > 24000)
                    {
                        gs_tBatListen.fRealSOC = KalmanFilter(&gs_tBatListen.tKalman, BatterySOCCheck(&gs_tBatListen.tChar, gs_tBatListen.fCurVoltage));
                        gs_tBatListen.fMapRatio = gs_tBatListen.fSOC / gs_tBatListen.fRealSOC;
                    }
                    else
                    {
                        gs_tBatListen.fRealSOC = gs_tBatListen.fRealSOC - gs_tBatListen.fCurCurrent * 25 / 3600;
                    }
                }
                else
                {
                    gs_tBatListen.lKalmanCnt = 0;
                    gs_tBatListen.fRealSOC = gs_tBatListen.fRealSOC - gs_tBatListen.fCurCurrent * 25 / 3600;
                }
            }
            
            fPreSoc = gs_tBatListen.fSOC;
            fTmp = gs_tBatListen.fSOC - gs_tBatListen.fCurCurrent * 25 * gs_tBatListen.fMapRatio / 3600;   //全映射
            // gs_tBatListen.fSOC = gs_tBatListen.fSOC - gs_tBatListen.fCurCurrent * 25 / 3600;
            // fTmp = gs_tBatListen.fSOC * 0.99984f + gs_tBatListen.fRealSOC * 0.00016f;
            if(fTmp > fPreSoc)
            {
                gs_tBatListen.fSOC = fPreSoc;
            }
            else
            {
                gs_tBatListen.fSOC = fTmp;
            }
            sPowerManager.sBatteryInfo.BatteryRemaingCapacity = gs_tBatListen.fSOC;
        }
    }

    /************** 对存储到eeprom的数据进行处理 ***************/
    gs_tBatListen.hSaveCnt++;
    if(gs_tBatListen.hSaveCnt >= 4800)           //2分钟记录一次数据，并存到eeprom
    {
        gs_tBatListen.hSaveCnt = 0;

        if(sPowerManager.sChargeInfo.eChargerConnectState == DISCONNECT)        //使用电池供电时,开始叠加电池循环
        {
            g_tBatterySaveInfo.lDischargeSum += (g_tBatterySaveInfo.hSoc > gs_tBatListen.fSOC) ? \
                (g_tBatterySaveInfo.hSoc - gs_tBatListen.fSOC) : 0;
        }
        g_tBatterySaveInfo.hSoc = gs_tBatListen.fSOC;
        g_tBatterySaveInfo.hCycle = g_tBatterySaveInfo.lDischargeSum / g_tBatterySaveInfo.hCapacity;
        g_tBatterySaveInfo.hSaveInfo = 1;
    }
}

PUBLIC void LeadAcidBatteryChargeListen(void)
{
    gs_tBatListen.fChargeCurrentSum += GetChargeCurrent();
    gs_tBatListen.fChargeVoltageSum += GetChargeVoltage();
    gs_tBatListen.ChargeCnt++;
}

PRIVATE void BatteryChargeHandle(void)
{
    if(sPowerManager.sBatteryInfo.BMS_icType != LEAD_ACID_BAT)
        return;
		
	float fTmp = sPowerManager.sChargeInfo.ChargeCurrentBias;
    gs_tBatListen.fChargeCurCurrent = gs_tBatListen.fChargeCurrentSum / gs_tBatListen.ChargeCnt - fTmp / 1000;
    gs_tBatListen.fChargeCurVoltage = gs_tBatListen.fChargeVoltageSum / gs_tBatListen.ChargeCnt;
    gs_tBatListen.ChargeCnt = 0;
    gs_tBatListen.fChargeCurrentSum = 0;
    gs_tBatListen.fChargeVoltageSum = 0;

    if(sPowerManager.sChargeInfo.eChargerConnectState != CONNECT)       //充电器已插入
        return;

    if(ReadChargeMosState() == 0)               //没有使能充电
    {
        if(gs_tBatListen.lChgTime >= 12000)             //发生过充电行为
        {
            g_tBatterySaveInfo.lChgSum += 1;
            if(gs_tBatListen.fSOC < (gs_tBatListen.fFC - gs_tBatListen.fFC / 100))      //如果是欠充电
            {
                g_tBatterySaveInfo.lNonFullChg += 1;
            }
        }
        gs_tBatListen.lChgTime = 0;
        return;
    }

    gs_tBatListen.lChgTime++;
    if((gs_tBatListen.fChargeCurVoltage > 29.0f) && (gs_tBatListen.fChargeCurCurrent < gs_tBatListen.fChargeFullCurrent))
    {
        gs_tBatListen.FullChargeCnt++;
    }
    else
    {
        gs_tBatListen.FullChargeCnt = 0;
    }

    if(gs_tBatListen.FullChargeCnt >= 50)       //电充满
    {
        gs_tBatListen.FullChargeCnt = 0;
        if(gs_tBatListen.fSOC != gs_tBatListen.fFC)
        {
            g_tBatterySaveInfo.hSaveInfo = 1;
        }
        gs_tBatListen.fSOC = gs_tBatListen.fFC;
        gSensorData.BatteryLevel0x400D = gs_tBatListen.fSOC * 100 / gs_tBatListen.fFC;
        sPowerManager.sBatteryInfo.BatteryLevelRaw = gSensorData.BatteryLevel0x400D;
    }
    else
    {
        if(gs_tBatListen.fSOC < gs_tBatListen.fFC)
        {
            if(gs_tBatListen.fChargeCurVoltage >= (gs_tBatListen.tChar.fCurve2Left + 1))     //先将过放的电能充回，再开始计算电量
            {
                gs_tBatListen.fSOC = gs_tBatListen.fSOC + gs_tBatListen.fChargeCurCurrent * 25 * gs_tBatListen.fMapRatio / 3600;
                gs_tBatListen.fRealSOC = gs_tBatListen.fRealSOC + gs_tBatListen.fChargeCurCurrent * 25 / 3600;
                gs_tBatListen.fRealSOC = (gs_tBatListen.fRealSOC >= gs_tBatListen.fFC) ? gs_tBatListen.fFC : gs_tBatListen.fRealSOC;
                if(gs_tBatListen.fSOC >= (gs_tBatListen.fFC - gs_tBatListen.fFC / 100))
                {
                    gs_tBatListen.fSOC = gs_tBatListen.fFC - gs_tBatListen.fFC / 100 - 5;
                }
            }
        }
    }
    
    sPowerManager.sBatteryInfo.BatteryRemaingCapacity = gs_tBatListen.fSOC;
}

