#include <time.h>
#include "bsp_rtc.h"

#ifndef MYRTC_CLK_SOURCE
#define MYRTC_CLK_SOURCE RCC_RTCCLKSource_LSE             // 外部低速时钟 (32.768kHz)  (推荐)
// #define RTC_CLK_SOURCE RCC_RTCCLKSource_HSE_Div128      // 外部高速时钟 (128分频) (更不推荐, 主电源VDD掉电, 受影响)
// #define MYRTC_CLK_SOURCE RCC_RTCCLKSource_LSI           // 内部低速时钟 (40kHz) (备选, LSI无法由备用电源供电，故主电源掉电时，RTC走时会暂停)
#endif

#ifndef MYRTC_TIME_ZONE_OFFSET
#define MYRTC_TIME_ZONE_OFFSET  8 * 60 * 60              // 北京时间(东八区) 时区偏移
#endif

MyRTC_TimeTypeDef MyRTC_Time = {2023, 1, 1, 23, 59, 55};    //定义全局的时间数组，数组内容分别为年、月、日、时、分、秒


// 一些基本概念
// RTC时钟源 (RTCCLK)
// - HSE/128: 外部高速时钟 (通常为8MHz) / 128 (分频) = 8MHz / 128 
//   这里必须要除以 128, 因为外部高速时钟频率太高, RTCCLK 的20位分频器无法直接分频到 1Hz,
//   所以, 进入 RTCCLK 之前需要先进行一次 128分频, 后续 RTCCLK 的20位分频器进行适当分频 就能使频率到 1Hz
// - LSE: 外部低速时钟 (通常为 32.768kHz)  (推荐, 分频使用 32768 - 1 即可使频率为 1Hz, 需手工开启)
//        因为 32768 = 2^15 (所以, 3278 经过一个 15位分频器自然溢出后, 就能方便的得到 1Hz 频率)
// - LSI: 内部低速时钟 (40kHz)
//        到 RTCCLK 后, 后续经过 40K的分频, 就能使频率为 1Hz
//        内部晶振精度不如外部, 适合当给 RTC备选, 给 看门狗(IWDG)比较合适
// 等待操作
// - WaitForSynchro: 等待同步
// - WaitForLastTask: 等待上一次操作完成
// RTC预分频器
// - PRL: 重装寄存器
// - DIV: 余数寄存器 (自减计数)
//        DIV = PRL + 1
// 注意:
// - 必须设置 RTC_CRL 寄存器的 CNF位, 使RTC进入配置模式后, 才能写入 PRL, CNT, ALR 寄存器
// - 读取RTC寄存器时, 若RTC的APB1接口曾经处于禁止状态, 则软件首先必须等待RTC_CRL寄存器的RSF标志位(寄存器同步标志)被硬件置1


/**
  * 函    数：RTC初始化
  * 参    数：无
  * 返 回 值：无
  */
void MyRTC_Init(void)
{
	// 1. 配置时钟 (RTC, BKP)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);		//开启PWR的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP, ENABLE);		//开启BKP的时钟
	
	// 2. 备份寄存器的访问使能
	PWR_BackupAccessCmd(ENABLE);							//使用PWR开启对备份寄存器的访问
	
    // 3. RTC 首次配置
	if (BKP_ReadBackupRegister(BKP_DR1) != 0x1234)			//通过写入备份寄存器的标志位，判断RTC是否是第一次配置
	{
        // 设置时钟源
        if (MYRTC_CLK_SOURCE == RCC_RTCCLKSource_LSE) {
            RCC_LSEConfig(RCC_LSE_ON);							        //开启LSE时钟
            while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) != SET);	        //等待LSE准备就绪
        }
        else if (MYRTC_CLK_SOURCE == RCC_RTCCLKSource_LSI) {
            RCC_LSICmd(ENABLE);								            //开启LSI时钟
            while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) != SET);          //等待LSI准备就绪
        }
        else if (MYRTC_CLK_SOURCE == RCC_RTCCLKSource_HSE_Div128) {
            RCC_HSEConfig(RCC_HSE_ON);                                  //开启HSE时钟
            while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) != SET);          //等待HSE准备就绪
        }

		RCC_RTCCLKConfig(MYRTC_CLK_SOURCE);				    //选择RTCCLK来源
		RCC_RTCCLKCmd(ENABLE);								//RTCCLK使能
		
		RTC_WaitForSynchro();								//等待同步
		RTC_WaitForLastTask();								//等待上一次操作完成
		
        // 设置RTC预分频 (使频率为1Hz)
        if (MYRTC_CLK_SOURCE == RCC_RTCCLKSource_LSE) {
            RTC_SetPrescaler(32768 - 1);						        //外部低速时钟 LSE (32.768kHz)
        }
        else if (MYRTC_CLK_SOURCE == RCC_RTCCLKSource_LSI) {
            RTC_SetPrescaler(40000 - 1);						        //内部低速时钟 LSI (40kHz)
        }
        else if (MYRTC_CLK_SOURCE == RCC_RTCCLKSource_HSE_Div128) {
            RTC_SetPrescaler(62500 - 1);						        //外部高速时钟 HSE/128 (8MHz/128 = 8000000 / 128 = 62500 = 62.5kHz)
        }
		RTC_WaitForLastTask();								//等待上一次操作完成
		
		MyRTC_SetTime();									//设置时间，调用此函数，全局数组里时间值刷新到RTC硬件电路
		
		BKP_WriteBackupRegister(BKP_DR1, 0x1234);			//在备份寄存器写入自己规定的标志位，用于判断RTC是不是第一次执行配置
	}
	else													//RTC不是第一次配置
	{
		RTC_WaitForSynchro();								//等待同步
		RTC_WaitForLastTask();								//等待上一次操作完成
	}
}


/**
  * 函    数：RTC设置时间
  * 参    数：无
  * 返 回 值：无
  * 说    明：调用此函数后，全局数组里时间值将刷新到RTC硬件电路
  */
void MyRTC_SetTime(void)
{
	time_t time_cnt;		//定义秒计数器数据类型
	struct tm time_date;	//定义日期时间数据类型
	
	time_date.tm_year = MyRTC_Time.Year - 1900;		//将数组的时间赋值给日期时间结构体
	time_date.tm_mon = MyRTC_Time.Month - 1;
	time_date.tm_mday = MyRTC_Time.Day;
	time_date.tm_hour = MyRTC_Time.Hour;
	time_date.tm_min = MyRTC_Time.Minute;
	time_date.tm_sec = MyRTC_Time.Second;
	
	time_cnt = mktime(&time_date) - MYRTC_TIME_ZONE_OFFSET;	//调用mktime函数，将日期时间转换为秒计数器格式
	
	RTC_SetCounter(time_cnt);						//将秒计数器写入到RTC的CNT中
	RTC_WaitForLastTask();							//等待上一次操作完成
}

/**
  * 函    数：RTC读取时间
  * 参    数：无
  * 返 回 值：无
  * 说    明：调用此函数后，RTC硬件电路里时间值将刷新到全局数组
  */
void MyRTC_ReadTime(void)
{
	time_t time_cnt;		//定义秒计数器数据类型
	struct tm time_date;	//定义日期时间数据类型
	
	time_cnt = RTC_GetCounter() + MYRTC_TIME_ZONE_OFFSET;		//读取RTC的CNT，获取当前的秒计数器
	
	time_date = *localtime(&time_cnt);				//使用localtime函数，将秒计数器转换为日期时间格式
	
	MyRTC_Time.Year = time_date.tm_year + 1900;		//将日期时间结构体赋值给数组的时间
	MyRTC_Time.Month = time_date.tm_mon + 1;
	MyRTC_Time.Day = time_date.tm_mday;
	MyRTC_Time.Hour = time_date.tm_hour;
	MyRTC_Time.Minute = time_date.tm_min;
	MyRTC_Time.Second = time_date.tm_sec;
}

