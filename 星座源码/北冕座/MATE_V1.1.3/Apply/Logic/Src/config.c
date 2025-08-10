#include "config.h"

/* 工程中所用到所有IO
	Thruster
		  PWM0	PB0
		  PWM1	PB4
		  PWM2	PB1
		  PWM3	PB5
		  PWM4	PB6
		  PWM5	PB7
		  PWM6	PB8
		  PWM7	PB9
	JY901S
		PA2	PA3 Uart2
	MS5837
		SCL		SDA
		PC6		PC7
	ADC
		PA0
	Uart1			Uart2			Uart3			Uart4			Uart5
		TX PA9 			TX PA2			TX PB10			TX PC10			TX PC12
		RX PA10			RX PA3			RX PB11			RX PC11			RX PD2
*/
Expect_angle Exp_AngleInfo = {0,0,0};

//定义抽屉变量
volatile char x_y_z_pitch = 0;
volatile char left_rocker = 0;
volatile char right_rocker = 0;
volatile char Mode_control = 0;
volatile char State_control = 0;
volatile char Light_control = 0;
volatile char speed_kH = 0;
volatile char speed_kV = 0;
volatile char DepthFlag = 0;
volatile char SpeedMode = 0;
volatile char cnt = 0;
volatile float DepthPID_out = 0.0;
volatile float DepthFliter = 0.0;
volatile char PitchFlag = 0;
volatile float PitchPID_out = 0.0;
volatile char floatflag = 0;
volatile char sinkflag = 0;
volatile char pHflag = 0;
tagGPIO_T demoGPIO[] =
{
	
	[0]=
	{ //红
		.tGPIOInit.Pin 		= GPIO_PIN_13,				/* GPIO引脚 */
		.tGPIOInit.Mode 	= GPIO_MODE_OUTPUT_PP,		/* GPIO模式 */
		.tGPIOInit.Pull 	= GPIO_NOPULL,				/* GPIO上下拉设置，是否需要上下拉看硬件 */
		.tGPIOInit.Speed 	= GPIO_SPEED_FREQ_HIGH,		/* GPIO速度 */	
		.tGPIOPort 			= GPIOD,					/* GPIO分组 */
	},
    [1]=
	{ 
		.tGPIOInit.Pin 		= GPIO_PIN_14,				/* GPIO引脚 */
		.tGPIOInit.Mode 	= GPIO_MODE_OUTPUT_PP,		/* GPIO模式 */
		.tGPIOInit.Pull 	= GPIO_NOPULL,				/* GPIO上下拉设置，是否需要上下拉看硬件 */
		.tGPIOInit.Speed 	= GPIO_SPEED_FREQ_HIGH,		/* GPIO速度 */	
		.tGPIOPort 			= GPIOD,					/* GPIO分组 */
	},
    [2]=
	{ 
		.tGPIOInit.Pin 		= GPIO_PIN_15,				/* GPIO引脚 */
		.tGPIOInit.Mode 	= GPIO_MODE_OUTPUT_PP,		/* GPIO模式 */
		.tGPIOInit.Pull 	= GPIO_NOPULL,				/* GPIO上下拉设置，是否需要上下拉看硬件 */
		.tGPIOInit.Speed 	= GPIO_SPEED_FREQ_HIGH,		/* GPIO速度 */	
		.tGPIOPort 			= GPIOD,					/* GPIO分组 */
	},
    
};

/* 深度环PID */
tagPID_T DepthPID = 
{
	.fKp = 12,
	.fKi = 0,   
	.fKd = 1,
	.dt = 0.01,
	.second_ord_lim = 100,
};

/* 艏向PID */
tagPID_T YawPID = 
{
	.fKp = 1,
	.fKi = 0,
	.fKd = 0.1,
};

/* 角度环PID */
tagPID_T AngleLoopPID = 
{
	.fKp = 1,
	.fKi = 0,
	.fKd = 0,
};

/* 位置环PID */
tagPID_T PositionLoopPID = 
{
	.fKp = 1,
	.fKi = 0,
	.fKd = 0,
};

/* 平衡PID */
tagPID_T RollPID = 
{
	.fKp = 3,
	.fKi = 0,
	.fKd = 0.1,
};

/* 俯仰PID */
tagPID_T PitchPID = 
{
	.fKp = 1.0,
	.fKi = 0,
	.fKd = 0.1,
};

/* 串口1初始化句柄 */
tagUART_T Uart1 = 
{	
	.tUARTHandle.Instance				= USART1,
	//串口DMA接收参数配置
	.tUartDMA.bRxEnable					= true,						/* DMA接收使能 */
	.tUartDMA.bTxEnable					= true,
	//.tRxInfo.usDMARxMAXSize             = 100,              		/* 接收数据长度 长度保持在协议最长字节*2以上，确保缓存池一定能够稳定接收一个完整的数据帧*/
};

/* 串口3初始化句柄 */
tagUART_T Uart3 = 
{
	.tUARTHandle.Instance				= USART3,
	//串口DMA接收参数配置
	.tUartDMA.bRxEnable					= true,						/* DMA接收使能 */
	//.tUartDMA.bTxEnable					= true,
	//.tRxInfo.usDMARxMAXSize             = 100,              		/* 接收数据长度 长度保持在协议最长字节*2以上，确保缓存池一定能够稳定接收一个完整的数据帧*/
};

/* JY901S参数设置 */
tagJY901_T JY901S = 
{
	.tConfig.ucBaud 	= JY901_RXBAUD_9600,
	.tConfig.ucRate		= JY901_RX_2HZ,
	//.tConfig.usType		= JY901_OUTPUT_ANGLE,
	.tConfig.usType		= JY901_OUTPUT_GYRO,

	.tUART.tRxInfo.usDMARxMAXSize             	= 200,                 /* 接收数据长度 长度保持在协议最长字节*2以上，确保缓存池一定能够稳定接收一个完整的数据帧*/

    .tUART.tUartDMA.bRxEnable					= true,					/* DMA接收使能 */
};

/* MS5837示例 */
tagMS5837_T MS5837 = 
{
    /* 采样分辨率设置 */
	.setOSR = MS5837_OSR4096,

	/* SCL线 */
	.tIIC.tIICSoft[0].tGPIOInit.Pin 		= GPIO_PIN_6,				/* GPIO引脚 */
	.tIIC.tIICSoft[0].tGPIOInit.Mode 		= GPIO_MODE_OUTPUT_PP,		/* GPIO模式 */
	.tIIC.tIICSoft[0].tGPIOInit.Pull 		= GPIO_NOPULL,				/* GPIO上下拉设置，是否需要上下拉看硬件 */
	.tIIC.tIICSoft[0].tGPIOInit.Speed 		= GPIO_SPEED_FREQ_HIGH,		/* GPIO速度 */	
	.tIIC.tIICSoft[0].tGPIOPort 			= GPIOC,					/* GPIO分组 */

	/* SDA线 */
	.tIIC.tIICSoft[1].tGPIOInit.Pin 		= GPIO_PIN_7,				/* GPIO引脚 */
	.tIIC.tIICSoft[1].tGPIOInit.Mode		= GPIO_MODE_INPUT,			/* GPIO模式 */
	.tIIC.tIICSoft[1].tGPIOInit.Pull		= GPIO_NOPULL,				/* GPIO上下拉设置，是否需要上下拉看硬件 */
	.tIIC.tIICSoft[1].tGPIOInit.Speed		= GPIO_SPEED_FREQ_HIGH,		/* GPIO速度 */	
	.tIIC.tIICSoft[1].tGPIOPort 			= GPIOC,					/* GPIO分组 */
};

tagPWM_T PWM[] =
{
	//水平推
	[0] =
	{
		.tPWMHandle.Instance	= TIM3,         	/* 定时器4 */
		.fDuty					= 7.5,				/* 初始占空比（%） */
		.ulFreq					= 50,				/* 频率（Hz） */
		.ucChannel				= TIM_CHANNEL_3,	/* 通道 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_0,		/* IO映射 */
		.tGPIO.tGPIOPort		= GPIOB,			/* IO组映射 */
		.tGPIO.ucAFMode			= NO_REMAP,			/* IO重映射模式 */
	},	 	
	[1] =
	{
		.tPWMHandle.Instance	= TIM3,         	/* 定时器1 */
		.fDuty					= 7.5,				/* 初始占空比（%） */
		.ulFreq					= 50,				/* 频率（Hz） */
		.ucChannel				= TIM_CHANNEL_1,	/* 通道 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_4,		/* IO映射 */
		.tGPIO.tGPIOPort		= GPIOB,			/* IO组映射 */
		.tGPIO.ucAFMode			= PARTIAL_REMAP,		    /* IO重映射模式 */
	},	  
	[2] =
	{
		.tPWMHandle.Instance	= TIM3,         	/* 定时器4 */
		.fDuty					= 7.5,				/* 初始占空比（%） */
		.ulFreq					= 50,	 			/* 频率（Hz） */
		.ucChannel				= TIM_CHANNEL_4,	/* 通道 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_1,		/* IO映射 */
		.tGPIO.tGPIOPort		= GPIOB,			/* IO组映射 */
		.tGPIO.ucAFMode			= NO_REMAP,			/* IO重映射模式 */
	},
	[3] =
	{
		
		.tPWMHandle.Instance	= TIM3,         	/* 定时器1 */
		.fDuty					= 7.5,				/* 初始占空比（%） */
		.ulFreq					= 50,				/* 频率（Hz） */
		.ucChannel				= TIM_CHANNEL_2,	/* 通道 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_5,		/* IO映射 */
		.tGPIO.tGPIOPort		= GPIOB,			/* IO组映射 */
		.tGPIO.ucAFMode			= PARTIAL_REMAP,		    /* IO重映射模式 */
	},

	//机械爪
	[4] =
	{
		.tPWMHandle.Instance	= TIM4,         	/* 定时器4 */
		.fDuty					= 7.5,				/* 初始占空比（%） */ //2.5%带宽为500
		.ulFreq					= 50,				/* 频率（Hz） */
		.ucChannel				= TIM_CHANNEL_1,	/* 通道 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_6,		/* IO映射 */
		.tGPIO.tGPIOPort		= GPIOB,			/* IO组映射 */
		.tGPIO.ucAFMode			= NO_REMAP,			/* IO重映射模式 */
	},
	[5] =
	{
		.tPWMHandle.Instance	= TIM4,         	/* 定时器4 */
		.fDuty					= 3.0,				/* 初始占空比（%） */
		.ulFreq					= 50,				/* 频率（Hz） */
		.ucChannel				= TIM_CHANNEL_2,	/* 通道 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_7,		/* IO映射 */
		.tGPIO.tGPIOPort		= GPIOB,			/* IO组映射 */
		.tGPIO.ucAFMode			= NO_REMAP,		
	},
	[6] =
	{
		.tPWMHandle.Instance	= TIM1,         	/* 定时器1 */
		.fDuty					= 7.5,				/* 初始占空比（%） */
		.ulFreq					= 50,				/* 频率（Hz） */
		.ucChannel				= TIM_CHANNEL_2,	/* 通道 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_11,		/* IO映射 */
		.tGPIO.tGPIOPort		= GPIOE,			/* IO组映射 */
		.tGPIO.ucAFMode			= FULL_REMAP,		/* IO重映射模式 */	
	},	
	[7] =
	{
		.tPWMHandle.Instance	= TIM4,         	/* 定时器4 */
		.fDuty					= 7.5,				/* 初始占空比（%） */
		.ulFreq					= 50,				/* 频率（Hz） */
		.ucChannel				= TIM_CHANNEL_4,	/* 通道 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_9,		/* IO映射 */
		.tGPIO.tGPIOPort		= GPIOB,			/* IO组映射 */
		.tGPIO.ucAFMode			= NO_REMAP,			/* IO重映射模式 */
	},
	
	//垂推
	[8] =
	{
		.tPWMHandle.Instance	= TIM1,         	/* 定时器1 */
		.fDuty					= 7.5,				/* 初始占空比（%） */
		.ulFreq					= 50,				/* 频率（Hz） */
		.ucChannel				= TIM_CHANNEL_3,	/* 通道 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_13,		/* IO映射 */
		.tGPIO.tGPIOPort		= GPIOE,			/* IO组映射 */
		.tGPIO.ucAFMode			= FULL_REMAP,		/* IO重映射模式 */
	},
	[9] =
	{
		.tPWMHandle.Instance	= TIM1,         	/* 定时器1 */
		.fDuty					= 7.5,				/* 初始占空比（%） */
		.ulFreq					= 50,				/* 频率（Hz） */
		.ucChannel				= TIM_CHANNEL_1,	/* 通道 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_9,		/* IO映射 */
		.tGPIO.tGPIOPort		= GPIOE,			/* IO组映射 */
		.tGPIO.ucAFMode			= FULL_REMAP,		/* IO重映射模式 */
		
	},	
	[10] =
	{
		
		.tPWMHandle.Instance	= TIM8,         	/* 定时器1 */
		.fDuty					= 7.5,				/* 初始占空比（%） */
		.ulFreq					= 50,				/* 频率（Hz） */
		.ucChannel				= TIM_CHANNEL_3,	/* 通道 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_8,		/* IO映射 */
		.tGPIO.tGPIOPort		= GPIOC,			/* IO组映射 */
		.tGPIO.ucAFMode			= FULL_REMAP,		/* IO重映射模式 */
	}, 
	[11] =
	{
		.tPWMHandle.Instance	= TIM1,         	/* 定时器1 */
		.fDuty					= 7.5,				/* 初始占空比（%） */
		.ulFreq					= 50,				/* 频率（Hz） */
		.ucChannel				= TIM_CHANNEL_4,	/* 通道 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_14,		/* IO映射 */
		.tGPIO.tGPIOPort		= GPIOE,			/* IO组映射 */
		.tGPIO.ucAFMode			= FULL_REMAP,		/* IO重映射模式 */
	},
	[12] = 
	{
		.tPWMHandle.Instance	= TIM4,         	/* 定时器4 */
		.fDuty					= 6.5,				/* 初始占空比（%） */
		.ulFreq					= 50,				/* 频率（Hz） */
		.ucChannel				= TIM_CHANNEL_3,	/* 通道 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_8,		/* IO映射 */
		.tGPIO.tGPIOPort		= GPIOB,			/* IO组映射 */
		.tGPIO.ucAFMode			= NO_REMAP,			/* IO重映射模式 */
	},
	[13] =
	{
		.tPWMHandle.Instance	= TIM8,         	/* 定时器1 */
		.fDuty					= 10.0,				/* 初始占空比（%） */
		.ulFreq					= 50,				/* 频率（Hz） */
		.ucChannel				= TIM_CHANNEL_4,	/* 通道 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_9,		/* IO映射 */
		.tGPIO.tGPIOPort		= GPIOC,			/* IO组映射 */
		.tGPIO.ucAFMode			= FULL_REMAP,		    /* IO重映射模式 */
	}
};

tagIWDG_T demoIWDG = 
	{
		.usResetTime = 2000,			/* 2S喂狗溢出时间 */
	};
/* ADC句柄示例 */
tagADC_T pH_adc= 
{
    
        .tGPIO.tGPIOInit.Pin                  = GPIO_PIN_1,           /* GPIO引脚 */
        .tGPIO.tGPIOInit.Mode                 = GPIO_MODE_ANALOG,     /* GPIO模式 */
        .tGPIO.tGPIOInit.Pull                 = GPIO_NOPULL,          /* GPIO上下拉设置，是否需要上下拉看硬件 */
        .tGPIO.tGPIOInit.Speed                = GPIO_SPEED_FREQ_HIGH, /* GPIO速度 */    
        .tGPIO.tGPIOPort                      = GPIOA,                /* GPIO分组 */
        
        .tADCHandle.Instance                  = ADC1,
        .tADCHandle.Init.DataAlign            = ADC_DATAALIGN_RIGHT,  /* 右对齐 */
        .tADCHandle.Init.ScanConvMode         = DISABLE,              /* 非扫描模式 */
        .tADCHandle.Init.ContinuousConvMode   = DISABLE,              /* 关闭连续转换 */
        .tADCHandle.Init.NbrOfConversion      = 1,                    /* 1个转换在规则序列中 也就是只转换规则序列1 */ 
        .tADCHandle.Init.DiscontinuousConvMode = DISABLE,              /* 禁止不连续采样模式 */
        .tADCHandle.Init.NbrOfDiscConversion  = 0,                    /* 不连续采样通道数为0 */
        .tADCHandle.Init.ExternalTrigConv     = ADC_SOFTWARE_START,   /* 软件触发 */
        
        .tADCChannel.Channel                 = ADC_CHANNEL_1,        /* 通道 */
        .tADCChannel.Rank                    = 1,                    /* 第1个序列，序列1 */
        .tADCChannel.SamplingTime            = ADC_SAMPLETIME_239CYCLES_5, /* 采样时间 */
        
        .ucAverageNum                        = 10                     /* 均值滤波均值数 */
    
};
