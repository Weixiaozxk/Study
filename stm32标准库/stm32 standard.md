### STM32 标准库
以STM32F4系列为例
ps:强制类型转换方式:uint16_t (value*100)%100;
#### 一、GPIO
##### 1.八大模式 
![alt text](image.png)
开漏输出：在输出驱动器中，P-MOS无效，N-MOS导通时的输出，高电平没有驱动能力
推挽输出：在输出驱动器中，P-MOS与N-MOS均为导通
##### 2.GPIO配置
```c
	GPIO_InitTypeDef  GPIO_InitStructure;   //初始化GPIO结构体
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   //初始化引脚时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;   //初始化GPIO引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   //配置GPIO为输出模式  
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //配置GPIO为推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //配置GPIO传输速度
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    //配置上下拉电阻
	GPIO_Init(GPIOB, &GPIO_InitStructure);      //GPIO总初始化
```
常用函数
```c
    /**
      * GPIO_TypeDef* GPIOx 
      * GPIO_Pin
      * BitAction BitVal: Bit_RESET 低电平
                         Bit_SET 高电平
    **/
    GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal)
    /**
     * 函数功能：GPIO输出高电平
     * GPIO_TypeDef* GPIOx
     * uint16_t GPIO_Pin：引脚
     **/
    GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
    /**
     * 函数功能：GPIO输出低电平
     * GPIO_TypeDef* GPIOx
     * uint16_t GPIO_Pin：引脚
     **/
     GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
    /**
     * 函数功能：GPIO翻转电平
     * GPIO_TypeDef* GPIOx
     * uint16_t GPIO_Pin：引脚
     **/  
     GPIO_ToggleBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
``` 
#### 二、LED
##### 1.led.c
```c
//LED初始化
void LED_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;   //初始化GPIO结构体
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   //初始化引脚时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;   //初始化GPIO引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   //配置GPIO为输出模式  
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //配置GPIO为推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //配置GPIO传输速度
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    //配置上下拉电阻
	GPIO_Init(GPIOB, &GPIO_InitStructure);      //GPIO总初始化    
}

//开启LED
void LED_ON(void)
{
    GPIO_SetBits(GPIOA,GPIO_Pin_1);
}
//关闭LED
void LED_OFF(void)
{
    GPIO_ResetBits(GPIOA,GPIO_Pin_1);
}
//翻转LED
void LED_Turn(void)
{
    if(GPIO输出寄存器（GPIO_ReadOutputDataBit）==0)
    {
        GPIO_ToggleBits(GPIOA, uint16_t GPIO_Pin_1)；
    }
}
```
##### 2.led.h
```c
#ifndef __LED_H
#define __LED_H

void LED_init(void);
void LED_ON(void);
void LED_OFF(void);
void LED_Turn(void);
#endif
```
#### 三、KEY(高低电平版)
##### 1.key.c
```c
#include "stm32f40x.h"

void Key_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;   //初始化GPIO结构体
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   //初始化引脚时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;   //初始化GPIO引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   //配置GPIO为输出模式  
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //配置GPIO为推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //配置GPIO传输速度
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    //配置上下拉电阻
	GPIO_Init(GPIOB, &GPIO_InitStructure);      //GPIO总初始化       
}   
uint8_t Key_GetNum(void)
{
    uint8_t KeyNum=0;
    if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5==0))
    {
        delay_ms(20);
        while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5)==0);
        delay_ms(20);
        KeyNum=1;
    }

    if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6==0))
    {
        delay_ms(20);
        while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6)==0);
        delay_ms(20);
        KeyNum=2;
    }

    if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7==0))
    {
        delay_ms(20);
        while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)==0);
        delay_ms(20);
        KeyNum=3;
    }
    return KeyNum;
}
```
##### 2.key.h 
```c
#ifndef __KEY_H
#define __KEY_H

void Key_init(void);
uint8_t Key_GetNum(void);
#endif
```
##### 3.main.h
```c
#include "key.h"
#include "led.h"
uint8_t KeyNum;

int main()
{
        Key_init();
        Key_GetNum();

    while(1)
    {
        KeyNum=Key_GetNum();
        if(KeyNum==1)   //按键1
        {
            LED_ON();
        }
        if(KeyNum==2)   //按键2
        {
            LED_OFF();
        }
        if(KeyNum==3)   //按键3
        {
            LED_Turn();
        }
    }
}
```

#### 四、EXTI
RCC->AFIO（复用功能重映射）->EXTI->NVIC->CPU
F4系列中没有AFIO需要配置SYSCFG存储器
EXIT函数
```C
        EXTI_InitTypeDef EXTI_InitStructure;//结构体类型名EXTI_InitTypeDef，变量名EXTI_InitStructure
	EXTI_InitStructure.EXTI_Line = EXTI_Line14; //根据所配置的GPIO引脚选择EXIT_Line数字
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //中断模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//GPIO_Mode_IPU设置为高电平，触发中断是下降
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

    //检查外部中断状态
    EXTI_GetITStatus(uint32_t EXTI_Line);
    // 清除中断标志位
    EXTI_ClearITPendingBit(uint32_t EXTI_Line);
```
##### 1.EXTI.c
带有AFIO板卡中断函数的编写
```c
#include "stm32f10x.h"

void EXTI_init(void)
{
	//第一步，时钟配置
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); //开启GPIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//开启AFIO时钟
	//EXTI和NVIC两个外设的时钟是一直开的 ，NVIC内核外设都是不需要开启时钟
 
	//第二步，配置GPIO
	//首先定义结构体
	GPIO_InitTypeDef GPIO_InitStructure;  //结构体名字GPIO_InitStructure
	//将结构体成员引出来
	//对于EXTI来说，模式为浮空输入|上拉输入|下拉输入；不知该写什么模式，可以看参考手册中的外设GPIO配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //由于GPIO为输入模式传输速度无限制
	//最后初始化GPIO
	GPIO_Init(GPIOB,&GPIO_InitStructure);	
	
	//第三步，配置AFIO外设中断引脚选择
	//AFIO的库函数是和GPIO在一个文件里，可以查看Library文件中的gpio.h查看函数
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource14);//代表连接PB14号口的第14个中断线路
	
	//第四步，配置EXTI,这样PB14的电平信号就能够通过EXTI通向下一级的NVIC了
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line14;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//因为上面是GPIO_Mode_IPU设置为高电平，所以触发中断是下降
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	//第五步，配置NVIC，NVIC是内核外设，所以它的库函数在misc.h
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //分组方式，整个芯片只能用一种。如放在模块中进行分组，要确保每个模块分组都选的是同一个；或者将这个代码放在主函数的最开始
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//查询中断向量表获得
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}

void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line14) == SET)
    {

        //事件

        EXTI_ClearITPendingBit(EXTI_Line14);
    }
}
```
F4为例不带有AFIO
```c
#include "stm32f40x.h"

void EXTI_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); //开启GPIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//开启中断时钟

    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_D;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //配置中断
    NVIC_InitTypeDef   NVIC_InitStructure;
    EXTI_InitTypeDef   EXTI_InitStructure;
        
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);

    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);    
}

void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) == SET)
    {
        
        //事件

        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
```

##### 5.EXTI.h
```c
#ifndef __EXTI_H
#define __EXTI_H
void EXTI_init(void);
#endif
```

#### 五、TIM
常用函数
```c
/*定时器结构体定义*/
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
/* 配置定时器参数 */
TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseStructure.TIM_Period = 10000;
TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1; // TIMER3时钟频率84MHz
/*定时器初始化*/
TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
/*使能中断*/
TIM_ITConfig(TIM3,TIM_IT_Update, ENABLE);
/***使能计数器***/
TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState);
```
计算公式：
定时频率=主频/(PSC+1)/(ARR+1)

##### 1.timer.c
```c
void Timer_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  //初始化结构体
	NVIC_InitTypeDef NVIC_InitStructure;    //初始化NVIC结构体
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE); 
	
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;    //设置向上计数
	TIM_TimeBaseStructure.TIM_Period = 10000;   //自动重装载值
	TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;     //预分频系数
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);     //初始化定时器
	
	TIM_ITConfig(TIM3,TIM_IT_Update, ENABLE);   //使能中断
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
	NVIC_Init(&NVIC_InitStructure); 
	
	TIM_Cmd(TIM3,ENABLE);   //使能定时器
}

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) //获取中断标志位
	{ 
        // {}中为中断处理
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除更新中断
}
```
##### 2.timer.h
```c
#ifndef __TIMER_H
#define __TIMER_H
void Timer_init(void);
#endif
```

#### 六、PWM
常用函数
```c
    /*结构体*/
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    //初始化TIM3 Channel2 PWM模式         
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
```

PWM频率=定时频率
PWM占空比=CCR/(ARR+1)

##### 1.PWM.c
```c
void TIM3_PWM_Init(u16 arr,u16 psc)
{  
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,  ENABLE);	//使能定时器3时钟
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   //使能GPIO外设
	   
 
   //设置该引脚为复用输出功能,输出TIM3 CH2的PWM脉冲波形	GPIOB5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3); //复用
 
   //初始化TIM3
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc - 1; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 时钟分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM3 Channel2 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2
 
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
 
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3
	
}  
```
输出函数
```c
    void TIM_SetCompare2(TIM_TypeDef* TIMx, uint32_t Compare2); //(TIMx,CRR)
```

##### 2.PWM.h
```c
#ifndef __PWM_H
#define __PWM_H
void TIM3_PWM_Init(u16 arr,u16 psc);
#endif
```
#### 七、ADC
ADC采样通道分为规则组与注入组，规则组有16个通道，一般ADC采样选择规则组，注入组有4个通道，不常用。
规则组：用于设置和转换一组指定通道的组。这些通道按照预定义的顺序和参数逐个进行转换。规则组的转换通常是连续的，且可以循环进行。规则组适用于需要持续监测或周期性采样的场景。
注入组：注入组则用于在规则组转换过程中临时插入对某些通道的转换。当需要中断规则组的转换，对某些特定通道进行快速采样时，可以使用注入组。注入组的转换可以看作是规则组转换过程中的一个中断或插入操作。
![alt text](image-2.png)
ADC规则组的四种转换模式
>|单次转换|非扫描模式|
>|-----------|-----------|
>|单次转换|扫描模式|
>|连续转换|非扫描模式|
>|连续转换|扫描模式|
常用配置函数
```c
/*ADC时钟开启函数*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
/*ADC公共配置*/
    ADC_CommonInitTypeDef ADC_CommonInitStruct; //ADC初始化公共配置结构体

    ADC_DeInit();//ADC复位

    ADC_CommonInitStruct.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled;  //禁用DMA访问模式
    ADC_CommonInitStruct.ADC_Mode=ADC_Mode_Independent;     //设置ADC为独立模式
    ADC_CommonInitStruct.ADC_Prescaler=ADC_Prescaler_Div4;  //预分频：设置四分频
    ADC_CommonInitStruct.ADC_TwoSamplingDelay=ADC_TwoSamplingDelay_5Cycles; //采样时钟周期

    ADC_CommonInit(&ADC_CommonInitStruct);  //启用ADC公共配置
/*ADC特定配置*/
    ADC_InitTypeDef ADC_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);    //ADC初始化特定配置结构体

    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;    //禁用连续转换，使用单次转换
    ADC_InitStruct.ADC_DataAlign=ADC_DataAlign_Right;   //使用右对齐方式
    ADC_InitStruct.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;  //禁用外部触发转换，使用软件触发转换
    ADC_InitStruct.ADC_NbrOfConversion=1;   //设置转换数量（采集通道数量）单采集设置1，不采用扫描模式
    ADC_InitStruct.ADC_Resolution=ADC_Resolution_12b;   //设置ADC分辨率
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;  //禁用or使用扫描模式（单or多）

    ADC_Init(ADC1, &ADC_InitStruct);    //启用ADC特定配置

/*ADC1使能*/
    ADC_Cmd(ADC1, ENABLE);
```

##### （单通道）AD.c
```c
void Adc_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStruct;
    ADC_InitTypeDef ADC_InitStruct;

    RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);  

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    //不上拉不下拉电阻
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_DeInit();//ADC复位

    ADC_CommonInitStruct.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled;  //禁用DMA访问模式
    ADC_CommonInitStruct.ADC_Mode=ADC_Mode_Independent;     //设置ADC为独立模式
    ADC_CommonInitStruct.ADC_Prescaler=ADC_Prescaler_Div4;  //预分频：设置四分频
    ADC_CommonInitStruct.ADC_TwoSamplingDelay=ADC_TwoSamplingDelay_5Cycles; //采样时钟周期

    ADC_CommonInit(&ADC_CommonInitStruct);


    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;    //禁用连续转换，使用单次转换
    ADC_InitStruct.ADC_DataAlign=ADC_DataAlign_Right;   //使用右对齐方式
    ADC_InitStruct.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;  //禁用外部触发转换，使用软件触发转换
    ADC_InitStruct.ADC_NbrOfConversion=1;   //设置转换数量（采集通道数量）单采集设置1，不采用扫描模式
    ADC_InitStruct.ADC_Resolution=ADC_Resolution_12b;   //设置ADC分辨率
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;  //禁用or使用扫描模式（单or多）

    ADC_Init(ADC1, &ADC_InitStruct);

    ADC_Cmd(ADC1, ENABLE);
}

/**********************************************************
 * 函 数 名 称：Get_ADC_Value
 * 函 数 功 能：读取ADC值
 * 传 入 参 数：ADC_CHANNEL_x=要采集的通道
**********************************************************/
unsigned int Get_ADC_Value(uint8_t  ADC_CHANNEL_x)
{
    unsigned int adc_value = 0;
    //设置采集通道
    ADC_RegularChannelConfig(ADC1, CHx, 1, ADC_SampleTime_480Cycles );
    //开始软件转换
    ADC_SoftwareStartConv(ADC1);
    // 等待转换结束
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
    
    //读取采样值
    adc_value = ADC_GetConversionValue(ADC1);
    
    //返回采样值
    return adc_value;
}

 /**********************************************************
 * 函 数 名 称：Get_Adc_Average
 * 函 数 功 能：计算得出某个通道给定次数采样的平均值 
 * 传 入 参 数：CHx                ：通道数字
 *             times              ：采集次数
**********************************************************/
uint16_t Get_Adc_Average(uint8_t CHx,uint8_t times)
{
         
        uint32_t value = 0;
        uint8_t t;
        
        for(t=0;t<times;t++)
        {
            value += Get_Adc(CHx);
            delay_ms(5);
        }
        return value/times;
}
```
##### （单通道）AD.h
```c
#ifndef __AD_H
#define __AD_H
void Adc_Init(void);
unsigned int Get_ADC_Value(uint8_t  ADC_CHANNEL_x);
uint16_t Get_Adc_Average(uint8_t CHx,uint8_t times)
#endif
```
##### （单通道）main.c
```c
int main(void)
{
	Adc_Init();
	
	while(1)
	{
		uint16_t value = Get_Adc_Average(ADC_Channel_0,20);
		

	}
	

}
```

##### （多通道）AD.c
```c
void Adc_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStruct;
    ADC_InitTypeDef ADC_InitStruct;

    RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);  

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    //不上拉不下拉电阻
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_DeInit();//ADC复位

    ADC_CommonInitStruct.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled;  //禁用DMA访问模式
    ADC_CommonInitStruct.ADC_Mode=ADC_Mode_Independent;     //设置ADC为独立模式
    ADC_CommonInitStruct.ADC_Prescaler=ADC_Prescaler_Div4;  //预分频：设置四分频
    ADC_CommonInitStruct.ADC_TwoSamplingDelay=ADC_TwoSamplingDelay_5Cycles; //采样时钟周期

    ADC_CommonInit(&ADC_CommonInitStruct);


    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;    //禁用连续转换，使用单次转换
    ADC_InitStruct.ADC_DataAlign=ADC_DataAlign_Right;   //使用右对齐方式
    ADC_InitStruct.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;  //禁用外部触发转换，使用软件触发转换
    ADC_InitStruct.ADC_NbrOfConversion=1;   //设置转换数量（采集通道数量）单采集设置1，不采用扫描模式
    ADC_InitStruct.ADC_Resolution=ADC_Resolution_12b;   //设置ADC分辨率
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;  //禁用or使用扫描模式（单or多）

    ADC_Init(ADC1, &ADC_InitStruct);

    ADC_Cmd(ADC1, ENABLE);
}

/**********************************************************
 * 函 数 名 称：Get_ADC_Value
 * 函 数 功 能：读取ADC值
 * 传 入 参 数：ADC_CHANNEL_x=要采集的通道
**********************************************************/
unsigned int Get_ADC_Value(uint8_t  ADC_CHANNEL_x)
{
    unsigned int adc_value = 0;
    //设置采集通道
    ADC_RegularChannelConfig(ADC1, CHx, 1, ADC_SampleTime_480Cycles );
    //开始软件转换
    ADC_SoftwareStartConv(ADC1);
    // 等待转换结束
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
    
    //读取采样值
    adc_value = ADC_GetConversionValue(ADC1);
    
    //返回采样值
    return adc_value;
}

 /**********************************************************
 * 函 数 名 称：Get_Adc_Average
 * 函 数 功 能：计算得出某个通道给定次数采样的平均值 
 * 传 入 参 数：CHx                ：通道数字
 *             times              ：采集次数
**********************************************************/
uint16_t Get_Adc_Average(uint8_t CHx,uint8_t times)
{
         
        uint32_t value = 0;
        uint8_t t;
        
        for(t=0;t<times;t++)
        {
            value += Get_Adc(CHx);
            delay_ms(5);
        }
        return value/times;
}
```
##### （多通道）AD.h
```c
#ifndef __AD_H
#define __AD_H
void Adc_Init(void);
unsigned int Get_ADC_Value(uint8_t  ADC_CHANNEL_x);
uint16_t Get_Adc_Average(uint8_t CHx,uint8_t times)
#endif
```

##### （多通道）main.c
```c
int main(void)
{
	Adc_Init();
	
	while(1)
	{
		uint16_t value1  = Get_Adc_Average(ADC_Channel_0,20);
		uint16_t value2  = Get_Adc_Average(ADC_Channel_1,20);
        uint16_t value3  = Get_Adc_Average(ADC_Channel_2,20);
        uint16_t value3  = Get_Adc_Average(ADC_Channel_3,20);       
	}
}
```

#### 八、iic

