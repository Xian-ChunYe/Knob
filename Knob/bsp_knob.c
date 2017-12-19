#include "user_task_config.h"
#include "board_config.h"
#include "usart_app.h"

extern u8 ATT_BinSemaphore;
//变量定义
struct
{
	u8 old_att_value;
	u8 att_value;
	u8 att_scale[50];
	u8 att_write;
	u8 att_read;	
}digital_knob;


static void bsp_InitKeyVar(void);
static void bsp_InitKeyHard(void);
static void bsp_DetectKey(uint8_t i);

/*
*********************************************************************************************************
*	函 数 名: IsKeyDownX
*	功能说明: 判断按键是否按下
*	形    参: 无
*	返 回 值: 返回值1 表示按下，0表示未按下
*********************************************************************************************************
*/

static void bsp_InitKnobVar(void)
{
    u16 att=0;
   
    STMFLASH_Read(ATT_SAVE_ADDR,&Boards_Info.ATT,1);  
    if(Boards_Info.ATT>20)Boards_Info.ATT=20;
    
    digital_knob.att_value = Boards_Info.ATT*10;
	digital_knob.old_att_value = Boards_Info.ATT*10;
	digital_knob.att_read = 1;
	digital_knob.att_write = 1;
	memset(digital_knob.att_scale,0,50);
    
    digital_knob.att_scale[0] = GPIOC->IDR&0X03;
}	

static void bsp_InitKnobHard(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
   	GPIO_InitTypeDef  GPIO_InitStructure;
    
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //使能E端口时钟
	
   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;	 			//  RES
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//速度50MHz
 	GPIO_Init(GPIOC, &GPIO_InitStructure);	  				//初始化PC6

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟

    //GPIOA.7 中断线以及中断初始化配置   上升沿触发
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource0);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource1);
    
  	EXTI_InitStructure.EXTI_Line=EXTI_Line0;	//KEY
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

  	EXTI_InitStructure.EXTI_Line=EXTI_Line1;	//KEY
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

  	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//使能按键WK_UP所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;	//抢占优先级5， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;					//子优先级0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure); 
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;			//使能按键WK_UP所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//抢占优先级5， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;					//子优先级0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure); 
}

//外部中断0服务程序
void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {	 
        EXTI_ClearITPendingBit(EXTI_Line0);  //清除LINE7上的中断标志位 
        digital_knob.att_scale[digital_knob.att_write] = GPIOC->IDR&0X03;
        digital_knob.att_write++;
        if(digital_knob.att_write>=50)
        {
            digital_knob.att_write = 1;
        }
    }
}

//外部中断1服务程序
void EXTI1_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line1);  //清除LINE7上的中断标志位 
        digital_knob.att_scale[digital_knob.att_write] = GPIOC->IDR&0X03;
        digital_knob.att_write++;
        if(digital_knob.att_write>=50)
        {
            digital_knob.att_write = 1;
        }
    }
}

/*
*********************************************************************************************************
*	函 数 名: bsp_InitKey
*	功能说明: 初始化按键. 该函数被 bsp_Init() 调用。
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitKnob(void)
{
	bsp_InitKnobVar();		/* 初始化按键变量 */
	bsp_InitKnobHard();		/* 初始化按键硬件 */
}


/*
*********************************************************************************************************
*	函 数 名: bsp_GetKey
*	功能说明: 从按键FIFO缓冲区读取一个键值并判断。
*	形    参:  无
*	返 回 值: 
*********************************************************************************************************
*/
static uint8_t bsp_ATT_Knob_Deal(void)
{
	u8 ret=0,old_ret=0;

    if(digital_knob.att_read != digital_knob.att_write)
    {
        EXTI->IMR&=~(1<<0);//屏蔽line0上的中断 
        EXTI->IMR&=~(1<<1);//屏蔽line1上的中断 
        
        while(digital_knob.att_read != digital_knob.att_write)
        {
            ret = digital_knob.att_scale[digital_knob.att_read];
            old_ret = digital_knob.att_scale[digital_knob.att_read-1];
            
            switch(ret)
            {
                case 0:
                    if(old_ret == 1)
                    {
                        if(digital_knob.old_att_value < 200)
                        digital_knob.att_value = digital_knob.att_value+10;
                    }	
                    else if(old_ret == 2)				
                    {
                        if(digital_knob.old_att_value >0)
                        digital_knob.att_value = digital_knob.att_value-10;
                    }
                    break;
                    
                case 1:
                    if(old_ret == 3)
                    {
                        if(digital_knob.old_att_value < 200)
                        digital_knob.att_value = digital_knob.att_value+10;
                    }	
                    else if(old_ret == 0)				
                    {
                        if(digital_knob.old_att_value >0)
                        digital_knob.att_value = digital_knob.att_value-10;
                    }
                    break;
                    
                case 3:
                    if(old_ret == 2)
                    {
                        if(digital_knob.old_att_value < 200)
                        digital_knob.att_value = digital_knob.att_value+10;
                    }	
                    else if(old_ret == 1)				
                    {
                        if(digital_knob.old_att_value >0)
                        digital_knob.att_value = digital_knob.att_value-10;
                    }
                    break;
                    
                case 2:
                    if(old_ret == 0)
                    {
                        if(digital_knob.old_att_value < 200)
                        digital_knob.att_value = digital_knob.att_value+10;
                    }	
                    else if(old_ret == 3)				
                    {
                        if(digital_knob.old_att_value >0)
                        digital_knob.att_value = digital_knob.att_value-10;
                    }
                    break;
            }
            digital_knob.old_att_value =  digital_knob.att_value;  
            digital_knob.att_read++;
        }
    
        memset(digital_knob.att_scale,0,50);
        digital_knob.att_scale[0] = ret;
        digital_knob.att_read = 1;
        digital_knob.att_write = 1;

        if(Boards_Info.ATT != digital_knob.att_value/10)
        {
            Boards_Info.ATT = digital_knob.att_value/10;
            STMFLASH_Write(ATT_SAVE_ADDR,&Boards_Info.ATT,1); 
            ATT_BinSemaphore =1;
            OLED_BinSemaphore=2;
        }
        
        EXTI->IMR|=1<<0;//不屏蔽line0上的中断
        EXTI->IMR|=1<<1;//不屏蔽line1上的中断 
    }
}

void bsp_Knob_Deal(void)
{
	bsp_ATT_Knob_Deal();
}
