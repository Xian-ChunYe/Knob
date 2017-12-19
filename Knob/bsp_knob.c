#include "user_task_config.h"
#include "board_config.h"
#include "usart_app.h"

extern u8 ATT_BinSemaphore;
//��������
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
*	�� �� ��: IsKeyDownX
*	����˵��: �жϰ����Ƿ���
*	��    ��: ��
*	�� �� ֵ: ����ֵ1 ��ʾ���£�0��ʾδ����
*********************************************************************************************************
*/
static u8 GetKnobScale_ATT(void) {return (((GPIOC->IDR & GPIO_Pin_1)) + ((GPIOC->IDR & GPIO_Pin_0)));}  

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
    
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //ʹ��E�˿�ʱ��
	
   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;	 			//  RES
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//�ٶ�50MHz
 	GPIO_Init(GPIOC, &GPIO_InitStructure);	  				//��ʼ��PC6

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//ʹ�ܸ��ù���ʱ��

    //GPIOA.7 �ж����Լ��жϳ�ʼ������   �����ش���
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource0);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource1);
    
  	EXTI_InitStructure.EXTI_Line=EXTI_Line0;	//KEY
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

  	EXTI_InitStructure.EXTI_Line=EXTI_Line1;	//KEY
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

  	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//ʹ�ܰ���WK_UP���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;	//��ռ���ȼ�5�� 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;					//�����ȼ�0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure); 
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;			//ʹ�ܰ���WK_UP���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//��ռ���ȼ�5�� 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;					//�����ȼ�0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure); 
}

//�ⲿ�ж�0�������
void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {	 
        EXTI_ClearITPendingBit(EXTI_Line0);  //���LINE7�ϵ��жϱ�־λ 
        digital_knob.att_scale[digital_knob.att_write] = GPIOC->IDR&0X03;
        digital_knob.att_write++;
        if(digital_knob.att_write>=50)
        {
            digital_knob.att_write = 1;
        }
    }
}

//�ⲿ�ж�1�������
void EXTI1_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line1);  //���LINE7�ϵ��жϱ�־λ 
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
*	�� �� ��: bsp_InitKey
*	����˵��: ��ʼ������. �ú����� bsp_Init() ���á�
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitKnob(void)
{
	bsp_InitKnobVar();		/* ��ʼ���������� */
	bsp_InitKnobHard();		/* ��ʼ������Ӳ�� */
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_GetKey
*	����˵��: �Ӱ���FIFO��������ȡһ����ֵ���жϡ�
*	��    ��:  ��
*	�� �� ֵ: 
*********************************************************************************************************
*/
static uint8_t bsp_ATT_Knob_Deal(void)
{
	u8 ret=0,old_ret=0;

    if(digital_knob.att_read != digital_knob.att_write)
    {
        EXTI->IMR&=~(1<<0);//����line0�ϵ��ж� 
        EXTI->IMR&=~(1<<1);//����line1�ϵ��ж� 
        
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
        
        EXTI->IMR|=1<<0;//������line0�ϵ��ж�
        EXTI->IMR|=1<<1;//������line1�ϵ��ж� 
    }
}

void bsp_Knob_Deal(void)
{
	bsp_ATT_Knob_Deal();
}
