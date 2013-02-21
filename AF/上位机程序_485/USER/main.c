
#include "rs485.h"
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"	 
#include "24cxx.h"
#include "includes.h" 	 
#include <math.h>
#include "adc.h"
#include "timer.h"
//32

/////////////////////////UCOSII��������///////////////////////////////////
//START ����
//�����������ȼ�
#define START_TASK_PRIO      			10 //��ʼ��������ȼ�����Ϊ���
//���������ջ��С
#define START_STK_SIZE  				64
//�����ջ	
OS_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *pdata);	
 			   
//LED����
//�����������ȼ�
#define TAKE_TASK_PRIO       			7 
//���������ջ��С
#define TAKE_STK_SIZE  		    		64
//�����ջ
OS_STK TAKE_TASK_STK[TAKE_STK_SIZE];
//������
void Heartbeat_task(void *pdata);

//��������
//�����������ȼ�
#define Receive_TASK_PRIO       			8 
//���������ջ��С
#define Receive_STK_SIZE  		    		64
//�����ջ
OS_STK Receive_TASK_STK[Receive_STK_SIZE];
//������
void  Receive_task(void *pdata);





//������
//�����������ȼ�
#define MAIN_TASK_PRIO       			4 
//���������ջ��С
#define MAIN_STK_SIZE  					128
//�����ջ	
OS_STK MAIN_TASK_STK[MAIN_STK_SIZE];
//������
void main_task(void *pdata);

//�ź���������
//�����������ȼ�
#define MASTER_TASK_PRIO       			3 
//���������ջ��С
#define MASTER_STK_SIZE  		 		64
//�����ջ	
OS_STK MASTER_TASK_STK[MASTER_STK_SIZE];
//������
 void master_task(void *pdata);
 
OS_EVENT *Heartbeat;			 //�����ź���
OS_EVENT * RS485_MBOX;			//	rs485�����ź���

//����ɨ������
//�����������ȼ�
#define KEY_TASK_PRIO       			2 
//���������ջ��С
#define KEY_STK_SIZE  					64
//�����ջ	
OS_STK KEY_TASK_STK[KEY_STK_SIZE];
//������
void key_task(void *pdata);


//////////////////////////////////////////////////////////////////////////////
 #define TIM3_IRQChannel              ((u8)0x1D)  /* TIM3 global Interrupt */
#define TIM_CounterMode_Up                 ((uint16_t)0x0000)
#define TIM_CKD_DIV1                       ((uint16_t)0x0000)
 typedef struct
{
  uint16_t TIM_Prescaler;         /*!< Specifies the prescaler value used to divide the TIM clock.
                                       This parameter can be a number between 0x0000 and 0xFFFF */

  uint16_t TIM_CounterMode;       /*!< Specifies the counter mode.
                                       This parameter can be a value of @ref TIM_Counter_Mode */

  uint16_t TIM_Period;            /*!< Specifies the period value to be loaded into the active
                                       Auto-Reload Register at the next update event.
                                       This parameter must be a number between 0x0000 and 0xFFFF.  */ 

  uint16_t TIM_ClockDivision;     /*!< Specifies the clock division.
                                      This parameter can be a value of @ref TIM_Clock_Division_CKD */

  uint8_t TIM_RepetitionCounter;  /*!< Specifies the repetition counter value. Each time the RCR downcounter
                                       reaches zero, an update event is generated and counting restarts
                                       from the RCR value (N).
                                       This means in PWM mode that (N+1) corresponds to:
                                          - the number of PWM periods in edge-aligned mode
                                          - the number of half PWM period in center-aligned mode
                                       This parameter must be a number between 0x00 and 0xFF. 
                                       @note This parameter is valid only for TIM1 and TIM8. */
} TIM_TimeBaseInitTypeDef;   
 ////////////////////////////////////////////////////////////////////   	   
#define LEN 7
#define ID  3
#define TIM_IT_Update                      ((uint16_t)0x0001)
typedef struct  
{ 
  u8 start;
  u8 myid;      //��������ID��
  u8 source;
  u8 destination; //Ŀ�ĵ�����
  u8 send;      //�Ƿ��Ƿ�������1Ϊ�ǣ�0Ϊ����
  u8 relay;    //�ڼ��������
  u8 message;     //������Ϣ
  u8 master;      //��������
  u8 end;   
}box;

box mybox;

u8 token[33];//����������

u8 cont=0;//���ڸ��������ŵļǴ�����

u16  dog_clock=10;

u8 rs485buf[LEN],lon=LEN;

//���ջ����� 	
u8 RS485_RX_BUF[64];  	//���ջ���,���64���ֽ�.
//���յ������ݳ���
u8 RS485_RX_CNT=0;  
//////////////////////////////////////////
void initmybox(void);
void order_trans_rs485(u8,u8,u8,u8,u8,u8,u8);
int rs485_trans_order(u8 *);
int subcontrol(u8,u8);
void modfiy_token_array(u8,u8);
void turn_master_id(void);
/////////////////////////////////////////////
int main(void)
 {	 
  
	 
	
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_Configuration(); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
 	LED_Init();			     //LED�˿ڳ�ʼ��

	RS485_Init(9600);	//��ʼ��RS485
	TIM3_Int_Init(4999,7199);//10Khz�ļ���Ƶ�ʣ�����5K��Ϊ500ms  
	 initmybox();
	OSInit();  	 			//��ʼ��UCOSII
			  
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//������ʼ����
	OSStart();	    
}							    
//��ʼ����
void start_task(void *pdata)
{
    OS_CPU_SR cpu_sr=0;  	    
	pdata = pdata; 	
	 Heartbeat=OSSemCreate(0);
	 RS485_MBOX=OSMboxCreate((void*)0);
	OSStatInit();					//��ʼ��ͳ������.�������ʱ1��������	
 	OS_ENTER_CRITICAL();			//�����ٽ���(�޷����жϴ��)    			   
 	OSTaskCreate(master_task,(void *)0,(OS_STK*)&MASTER_TASK_STK[MASTER_STK_SIZE-1],MASTER_TASK_PRIO);	 				   
 	OSTaskCreate(Heartbeat_task,(void *)0,(OS_STK*)&TAKE_TASK_STK[TAKE_STK_SIZE-1],TAKE_TASK_PRIO);
	OSTaskCreate(Receive_task,(void *)0,(OS_STK*)&Receive_TASK_STK[Receive_STK_SIZE-1],Receive_TASK_PRIO);
 	OSTaskSuspend(START_TASK_PRIO);	//������ʼ����.
	OS_EXIT_CRITICAL();				//�˳��ٽ���(���Ա��жϴ��)
}
//LED����
void Heartbeat_task(void *pdata)//master����������
{		// u8 key;
        u8 i=0;
		u8 err;
	//  u8 rs485buf[5];
	while(1)
	{
	OSSemPend(Heartbeat,0,&err);
		//key=KEY_Scan(0);
		//if(key==KEY_RIGHT)//KEY0����,����һ������
		for(i=1;i<33;i++)
		{	
	       order_trans_rs485(mybox.start,mybox.myid,i,0,0,0,mybox.end);//����5���ֽ�
		    delay_us(10000);
		  // LCD_ShowxNum(60+i*32,190,0,7,16,0X80);
		}		 
				   
	}
}

void Receive_task(void *pdate)//��������
{   u8 err;
	 u8 *msg;
	 int flag1,flag2;
	
    while(1)
    	{
		 if(mybox.master==1)OSTaskSuspend(Receive_TASK_PRIO);
	 msg=(u8 *)OSMboxPend(RS485_MBOX,0,&err);//���յ�������
	 flag1=rs485_trans_order(msg);
	 	 
	 if(flag1==0);/***�������Ǹ�����ͨ�ţ���Ϣ����***/
	 if(flag1==1)/***�Ǳ�����Ϣ***/
	 	{		LED1=!LED1;	  
		       dog_clock=10;	 //ι��
	 	      flag2=subcontrol(mybox.relay,mybox.message);
		       if(flag2==0);/******������Ϣ���������ţ������������¶�λ�������������ظ��������ӻ�Ҳ����    ι��*********/
               if(flag2==1);/*****��λ�����������ι��*****/		  
	 	}

	/* ʱ�䵽�ﺯ��,�ж��Ƿ������Ѿ�����*/

	
	/*	if(key)//���յ�������
		{
			if(key>5)key=5;//�����5������.
 			for(i=0;i<key;i++)LCD_ShowxNum(60+i*32,230,rs485buf[i],3,16,0X80);	//��ʾ����
 		}
		t++; 
		delay_ms(10);
		if(t==20)
		{
			LED0=!LED0;//��ʾϵͳ��������	
			t=0;
			cnt++;
			LCD_ShowxNum(60+48,150,cnt,3,16,0X80);	//��ʾ����
		}
    	}*/
	}
	}
 /**************��������**********************/
  void master_task(void *pdata)
  {	  OS_CPU_SR cpu_sr=0;
   while(1){
  	if(mybox.master==1)
	{
	 OSSemPost(Heartbeat);
			delay_ms(100);
			LED0=!LED0;
                                              //�������ճ���
	}
	if(mybox.master==0)
	{
		OS_ENTER_CRITICAL();
		 OSTaskResume(Receive_TASK_PRIO );//�����ӻ�����״̬
		OSTaskSuspend(MASTER_TASK_PRIO );//����������״̬.
		OS_EXIT_CRITICAL();	
	}
	}//while
  }



 /***********************************/


void initmybox()//��ʼ��������Ϣ
{  	 
  u8 i;
  
  for(i=1;i<33;i++)token[i]=0;//��ʼ������
 // if(ID==1){mybox.master=1;token[1]=1;}
  // if(ID!=1)mybox.master=0;
  mybox.master=0;
   token[1]=1;
 mybox.start='&';
 mybox.myid=ID;
 mybox.source=0;
 mybox.destination=0;
 mybox.send=0;
 mybox.relay=0;
 mybox.message=0;
 mybox.end='*';	
						
}


 void order_trans_rs485(u8 start,u8 source,u8 destination, u8 send,u8 relay,u8 message,u8 end)//���������������������RS485��Ϣ�����͸�Ŀ�Ĵӻ�
{  	 OS_CPU_SR cpu_sr=0;
    OS_ENTER_CRITICAL();
    rs485buf[0]=start;
	rs485buf[1]=source;
	rs485buf[2]=destination;
	rs485buf[3]=send;
	rs485buf[4]=relay;
	rs485buf[5]=message;
	rs485buf[6]=end;
	RS485_Send_Data(rs485buf,7);//����5���ֽ�
	OS_EXIT_CRITICAL();	
}

int rs485_trans_order(u8 *tx_r485)//�������������͹������źţ������͸���λ��
{ 
  
   if(mybox.myid==tx_r485[2])//�ж��Ƿ��Ƿ�����������Ϣ
   	{
   	 mybox.source=tx_r485[1];
   	 mybox.send=tx_r485[3];
     mybox.relay=tx_r485[4];
     mybox.message=tx_r485[5];
 return 1;
   	}
   else return 0;
}


void modfiy_token_array(u8 i,u8 j)//ԭ����λΪ0��������λΪ1
{
u8 k;
for(k=1;k<33;k++)
if(token[k]==1)
{token[k]=0;
break;
} 
token[i]=j;

}


int subcontrol(u8 i,u8 j)//������λ����ָ��
{
 if(mybox.send==0)//��̬����
   	{

	return 0;
    }
   if(mybox.send==1) //��λ������
   	{ 
   	if(i==1&&j==1);
    if(i==1&&j==0);
    if(i==2&&j==1);
    if(i==2&&j==0);
	return 1;
   	}
if(mybox.send==2)//�޸�������¼����
 {
modfiy_token_array(mybox.relay,mybox.message);
return 2;
 }
return 3; //����ʧ��
}


/*****δ���******/
void turn_master_id()//�ı䵱ǰ����ϵͳ��������ID��
{
u8 i,j,flag=0;

for(i=1;i<33;i++)
if(token[i]==1)
	{ 
	  flag=i+cont;
      if(ID==(flag)){
        token[i]=0;
		token[ID]=1;
		cont=0;
	  for(j=1;j<33;j++)
     {  order_trans_rs485(mybox.start,ID,j,2,ID,1,mybox.end);
	   delay_us(10000);
	  }
	 //  LED1=!LED1;
	   mybox.master=1;
	    OSTaskResume(MASTER_TASK_PRIO );
      	}
   }
}
//ͨ�ö�ʱ��3�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3!
//��ʱ��3�жϷ������	

  void TIM3_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��
	
	//��ʱ��TIM3��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���


	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx					 
}
 
 void TIM3_IRQHandler(void)   //TIM3�ж�
{	 
	OSIntEnter();   
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
		{	  
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx�����жϱ�־
	if(mybox.master==0)	
		{
		
		if(dog_clock==0)
		   { 
		    LED0=!LED0;
			turn_master_id();
			  cont++;
			}
			if(dog_clock>0)dog_clock--;
		 }
		}
   	OSIntExit();  
}
//////////////////////////////////////////////////////////
	void USART2_IRQHandler(void)
{
 
	u8 RS485_RX_BUF[64];
	#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
	OSIntEnter();    
      #endif
 		
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //���յ�����
	{	 
	 			 
		 RS485_RX_BUF[RS485_RX_CNT++]=USART_ReceiveData(USART2); 	//��ȡ���յ�������
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='&'){RS485_RX_BUF[0]='&'; RS485_RX_CNT=1;}
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='*')
		{
				RS485_RX_CNT=0;
				OSMboxPost(RS485_MBOX,(void*)&RS485_RX_BUF);
		} 
	}  	
	#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
	OSIntExit();  											 
#endif

} 

void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	RS485_TX_EN=1;			//����Ϊ����ģʽ
  	for(t=0;t<len;t++)		//ѭ����������
	{		   
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	  
		USART_SendData(USART2,buf[t]);
	}	 
 
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);		
	RS485_RX_CNT=0;	  
	RS485_TX_EN=0;				//����Ϊ����ģʽ	

}


