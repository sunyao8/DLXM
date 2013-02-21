
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

/////////////////////////UCOSII任务设置///////////////////////////////////
//START 任务
//设置任务优先级
#define START_TASK_PRIO      			10 //开始任务的优先级设置为最低
//设置任务堆栈大小
#define START_STK_SIZE  				64
//任务堆栈	
OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *pdata);	
 			   
//LED任务
//设置任务优先级
#define TAKE_TASK_PRIO       			7 
//设置任务堆栈大小
#define TAKE_STK_SIZE  		    		64
//任务堆栈
OS_STK TAKE_TASK_STK[TAKE_STK_SIZE];
//任务函数
void Heartbeat_task(void *pdata);

//接收任务
//设置任务优先级
#define Receive_TASK_PRIO       			8 
//设置任务堆栈大小
#define Receive_STK_SIZE  		    		64
//任务堆栈
OS_STK Receive_TASK_STK[Receive_STK_SIZE];
//任务函数
void  Receive_task(void *pdata);





//主任务
//设置任务优先级
#define MAIN_TASK_PRIO       			4 
//设置任务堆栈大小
#define MAIN_STK_SIZE  					128
//任务堆栈	
OS_STK MAIN_TASK_STK[MAIN_STK_SIZE];
//任务函数
void main_task(void *pdata);

//信号量集任务
//设置任务优先级
#define MASTER_TASK_PRIO       			3 
//设置任务堆栈大小
#define MASTER_STK_SIZE  		 		64
//任务堆栈	
OS_STK MASTER_TASK_STK[MASTER_STK_SIZE];
//任务函数
 void master_task(void *pdata);
 
OS_EVENT *Heartbeat;			 //心跳信号量
OS_EVENT * RS485_MBOX;			//	rs485邮箱信号量

//按键扫描任务
//设置任务优先级
#define KEY_TASK_PRIO       			2 
//设置任务堆栈大小
#define KEY_STK_SIZE  					64
//任务堆栈	
OS_STK KEY_TASK_STK[KEY_STK_SIZE];
//任务函数
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
  u8 myid;      //本电容箱ID号
  u8 source;
  u8 destination; //目的电容箱
  u8 send;      //是否是发送命令1为是，0为不是
  u8 relay;    //第几组电容器
  u8 message;     //开关信息
  u8 master;      //主机令牌
  u8 end;   
}box;

box mybox;

u8 token[33];//主机号令牌

u8 cont=0;//用于更改主机号的记次数器

u16  dog_clock=10;

u8 rs485buf[LEN],lon=LEN;

//接收缓存区 	
u8 RS485_RX_BUF[64];  	//接收缓冲,最大64个字节.
//接收到的数据长度
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
  
	 
	
	delay_init();	    	 //延时函数初始化	  
	NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
 	LED_Init();			     //LED端口初始化

	RS485_Init(9600);	//初始化RS485
	TIM3_Int_Init(4999,7199);//10Khz的计数频率，计数5K次为500ms  
	 initmybox();
	OSInit();  	 			//初始化UCOSII
			  
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
	OSStart();	    
}							    
//开始任务
void start_task(void *pdata)
{
    OS_CPU_SR cpu_sr=0;  	    
	pdata = pdata; 	
	 Heartbeat=OSSemCreate(0);
	 RS485_MBOX=OSMboxCreate((void*)0);
	OSStatInit();					//初始化统计任务.这里会延时1秒钟左右	
 	OS_ENTER_CRITICAL();			//进入临界区(无法被中断打断)    			   
 	OSTaskCreate(master_task,(void *)0,(OS_STK*)&MASTER_TASK_STK[MASTER_STK_SIZE-1],MASTER_TASK_PRIO);	 				   
 	OSTaskCreate(Heartbeat_task,(void *)0,(OS_STK*)&TAKE_TASK_STK[TAKE_STK_SIZE-1],TAKE_TASK_PRIO);
	OSTaskCreate(Receive_task,(void *)0,(OS_STK*)&Receive_TASK_STK[Receive_STK_SIZE-1],Receive_TASK_PRIO);
 	OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务.
	OS_EXIT_CRITICAL();				//退出临界区(可以被中断打断)
}
//LED任务
void Heartbeat_task(void *pdata)//master任务发送任务
{		// u8 key;
        u8 i=0;
		u8 err;
	//  u8 rs485buf[5];
	while(1)
	{
	OSSemPend(Heartbeat,0,&err);
		//key=KEY_Scan(0);
		//if(key==KEY_RIGHT)//KEY0按下,发送一次数据
		for(i=1;i<33;i++)
		{	
	       order_trans_rs485(mybox.start,mybox.myid,i,0,0,0,mybox.end);//发送5个字节
		    delay_us(10000);
		  // LCD_ShowxNum(60+i*32,190,0,7,16,0X80);
		}		 
				   
	}
}

void Receive_task(void *pdate)//接收任务
{   u8 err;
	 u8 *msg;
	 int flag1,flag2;
	
    while(1)
    	{
		 if(mybox.master==1)OSTaskSuspend(Receive_TASK_PRIO);
	 msg=(u8 *)OSMboxPend(RS485_MBOX,0,&err);//接收到有数据
	 flag1=rs485_trans_order(msg);
	 	 
	 if(flag1==0);/***主机不是给本机通信，信息舍弃***/
	 if(flag1==1)/***是本机信息***/
	 	{		LED1=!LED1;	  
		       dog_clock=10;	 //喂狗
	 	      flag2=subcontrol(mybox.relay,mybox.message);
		       if(flag2==0);/******心跳信息，主机活着，不必启动从新定位主机函数，返回给主机本从机也存在    喂狗*********/
               if(flag2==1);/*****下位机，控制命令，喂狗*****/		  
	 	}

	/* 时间到达函数,判断是否主机已经死亡*/

	
	/*	if(key)//接收到有数据
		{
			if(key>5)key=5;//最大是5个数据.
 			for(i=0;i<key;i++)LCD_ShowxNum(60+i*32,230,rs485buf[i],3,16,0X80);	//显示数据
 		}
		t++; 
		delay_ms(10);
		if(t==20)
		{
			LED0=!LED0;//提示系统正在运行	
			t=0;
			cnt++;
			LCD_ShowxNum(60+48,150,cnt,3,16,0X80);	//显示数据
		}
    	}*/
	}
	}
 /**************主机任务**********************/
  void master_task(void *pdata)
  {	  OS_CPU_SR cpu_sr=0;
   while(1){
  	if(mybox.master==1)
	{
	 OSSemPost(Heartbeat);
			delay_ms(100);
			LED0=!LED0;
                                              //启动接收程序
	}
	if(mybox.master==0)
	{
		OS_ENTER_CRITICAL();
		 OSTaskResume(Receive_TASK_PRIO );//启动从机任务状态
		OSTaskSuspend(MASTER_TASK_PRIO );//挂起主机任状态.
		OS_EXIT_CRITICAL();	
	}
	}//while
  }



 /***********************************/


void initmybox()//初始化自身信息
{  	 
  u8 i;
  
  for(i=1;i<33;i++)token[i]=0;//初始化令牌
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


 void order_trans_rs485(u8 start,u8 source,u8 destination, u8 send,u8 relay,u8 message,u8 end)//主机程序，主机命令解析成RS485信息，发送给目的从机
{  	 OS_CPU_SR cpu_sr=0;
    OS_ENTER_CRITICAL();
    rs485buf[0]=start;
	rs485buf[1]=source;
	rs485buf[2]=destination;
	rs485buf[3]=send;
	rs485buf[4]=relay;
	rs485buf[5]=message;
	rs485buf[6]=end;
	RS485_Send_Data(rs485buf,7);//发送5个字节
	OS_EXIT_CRITICAL();	
}

int rs485_trans_order(u8 *tx_r485)//解析由主机发送过来的信号，并发送给下位机
{ 
  
   if(mybox.myid==tx_r485[2])//判断是否是发给本机的信息
   	{
   	 mybox.source=tx_r485[1];
   	 mybox.send=tx_r485[3];
     mybox.relay=tx_r485[4];
     mybox.message=tx_r485[5];
 return 1;
   	}
   else return 0;
}


void modfiy_token_array(u8 i,u8 j)//原主机位为0，新主机位为1
{
u8 k;
for(k=1;k<33;k++)
if(token[k]==1)
{token[k]=0;
break;
} 
token[i]=j;

}


int subcontrol(u8 i,u8 j)//给下下位机放指令
{
 if(mybox.send==0)//心态脉搏
   	{

	return 0;
    }
   if(mybox.send==1) //下位机控制
   	{ 
   	if(i==1&&j==1);
    if(i==1&&j==0);
    if(i==2&&j==1);
    if(i==2&&j==0);
	return 1;
   	}
if(mybox.send==2)//修改主机记录数组
 {
modfiy_token_array(mybox.relay,mybox.message);
return 2;
 }
return 3; //操作失败
}


/*****未完成******/
void turn_master_id()//改变当前整个系统中主机的ID号
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
//通用定时器3中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!
//定时器3中断服务程序	

  void TIM3_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能
	
	//定时器TIM3初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


	TIM_Cmd(TIM3, ENABLE);  //使能TIMx					 
}
 
 void TIM3_IRQHandler(void)   //TIM3中断
{	 
	OSIntEnter();   
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
		{	  
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx更新中断标志
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
	#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntEnter();    
      #endif
 		
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //接收到数据
	{	 
	 			 
		 RS485_RX_BUF[RS485_RX_CNT++]=USART_ReceiveData(USART2); 	//读取接收到的数据
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='&'){RS485_RX_BUF[0]='&'; RS485_RX_CNT=1;}
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='*')
		{
				RS485_RX_CNT=0;
				OSMboxPost(RS485_MBOX,(void*)&RS485_RX_BUF);
		} 
	}  	
	#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntExit();  											 
#endif

} 

void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	RS485_TX_EN=1;			//设置为发送模式
  	for(t=0;t<len;t++)		//循环发送数据
	{		   
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	  
		USART_SendData(USART2,buf[t]);
	}	 
 
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);		
	RS485_RX_CNT=0;	  
	RS485_TX_EN=0;				//设置为接收模式	

}


