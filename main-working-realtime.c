
// MIDI In A10


#define STM32F10X_MD

#include "stm32f10x_lib.h"





//s8 temp=0;

u8 timeout= 0;
u16 phase = 0;

u8 servoPhase[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//s8 offset[]={10,13,18,12,12,9,14,17,21,16,9,15,17,12,15,4,0,-8,-8,-2,3,9,16,10,24,18,10,9,15,7};
s8 offset[]={23,22,31,24,26,19,27,28,32,26,20,26,29,25,27,9,8,-1,-3,4,8,18,27,15,31,25,18,14,20,14};


//Long hold back
//const u8 timeline[]={100,100,100,100,100,100,160,160,160,160,125,125,125,125};

//'realtime', no sustain
const u8 timeline[]={173,173,173,173,173,113,113,113,113,113};



// to suppress warnings
#define servo_gpio(x) (GPIO_TypeDef *)(servo_gpio_port[x])

const GPIO_TypeDef * const servo_gpio_port[] = {
  GPIOB,
  GPIOB,
  GPIOB,
  GPIOB,
  GPIOA,
  GPIOA,
  GPIOA,
  GPIOA,
  GPIOA,
  GPIOA,
  GPIOA,
  GPIOA,
  GPIOC,
  GPIOC,
  GPIOB,
  GPIOB,
  GPIOB,
  GPIOB,
  GPIOB,
  GPIOB,
  GPIOB,
  GPIOA,
  GPIOA,
  GPIOA,
  GPIOA,
  GPIOA,
  GPIOB,
  GPIOB,
  GPIOB,
  GPIOB
};

const u16 servo_pin[]={
  GPIO_Pin_11,
  GPIO_Pin_10,
  GPIO_Pin_1,
  GPIO_Pin_0,
  GPIO_Pin_7,
  GPIO_Pin_6,
  GPIO_Pin_5,
  GPIO_Pin_4,
  GPIO_Pin_3,
  GPIO_Pin_2,
  GPIO_Pin_1,
  GPIO_Pin_0,
  GPIO_Pin_15,
  GPIO_Pin_14,
  GPIO_Pin_9,
  GPIO_Pin_8,
  GPIO_Pin_7,
  GPIO_Pin_6,
  GPIO_Pin_5,
  GPIO_Pin_4,
  GPIO_Pin_3,
  GPIO_Pin_15,
  GPIO_Pin_12,
  GPIO_Pin_11,
  GPIO_Pin_9,
  GPIO_Pin_8,
  GPIO_Pin_15,
  GPIO_Pin_14,
  GPIO_Pin_13,
  GPIO_Pin_12
};

u8 const noteToServo[]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,-1,1,-1,-1,-1,-1,2,-1,3,-1,4,5,-1,6,-1,7,8,9,10,11,12,13,14,29,28,27,26,25,24,23,22,21,20,19,18,17,-1,16,-1,15,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};

void USART1_IRQHandler(void)
{
  static u8 status=0;
  static u8 bytenumber =0;
  static u8 bytetwo =0;

  u8 i = USART_ReceiveData(USART1);

  if (i & 0x80) {
    status = i;
    bytenumber = 1;

  } else {
    if (bytenumber ==1) {
      bytetwo=i;
      bytenumber= 2;
    } else if (bytenumber==2){

      if ((status & 0xF0) == 0x90 && i!=0) {
        //noteOn
        //u8 note=bytetwo%30;
        u8 note=noteToServo[bytetwo];
        if (note!=255 && (!servoPhase[note] || servoPhase[note] >11)) {
          servoPhase[note]=1;
          GPIO_SetBits(GPIOC, GPIO_Pin_13);
          timeout=15;
        }
      }

      if (status == 0xAA) {
        offset[bytetwo%30]=(signed)i -64;
        servoPhase[bytetwo%30]=8;
      }

      bytenumber =1; //running status
    }
  }


}


void TIM3_IRQHandler(void){
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);

  if (phase == 0) {

    // Don't power up inactive servos
    for (u8 i=0; i<30; i++){
      if (servoPhase[i]!=0) {
        GPIO_SetBits(servo_gpio(i), servo_pin[i]);
      }
    }

  }
  for (u8 i=0;i<15;i++) {
    if (servoPhase[i] && phase == timeline[servoPhase[i]] + offset[i] ){
      GPIO_ResetBits(servo_gpio(i), servo_pin[i]);
    }
  }
  //Second half are mirrored
  for (u8 i=15;i<30;i++) {
    if (servoPhase[i] && phase == 250-timeline[servoPhase[i]] + offset[i] ){
      GPIO_ResetBits(servo_gpio(i), servo_pin[i]);
    }
  }

  phase++;
  if (phase>4000) {
    phase=0;
    for (u8 i=0;i<30;i++) {
      if (servoPhase[i]) servoPhase[i]++;
      if(servoPhase[i]==sizeof(timeline)*sizeof(u8)) servoPhase[i]=0;
    }

    if (--timeout==0) {
      GPIO_ResetBits(GPIOC, GPIO_Pin_13);
    }
  }
}

static void clock_init ( void ){
  RCC_HSEConfig(RCC_HSE_ON);
  RCC_WaitForHSEStartUp();
  RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9);
  RCC_PLLCmd(ENABLE);
 
  while(1) if((RCC->CR)&(1<<25)) break; //PLLRDY

  FLASH->ACR=0x2;

  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

  while(1) if(((RCC->CFGR)&0xF)==0xA) break;
}

int main()
{

  clock_init();


  // Disable JTAG/SW debug pins to enable the use of PA15, PB3, PB4
  // This means you have to reset the chip before programming via st-link
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
  

  GPIO_InitTypeDef GPIO_InitStruct;
  USART_InitTypeDef USART_InitStruct;
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;


  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_All;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;

  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_Init(GPIOC, &GPIO_InitStruct);


  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  // GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
  // GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  // GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  // GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_10;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStruct);


  USART_StructInit(&USART_InitStruct);

	USART_InitStruct.USART_BaudRate = 31250;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART1, &USART_InitStruct); 

  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);


  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 

  USART_Cmd(USART1, ENABLE);






  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_Period = 700;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);


  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_Cmd(TIM3, ENABLE);










  while (1){ }


}




