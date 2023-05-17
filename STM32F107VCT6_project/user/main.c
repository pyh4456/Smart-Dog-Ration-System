#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_exti.h"
#include "lcd.h"
#include "touch.h"
#include <stdbool.h>
#include <string.h>

#define BUF_SIZE 30

bool phoneFlag = false;
volatile uint32_t value[] = {0, 0};  // value[0] : water level(PC0), value[1] : light(PC1), value[2] :  light
int color[12] = {WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY};
int32_t offset = 0;
int offset_flag = 0;
int w_flag = 0, f_flag = 0, l_flag = 0, phone_flag = 0;
int32_t val=0;
int32_t val2=0;
uint64_t load_cell;


void GPIO_Configure(void);
void EXTI_Configure(void);
void USART1_Init(void);
void USART2_Init(void);
void RCC_Configure(void);
void ADC_Configure(void);
void DMA_Configure(void);
void NVIC_Configure(void);

void EXTI15_10_IRQHandler(void);

void Motor_Init(void);
void openDoor(void);
void closeDoor(void);

void sendPhone(char*);
void Delay(uint32_t);
void TIM2_us_Delay(uint32_t delay);

void sendDataUSART1(uint16_t data);
void sendDataUSART2(uint16_t data);

uint64_t Sensor_Read(void);
uint32_t get_sensor_average(uint8_t num);
uint32_t get_sensor_Tare(uint8_t num);

void RCC_Configure(void)
{
   // Enable clock
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
}

// configuration pin setting 
void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // water level sensor-PC1, light sensor-PC2 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // load cell data- PC3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // load cell clock
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // user button1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;// INPUT PULL-DOWN
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    // LED
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //OUTPUT PUSH-PULL
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    // PB0 : servo motor1, PB7 : servo motor2
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_7 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //USART1 TX  --> pin 09(USART1)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // USART1 TX - alternate function
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //USART1 RX --> pin 10(USART1)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // USART1 RX - PULL-DOWN
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //Tx : PA2 - send signal with bluetooth module ( FB755AC )
    //Rx : PA3 // receive signal with blutooth modul ( FB755AC )
    //USART2 TX --> pin 2 (USART2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // USART2 TX - alternate function
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //USART2 RX --> pint 3 (USART2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // USART2 RX - PULL-DOWN
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// External Interrupt settings
void EXTI_Configure(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;

    // USER button - PD11
    // interrupt detect by falling trigger
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource11);
    EXTI_InitStructure.EXTI_Line = EXTI_Line11;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}


// NVIC - prioritizing interrupts
void NVIC_Configure(void) {

    NVIC_InitTypeDef NVIC_InitStructure;
    
    // Configure the NVIC Preemption Priority Bits
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // User S1 Button
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    //PC <-> board (USART1)
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  
    //Phone <-> board (USART2)
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


/*
  Set the ADC mode to be connected to the light sensor, whether to use an external trigger, and the cycle rate.

  Since one illuminance sensor uses ADC12_IN10, it is set to 10 channels..

  Since we uses DMA, uses the ADC_DMACmd() function.
  After setting Calibration to Reset, the operation is set to start Calibration.
*/
void ADC_Configure(void){
  
  ADC_InitTypeDef ADC_Config;
  
  ADC_DeInit(ADC1);
  
  ADC_Config.ADC_Mode = ADC_Mode_Independent;
  ADC_Config.ADC_ScanConvMode = ENABLE;
  ADC_Config.ADC_ContinuousConvMode = ENABLE;
  ADC_Config.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_Config.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_Config.ADC_NbrOfChannel = 2;
  
  ADC_Init(ADC1, &ADC_Config);
  
  // ADC1 regualr Channel Configuration
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11,1, ADC_SampleTime_239Cycles5);  //water level
  
  // ADC1 regualr Channel Configuration
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12,2, ADC_SampleTime_239Cycles5);  //light value
  ADC_Cmd(ADC1, ENABLE);
  
  // Enable ADC1 DMA
  ADC_DMACmd(ADC1,ENABLE);
  
  // Enable ADC1 reset calibration register
  ADC_ResetCalibration(ADC1);
  
  // Check the end of ADC1 reset calibration register
  while (ADC_GetResetCalibrationStatus(ADC1));
  
  // Start ADC1 calibration
  ADC_StartCalibration(ADC1);
  // Check the end of ADC1 calibration
  while (ADC_GetCalibrationStatus(ADC1));
  
  // Start ADC1 Software Conversion
  ADC_SoftwareStartConvCmd(ADC1,ENABLE);
}

void DMA_Configure(void){
  DMA_InitTypeDef DMA_InitStructure;
  DMA_DeInit(DMA1_Channel1);
  
  // amount of memory to store in the variable. set the value to 2 to use 2 values.
  DMA_InitStructure.DMA_BufferSize = 2;
  
  // memory base address of the variable
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)value;
  
  // The amount of data to be stored in the variable
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  
  // Use incremented memory address register
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  
  // Circular mode is used for circular buffers or continuous data processing.
  // The number of data to be transmitted is automatically reloaded in the channel setting stage, 
  // and DMA requests are continuously generated.
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  
  // Determines the memory address from which register to receive information
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
  
  // Set the bit unit of the sent peripheral.
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  
  // Determines the priority of the DMA channel.
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  // Enable DMA1 Channel1
  DMA_Cmd(DMA1_Channel1,ENABLE);
}

void USART1_Init(void)  //connect to putty
{
    USART_InitTypeDef USART1_InitStructure;

   // Enable the USART1 peripheral
   USART_Cmd(USART1, ENABLE);
   
    // TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
    USART1_InitStructure.USART_BaudRate = 9600;
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART1_InitStructure.USART_Parity = USART_Parity_No;
    USART1_InitStructure.USART_StopBits = USART_StopBits_1;
    USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART1_InitStructure);    
   
    // TODO: Enable the USART1 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 
}

void USART2_Init(void)  //connect to bluetooth module
{
   USART_InitTypeDef USART2_InitStructure;

   // Initialize the USART2 using the structure 'USART_InitTypeDef' and the function 'USART_Init'
   USART2_InitStructure.USART_BaudRate = 9600;
   USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   USART2_InitStructure.USART_Parity = USART_Parity_No;
   USART2_InitStructure.USART_StopBits = USART_StopBits_1;
   USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_Init(USART2, &USART2_InitStructure);

   // Enable the USART2 peripheral
   USART_Cmd(USART2, ENABLE);
   // Enable the USART2 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
   USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // USART RX   Interrupt     
}

void Motor_Init() {
    
  //TIM40 config
   TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
   uint16_t prescale= (uint16_t) (SystemCoreClock/ 1000000)-1;
    
    TIM_TimeBaseStructure.TIM_Period= 20000-1;
    TIM_TimeBaseStructure.TIM_Prescaler= prescale;
    TIM_TimeBaseStructure.TIM_ClockDivision= 0;
    TIM_TimeBaseStructure.TIM_CounterMode= TIM_CounterMode_Down;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    // declare the variable about OCinit
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode= TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity= TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState= TIM_OutputState_Enable;
    
    TIM_OCInitStructure.TIM_Pulse= 1500; // us 1.5ms
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);  
  
  
    //TIM3 config
    prescale= (uint16_t) (SystemCoreClock/ 1000000)-1;
    
    TIM_TimeBaseStructure.TIM_Period= 20000-1;
    TIM_TimeBaseStructure.TIM_Prescaler= prescale;
    TIM_TimeBaseStructure.TIM_ClockDivision= 0;
    TIM_TimeBaseStructure.TIM_CounterMode= TIM_CounterMode_Down;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    // declare the variable about OCinit
    TIM_OCInitStructure.TIM_OCMode= TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity= TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState= TIM_OutputState_Enable;
    
    TIM_OCInitStructure.TIM_Pulse= 1500; // us 1.5ms
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    
} // Right Side : Port B9, Left Side : Port B7

// TIM3 : PB0, TIM4 : PB7
void change_pwm_duty_cycle(int percentx10, TIM_TypeDef* tim){
  int pwm_pulse;
  pwm_pulse = percentx10 * 20000 / 100;
 
  // declare the variable about OCinit
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode= TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity= TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState= TIM_OutputState_Enable;
  
  TIM_OCInitStructure.TIM_Pulse = pwm_pulse;
  
  if(tim == TIM3){
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  }
  if(tim == TIM4){
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  }
  
}

//PC <-> board
void USART1_IRQHandler() {
   uint16_t word1;
    while(USART_GetITStatus(USART1,USART_IT_RXNE) ==RESET);
       // the most recent received data by the USART1 peripheral
        word1= USART_ReceiveData(USART1);
        sendDataUSART1(word1);
        sendDataUSART2(word1);
        // clear 'Read data register not empty' flag
       USART_ClearITPendingBit(USART1,USART_IT_RXNE);

}

//board <-> bluetooth
void USART2_IRQHandler() {
   uint16_t word;
    while(USART_GetITStatus(USART2,USART_IT_RXNE)==RESET);
        phone_flag = 1;
       // the most recent received data by the USART1 peripheral
        word = USART_ReceiveData(USART2);
        //while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        sendDataUSART1(word);
        if(word == 'w' || word == 'W') {        //
          w_flag = 1;
        }
        else if(word == 'f' || word == 'F') {   //
          f_flag = 1;
        }
        else if(word == 'l' || word == 'L') {   //
          l_flag = 1;
        }
        else if(word == 'e' || word == 'E'){
          w_flag = 0, f_flag = 0, l_flag = 0;
        }
        // clear 'Read data register not empty' flag
       USART_ClearITPendingBit(USART2,USART_IT_RXNE);
}

void EXTI15_10_IRQHandler(void) { // when the button is pressed
   if (EXTI_GetITStatus(EXTI_Line11) != RESET){

     if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11) == Bit_RESET) {
       //send amount of water
       if ((uint32_t)value[0] < 100) {      
         sendPhone("Lack of water.\r\n");
       }
       else {     //water level low
         sendPhone("Enough water.\r\n");
       }

       //send amount of light
       if ((uint32_t)value[1] > 2000) {        
         sendPhone("the surroundings are dark.\r\n");
       }        
       else {    //water level middle
         sendPhone("the surroundings are bright.\r\n");
       }
       
       //send amount of food
       if (load_cell > 33150) {       
         sendPhone("Enough food.\r\n");
       }        
       else {    //water level middle
         sendPhone("Lack of food.\r\n");
       }
     }
   }
   EXTI_ClearITPendingBit(EXTI_Line11);
}

void sendPhone(char* buf) {     //send messege to bluetooth module
  char *info = buf;
  while (*info) {
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) ;
    USART_SendData(USART2, *info++);
  }
}

void Delay(uint32_t num) {
   int i =0;
   for (i = 0; i < num; i++) {}
}

void sendDataUSART1(uint16_t data) {    //send data to USART1 (putty)
   USART_SendData(USART1, data);
}

void sendDataUSART2(uint16_t data) {    //send data to USART2 (bluetooth)
   USART_SendData(USART2, data);
}

uint64_t Sensor_Read(void){ 
 uint64_t Count; 
 unsigned char i; 
 
 GPIO_SetBits(GPIOC,GPIO_Pin_3);
 GPIO_ResetBits(GPIOC,GPIO_Pin_8);
 Count=0; 
 TIM2_us_Delay(30);
 while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3)); 
 for (i=0;i<24;i++){ 
  GPIO_SetBits(GPIOC,GPIO_Pin_8);
  TIM2_us_Delay(30);
  Count=Count<<1; 
  GPIO_ResetBits(GPIOC,GPIO_Pin_8);
  if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3))Count++; 
  TIM2_us_Delay(30);
  
 } 
 GPIO_SetBits(GPIOC,GPIO_Pin_8);
 TIM2_us_Delay(30);
 Count=Count^0x800000;
 Count=Count/250;
 GPIO_ResetBits(GPIOC,GPIO_Pin_8); 
 TIM2_us_Delay(30);
 return Count;
}

void TIM2_us_Delay(uint32_t delay){	//function makes delay in microseconds
    RCC->APB1ENR |=1; //Start the clock for the timer peripheral
    TIM2->ARR = (int)(delay/0.0625); // Total period of the timer
    TIM2->CNT = 0;
    TIM2->CR1 |= 1; //Start the Timer
    while(!(TIM2->SR & TIM_SR_UIF)){} //Polling the update interrupt flag
    TIM2->SR &= ~(0x0001); //Reset the update interrupt flag
}


int main() {
  SystemInit();
  RCC_Configure();
  GPIO_Configure();
  ADC_Configure();
  DMA_Configure();
  EXTI_Configure();
  NVIC_Configure();
  USART1_Init();
  USART2_Init();
  Motor_Init();
  LCD_Init();
  Touch_Configuration();
  LCD_Clear(WHITE);

  LCD_ShowString(100,20, "THU_TEAM8", BLACK, WHITE);
    
  change_pwm_duty_cycle(12, TIM4);
  change_pwm_duty_cycle(5, TIM3);
  
  while(1){ 
    //make up the screen
    load_cell = Sensor_Read();
    
    
    LCD_ShowString(100,40, "Water Level", BLACK, WHITE);
    LCD_ShowNum(100,60,(uint32_t)value[0],4,BLACK,WHITE);
    LCD_ShowString(100,80, "Light", BLACK, WHITE);
    LCD_ShowNum(100,100,(uint32_t)value[1],4,BLACK,WHITE);
    LCD_ShowString(100,200, "Load cell", BLACK, WHITE);
    LCD_ShowNum(100,220, load_cell - 3000,4,BLACK,WHITE);

    //Turn on the light when it's dark 
    if((uint32_t)value[1] > 2000 || l_flag == 1){
      GPIO_SetBits(GPIOD,GPIO_Pin_2);
      GPIO_SetBits(GPIOD,GPIO_Pin_3);
      GPIO_SetBits(GPIOD,GPIO_Pin_4);
      GPIO_SetBits(GPIOD,GPIO_Pin_7);
    }else{
      GPIO_ResetBits(GPIOD,GPIO_Pin_2);
      GPIO_ResetBits(GPIOD,GPIO_Pin_3);
      GPIO_ResetBits(GPIOD,GPIO_Pin_4);
      GPIO_ResetBits(GPIOD,GPIO_Pin_7);
    }
    
    //Open water door when water level is low
    if((uint32_t)value[0] < 100 || w_flag == 1){
      change_pwm_duty_cycle(7, TIM4);
    }else{
      change_pwm_duty_cycle(12, TIM4);
    }
    
    //Open food door when load cell value is low
    if(load_cell < 33150  || f_flag == 1){
      change_pwm_duty_cycle(8, TIM3);
    }else{
      change_pwm_duty_cycle(5, TIM3);
    } 
    
    // w_flag = 0, f_flag = 0;
    
    Delay(200000);
    
  }
}
