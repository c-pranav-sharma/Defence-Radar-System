#include "stm32f4xx.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h> 

extern uint32_t SystemCoreClock; 

// --- Constants (No changes here) ---
#define SSD1306_I2C_ADDR      (0x3C << 1)
#define SSD1306_HEIGHT        64
#define SSD1306_WIDTH         128
#define SSD1306_BUFFER_SIZE   (SSD1306_WIDTH * SSD1306_HEIGHT / 8)
#define RADAR_CENTER_X        64
#define RADAR_CENTER_Y        63
#define RADAR_MAX_RADIUS      60
#define MAX_BLIPS             20
#define BLIP_LIFETIME         50

// Standard F4 Clock: APB1 Timer Clock is 84MHz when SystemCoreClock is 168MHz.
// Assuming F_TIM_PCLK1_x = 84MHz
#define F_TIM_CLK (SystemCoreClock / 2) * 2 

// --- Type Definitions / Global Variables / Forward Declarations ---
typedef enum { Black = 0x00, White = 0x01 } SSD1306_COLOR;
typedef struct { int x, y, lifetime; } Blip;
volatile uint32_t msTicks = 0;
static uint8_t SSD1306_Buffer[SSD1306_BUFFER_SIZE];
static Blip blips[MAX_BLIPS];
static int blip_index = 0;
volatile bool g_update_flag = false;
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR c);
void ssd1306_Line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, SSD1306_COLOR c);
float getUltrasonicDistance(void);
void TIM2_PWM_Init(void);


// --- Core MCU & Peripheral Functions (No relevant logic changes) ---

void SysTick_Handler(void) { msTicks++; }
void ssd1306_Delay(uint32_t ms){ uint32_t start=msTicks; while((msTicks-start)<ms); }

void DWT_Delay_Init(void){ CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk; DWT->CYCCNT=0; DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk; }
static inline uint32_t micros(void){ return (uint32_t)(DWT->CYCCNT/(SystemCoreClock/1000000U)); }
void delay_us(uint32_t us){ uint32_t start=micros(); while((micros()-start)<us); }

void I2C1_Init(void){
    RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN;RCC->APB1ENR|=RCC_APB1ENR_I2C1EN;
    GPIOB->MODER&=~((3U<<(6*2))|(3U<<(7*2)));GPIOB->MODER|=((2U<<(6*2))|(2U<<(7*2)));
    GPIOB->OTYPER|=((1U<<6)|(1U<<7));
    GPIOB->PUPDR&=~((3U<<(6*2))|(3U<<(7*2)));GPIOB->PUPDR|=((1U<<(6*2))|(1U<<(7*2)));
    GPIOB->AFR[0]&=~((0xF<<(6*4))|(0xF<<(7*4)));GPIOB->AFR[0]|=((4U<<(6*4))|(4U<<(7*4)));
    I2C1->CR1|=I2C_CR1_SWRST; I2C1->CR1&=~I2C_CR1_SWRST;
    uint32_t pclk1_mhz=SystemCoreClock/1000000U;
    I2C1->CR2=pclk1_mhz;
    I2C1->CCR=420; // Explicitly set to 420 for 100kHz on 84MHz PCLK1
    I2C1->TRISE=pclk1_mhz+1;
    I2C1->CR1|=I2C_CR1_PE;
}

void I2C1_Write(uint8_t addr,uint8_t* data,size_t n){
    while(I2C1->SR2&I2C_SR2_BUSY);I2C1->CR1|=I2C_CR1_START;while(!(I2C1->SR1&I2C_SR1_SB));
    I2C1->DR=addr;
    while(!(I2C1->SR1&I2C_SR1_ADDR)){if(I2C1->SR1&I2C_SR1_AF){I2C1->CR1|=I2C_CR1_STOP;I2C1->SR1&=~I2C_SR1_AF;return;}}(void)I2C1->SR2;
    for(size_t i=0;i<n;++i){ while(!(I2C1->SR1&I2C_SR1_TXE)); I2C1->DR=data[i]; }
    while(!(I2C1->SR1&I2C_SR1_TXE)); while(!(I2C1->SR1&I2C_SR1_BTF));
    I2C1->CR1|=I2C_CR1_STOP;
}
void ssd1306_WriteCommand(uint8_t b){ uint8_t buf[2]={0x00,b}; I2C1_Write(SSD1306_I2C_ADDR,buf,2); }
void ssd1306_Init(void){ 
    ssd1306_Delay(100);
    uint8_t cmds[] = {0xAE,0x20,0x00,0xC8,0x40,0x81,0xFF,0xA1,0xA6,0xA8,0x3F,0xD3,0x00,0xD5,0xF0,0xD9,0x22,0xDA,0x12,0xDB,0x20,0x8D,0x14,0xAF};
    for(int i=0;i<sizeof(cmds);i++) ssd1306_WriteCommand(cmds[i]);
}
void ssd1306_UpdateScreen(void){ 
    uint8_t d[129]; d[0]=0x40;
    for(uint8_t page=0;page<8;++page){
        ssd1306_WriteCommand(0xB0+page);ssd1306_WriteCommand(0x00);ssd1306_WriteCommand(0x10);
        memcpy(d+1,&SSD1306_Buffer[128*page],128);
        I2C1_Write(SSD1306_I2C_ADDR,d,129);
    }
}
void ssd1306_DrawPixel(uint8_t x,uint8_t y,SSD1306_COLOR c){ 
    if(x>=SSD1306_WIDTH||y>=SSD1306_HEIGHT)return;
    if(c==White){ SSD1306_Buffer[x+(y/8)*SSD1306_WIDTH]|=(1<<(y%8)); }
    else { SSD1306_Buffer[x+(y/8)*SSD1306_WIDTH]&=~(1<<(y%8)); }
}

float getUltrasonicDistance(void){ 
    GPIOB->BSRR=(1U<<0); delay_us(10); GPIOB->BSRR=(1U<<(0+16));
    uint32_t start_time=micros();while(!(GPIOB->IDR&(1U<<1))){ if((micros()-start_time)>30000)return-1.0f; }
    uint32_t echo_start=micros();while(GPIOB->IDR&(1U<<1)){ if((micros()-echo_start)>30000)return-1.0f; }
    uint32_t echo_end=micros();
    float distance_cm=((float)(echo_end-echo_start)*0.0343f)/2.0f;
    return distance_cm;
}

// FIX: Set PSC to 83 to guarantee 1us tick with 84MHz Timer Clock
void TIM2_PWM_Init(void){ 
    RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;
    
    // PA5 GPIO setup for AF1
    GPIOA->MODER&=~(3U<<(5*2));
    GPIOA->MODER|=(2U<<(5*2)); // Alternate Function mode
    GPIOA->AFR[0]&=~(0xF<<(5*4));
    GPIOA->AFR[0]|=(1U<<(5*4)); // AF1 is TIM2_CH1
    
    // Timer Clock = 84MHz. PSC = (84MHz / 1MHz) - 1 = 83.
    TIM2->PSC=(F_TIM_CLK/1000000U)-1; // 83
    
    // ARR = (1MHz / 50Hz) - 1 = 19999. (20ms period)
    TIM2->ARR=20000-1;
    
    TIM2->CCR1=1500; // Initial duty cycle (1.5ms pulse width)
    
    TIM2->CCMR1|=(6<<4); // PWM mode 1
    TIM2->CCER|=TIM_CCER_CC1E; // Enable output on CH1
    TIM2->CR1|=TIM_CR1_CEN; // Enable timer
    TIM2->EGR|=TIM_EGR_UG; // Update registers
}

void TIM4_Init_Interrupt(void){ 
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = 840 - 1; 
    TIM4->ARR = 1600 - 1;
    TIM4->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM4_IRQn, 2);
    NVIC_EnableIRQ(TIM4_IRQn);
    TIM4->CR1 |= TIM_CR1_CEN;
}

// --- Graphics & Logic Functions (No changes needed) ---

void TIM4_IRQHandler(void){ 
    if (TIM4->SR & TIM_SR_UIF) {
        TIM4->SR &= ~TIM_SR_UIF;
        g_update_flag = true;
    }
}
void ssd1306_Line(int16_t x0,int16_t y0,int16_t x1,int16_t y1,SSD1306_COLOR c){ 
    int16_t dx=abs(x1-x0), sx=x0<x1?1:-1;
    int16_t dy=-abs(y1-y0), sy=y0<y1?1:-1;
    int16_t err=dx+dy, e2;
    while(1){
        ssd1306_DrawPixel(x0,y0,c);
        if(x0==x1&&y0==y1)break;
        e2=2*err;if(e2>=dy){err+=dy; x0+=sx;}if(e2<=dx){err+=dx; y0+=sy;}
    }
}
void drawLargeDigit(int x,int y,int digit){ 
    const uint16_t large_font[10][12] = {{0x0FF0,0x1008,0x1008,0x1008,0x1008,0x1008,0x1008,0x1008,0x1008,0x1008,0x1008,0x0FF0}, {0x0180,0x0380,0x0180,0x0180,0x0180,0x0180,0x0180,0x0180,0x0180,0x0180,0x0180,0x07E0}, {0x0FF0,0x1008,0x0008,0x0008,0x0010,0x0020,0x0040,0x0080,0x0100,0x0200,0x0400,0x1FFC}, {0x0FF0,0x1008,0x0008,0x0008,0x0008,0x07F0,0x0008,0x0008,0x0008,0x0008,0x1008,0x0FF0}, {0x000C,0x001C,0x002C,0x004C,0x008C,0x010C,0x020C,0x040C,0x1FFC,0x000C,0x000C,0x000C}, {0x1FFC,0x1000,0x1000,0x1000,0x1000,0x0FF0,0x0008,0x0008,0x0008,0x0008,0x1008,0x0FF0}, {0x0FF0,0x1008,0x1000,0x1000,0x1000,0x0FF0,0x1008,0x1008,0x1008,0x1008,0x1008,0x0FF0}, {0x1FFC,0x0004,0x0004,0x0008,0x0010,0x0020,0x0040,0x0080,0x0100,0x0100,0x0100,0x0100}, {0x0FF0,0x1008,0x1008,0x1008,0x1008,0x0FF0,0x1008,0x1008,0x1008,0x1008,0x1008,0x0FF0}, {0x0FF0,0x1008,0x1008,0x1008,0x1008,0x0FF8,0x0008,0x0008,0x0008,0x1008,0x1008,0x0FF0}};
    if(digit<0||digit>9)return;for(int i=0;i<12;i++)for(int j=0;j<16;j++)if((large_font[digit][i]>>j)&1)ssd1306_DrawPixel(x+i,y+j,White);
}
void drawDistance(float dist){ 
    char buf[4];if(dist<0)return;sprintf(buf,"%3.0f",dist);
    for(int i=0;i<3;i++)if(buf[i]>='0'&&buf[i]<='9')drawLargeDigit(2+i*14,2,buf[i]-'0');
}
void drawSolidArc(int xc,int yc,int r){ 
    int x=-r,y=0,err=2-2*r;do{if(yc+y<=yc)ssd1306_DrawPixel(xc-x,yc+y,White);if(yc-x<=yc)ssd1306_DrawPixel(xc-y,yc-x,White);if(yc-y<=yc)ssd1306_DrawPixel(xc+x,yc-y,White);if(yc+x<=yc)ssd1306_DrawPixel(xc+y,yc+x,White);int e2=err;if(e2<=y)err+=++y*2+1;if(e2>x||err>y)err+=++x*2+1;}while(x<0);
}
void drawRadarBackground(void){ 
    drawSolidArc(RADAR_CENTER_X,RADAR_CENTER_Y,60);drawSolidArc(RADAR_CENTER_X,RADAR_CENTER_Y,40);drawSolidArc(RADAR_CENTER_X,RADAR_CENTER_Y,20);
    const float PI = 3.14159f;float a45=PI/4,a90=PI/2,a135=3.0f*PI/4;
    int x45=RADAR_CENTER_X-(int)(RADAR_MAX_RADIUS*cosf(a45));int y45=RADAR_CENTER_Y-(int)(RADAR_MAX_RADIUS*sinf(a45));ssd1306_Line(RADAR_CENTER_X,RADAR_CENTER_Y,x45,y45,White);
    int x90=RADAR_CENTER_X, y90=RADAR_CENTER_Y-RADAR_MAX_RADIUS;ssd1306_Line(RADAR_CENTER_X,RADAR_CENTER_Y,x90,y90,White);
    int x135=RADAR_CENTER_X-(int)(RADAR_MAX_RADIUS*cosf(a135));int y135=RADAR_CENTER_Y-(int)(RADAR_MAX_RADIUS*sinf(a135));ssd1306_Line(RADAR_CENTER_X,RADAR_CENTER_Y,x135,y135,White);
}
void addBlip(int x,int y){ 
    blips[blip_index].x=x;blips[blip_index].y=y;blips[blip_index].lifetime=BLIP_LIFETIME;blip_index=(blip_index+1)%MAX_BLIPS;
}
void updateAndDrawBlips(void){ 
    for(int i=0;i<MAX_BLIPS;i++)if(blips[i].lifetime>0){ssd1306_DrawPixel(blips[i].x,blips[i].y,White);blips[i].lifetime--;}
}

int main(void){
    SysTick_Config(SystemCoreClock/1000); 

    // Initialize clocks and GPIO for peripherals
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    // PB0 (TRIG) setup
    GPIOB->MODER &= ~(3U << (0*2));
    GPIOB->MODER |= (1U << (0*2));
    
    // PB1 (ECHO) setup (Input Pull-Down for robustness)
    GPIOB->MODER &= ~(3U << (1*2));
    GPIOB->PUPDR &= ~(3U << (1*2));
    GPIOB->PUPDR |= (2U << (1*2)); 

    // PC13 (LED) setup
    GPIOC->MODER &= ~(3U << (13 * 2));
    GPIOC->MODER |= (1U << (13 * 2));

    DWT_Delay_Init();
    I2C1_Init();
    ssd1306_Init();
    TIM2_PWM_Init(); // Servo Init with corrected frequency
    TIM4_Init_Interrupt();

    float distance = -1.0f;
    uint32_t last_ping_time = 0;
    for(int i=0; i<MAX_BLIPS; i++) blips[i].lifetime = 0;

    while (1) {
        if (g_update_flag) {
            g_update_flag = false;

            uint32_t t = msTicks;
            const float PI = 3.14159f;
            
            // Servo control: uses 1000us to 2000us values (CCR1)
            float phase = fmodf((float)t, 4000.0f) / 4000.0f;
            TIM2->CCR1 = 1500 + 500 * cosf(phase * 2.0f * PI);

            if ((msTicks - last_ping_time) >= 40) {
                last_ping_time = msTicks;
                distance = getUltrasonicDistance();
                if (distance < 1 || distance > 70) {
                    distance = -1.0f;
                }
            }

            if (distance > 0) {
                GPIOC->BSRR = (1U << 13);
            } else {
                GPIOC->BSRR = (1U << (13 + 16));
            }

            memset(SSD1306_Buffer, 0, sizeof(SSD1306_Buffer));

            // Angle calculation using 1000us to 2000us CCR1 values
            float angle_rad = (PI * (2000 - TIM2->CCR1)) / 1000.0f;
            
            if (distance > 0) {
                int dot_dist = (int)((distance / 70.0f) * RADAR_MAX_RADIUS);
                int x = RADAR_CENTER_X - (int)(dot_dist * cosf(angle_rad));
                int y = RADAR_CENTER_Y - (int)(dot_dist * sinf(angle_rad));
                addBlip(x, y);
            }
            
            drawRadarBackground();
            updateAndDrawBlips();
            
            int lx = RADAR_CENTER_X - (int)(RADAR_MAX_RADIUS * cosf(angle_rad));
            int ly = RADAR_CENTER_Y - (int)(RADAR_MAX_RADIUS * sinf(angle_rad));
            ssd1306_Line(RADAR_CENTER_X, RADAR_CENTER_Y, lx, ly, White);

            drawDistance(distance);
            ssd1306_UpdateScreen();
        }
    }
}