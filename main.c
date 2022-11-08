#include "msp.h"
#include "Clock.h"
#include <stdio.h>

#define LED_RED 1
#define LED_GREEN (LED_RED << 1)
#define LED_BLUE (LED_RED << 2)

/**
 * main.c
 */

void led_init()
{
    P2->SEL0 &= ~0x07;
    P2->SEL1 &= ~0x07;
    P2->DIR |= 0x07;
    P2->OUT &= ~0x07;
}

void turn_on_led(int color)
{
    P2->OUT &= ~0x07;
    P2->OUT |= color;
}

void turn_off_led()
{
    P2->OUT &= ~0x07;
}

void switch_init()
{
    P1->SEL0 &= ~0x12;
    P1->SEL1 &= ~0x12;
    P1->DIR &= ~0x12;
    P1->REN |= 0x12;
    P1->OUT |= 0x12;
}

void systick_init(void)
{
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL = 0x00000005;
}

void ir_init()
{
    // even emitter
    P5->SEL0 &= ~0x08;
    P5->SEL1 &= ~0x08;
    P5->DIR |= 0x08;
    P5->OUT &= ~0x08;

    // even emitter
    P9->SEL0 &= ~0x04;
    P9->SEL1 &= ~0x04;
    P9->DIR |= 0x04;
    P9->OUT &= ~0x04;

    // sensor
    P7->SEL0 &= ~0xFF;
    P7->SEL1 &= ~0xFF;
    P7->DIR &= ~0xFF;
}

void printBinary(unsigned int num)
{
    //이진수
    int i;
    for (i = 7; i >= 0; --i)
    { //8자리 숫자까지 나타냄
        int result = num >> i & 1;
        printf("%d", result);
    }
    printf("\n");
}

int ir_read()
{

        // IR on
        P5->OUT |= 0x08;
        P9->OUT |= 0x04;

        // charge emitter capacitor
        P7->DIR = 0xFF;
        P7->OUT = 0xFF;
        Clock_Delay1us(10);

        // read sensor
        P7->DIR = 0x00;
        Clock_Delay1us(1000);
        // Read p7, white = 0
//        sensor = P7->IN & 0x10;
//
//        if (sensor)
//        {
//            P2->OUT |= 0x01;
//        }
//        else
//        {
//            P2->OUT &= ~0x07;
//        }

        // IR off
        P5->OUT &= ~0x08;
        P9->OUT &= ~0x04;
//        Clock_Delay1ms(10);
        return P7->IN;

}

void pwm_init34(uint16_t period, uint16_t duty3, uint16_t duty4){
    TIMER_A0->CCR[0] = period;

    TIMER_A0->EX0 = 0x0000;

    TIMER_A0->CCTL[3] = 0x0040;
    TIMER_A0->CCR[3] = duty3;
    TIMER_A0->CCTL[4] = 0x0040;
    TIMER_A0->CCR[4] = duty4;

    TIMER_A0->CTL = 0x02F0;

    P2->DIR |= 0xC0;
    P2->SEL0 |= 0xC0;
    P2->SEL1 &= ~0xC0;
}

void moter_init(void){
    P3->SEL0 &= ~0xC0; //nSLPR, nSleep
    P3->SEL1 &= ~0xC0;
    P3->DIR |= 0xC0;
    P3->OUT &= ~0xC0;

    P5->SEL0 &= ~0x30; // DIRR PH
    P5->SEL1 &= ~0x30;
    P5->DIR |= 0x30;
    P5->OUT &= ~0x30;

    P2->SEL0 &= ~0xC0; // PWMR EN
    P2->SEL1 &= ~0xC0;
    P2->DIR |= 0xC0;
    P2->OUT &= ~0xC0;

    pwm_init34(7500, 0, 0);
}

void run_moter(float speed){
    P5->OUT &= ~0x30; // PH=0
    P2->OUT |= 0xC0; // EN=1
    P3->OUT |= 0xC0; // nSleep=1
    Clock_Delay1us((int) (speed*10000));

    P2->OUT &= ~0xC0;
    Clock_Delay1us((int) ((1-speed)*10000));

}
void move(uint16_t leftDuty, uint16_t rightDuty){
    P3->OUT |= 0xC0;
    TIMER_A0->CCR[4]=leftDuty;
    TIMER_A0->CCR[3]=rightDuty;
}

void left_forward(){
    P5->OUT &= ~0x10;
}
void left_backward(){
    P5->OUT |= 0x10;
}
void right_forward(){
    P5->OUT &= ~0x20;
}
void right_backward(){
    P5->OUT |= 0x20;
}


void systick_wait1ms(void)
{
    SysTick->LOAD = 48000;
    SysTick->VAL = 0;
    while ((SysTick->CTRL & 0x00010000) == 0)
    {
    }

}
void systick_wait1s()
{
    int i;
    for (i = 0; i < 1000; i++)
    {
        systick_wait1ms();
    }
}


void main(void)
{
//	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

//        Clock_Delay1ms(50);
    Clock_Init48MHz();
    led_init();
    switch_init();
    systick_init();
    ir_init();
    moter_init();
    int i;
    int left;
    int right;
    int left_senser;
    int right_senser;
    while(1){
        for(i =0;i<50;i++){
            systick_wait1ms();
        }

        int sensor = ir_read();
        printBinary(sensor);
        left_senser = sensor >> 7 & 1;
        right_senser = sensor >> 0 & 1;

        if(left_senser && right_senser){
            move(0, 0);
        } else if(left_senser){
            left_backward();
            right_forward();
            move(1000,1000);
        } else if(right_senser){
            right_backward();
            left_forward();
            move(1000,1000);
        } else{
            move(1000,1000);
        }

    }

}
