#include "msp.h"
#include "Clock.h"
#include <stdio.h>

#define LED_RED 1
#define LED_GREEN (LED_RED << 1)
#define LED_BLUE (LED_RED << 2)

/**
 * main.c
 */

uint16_t first_left;
uint16_t first_right;

uint16_t period_left;
uint16_t period_right;
uint32_t left_count, right_count;
uint16_t left_sensor4, left_sensor3, left_sensor2, left_sensor1; // 숫자가 클 수록 바깥쪽
uint16_t right_sensor1, right_sensor2, right_sensor3, right_sensor4; // 숫자가 클 수록 바깥쪽

int slow_speed = 1000; // 바꾸지 말 것, 정확한 거리를 이동할 필요가 있는 작업에 사용됨
int default_speed = 3000;

int rotate_speed = 1500; // 최적화 필요
int interval=10;

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

void pwm_init34(uint16_t period, uint16_t duty3, uint16_t duty4)
{
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

void moter_init(void)
{
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

void move(uint16_t leftDuty, uint16_t rightDuty)
{
    // set wheel speed
    P3->OUT |= 0xC0;
    TIMER_A0->CCR[4] = leftDuty;
    TIMER_A0->CCR[3] = rightDuty;
}

// set wheel rotation direction
void left_forward()
{
    P5->OUT &= ~0x10;
}
void left_backward()
{
    P5->OUT |= 0x10;
}
void right_forward()
{
    P5->OUT &= ~0x20;
}
void right_backward()
{
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

int ir_read();
void (*TimerA2Task)(void);
void TimerA2_Init(void (*task)(void), uint16_t period);
void TA2_0_IRQHandler(void);
void task();
void timer_A3_capture_init();
void TA3_0_IRQHandler(void);
void TA3_N_IRQHandler(void);
uint32_t get_left_rpm();
uint32_t get_right_rpm();

// 주행을 위해 추상화된 함수들 직진, 회전 등
void rotate(int degree); // 현재 위치를 기준으로 회전, 시계 방향이 + 방향
void moveSimple(int speed, int checkObstacle, int time); // checkObstacle: 좌우에 장애물이 있으면 엄춤
void moveCurve(int speed, int checkObstacle, int time); // checkObstacle: 좌우에 장애물이 있으면 엄춤
void moveCurve2(int speed, int checkObstacle, int time); // 곡면이 강한 커브길

void track1(){
    turn_on_led(LED_BLUE);
    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 300); // 센서가 바퀴의 중심축보다 조금 앞에 있어서 조금 앞으로 간 다음에 회전해야함
    rotate(90);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(90);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 300);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(90);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(-90);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(90);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(-90);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 300);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(-90);

}

void track2(){
    turn_on_led(LED_GREEN);

    // hair pin curve 1
    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 400);
    rotate(-90);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 500);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 200); // 지나치게 많이 꺽이는 커브는 ㄷ자 도로처럼 다뤄줘야함
    rotate(70);
    moveCurve(default_speed, 1, 999999);

    moveSimple(slow_speed, 0, 200);
    rotate(70);
    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 300);

    // hair pin curve2
    moveCurve(default_speed, 1, 999999);
    rotate(-20);
    moveSimple(slow_speed, 0, 500);

    moveCurve2(slow_speed*1.5, 1, 999999);
    // moveCurve2는 많이 꺽이는 커브를 이동하기 위한 함수인데 너무 빠르게 달리면 이탈할 수 있으니 속도 값을 한계까지 잘 최적화해야함
    rotate(-15);
    moveSimple(slow_speed, 0, 800);

    // hair pin curve3
    moveCurve2(slow_speed*1.5, 1, 999999);
    moveSimple(slow_speed, 0, 500);
    rotate(30);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 200);
    rotate(-90);
    moveCurve(slow_speed*1.5, 1, 999999);

    moveSimple(slow_speed, 0, 200);
    rotate(-50);
    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 500);


    moveCurve2(slow_speed*1.5, 1, 999999);
    moveSimple(slow_speed, 0, 200);
    rotate(70);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 200);
    rotate(70);
    moveCurve2(slow_speed*1.5, 1, 1000);



}

void track3(){
    turn_on_led(LED_BLUE);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(90);

    moveCurve2(slow_speed*1.5, 1, 999999);

    moveSimple(slow_speed, 0, 300);
    rotate(-90);
    moveCurve2(slow_speed*1.5, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(90);
    moveCurve2(slow_speed*1.5, 1, 999999);

    moveSimple(slow_speed, 0, 300);
    rotate(-90);
    moveCurve2(slow_speed*1.5, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(90);
    moveCurve2(slow_speed*1.5, 1, 999999);

    moveSimple(slow_speed, 0, 300);
    rotate(-90);
    moveCurve2(slow_speed*1.5, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(90);
    moveCurve2(slow_speed*1.5, 1, 999999);

    moveSimple(slow_speed, 0, 300);
    rotate(-90);
    moveCurve2(slow_speed*1.5, 1, 999999);

    moveSimple(slow_speed, 0, 300);
    rotate(-90);


}

void track4(){
    turn_on_led(LED_GREEN);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(90);

    moveCurve(default_speed, 1, 999999);

    rotate(90);
    moveSimple(slow_speed, 0, 800);
    rotate(75);

    moveCurve2(slow_speed*1.5, 1, 999999);
    rotate(-15);
    moveSimple(slow_speed, 0, 300);
    rotate(-90);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(90);

    moveCurve2(slow_speed*1.5, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(90);
}
void track5(){
    turn_on_led(LED_BLUE);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(-90);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 500);
    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(-90);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(-90);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 500);
    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(90);
}

void track6(){
    turn_on_led(LED_GREEN);

    moveCurve(default_speed, 1, 999999);
    rotate(90);
    moveSimple(slow_speed, 0, 1200);
    rotate(30);
    moveCurve2(default_speed*0.7, 1, 999999);

    rotate(-30);
    moveSimple(slow_speed, 0, 600);
    rotate(-90);

    moveCurve2(default_speed*0.7, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(90);

    moveCurve2(default_speed*0.7, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(90);

    moveCurve2(default_speed*0.7, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(-90);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 300);
    rotate(90);

    moveCurve(default_speed, 1, 999999);
    moveSimple(slow_speed, 0, 600);
    moveCurve2(slow_speed, 0, 600);
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
    // TimerA2_Init(&task, 50000);
    timer_A3_capture_init(); // check rotation

    Clock_Delay1ms(700);
    track1();
    track2();
    track3();
    track4();
    track5();
    track6();

    track1();
    track2();
    track3();
    track4();
    track5();
    track6();
//    moveCurve2(slow_speed*1.5, 1, 999999);

    return;

}

void rotate(int degree) // 현재 위치를 기준으로 회전, 시계 방향이 + 방향
{
    if (degree == 0)
        return;
    move(0, 0);
    if (degree > 0)
    {
        left_forward();
        right_backward();
        left_count = 0;
        move(rotate_speed, rotate_speed);
        while (1)
        {
            if (left_count >= degree * 2)
            {
                move(0, 0);
                break;
            }
        }
    }
    if (degree < 0)
    {
        left_backward();
        right_forward();
        move(rotate_speed, rotate_speed);
        left_count = 0;
        while (1)
        {
            // printf("%d\n", left_count);
            if (left_count >= -degree * 2)
            {
                move(0, 0);
                break;
            }
        }
    }
}
void moveSimple(int speed, int checkObstacle, int time) // checkObstacle: 좌우에 장애물이 있으면 엄춤
{
    int i;
    for (i = 0; i < time / interval; i++)
    {
        if (checkObstacle)
        {
            ir_read();
            if (left_sensor3 == 1 || right_sensor3 == 1)
            {
                move(0, 0);
                break;
            }

        }
        right_forward();
        left_forward();
        move(speed, speed);
        Clock_Delay1ms(interval);
    }
    move(0, 0);
}
void moveCurve(int speed, int checkObstacle, int time) // checkObstacle: 좌우에 장애물이 있으면 엄춤
{
    int i;
    for (i = 0; i < time / interval; i++)
    {
        printBinary(ir_read());
        if (checkObstacle)
        {
            ir_read();
            printf("%d %d", left_sensor4, right_sensor4);
            if ( (left_sensor4)
                    || (right_sensor4)
                    || (left_sensor3 && right_sensor3))
            {
                move(0, 0);
                Clock_Delay1ms(300); // 관성 때분에 장애물 만나면 잠깐 정지하게 했는데 더 짧게 정지해도 될듯
                break;
            }

        }
        right_forward();
        left_forward();
        move(speed, speed);
        if(0){

        }
        else if (left_sensor2 && !right_sensor2)
        {
            move(speed * 0.5, speed*1.5);
            Clock_Delay1ms(interval);
        }
        else if ( !left_sensor2 && right_sensor2)
        {
            move(speed*1.5, speed * 0.5);

            Clock_Delay1ms(interval);
        } else {
            Clock_Delay1ms(interval);
        }
    }
    move(0, 0);
}

void moveCurve2(int speed, int checkObstacle, int time) // checkObstacle: 좌우에 장애물이 있으면 엄춤
{
    int i;
    for (i = 0; i < time / interval; i++)
    {
        printBinary(ir_read());
        if (checkObstacle)
        {
            ir_read();
            printf("%d %d", left_sensor4, right_sensor4);
            if ( (left_sensor4)
                    || (right_sensor4)
                    || (left_sensor3 && right_sensor3))
            {
                move(0, 0);
                Clock_Delay1ms(300); // 관성 때분에 장애물 만나면 잠깐 정지하게 했는데 더 짧게 정지해도 될듯
                break;
            }

        }
        right_forward();
        left_forward();
        move(speed, speed);
        if(0){

        }
        else if (left_sensor3 && !right_sensor3)
        {
            right_forward();
            left_backward();
            move(speed, speed);
            Clock_Delay1ms(interval);
        }
        else if ( !left_sensor3 && right_sensor3)
        {

            right_backward();
            left_forward();
            move(speed, speed);
            Clock_Delay1ms(interval);
        }
        else if (left_sensor2 && !right_sensor2)
        {
            move(speed * 0.2, speed*1.5);
            Clock_Delay1ms(interval);
        }
        else if ( !left_sensor2 && right_sensor2)
        {
            move(speed*1.5, speed * 0.2);
            Clock_Delay1ms(interval);
        }
        else {
            Clock_Delay1ms(interval);
        }
    }
    move(0, 0);
}

void TimerA2_Init(void (*task)(void), uint16_t period)
{
    TimerA2Task = task;
    TIMER_A2->CTL = 0x0280;
    TIMER_A2->CCTL[0] = 0x0010;
    TIMER_A2->CCR[0] = (period - 1);
    TIMER_A2->EX0 = 0x0005;
    NVIC->IP[3] = (NVIC->IP[3] & 0xFFFFFF00) | 0x00000040;
    NVIC->ISER[0] = 0x00001000;
    TIMER_A2->CTL |= 0x0014;
}
void TA2_0_IRQHandler(void)
{
    TIMER_A2->CCTL[0] &= ~0x0001;
    (*TimerA2Task)();
}
void task()
{
    printf("interrupt\n");
}
void timer_A3_capture_init()
{
    P10->SEL0 |= 0x30;
    P10->SEL1 &= ~0x30;
    P10->DIR &= ~0x30;

    TIMER_A3->CTL &= ~0x0030;
    TIMER_A3->CTL = 0x0200;

    TIMER_A3->CCTL[0] = 0x4910;
    TIMER_A3->CCTL[1] = 0x4910;
    TIMER_A3->EX0 &= ~0x0007;

    NVIC->IP[3] = (NVIC->IP[3] & 0x0000FFFF) | 0x40400000;
    NVIC->ISER[0] = 0x0000C000;
    TIMER_A3->CTL |= 0x0024;

}
void TA3_0_IRQHandler(void)
{
    TIMER_A3->CCTL[0] &= ~0x0001;
    period_right = TIMER_A3->CCR[0] - first_right;
    first_right = TIMER_A3->CCR[0];
    right_count++;
}
void TA3_N_IRQHandler(void)
{
    TIMER_A3->CCTL[1] &= ~0x0001;
    period_left = TIMER_A3->CCR[1] - first_left;
    first_left = TIMER_A3->CCR[1];
    left_count++;
}
uint32_t get_left_rpm()
{
    return 2000000 / period_left;
}
uint32_t get_right_rpm()
{
    return 2000000 / period_right;
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

    // IR off
    P5->OUT &= ~0x08;
    P9->OUT &= ~0x04;
//        Clock_Delay1ms(10);
    int sensor = P7->IN;
    left_sensor4 = sensor >> 7 & 1;
    left_sensor3 = sensor >> 6 & 1;
    left_sensor2 = sensor >> 5 & 1;
    left_sensor1 = sensor >> 4 & 1;
    right_sensor4 = sensor >> 0 & 1;
    right_sensor3 = sensor >> 1 & 1;
    right_sensor2 = sensor >> 2 & 1;
    right_sensor1 = sensor >> 3 & 1;
    return P7->IN;

}
