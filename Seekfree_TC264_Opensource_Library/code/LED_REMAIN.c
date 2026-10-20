/*
 * sata.c
 *
 *  Created on: 2026Äê10ÔÂ18ÈÕ
 *      Author: Freed
 */
#include "LED_REMAIN.h"
void LED_GPIO_INIT_0()
{
    gpio_init(P21_5,GPO,0,GPO_PUSH_PULL);
    gpio_init(P21_4,GPO,0,GPO_PUSH_PULL);
    gpio_init(P20_8,GPO,0,GPO_PUSH_PULL);
    gpio_init(P20_9,GPO,0,GPO_PUSH_PULL);
}
void LED_GPIO_INIT_1()
{
    gpio_init(P21_5,GPO,1,GPO_PUSH_PULL);
    gpio_init(P21_4,GPO,1,GPO_PUSH_PULL);
    gpio_init(P20_8,GPO,1,GPO_PUSH_PULL);
    gpio_init(P20_9,GPO,1,GPO_PUSH_PULL);
}
void LED_GPIO_WHILE_L()
{
    while(1)
        {
            for(int i=0;i<4;i++)
            {
                system_delay_ms(1000);
                if(i==0)
                {
                    gpio_set_level(P21_4, 0);
                    gpio_set_level(P21_5, 1);
                    gpio_set_level(P20_8, 1);
                    gpio_set_level(P20_9, 1);
                }
                else if(i==1)
                {
                    gpio_set_level(P21_4, 1);
                    gpio_set_level(P21_5, 0);
                    gpio_set_level(P20_8, 1);
                    gpio_set_level(P20_9, 1);
                }
                else if(i==2)
                {
                    gpio_set_level(P21_4, 1);
                    gpio_set_level(P21_5, 1);
                    gpio_set_level(P20_8, 0);
                    gpio_set_level(P20_9, 1);
                }
                else if(i==3)
                {
                    gpio_set_level(P21_4, 1);
                    gpio_set_level(P21_5, 1);
                    gpio_set_level(P20_8, 1);
                    gpio_set_level(P20_9, 0);
                }
            }
        }
}
void LED_PWM_INIT()
{
    pwm_init(ATOM0_CH3_P21_5,50,10000);
    pwm_init(ATOM0_CH2_P21_4,50,10000);
    pwm_init(ATOM2_CH7_P20_8,50,10000);
    pwm_init(ATOM1_CH5_P20_9,50,10000);
}
void LED_PWM_WHILE_H()
{
    while(1)
        {
            for(int j=10000;j>=0;j-=100)
            {
               pwm_set_duty(ATOM0_CH3_P21_5,j);
               system_delay_ms(20);
            }
            for(int i=0;i<=10000;i+=100)
            {
                pwm_set_duty(ATOM0_CH3_P21_5,i);
                system_delay_ms(20);
            }
            for(int j=10000;j>=0;j-=100)
            {
                pwm_set_duty(ATOM0_CH2_P21_4,j);
                system_delay_ms(20);
            }
            for(int i=0;i<=10000;i+=100)
            {
                pwm_set_duty(ATOM0_CH2_P21_4,i);
                system_delay_ms(20);
            }
            for(int j=10000;j>=0;j-=100)
            {
                pwm_set_duty(ATOM2_CH7_P20_8,j);
                system_delay_ms(20);
            }
            for(int i=0;i<=10000;i+=100)
            {
                pwm_set_duty(ATOM2_CH7_P20_8,i);
                system_delay_ms(20);
            }
            for(int j=10000;j>=0;j-=100)
            {
                pwm_set_duty(ATOM1_CH5_P20_9,j);
                system_delay_ms(20);
            }
            for(int i=0;i<=10000;i+=100)
            {
                pwm_set_duty(ATOM1_CH5_P20_9,i);
                system_delay_ms(20);
            }
         }
}
void sata()
{

}
