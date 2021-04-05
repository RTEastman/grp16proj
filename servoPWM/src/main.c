/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f0xx.h"
#include "stm32f0_discovery.h"
#include <stdio.h>

// Section 6.2
// -------------------------------------------
// Should enable clock to GPIO port A, configure the modes of the three
// pins corresponding to TIM1_CH1, TIM1_CH2 and TIM1_CH3 as alternate function.
void setup_gpio() {
    /* Student code goes here */


    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~((3<<(0*2))+(3<<(1*2))+(3<<(2*2)));    //clear the bits first
    GPIOA->MODER |= ((2<<(0*2))+(2<<(1*2))+(2<<(2*2)));    //sets pins to alternative function mode (10 aka 2) (PA0-3 will need this)
    GPIOA->AFR[0] |= ((2<<(0*4))+(2<<(1*4))+(2<<(2*4)));	//set to AF2 (PA0-3 need AF2 aswell for TIM2, probably with AFR[0] instead of AFR[1])

}

// Should use TIM1 to PSC so that the clock is 1 KHz, and choose the		\
// value of ARR so that the PWM frequency is 10 Hz. The duty cycle of each	|
// channel will be set by writing a value between 0 and 99 to the CCRx		|___ Original comments from ECE 362
// registers. Note since we have a common anode configuration,				|
// CCRx of 100 will result in an off LED and								|
// a CCRx of 0 will result in maximum brightness.							/
void setup_pwm() {
    /* Student code goes here */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;		//for PA0-3, we need TIM2. (If need be, PA2 and PA3 can be TIM15 on AF0 instead of AF2)
    TIM2->BDTR |= (1<<15);//TIM2_BTDR_MOE (may or may not exist)

    //PSC and ARR are the parameters used to change the resolution of the PWM. (Can also change PWM period/frequency, but please don't do that here)
    //Note: for our current servos, we need 20ms period (50Hz), meaning (ARR+1)*(PSC+1)=960000 for a 48MHz clock.
    //I experimentally found that an 11.72% duty cycle equates to ~180deg, while a 1.95% d.c. equates to ~0deg.
    //Theoretically, the range of angles are defined by pulses between 0.5ms-2.5ms, but that range might be slightly over 180deg (i.e. 0deg to 190deg).
    //The current values were calculated so that the CCR for 180deg would be 90,
    //which is a good resolution, but my reasoning for that specific value didn't work out, so 90 is arbitrary now.
    //IMPORTANT: if you're going to change these values, then you have to change the linear equation used to convert degrees to CCR in the main function.
    TIM2->PSC = 1249;//3749;//20*479;
    TIM2->ARR = 767;//255;//99;

    TIM2->CR1 &= ~TIM_CR1_CMS;
    TIM2->CR1 &= ~(1<<4); //set DIR to 0 (counts up)


    //set pwm 1 mode
    TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
    TIM2->CCMR2 &= ~TIM_CCMR2_OC3M;

    TIM2->CCMR1 |= TIM_CCMR1_OC1M_2 + TIM_CCMR1_OC1M_1;	//PWM mode 1
    TIM2->CCMR1 |= TIM_CCMR1_OC2M_2 + TIM_CCMR1_OC2M_1;	//Output compare 2 mode (PWM mode 1 for channel 2?)
    TIM2->CCMR2 |= TIM_CCMR2_OC3M_2 + TIM_CCMR2_OC3M_1;	//Output compare 3 mode (PWM mode 1 for channel 3?)

    TIM2->CCER |= ((TIM_CCER_CC1E)+(TIM_CCER_CC2E)+(TIM_CCER_CC3E)); //OCx output to specified pins enabled

    TIM2->CR1 |= 1; //CEN: counter enable

}
int main(void)
{

	setup_gpio();
	setup_pwm();
	int deg = 0; //use this variable to set the desired angle of the servo between 0-180

	//This code is unnecessary, its just a fun thing used to sweep the servo between 0-180 degrees continuously with 1 degree increments/decrements.
	//It's great for checking the resolution of the PWM and the servo.
	int cnt = 0;		//counter
	int cntSet = 1000; 	//sets the value cnt has to count to before a degree change happens (lower is faster)
	int cw = 1;			//"clockwise" flag
	int ccw = 0;		//"counter clockwise" flag
	while(1){
		if(deg == 0){cw = 1; ccw = 0;}
		else if(deg == 180){cw = 0; ccw = 1;}

		if(cw == 1){
			if(cnt == cntSet){
				deg += 1;
				cnt = 0;
			}
			cnt++;
		}
		else if(ccw == 1){
			if(cnt == cntSet){
				deg -= 1;
				cnt = 0;
			}
			cnt++;
		}

	//with current settings, min is 2 and max is 13
	//new settings, 0 to 255 ccr: min is 5, "good" max is 30 (bad max is 33), 18 is 90deg
	//newer-new settings should have 180deg be ccr=90, which it does. I found that 0deg is ccr=15
	//solving linearly with CCR 90 = 180deg & ccr 15 = 0deg:
		int xCCR = (.4166666667*deg) + 15;
	//This is where the duty cycle of the PWM is set. Remember: Duty Cycle = CCR / ARR, multiply by 100 to get percent
		TIM2->CCR1 = xCCR;   //PA0
		TIM2->CCR2 = 20;  //PA1
		TIM2->CCR3 = 20; //PA2
	}
}
