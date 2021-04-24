#include "stm32f0xx.h"
#include "stm32f0_discovery.h"

#define FAIL -1
#define SUCCESS 0
#define WR 0
#define RD 1

void micro_wait(unsigned int);

void I2C1_waitidle(void) {
    while ((I2C1->ISR & I2C_ISR_BUSY) == I2C_ISR_BUSY);  // while busy, wait.
}

int I2C1_checknack(void) {
    if (I2C1->ISR & I2C_ISR_NACKF)
        return 1;
    return 0;
}

void I2C1_clearnack(void) {
    I2C1->ICR |= I2C_ICR_NACKCF;
}

void init_I2C1() {
    // Student code goes here
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER |= (GPIO_Mode_AF << 12) | (GPIO_Mode_AF << 14);
    GPIOB->AFR[0] |= (0b1 << 24) | (0b1 << 28);
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    I2C1->CR2 &= ~(I2C_CR2_ADD10);
    I2C1->CR2 |= I2C_CR2_NACK;
    //I2C1->CR2 |= I2C_CR2_AUTOEND;

    I2C1->TIMINGR |= (0b0000 << 28) | (0b100 <<20) | (0b10 << 16) | (0xC4 << 8) | 0b11000111;
    I2C1->OAR1 &= ~(0b1 << 15);
    I2C1->OAR1 |= (0b1 << 15) | 0x2;
    I2C1->OAR2 &= ~(0b1 << 15);
    I2C1->CR1 |= I2C_CR1_PE;

    micro_wait(30);
    char em[2];
	I2C1_start(0x52, RD);
	I2C1_readdata(&em,2);
	//I2C1_stop();
	micro_wait(30);
	I2C1->CR1 &= ~I2C_CR1_PE;
	micro_wait(30);
	I2C1->CR1 |= I2C_CR1_PE;

    //---------End-----------
    //I2C1_start();
}


void I2C1_start(uint8_t addr, uint32_t dir) {
    // Student code goes here
    I2C1->CR2 &= ~(0b11111110);
    I2C1->CR2 &= ~(I2C_CR2_RD_WRN);
    I2C1->CR2 |= addr<<1;
    if(dir == RD){
        I2C1->CR2 |= I2C_CR2_RD_WRN;
    }
    I2C1->CR2 |= I2C_CR2_START;
    //---------End-----------
}

// See lab document for description
void I2C1_stop() {
    // Student code goes here
    int plzstop = 0;
    if(I2C1->ISR & I2C_ISR_STOPF){
        return;
    }
    I2C1->CR2 |= I2C_CR2_STOP;
    while(!(I2C1->ISR & I2C_ISR_STOPF)){
        /*if(plzstop > 5000){
            I2C1->CR1 &= ~(I2C_CR1_PE);
            micro_wait(10);
            I2C1->CR1 |= (I2C_CR1_PE);
            break;
        }else{
            plzstop++;
        }*/
    }
    I2C1->ICR |= I2C_ICR_STOPCF;
    //---------End-----------
}

int I2C1_senddata(uint8_t* data, uint32_t size) {
    // Student code goes here
    I2C1->CR2 &= ~(I2C_CR2_NBYTES);
    I2C1->CR2 |= (I2C_CR2_NBYTES & (size<<16));
    for(int i = 0;i<size;i++){
        int timeout = 0;
        while(0==(I2C1->ISR & I2C_ISR_TXE)){
            timeout++;
            if(timeout > 1000000){
                return FAIL;
            }

        }
        I2C1->TXDR = data[i];

    }
    while(!((I2C1->ISR & I2C_ISR_NACKF) || (I2C1->ISR & I2C_ISR_TC))){
        ;
    }
    if(I2C1->ISR & I2C_ISR_NACKF){
        return FAIL;
    }

    return SUCCESS;

    //---------End-----------
}

// See lab document for description
int I2C1_readdata(int8_t* data, uint32_t size) {
    // Student code goes here
    I2C1->CR2 &= ~(I2C_CR2_NBYTES);
    I2C1->CR2 |= (I2C_CR2_NBYTES & (size<<16));
    for(int i = 0;i<size;i++){
            int timeout = 0;
            while(!(I2C1->ISR & I2C_ISR_RXNE)){
                timeout++;
                if(timeout > 1000000){
                    return FAIL;
                }
            }
            data[i] = I2C1->RXDR;

        }
    while(!((I2C1->ISR & I2C_ISR_NACKF) || (I2C1->ISR & I2C_ISR_TC))){
            ;
        }
    if(I2C1->ISR & I2C_ISR_NACKF){
            return FAIL;
    }
    return SUCCESS;
    //---------End-----------
}

void tof_params(){
    char data[2];
    data[0] = 8;
    data[1] = 1;
    I2C1_waitidle();
    I2C1_start(0x52, WR);
    I2C1_senddata(data, 1);
    micro_wait(35);
    I2C1_senddata(data+1, 1);
    I2C1_stop();
    micro_wait(100);
}

void setup_gpioc(){
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER |= 0b01010101;
    GPIOC->MODER |=(0b01<<14);
}

int tof(void)
{   //init_I2C1();  //moved to main.c
    //setup_gpioc();  //moved to main.c
    //tof_params();
    int reads = 0;
    char data[6];
    data[0] = 0;
    data[1] = 1;
    data[2] = 255;
    data[3] = 255;

    while(1){
        I2C1_waitidle();

        //micro_wait(40);
        //I2C1_senddata(data+1,1);

        I2C1_start(0x52, WR);
        int check1 = I2C1_senddata(data, 1);
        I2C1_stop();


        micro_wait(100);
        I2C1_waitidle();
        I2C1_start(0x52, RD);
        int check2 = I2C1_readdata(data+1, 2);
        I2C1_stop();
        unsigned int val = (data[1] << 8) | data[2];
        /**
        reads++;

        GPIOC->ODR &= ~(1<<7);
        if(val < 400){
            GPIOC->ODR |= 0b1111;

        } else if(val < 800){
            GPIOC->ODR &= ~0b1000;
            GPIOC->ODR |= 0b111;

        } else if(val < 1200){
            GPIOC->ODR &= ~0b1100;
            GPIOC->ODR |= 0b11;
        }else if(val < 1600){
            GPIOC->ODR &= ~0b1110;
            GPIOC->ODR |= 0b1;
        }else{
            GPIOC->ODR &= ~0b1111;
        }
        if(check1 + check2 < 0){
            GPIOC->ODR  |= (1<<7);
        }
        micro_wait(700);
        **/
        return val;
    }
}
