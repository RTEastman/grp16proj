#include "stm32f0xx.h"
#include "stm32f0_discovery.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "syscalls.h"
#include "fifo.h"
#include <stdlib.h>
#include "stm32f0xx.h"
#include "stm32f0_discovery.h"
#include "uart.h"

//#include "ece477.h"
//added from header file
struct sensorInfo {
	int barcode_info;
	int compartment;
	struct sensorInfo *next;
};

struct sensorInfo *barcode_storage;
//barcode_storage -> next = NULL;
struct sensorInfo *old = NULL;
struct sensorInfo *curr = NULL;

#define MAX_RECORDS 5
int counter = 0;
int compartment_record = 0;
int oldest = 0;
int ptr = 0;
int flap = 0;



#define UNK -1
#define NON_INTR 0
#define INTR 1
#define MAX_BUF_SIZE 	128

void setup_pwm();
void setup_gpio();

void TIM6_DAC_IRQHandler(void);
void USART1_IRQHandler(void);

/**int __io_putchar(int ch);
static int putchar_nonirq(int ch);

void prob3(void);
void prob4(void);
void prob5(void);

static struct fifo input_fifo;  // input buffer
static struct fifo output_fifo; // output buffer
**/

void micro_wait(unsigned int);
int interrupt_mode = UNK;   // which version of putchar/getchar to use.
int echo_mode = 0;          // should we echo input characters?

#define RATE 100000
#define N 1000
short int wavetable[N];
// Only need to receive no transmit
// stdin?

//=============================================================================  MAY USE THIS FUNCTION
// This is a version of printf() that will disable interrupts for the
// USART and write characters directly.  It is intended to handle fatal
// exceptional conditions.
// It's also an example of how to create a variadic function.
/**
static void safe_printf(const char *format, ...) {
    va_list ap;
    va_start(ap, format);
    char buf[80];
    int len = vsnprintf(buf, sizeof buf, format, ap);
    int saved_txeie = USART1->CR1 & USART_CR1_TXEIE;
    USART1->CR1 &= ~USART_CR1_TXEIE;
    int x;
    for(x=0; x<len; x++) {
        putchar_nonirq(buf[x]);
    }
    USART1->CR1 |= saved_txeie;
    va_end(ap);
}
**/
//=======================================================================
// Simply write a string one char at a time.
//=======================================================================
/**
static void putstr(const char *s) {
    while(*s)
        __io_putchar(*s++);
}
**/
//=======================================================================
// Insert a character into input FIFO and echo it.
// (or, if it's a backspace, remove a char and erase it from the line).
// If echo_mode is turned off, just insert the character and get out.
//=======================================================================
/**
static void insert_echo_char(char ch) {
    if (ch == '\r')
        ch = '\n';
    if (!echo_mode) {
        fifo_insert(&input_fifo, ch);
        return;
    }
    if (ch == '\b' || ch == '\177') {
        if (!fifo_empty(&input_fifo)) {
            char tmp = fifo_uninsert(&input_fifo);
            if (tmp == '\n')
                fifo_insert(&input_fifo, '\n');
            else if (tmp < 32)
                putstr("\b\b  \b\b");
            else
                putstr("\b \b");
        }
        return; // Don't put a backspace into buffer.
    } else if (ch == '\n') {
        __io_putchar('\n');
    } else if (ch == 0){
        putstr("^0");
    } else if (ch == 28) {
        putstr("^\\");
    } else if (ch < 32) {
        __io_putchar('^');
        __io_putchar('A'-1+ch);
    } else {
        __io_putchar(ch);
    }
    fifo_insert(&input_fifo, ch);
}

**/
//-----------------------------------------------------------------------------
// Section 6.2
//-----------------------------------------------------------------------------
// This should should perform the following
// 1) Enable clock to GPIO port A
// 2) Configure PA9 and PA10 to alternate function to use a USART
//    Note: Configure both MODER and AFRL registers
// 3) Enable clock to the USART module, it is up to you to determine
//    which RCC register to use
// 4) Disable the USART module (hint UE bit in CR1)
// 5) Configure USART for 8 bits, 1 stop bit and no parity bit
// 6) Use 16x oversampling
// 7) Configure for 9600 baud rate
// 8) Enable the USART for both transmit and receive
// 9) Enable the USART
// 10) Wait for TEACK and REACK to be set by hardware in the ISR register
// 11) Set the 'interrupt_mode' variable to NON_INTR
void tty_init(void) {
    // Disable buffers for stdio streams.  Otherwise, the first use of
    // each stream will result in a *malloc* of 2K.  Not good.
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    //1
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    //2
    GPIOA->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10);
    GPIOA->MODER |= 2<<(2*9) | 2<<(2*10);
    GPIOA->AFR[1] &= ~(0xff<<4);
    GPIOA->AFR[1] |= 1<<(4*1) | 1<<(4*2);
    //3
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    //4
    USART1->CR1 &= ~ USART_CR1_UE;
    //5
    USART1->CR1 &= ~(1<<28 | 1<<12);
    USART1->CR2 &= ~USART_CR2_STOP;
    USART1->CR1 &= ~ USART_CR1_PCE;
    //6
    USART1->CR1 &= ~USART_CR1_OVER8;
    //7
    USART1->BRR = 0x1388;
    //8
    USART1->CR1 |= USART_CR1_RE |  USART_CR1_TE;
    //9
    USART1->CR1 |=  USART_CR1_UE;
    //10
    while(!((USART1->ISR & USART_ISR_REACK)&&(USART1->ISR & USART_ISR_TEACK)));
    //11
    NVIC -> ISER[0] = (1 << USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 2);
    interrupt_mode = NON_INTR;
}

//=======================================================================
// Enable the USART RXNE interrupt.Receive data register not empty
// Remember to enable the right bit in the NVIC registers
//=======================================================================
/**
void enable_tty_irq(void) {
    // Student code goes here...
    USART1->CR1 |= USART_CR1_RXNEIE;
    NVIC->ISER[0] = (1<<USART1_IRQn);
    interrupt_mode = INTR;
}
**/
//-----------------------------------------------------------------------------
// Section 6.3
//-----------------------------------------------------------------------------
//=======================================================================
// This method should perform the following
// Transmit 'ch' using USART1, remember to wait for transmission register to be
// empty. Also this function must check if 'ch' is a new line character, if so
// it must transmit a carriage return before transmitting 'ch' using USART1.
// Think about this, why must we add a carriage return, what happens otherwise?
//=======================================================================
/**
static int putchar_nonirq(int ch) {
    // Student code goes here...
    if (ch == '\n'){
        while(!(USART1->ISR & USART_ISR_TXE));
        USART1->TDR = '\r';
    }
    while(!(USART1->ISR & USART_ISR_TXE));
    USART1->TDR = ch;
    return ch;
}**/

//-----------------------------------------------------------------------------
// Section 6.4
//-----------------------------------------------------------------------------
// See lab document for description
/**
static int getchar_nonirq(void) {
    // Student code goes here...
    if (USART1->ISR & USART_ISR_ORE){   //if overrun error indicated
        USART1->ICR &= ~USART_ICR_ORECF;        // clear the overrun error bit
    }
    while (!fifo_newline(&input_fifo)){
        while(!(USART1->ISR & USART_ISR_RXNE));//wait while USART->RDR is empty
        insert_echo_char(USART1->RDR);          // put char in RDR into Input buffer
    }
    return fifo_remove(&input_fifo);
}
**/
//-----------------------------------------------------------------------------
// Section 6.5
//-----------------------------------------------------------------------------
// See lab document for description
//=======================================================================
// IRQ invoked for USART1 activity.
/**
void USART1_IRQHandler(void) {
	pwm();
}
**/
// See lab document for description
/**
static int getchar_irq(void) {
    // Student code goes here...
    while(!fifo_newline(&input_fifo)){
        asm("wfi");
    }
    return fifo_remove(&input_fifo);    //read so remove
}
**/
// See lab document for description
/**
static int putchar_irq(char ch) {
    // Student code goes here...
  while (fifo_full(&output_fifo)){
      asm("wfi");
  }
  if (ch=='\n'){
      fifo_insert(&output_fifo, '\r');  //Insert into output FIFO
  }
  else{
      fifo_insert(&output_fifo, ch);
  }
  if (!(USART1->CR1 & USART_CR1_TXEIE)){
      USART1->CR1 |= USART_CR1_TXEIE;   //enable transmit data empty interrupt(let us know
                                              //when data is sent to shift register
      USART1_IRQHandler();              // send a char from output FIFO to UART->TDR
  }
  if (ch == '\n'){              // previously only inserted \r not \n, so we add it now
      while(fifo_full(&output_fifo)){
          asm("wfi");
      }
      fifo_insert(&output_fifo,'\n');
  }
  return ch;
}

**/
//=======================================================================
// Called by the Standard Peripheral library for a write()
/**
int __io_putchar(int ch) {
    if (interrupt_mode == INTR)
        return putchar_irq(ch);
    else
        return putchar_nonirq(ch);
}
**/
//=======================================================================
// Called by the Standard Peripheral library for a read()
/**int __io_getchar(void) {        //called in syscall read(). who calls read?
    // Choose the right implementation.
    if (interrupt_mode == INTR)
        return getchar_irq();
    else
        return getchar_nonirq();
}
**/

// This function should,
// enable clock to timer6,
// setup pre scalar and arr so that the interrupt is triggered every
// 10us, enable the timer 6 interrupt, and start the timer.
// What is this triggerring?
/**
void setup_timer6() {
    // Student code goes here...
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    //enable clock to timer6
    // frequency = 1e6
    TIM6->ARR = 24-1;
    TIM6->PSC = 20-1;
    //enable timer6 interrupt
    TIM6->DIER |= 1;
    NVIC->ISER[0] = 1<<TIM6_DAC_IRQn;
    //start the timer
    TIM6->CR1 |= 1;
}**/

//===========================================================================
int maskBarcode(int b_data) {
	//sscanf("");
	//uint8_t barcode_info = b_data;
	int mask = 0x3;
	int masked = b_data & mask;
	return masked;
	//int mask[2];
	/**mask = [1,1];
	int ct = -1;
	while(b_data[ct] != '\r' || b_data[ct] != '\n') {  //get counting variable to last element of barcode information
		ct--;
	}
	int ct2 = ct;
	ct--;  //advance one more spot than carriage return
	int ct3 = 1;
	while((ct2 - ct) > 2) {  //loop through mask and b_data array
		mask[ct3] = mask[ct3] & b_data[ct];
		ct3--;
		ct--;
	}
	return;**/
}

int storeBarcode(int b_data, int m_data) {
	//================ Debug bottom code (linked list strategy)
	/**
	struct sensorInfo *new_node =  malloc(sizeof(struct sensorInfo));
	//(struct sensorInfo*)

	//return;
	new_node -> barcode_info = b_data;  //looping at this line??
	//printf("b_data");
	//int locl = b_data;
	if (old == NULL) { //check if linked list has no old nodes yet
		new_node -> next = NULL; //initialize lonely node by itself
		old = new_node;
		curr = new_node;
	}
	else {
		new_node -> next = curr; //attach new node to back of existing linked list
		curr = new_node;  //make the newest node the "head"
	}
	barcode_storage = curr;
	new_node -> compartment = maskBarcode(m_data); //fill in with masking function
	return new_node -> compartment;
	**/
	//================
	//barcode_record[ptr] = b_data;
	int pre_incre = compartment_record;
	compartment_record |= (maskBarcode(b_data) << (counter * 2));
	/**if (ptr == MAX_RECORDS - 1)
		ptr = 0;
	else
		ptr++;**/
	counter++;
	int post_incre = compartment_record;
}

/**
 * This function will be invoked to take the oldest barcode information (which is at the end/tail of the linked list),
 * and disconnect (free) it from the rest of the linked list.
 * Returns the compartment information of the oldest node.
 */
int removeBarcode(void) {
	//=======================Debug below
	//struct sensorInfo* temp_node = old;
	/**
	if (barcode_storage == NULL) {
		return 0;
	}
	struct sensorInfo* tracking_node = curr;
	if (curr == old) {  //there is only 1 node in linked list; list should be empty after this
		free(curr);
		return 0;
	}
	while((tracking_node -> next != old) || (tracking_node -> next != NULL)) { //loop to the 2nd oldest node of linked list
		tracking_node = tracking_node -> next;
	}

	tracking_node -> next = NULL; //disconnect oldest node from rest of linked list.
	int compart_info = old -> compartment;
	free(old);
	old = tracking_node;**/
	//=======================
	if(counter == 0) {
		return 0;
	}
	int pre_queue = compartment_record;
	int compart_info = 3 & compartment_record;
	compartment_record = compartment_record >> 2;
	counter--;
	int post_queue = compartment_record;
	/**
	if (oldest == MAX_RECORDS)
		oldest = 0;
	else
		oldest++;**/
	return compart_info;
}

void choose_compartment_pwm(int compart_num) {
	//GPIOC -> ODR &= ~(GPIO_ODR_0);
	//int compartment_num = removeBarcode();
	int compartment_num = flap;
	if (compartment_num == 1) {//it's backwards
		//replace instance with pwm signal
		pwm(1,145);
		GPIOC -> ODR |= GPIO_ODR_0;
		GPIOC -> ODR &= ~(GPIO_ODR_1);
		GPIOC -> ODR &= ~(GPIO_ODR_2);
		//TIM2->CCR1 = 45; //problematic
		micro_wait(1500000);
		pwm(1,180);
		micro_wait(1000000);
	}
	if (compartment_num == 2) {//it's backwards
		//replace instance with pwm signal
		pwm(2,135);
		GPIOC -> ODR &= ~(GPIO_ODR_0);
		GPIOC -> ODR |= GPIO_ODR_1;
		GPIOC -> ODR &= ~(GPIO_ODR_2);

		micro_wait(1000000);
		pwm(2,180);
		micro_wait(1000000);
	}
	if (compartment_num == 3){
		//replace instance with pwm signal
		pwm(3,135);
		GPIOC -> ODR &= ~(GPIO_ODR_0);
		GPIOC -> ODR &= ~(GPIO_ODR_1);
		GPIOC -> ODR |= GPIO_ODR_2;

		micro_wait(2000000);
		pwm(3,180);
		micro_wait(1000000);
	}
	micro_wait(2000);
	flap = 0;



	return;
}

void USART1_IRQHandler(void) {
	int i = 0;
	char tmp = 0;
	char buf[MAX_BUF_SIZE];
	int barcode = 0;
	int temp_compartment;
	int mask_bits;
	/**for(int x= 0; x <= MAX_BUF_SIZE; x++) {  //every element of buf is set to ' '
		buf[x] = ' ';
	}**/
	while((tmp != '\r') && (tmp != '\n')) {
		tmp = UART_Recive();	//receive a char
		buf[i++] = tmp;	// write to buffer
	}
	/**
	if((tmp == '\r') || (tmp == '\n'))  //checks for newline or carriage return
	{**/

	//i = 0;
	sscanf(buf, "%d", &barcode);  //possibly a better alternative? Yes, it is
	mask_bits = barcode % 1000;
	flap = barcode & 0x3;
	//storeBarcode = store barcode data in linked list
	//choose_compartment_pwm = change pwm/servo motor arm angle
	//temp_compartment = storeBarcode(barcode, mask_bits);
	//}
	/**for(int x= 0; x <= MAX_BUF_SIZE; x++) {  //every element of buf is set to ' '
		buf[x] = ' ';
	}**/
}



void interrupt_init(void) {
	NVIC -> ISER[0] = (1 << USART1_IRQn); //reserved for second USART?
	NVIC -> ISER[0] = (1 << I2C1_IRQn); //reserved for other I2C
	NVIC -> ISER[0] = (1 << TIM2_IRQn); //interrupt for timer 8
}

void init_led_test(void) {
	RCC -> AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC -> MODER &= ~(GPIO_MODER_MODER0);
	GPIOC -> MODER &= ~(GPIO_MODER_MODER1);
	GPIOC -> MODER &= ~(GPIO_MODER_MODER2);
	GPIOC -> MODER |= GPIO_MODER_MODER0_0;
	GPIOC -> MODER |= GPIO_MODER_MODER1_0;
	GPIOC -> MODER |= GPIO_MODER_MODER2_0;
}

int get_compartment(void){
	return 3 & compartment_record;
}


void TIM6_DAC_IRQHandler() {
	//for testing
	int counter_global = counter;
	int queue = compartment_record;
	int distance = distance_sensor();
	//int compartment_num = get_compartment();
	int threshold_val;
	//int threshold_val = 500;

	if (flap == 2) {
		threshold_val = 150;//343
	}
	else if (flap == 1) {
		threshold_val = 520;//711
	}
	else if (flap == 3) {
		threshold_val = 700;//1155
	}
	else {
		TIM6 -> SR &= ~TIM_SR_UIF;  //clear interrupt
		return;
	}
	if (distance >= threshold_val && distance < 1800) {//was greater than
		choose_compartment_pwm(0);
	}
	TIM6 -> SR &= ~TIM_SR_UIF;  //clear interrupt
}


void setup_timers(void){
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	TIM6->PSC = 47;
	TIM6->ARR = 50000-1;
	TIM6->DIER |= TIM_DIER_UIE;
	TIM6->CR1 |= TIM_CR1_ARPE;
	TIM6->CR1 |= TIM_CR1_CEN;
	NVIC->ISER[0] |= (1<<TIM6_DAC_IRQn);
	NVIC_SetPriority(17,1);
}


int main(void)
{
    USART_Configuration();
    init_led_test();

    //interrupt_init();
	setup_gpio();  //from pwm.c
	setup_pwm();  //from pwm.c
	init_I2C1();  //from distance.c
	setup_gpioc(); //from distance.c
	setup_timers();
    TIM2->CCR4 = 75;  //for belt

	while(1)
	{
		;
	}
}

