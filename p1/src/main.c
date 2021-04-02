#include <stdlib.h>
#include "stm32f0xx.h"
#include "stm32f0_discovery.h"
#include "uart.h"

#define MAX_BUF_SIZE 	128

int main(void)
{
	int i = 0;
	int get_msg_flag = 0;
	char tmp = 0;
	char buf[MAX_BUF_SIZE];
	int barcode;
	
    USART_Configuration();
	
	while(1)
	{
		tmp = UART_Recive();	//receive a char
		buf[i++] = tmp;				// write to buffer
		if((tmp == '\r') || (tmp == '\n'))
		{
			i = 0;
			get_msg_flag = 1;
			
			//
			printf("Barcode = %s",buf);
			//barcode = atoi(buf);

			if (buf[0] != 0){

			}
			
			/*
			//proceed to mask
			if (barcode & 0x3 == 0x0){}
			else if (barcode & 0x3 == 0x1){}
			else if (barcode & 0x3 == 0x2){}
			else if (barcode & 0x3 == 0x3){}
			else{
				printf("unknown error. invalid barcode\n");
			}
			*/
		}
	}
}
