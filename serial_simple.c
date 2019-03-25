
#define UART_RESP_ERROR '!'             
#define UART_RESP_OK '+'
#define UART_CMD_SET_CALIBR_POINT 'C'
#define UART_CMD_ENABLE_DUMPOUT 'P'
#define UART_CMD_DISABLE_DUMPOUT 'D'

#define MODE_CMD_SET_CALIBR_POINT 1
uint8_t bSerialDumpEnable = 0;

volatile uint8_t uart_rx_tmp=0 ;   //receive
volatile uint8_t uart_tx_tmp=0 ;  //transmitpuffer

uint8_t iMode = 0;

ISR (USART_RXC_vect) 
{
	/* If error, responde error */
	if ( (1<<FE)|(1<<DOR)|(1<<PE) )
	uart_tx_tmp = UART_RESP_ERROR;   
    // Daten auslesen, dadurch wird das Interruptflag gelöscht              
    uart_rx_tmp = UDR;
}

 
ISR (USART_UDRE_vect) 
{
    if (uart_tx_tmp == 0) {        
        UCSRB &= ~(1<<UDRIE);       // UDRE Interrupt ausschalten 
    }
    else 
	{	
		uart_tx_tmp = 0;
		UDR = uart_tx_tmp; 
	}
} 


void uart_init(void)
{
 
#ifndef F_CPU
#error "F_CPU not defined";
#endif
#define UART_BAUD_RATE 9600
 
// Hilfsmakro zur UBRR-Berechnung ("Formel" laut Datenblatt)
#define UART_UBRR_CALC(BAUD_,FREQ_) ((FREQ_)/((BAUD_)*16L)-1)

    UCSRB |= (1<<RXCIE) | (1<<TXEN) | (1<<RXEN);    // UART TX und RX einschalten
    UCSRC |= (3<<UCSZ0);    // Asynchron 8N1 
 
    UBRRH = (uint8_t)( UART_UBRR_CALC( UART_BAUD_RATE, F_CPU ) >> 8 );
    UBRRL = (uint8_t)UART_UBRR_CALC( UART_BAUD_RATE, F_CPU );
}


void TransmitByte (unsigned char data)
{
  // Wait for empty transmit buffer 
  while (!(UCSRA & (1 << UDRE)));

  UDR = data;
}

 

void usart_write_str(char *str)
{
	
	while (*str)
	{
		TransmitByte(*str++);
	}
}



inline void check_uart(void){
		if (uart_rx_tmp != 0)
		{	//something received
			if (iMode & (UART_CMD_SET_CALIBR_POINT))
			{
				if (uart_rx_tmp >= '0' && uart_rx_tmp < '9')
				{
					TransmitByte(uart_rx_tmp);					
					uint8_t setval = uart_rx_tmp - 0x30  ;
					//itoa( setval, c_dbg, 10); 
					//DEBUG("\n");					
					//DEBUG(c_dbg);
					//DEBUG(" -- ");
				}
				
			}
			
			switch (uart_rx_tmp)
			{
				case UART_CMD_SET_CALIBR_POINT:
				iMode = MODE_CMD_SET_CALIBR_POINT;
				break;
				case UART_CMD_ENABLE_DUMPOUT:
				bSerialDumpEnable = 1;
				break;
				case UART_CMD_DISABLE_DUMPOUT:
				bSerialDumpEnable = 0;
				break;
				default:
				break;
			}
			TransmitByte(uart_rx_tmp); // echo
			uart_rx_tmp  = 0;
		}	
}
