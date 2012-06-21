#ifndef UARTUTIL_H
#define UARTUTIL_H
//---------------------------------------------------------------------
// UART
// By P Hunt 9/11/11
//---------------------------------------------------------------------
/* Received data is stored in array Buf  */
char Buf[80];
char *Receiveddata = Buf;
/* This is UART1 receive ISR */

void __attribute__((__interrupt__)) _U1RXInterrupt(void)

{

    IFS0bits.U1RXIF = 0;

/* Read the receive buffer till atleast one or more character can be read */ 
	
    while( DataRdyUART1())

      {

        ( *( Receiveddata)++) = ReadUART1();
		printf("Data Read %s  ",*( Receiveddata));

    } 
printf("\n");
}  
void uart_init(void)
{


// Setup interrupt
ConfigIntUART1(UART_RX_INT_EN & UART_RX_INT_PR6 & UART_TX_INT_DIS);
//specify that we want printf() to redirect to UART1
int __C30_UART = 1;

	//dsPIC33FJ* does not have dedicated ports for UART, we need to assign the pin's we're going to use


	//setup remapable pins for UART function
	_TRIS(pinTX) = 1;


	__builtin_write_OSCCONL(OSCCON & ~(1<<6));		//unlock registers
	RPOR5bits.RP10R = 0b00011;	//U1TX = RP11	0b00011 means RPn tied to UART1 Transmit
	RPINR18bits.U1RXR = 11;   	//U1RX = RP10 	15 specifies RPn pin used for UART1 Receive
	__builtin_write_OSCCONL(OSCCON | (1<<6));		//lock registers
//#define MasterClock 40000000
//int Baud;
// Baud =(40000000/(16 *19200)-1); // 19200bps

	OpenUART1(
		UART_EN &  UART_IDLE_CON & UART_IrDA_DISABLE & UART_MODE_FLOW & UART_UEN_00 &  UART_DIS_WAKE &  
			UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE & UART_NO_PAR_8BIT & UART_BRGH_SIXTEEN & UART_1STOPBIT , 
		UART_INT_TX & UART_IrDA_POL_INV_ZERO & UART_SYNC_BREAK_DISABLED &  UART_TX_ENABLE &  UART_INT_RX_CHAR &  UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR,
 
		//129		// ~=  40Mhz / (16 * 19200bps) - 1 
		//Baud		// ~=  40Mhz / (16 * 9600bps) - 1   with  1.4% error
//259// 9600 Baud
21//115200 Baud
	);

	__delay_ms(10); //UART requires some delay (at least one bit time) before sending data
/* Received data is stored in array Buf  */

}




void print_float_list(int n,float *a){
	int i;
	for(i=0;i<n;i++){
		printf("%f",(double)a[i]);
		if(i<n-1) printf(",");
	}
}

void uart_send_char(char c){
	WriteUART1(c);
	while(BusyUART1());
}




//HDLC encoding http://www.tinyos.net/tinyos-2.x/doc/html/tep113.html#hdlc
#define HDLC_ESC	0x7D	//escape char	
#define HDLC_SEP	0x7E	//packet separator char
#define HDLC_XOR	0x20	//xor char


void hdlc_send_sep(){
	uart_send_char(HDLC_SEP);
}

void hdlc_send_byte(unsigned char c ){
	if(HDLC_SEP == c || HDLC_ESC == c){
		uart_send_char(HDLC_ESC);
		uart_send_char(c ^ HDLC_XOR);
	}else{
		uart_send_char(c);
	}
}

void hdlc_send_word(unsigned int w){
	//send Most Significat Byte First
	hdlc_send_byte(w >> 8 );
	hdlc_send_byte(w & 0xFF);
}


void hdlc_send_float(float f){
	unsigned char* p = (unsigned char*)(&f);
	hdlc_send_byte(*p);p++;
	hdlc_send_byte(*p);p++;
	hdlc_send_byte(*p);p++;
	hdlc_send_byte(*p);
}

#endif
