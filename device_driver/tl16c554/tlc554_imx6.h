#ifndef __EXTUART_DRV_H__
#define __EXTUART_DRV_H__

									// DIVxxxx = TL16C554_XTAL/(xxxx-baud * 16)
#define DIV9600				24		// 9600 bps divisor (0 error rate)
#define DIV19200			12		// 19200 bps divisor (0 error rate)
#define DIV38400			6		// 38400 bps divisor (0 error rate)
#define DIV57600			4		// 57600 bps divisor (0 error rate)
#define DIV115200			2		// 115200 bps divisor (0 error rate)


#define UART_MAGIC	't'

#define IOCTL_UART_DECOUNT	_IO(UART_MAGIC, 0)
#define IOCTL_UART_CONFIG	_IOWR(UART_MAGIC, 1, uart_info_t)
#define IOCTL_UART_FLUSH	_IOWR(UART_MAGIC, 2, uart_info_t)
#define IOCTL_UART_DIR		_IOWR(UART_MAGIC, 3, uart_info_t)
#define IOCTL_UART_MAX		6

#define EX_UART_NUMBER		12

#define UART_BUFFERSIZE		1024

enum{
	EXUART_232=0,
	EXUART_485
};

enum{
	EX_UART0=0,
	EX_UART1,
	EX_UART2,
	EX_UART3,
	EX_UART4,
	EX_UART5,
	EX_UART6,
	EX_UART7,
	EX_UART8,
	EX_UART9,
	EX_UART10,
	EX_UART11
};


typedef struct uart_info_t
{
	unsigned int baud;

	unsigned char data;
	unsigned char stop;
	unsigned char parity;
	unsigned char type;		// 232/485
	unsigned char dir;				// 485_tx/rx pin control (for 422 1 fix, for 485 1:tx 0:rx) 

	int opt;
	int ret;

}__attribute__((packed))uart_info_t;



#endif


