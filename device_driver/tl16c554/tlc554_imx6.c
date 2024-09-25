/*
 * File Name : tl16c554_imx6.c
 *
 * Copyright 2021 by pwk
 *
 * Developer : PWK (pwk10000@naver.com)
 *
 * Classify : Device-Driver
 *
 * Version : 1.00
 *
 * Created Date : 2021-12-24
 *
 * File Description : External UART Control Driver(Linux-3.0.xx)
 *
 * Release List
 * 2021-12-24 : 1st Release
 */


/****************************************************************
* tl16c554 driver. for imx6(linux-3.0.26)
* internal uart...
  UART 0 - Terminal Port
  UART 1 - 
  UART 2 - 

* external uart...(TL16C554 * 3)
 - CH 1
  ExUART 0 
  ExUART 1 
  ExUART 2 
  ExUART 3 

 - CH 2
  ExUART 4
  ExUART 5
  ExUART 6
  ExUART 7

 - CH 3
  ExUART 8
  ExUART 9
  ExUART 10
  ExUART 11
****************************************************************/


#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/rational.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <mach/dma.h>
#include <mach/hardware.h>
#include <mach/imx-uart.h>

#include <linux/gpio.h>
#include <mach/gpio.h>

#include "tlc16c554_imx6.h"

//-------------------------------------------------------------------//
#define printm(fmt,args...) printk("[%s,%s,%d]"fmt,__FILE__,__FUNCTION__,__LINE__,## args)

//#define DEBUG_ANDUART_DRV
#ifdef DEBUG_ANDUART_DRV 
#define dp(fmt,args...) printk(fmt,## args)
#define dlp(fmt,args...) printk("[%s,%s,%d]"fmt,__FILE__,__FUNCTION__,__LINE__,## args)
#else
#define dp(fmt,args...)
#define dlp(fmt,args...)
#endif
//-------------------------------------------------------------------//


#define EINTCS1A gpio_to_irq(IMX_GPIO_NR(4,5))
#define EINTCS1B gpio_to_irq(IMX_GPIO_NR(4,6))
#define EINTCS1C gpio_to_irq(IMX_GPIO_NR(4,7))
#define EINTCS1D gpio_to_irq(IMX_GPIO_NR(4,8))

#define EINTCS2A gpio_to_irq(IMX_GPIO_NR(4,11))
#define EINTCS2B gpio_to_irq(IMX_GPIO_NR(2,13))
#define EINTCS2C gpio_to_irq(IMX_GPIO_NR(5,19))
#define EINTCS2D gpio_to_irq(IMX_GPIO_NR(5,20))

#define EINTCS3A gpio_to_irq(IMX_GPIO_NR(2,11))
#define EINTCS3B gpio_to_irq(IMX_GPIO_NR(2,12))
#define EINTCS3C gpio_to_irq(IMX_GPIO_NR(6,15))
#define EINTCS3D gpio_to_irq(IMX_GPIO_NR(4,14))

#define GPIO2_DR	(GPIO2_BASE_ADDR + 0x00)
#define GPIO2_GDIR	(GPIO2_BASE_ADDR + 0x04)
#define GPIO2_PSR	(GPIO2_BASE_ADDR + 0x08)
#define GPIO2_ICR1	(GPIO2_BASE_ADDR + 0x0C)
#define GPIO2_ICR2	(GPIO2_BASE_ADDR + 0x10)
#define GPIO2_IMR	(GPIO2_BASE_ADDR + 0x14)
#define GPIO2_ISR	(GPIO2_BASE_ADDR + 0x18)

#define GPIO4_DR	(GPIO4_BASE_ADDR + 0x00)
#define GPIO4_GDIR	(GPIO4_BASE_ADDR + 0x04)
#define GPIO4_PSR	(GPIO4_BASE_ADDR + 0x08)
#define GPIO4_ICR1	(GPIO4_BASE_ADDR + 0x0C)
#define GPIO4_ICR2	(GPIO4_BASE_ADDR + 0x10)
#define GPIO4_IMR	(GPIO4_BASE_ADDR + 0x14)
#define GPIO4_ISR	(GPIO4_BASE_ADDR + 0x18)


#define GPIO5_DR	(GPIO5_BASE_ADDR + 0x00)
#define GPIO5_GDIR	(GPIO5_BASE_ADDR + 0x04)
#define GPIO5_PSR	(GPIO5_BASE_ADDR + 0x08)
#define GPIO5_ICR1	(GPIO5_BASE_ADDR + 0x0C)
#define GPIO5_ICR2	(GPIO5_BASE_ADDR + 0x10)
#define GPIO5_IMR	(GPIO5_BASE_ADDR + 0x14)
#define GPIO5_ISR	(GPIO5_BASE_ADDR + 0x18)

#define GPIO6_DR	(GPIO6_BASE_ADDR + 0x00)
#define GPIO6_GDIR	(GPIO6_BASE_ADDR + 0x04)
#define GPIO6_PSR	(GPIO6_BASE_ADDR + 0x08)
#define GPIO6_ICR1	(GPIO6_BASE_ADDR + 0x0C)
#define GPIO6_ICR2	(GPIO6_BASE_ADDR + 0x10)
#define GPIO6_IMR	(GPIO6_BASE_ADDR + 0x14)
#define GPIO6_ISR	(GPIO6_BASE_ADDR + 0x18)


#define WHILE_CNT		10000
/* #include <plat/gpio-cfg.h> */
#define STRCAT(ABC) #ABC

#define DRIVER_AUTHOR		"Worked by pwk"
#define DRIVER_VERSION		"1.00"
#define DRIVER_DESC			"16C554 Driver Version "DRIVER_VERSION

#define ANDUART_DEVNAME		"ttyTL"

#define ANDUART_MAJOR		204

#define TL16C554DEV_PHY_ADDR	0x08000000 
#define TL16C554_MEM_SIZE		0x01000000

// DI reg field
#define DI_RX_RDY			0x20			// RX_RDY
#define DI_TX_RDY			0x10			// TX_RDY

// Minor number
enum{
	CH_A_MINOR=100,
	CH_B_MINOR,
	CH_C_MINOR,
	CH_D_MINOR,
	CH_E_MINOR,
	CH_F_MINOR,
	CH_G_MINOR,
	CH_H_MINOR,
	CH_I_MINOR,
	CH_J_MINOR,
	CH_K_MINOR,
	CH_L_MINOR
};

#define CH_A_OFFSET		0x0
#define CH_B_OFFSET		0x100000
#define CH_C_OFFSET		0x200000
#define CH_D_OFFSET		0x300000
#define CH_E_OFFSET		0x400000
#define CH_F_OFFSET		0x500000
#define CH_G_OFFSET		0x600000
#define CH_H_OFFSET		0x700000
#define CH_I_OFFSET		0x800000
#define CH_J_OFFSET		0x900000
#define CH_K_OFFSET		0xA00000
#define CH_L_OFFSET		0xB00000

#define REG_RBR			0x0000		// Receive Holding Register (R/O) 
#define REG_THR			0x0000		// Transmit Holding Register (W/O)
#define REG_DLL			0x0000		// Divisor Latch Low 
#define REG_DLM			0x0004		// Divisor Latch Middle 
#define REG_IER			0x0004		// Interrupt Enable Register 
#define REG_FCR			0x0008		// FIFO Control register (W/O) 
#define REG_IIR			0x0008		// Interupt identification Register (R/O) 
#define REG_LCR			0x000C		// Line Control Register 
#define REG_MCR			0x0010		// Modem Control Register 
#define REG_LSR			0x0014		// Line Status register 
#define REG_MSR			0x0018		// Modem Status Register 
#define REG_SCR			0x001C		// Scratchpad Register

// LCR reg field
#define LCR_DLAB	0x80		// Divisor latch access bit
#define LCR_SETBK	0x40		// Set break
#define LCR_SP		0x20		// Stick parity
#define LCR_EPS		0x10		// Even-parity select
#define LCR_PEN		0x08		// Parity enable
#define LCR_STB		0x04		// Number of stop bits
#define LCR_WLSB1	0x02		// Word-length select bit 1
#define LCR_WLSB0	0x01		// Word-length select bit 0


// IER reg field
#define IER_EDSSI		0x08		// Enable modem status interrupt
#define IER_ERLSI		0x04		// Enable receiver line status interrupt
#define IER_ETBEI		0x02		// Enable transmitter holding register empty interrupt
#define IER_ERBI		0x01		// Enable received data available interrupt

// FCR reg field
#define FCR_RTH		0x80		// Receiver Trigger (MSB)
#define FCR_RTL		0x40		// Receiver Trigger (LSB)
#define FCR_DMS		0x08		// DMA mode select
#define FCR_TFS		0x04		// Transmit FIFO reset
#define FCR_RFS		0x02		// Receiver FIFO reset
#define FCR_FE		0x01		// FIFO Enable


// IIR reg data but not field
#define IIR_RDA		0x04		// Received data available
#define IIR_CTI		0x0C		// Character time-out indicator

// LSR reg field
#define LSR_THRE		0x20		// Transmitter holding register empty
#define LSR_TEMT		0x40		// Transmitter holding register(THR)와 Transmitter Shift register(TSR)가 동시에 비어있을 경우 세트

// baudrate
#define TL16C554_XTAL	3686400	// XTAL 3.6864MHz

#define  GPIO_NTL16C554_RESET			6

unsigned int devBaseAddress = 0; 

#define MAXUARTBUFSIZE 	(1024*8)

typedef struct uart_buf_info
{
	int head;
	int tail;
	unsigned char lq[MAXUARTBUFSIZE];
	
}__attribute__ ((packed))ubuf_t;

static ubuf_t A_rxb;
static ubuf_t B_rxb;
static ubuf_t C_rxb;
static ubuf_t D_rxb;
static ubuf_t E_rxb;
static ubuf_t F_rxb;
static ubuf_t G_rxb;
static ubuf_t H_rxb;
static ubuf_t I_rxb;
static ubuf_t J_rxb;
static ubuf_t K_rxb;
static ubuf_t L_rxb;

static char exUartType[EX_UART_NUMBER];				// 232/485
static unsigned int exUartBaud[EX_UART_NUMBER];				// 


// file operation entry function - wrapper 
/////////////////////////////////////////////////////////////////////////////////
static int TL16C554_Open(struct inode* a_pINode, struct file* a_pFile);
static struct file_operations TL16C554_fops = {
	.open	= TL16C554_Open,
};

static int TL16C554x_Open(struct inode* a_pINode, struct file* a_pFile);
static int TL16C554x_Release(struct inode* a_pINode, struct file* a_pFile);
static int TL16C554x_Ioctl(struct inode* a_pINode, struct file* a_pFile, unsigned int a_cmd, unsigned long a_arg);

// file operation entry function - CH A
static ssize_t TL16C554A_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554A_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static struct file_operations TL16C554A_fops = {
	.open	= TL16C554x_Open,
	.release	= TL16C554x_Release,
	.read	= TL16C554A_Read,
	.write	= TL16C554A_Write,
	.unlocked_ioctl	= TL16C554x_Ioctl,
};


// file operation entry function - CH B
static ssize_t TL16C554B_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554B_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554B_fops = {
	.open	= TL16C554x_Open,
	.release	= TL16C554x_Release,
	.read	= TL16C554B_Read,
	.write	= TL16C554B_Write,
	.unlocked_ioctl	= TL16C554x_Ioctl,
};


// file operation entry function - CH C
static ssize_t TL16C554C_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554C_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554C_fops = {
	.open	= TL16C554x_Open,
	.release	= TL16C554x_Release,
	.read	= TL16C554C_Read,
	.write	= TL16C554C_Write,
	.unlocked_ioctl	= TL16C554x_Ioctl,
};


// file operation entry function - CH D
static ssize_t TL16C554D_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554D_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554D_fops = {
	.open	= TL16C554x_Open,
	.release	= TL16C554x_Release,
	.read	= TL16C554D_Read,
	.write	= TL16C554D_Write,
	.unlocked_ioctl	= TL16C554x_Ioctl,
};

// file operation entry function - CH E
static ssize_t TL16C554E_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554E_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554E_fops = {
	.open	= TL16C554x_Open,
	.release	= TL16C554x_Release,
	.read	= TL16C554E_Read,
	.write	= TL16C554E_Write,
	.unlocked_ioctl	= TL16C554x_Ioctl,
};


// file operation entry function - CH F
static ssize_t TL16C554F_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554F_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554F_fops = {
	.open	= TL16C554x_Open,
	.release	= TL16C554x_Release,
	.read	= TL16C554F_Read,
	.write	= TL16C554F_Write,
	.unlocked_ioctl	= TL16C554x_Ioctl,
};


// file operation entry function - CH G
static ssize_t TL16C554G_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554G_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554G_fops = {
	.open	= TL16C554x_Open,
	.release	= TL16C554x_Release,
	.read	= TL16C554G_Read,
	.write	= TL16C554G_Write,
	.unlocked_ioctl	= TL16C554x_Ioctl,
};


// file operation entry function - CH H
static ssize_t TL16C554H_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554H_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554H_fops = {
	.open	= TL16C554x_Open,
	.release	= TL16C554x_Release,
	.read	= TL16C554H_Read,
	.write	= TL16C554H_Write,
	.unlocked_ioctl	= TL16C554x_Ioctl,
};

#if 1
// file operation entry function - CH I
static ssize_t TL16C554I_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554I_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554I_fops = {
	.open	= TL16C554x_Open,
	.release	= TL16C554x_Release,
	.read	= TL16C554I_Read,
	.write	= TL16C554I_Write,
	.unlocked_ioctl	= TL16C554x_Ioctl,
};

// file operation entry function - CH J
static ssize_t TL16C554J_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554J_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554J_fops = {
	.open	= TL16C554x_Open,
	.release	= TL16C554x_Release,
	.read	= TL16C554J_Read,
	.write	= TL16C554J_Write,
	.unlocked_ioctl	= TL16C554x_Ioctl,
};


// file operation entry function - CH K
static ssize_t TL16C554K_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554K_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554K_fops = {
	.open	= TL16C554x_Open,
	.release	= TL16C554x_Release,
	.read	= TL16C554K_Read,
	.write	= TL16C554K_Write,
	.unlocked_ioctl	= TL16C554x_Ioctl,
};


// file operation entry function - CH L
static ssize_t TL16C554L_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554L_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554L_fops = {
	.open	= TL16C554x_Open,
	.release	= TL16C554x_Release,
	.read	= TL16C554L_Read,
	.write	= TL16C554L_Write,
	.unlocked_ioctl	= TL16C554x_Ioctl,
};
#endif

/////////////////////////////////////////////////////////////////////////////////

int Ubuf_Rx_Insert(ubuf_t *ub, unsigned char dat);
int Ubuf_Rx_Pullout(ubuf_t *ub, unsigned char *dat, int *nlen);

static unsigned int GetOffsetMinor(unsigned int minor)
{
	switch(minor){
		case CH_A_MINOR:
			return CH_A_OFFSET;
		case CH_B_MINOR:
			return CH_B_OFFSET;
		case CH_C_MINOR:
			return CH_C_OFFSET;
		case CH_D_MINOR:
			return CH_D_OFFSET;
		case CH_E_MINOR:
			return CH_E_OFFSET;
		case CH_F_MINOR:
			return CH_F_OFFSET;
		case CH_G_MINOR:
			return CH_G_OFFSET;
		case CH_H_MINOR:
			return CH_H_OFFSET;
		case CH_I_MINOR:
			return CH_I_OFFSET;
		case CH_J_MINOR:
			return CH_J_OFFSET;
		case CH_K_MINOR:
			return CH_K_OFFSET;
		case CH_L_MINOR:
			return CH_L_OFFSET;
		default:
			printm("Wrong minor number[%d]\n",minor);
			return 0;
	}

}

static int GetBaudrate(unsigned int baudrate)
{
	switch(baudrate){
		case 9600:
			return DIV9600;
		case 19200:
			return DIV19200;
		case 38400:
			return DIV38400;
		case 57600:
			return DIV57600;
		case 115200:
			return DIV115200;
		default:
			printm("Wrong baudrate[%d]\n",baudrate);
			return 0;
	}
}

static void SetUartType(unsigned char port,unsigned int baudrate,char type)
{
	if(port < EX_UART_NUMBER)
	{
		exUartType[port] = type;
		exUartBaud[port] = baudrate;
		dlp("Ex Uart Port[%d](COM%d)[%s]\n",port,port+3,type ? "RS-485":"RS-232");
	}
	else
	{
		printm("Wrong Port[%d]\n",port);
	}
}

static ubuf_t* SelRxBufPort(unsigned char port)
{
	switch(port){
		case EX_UART0:
			return &A_rxb;
		case EX_UART1:
			return &B_rxb;
		case EX_UART2:
			return &C_rxb;
		case EX_UART3:
			return &D_rxb;
		case EX_UART4:
			return &E_rxb;
		case EX_UART5:
			return &F_rxb;
		case EX_UART6:
			return &G_rxb;
		case EX_UART7:
			return &H_rxb;
		case EX_UART8:
			return &I_rxb;
		case EX_UART9:
			return &J_rxb;
		case EX_UART10:
			return &K_rxb;
		case EX_UART11:
			return &L_rxb;
		default:
			printm("Wrong serial port[%d]\n",port);
			return 0;
	}
}

static int InitQueue(unsigned int minor)
{
	unsigned int position = 0;
	ubuf_t* bufPtr = 0;

	position = minor - CH_A_MINOR;

	if((position >= 0) && (position < EX_UART_NUMBER))
	{
		bufPtr = SelRxBufPort(position);

		bufPtr->head = 0;
		bufPtr->tail = 0;
	}
	else
	{
		printm("## Ex Uart init queue fault - minor[%d]\n",minor);
	}
	return 0;
}

static void ConfigExUart(unsigned int offset,unsigned int baudrate, char type)
{
	// set LCR's bit 7
	outb(LCR_DLAB,(devBaseAddress + offset + REG_LCR));

	// program the divisor register (DLL and DLM)
	outb(0,(devBaseAddress + offset + REG_DLM));
	outb(GetBaudrate(baudrate),(devBaseAddress + offset + REG_DLL));

	// clear LCR's bit 7
	outb(0x00,(devBaseAddress + offset + REG_LCR));

	// config LCR to 8bit, 1stop, no parity
	outb((LCR_WLSB1|LCR_WLSB0),(devBaseAddress + offset + REG_LCR));

	// config FCR to enable FIFO
	outb(FCR_FE,(devBaseAddress + offset + REG_FCR));

	// config IER to Enable Rx interrupt
	outb(IER_ERBI,(devBaseAddress + offset + REG_IER));

	//dlp("offset[0x%x] IER[0x%x]\n",offset,inb(devBaseAddress + offset + REG_IER));

	// config MCR to don't use
	outb(0x00,(devBaseAddress + offset + REG_MCR));

	if(type == EXUART_232)
	{
		// config MCR to don't use
		outb(0x00,(devBaseAddress + offset + REG_MCR));
		//printk("Ex UART [%d] is RS-232\n",port);
	}
	else
	{
		// for 485 - Rx Enable(Low)
		outb(0x02,(devBaseAddress + offset + REG_MCR));   // RTS is Low 

		//printk("Ex UART [%d] is RS-485\n",port);
	}

#if 1
	// clear IIR
	inb(devBaseAddress + offset + REG_IIR);
	// clear LSR
	inb(devBaseAddress + offset + REG_LSR);
#endif
}

static int SetUartChannel(unsigned int baseAddress,unsigned int minor,int baudrate,char type)
{
	int offset = 0;

	//dp("TL16C554 set uart - minor[%d]\n",minor);

	if( (minor < CH_A_MINOR) || (minor > CH_L_MINOR))
	{
		printm("TL16C554 set uart fail - minor[%d]\n",minor);
		return -1;
	}

	offset = GetOffsetMinor(minor);

	ConfigExUart(offset,baudrate,type);
	SetUartType(minor - CH_A_MINOR,baudrate,type);

	return 1;
}



// ---------------- file operation function wrapper block
/*********************************************************************
  **********************************************************************/
int TL16C554_Open(struct inode* a_pINode, struct file* a_pFile)
{
//	printk("TL16C554_Open #\n");
	switch( MINOR(a_pINode->i_rdev))
	{
		case CH_A_MINOR:
			a_pFile->f_op = &TL16C554A_fops;
			break;
		case CH_B_MINOR:
			a_pFile->f_op = &TL16C554B_fops;
			break;
		case CH_C_MINOR:
			a_pFile->f_op = &TL16C554C_fops;			
			break;
		case CH_D_MINOR:
			a_pFile->f_op = &TL16C554D_fops;			
			break;
		case CH_E_MINOR:
			a_pFile->f_op = &TL16C554E_fops;			
			break;
		case CH_F_MINOR:
			a_pFile->f_op = &TL16C554F_fops;			
			break;
		case CH_G_MINOR:
			a_pFile->f_op = &TL16C554G_fops;			
			break;
		case CH_H_MINOR:
			a_pFile->f_op = &TL16C554H_fops;
			break;
		case CH_I_MINOR:
			a_pFile->f_op = &TL16C554I_fops;
			break;
		case CH_J_MINOR:
			a_pFile->f_op = &TL16C554J_fops;
			break;
		case CH_K_MINOR:
			a_pFile->f_op = &TL16C554K_fops;
			break;
		case CH_L_MINOR:
			a_pFile->f_op = &TL16C554L_fops;
			break;
		default:
			return -ENXIO;
	}

	if( (a_pFile->f_op != NULL) && (a_pFile->f_op->open != NULL) ) 
		return a_pFile->f_op->open(a_pINode, a_pFile);
	else 
		return -ENXIO;
}



int TL16C554x_Open(struct inode* a_pINode, struct file* a_pFile)
{
	unsigned int offset = 0;
	unsigned int minor = 0;

	minor = MINOR(a_pINode->i_rdev);
	offset = GetOffsetMinor(minor);

	//dlp("TL16C554x_Open - offset[0x%x]\n",offset);

	ConfigExUart(offset,9600,EXUART_232);
	SetUartType(minor - CH_A_MINOR,9600,EXUART_232);

	InitQueue(minor);

	return 0;
}
/*********************************************************************
**********************************************************************/
int TL16C554x_Release(struct inode* a_pINode, struct file* a_pFile)
{
//	printk("\nTL16C554A_Release \n");

	return 0;
}

/*********************************************************************
**********************************************************************/
int TL16C554x_Ioctl(struct inode* a_pINode, struct file* a_pFile, unsigned int a_cmd, unsigned long a_arg)
{
	int reVal = -1;
	int err = 0;

#if 1
	int size;
	if( _IOC_TYPE(a_cmd) != UART_MAGIC ) 
	{
		printm("## Ex Uart  - ioctl fail : MAGIC\n");
		return reVal;
	}
	if( _IOC_NR(a_cmd) >= IOCTL_UART_MAX )
	{
		printm("## Ex Uart  - ioctl fail :  UART MAX\n");
		return reVal;
	}

	size = _IOC_SIZE(a_cmd);

	if( _IOC_DIR(a_cmd) & _IOC_READ )
	{
		if(access_ok(VERIFY_WRITE,(void*)a_arg,size) < 0)
		{
			printm("## Ex Uart  - ioctl fail : write area \n");
			return -EINVAL;
		}
	}

	if( _IOC_DIR(a_cmd) & _IOC_WRITE )
	{
		if(access_ok( VERIFY_READ, (void *)a_arg, size) < 0)
		{
			printm("## Ex Uart  - ioctl fail : read area \n");
			return -EINVAL;
		}
	}
#endif
	switch(a_cmd)
	{
		case IOCTL_UART_DECOUNT:
			reVal = 1;
			break;
		case IOCTL_UART_CONFIG:
			{
				uart_info_t uartInfo;
				unsigned int minor = 0;
				
				//printm("Ex UART - ioctl config\n");

				minor = MINOR(a_pINode->i_rdev);

				if((err = copy_from_user((void *)&uartInfo, (void *)a_arg, sizeof(uart_info_t))) < 0)
				{
					printm("## Ex Uart config fail - copy_from_user() minor[%d]\n",minor);
					break;
				}

				if(SetUartChannel(devBaseAddress,minor,uartInfo.baud,uartInfo.type) < 0)
				{
					reVal = -1;
				}
				else
				{
					reVal = 1;
				}
				//printm("Ex UART - ioctl config end\n");
			}
			break;
		case IOCTL_UART_FLUSH:
			// buffer clear	
			reVal = 1;
			break;
		case IOCTL_UART_DIR:
			reVal = 1;
			break;
		default : 
			printm("## ioctl fault - cmd[%d]\n",a_cmd);
			break;
	}

	return reVal;

}

static int ReadData(char* dst, ubuf_t* src, int len, char* temp)
{
	int err = 0;

	Ubuf_Rx_Pullout(src, temp, &len);          // 상태값 & 길이값 return.

	if( len != 0 )
		err = copy_to_user((void *)dst, (void *)temp, len);
	else
		return 0;

	if(err != 0)
	{
		printm("## Ex Uart read - fail copy_to_user\n");
		return -1;
	}
	else
		return len;

}

static int WriteData(const char* a_pBuf,size_t a_length,unsigned int offset,char* temp,unsigned int baudrate,char type)
{
	int err = 0;
	int i;
	unsigned char reg = 0;

	int whileCnt = WHILE_CNT;

	err = copy_from_user((void *)temp, (void *)a_pBuf, a_length);

	if(err != 0)
	{
		printm("## Ex Uart write - fail copy_from_user, offset[0x%x]\n",offset);
		return -1;
	}

	if(type == EXUART_485)
	{
		// config IER to Disable Rx interrupt
		outb(0,(devBaseAddress + offset + REG_IER));
		outb(0x00,(devBaseAddress + offset + REG_MCR));   // RTS is High
	}

	for(i = 0; i < a_length ; i++)
	{

		while(whileCnt)
		{
			reg = readb(devBaseAddress + offset + REG_LSR);
			if( (reg & LSR_THRE) == LSR_THRE )
				break;

			whileCnt--;
		}

		if(whileCnt == 0)
		{
			printm("## Ex Uart write - tx timeout\n");
		}

		outb(temp[i],(devBaseAddress + offset + REG_THR));
		whileCnt = WHILE_CNT;
	}

	if(type == EXUART_485)
	{
		whileCnt = WHILE_CNT;
		while(whileCnt)
		{
			reg = readb(devBaseAddress + offset + REG_LSR);
			if( (reg & LSR_TEMT) == LSR_TEMT)
				break;
			whileCnt--;
		}

		if(whileCnt == 0)
		{
			printm("## Ex Uart write - last tx data timeout\n");
		}

		if(baudrate > 19200)
		{
			mdelay(1);
		}
		else
		{
			mdelay(2);
		}
		outb(0x02,(devBaseAddress + offset + REG_MCR));   // RTS is Low

		// config FCR to enable FIFO
		outb(FCR_FE | FCR_RFS ,(devBaseAddress + offset + REG_FCR));

		// config IER to Enable Rx interrupt
		outb(IER_ERBI,(devBaseAddress + offset + REG_IER));
	}

	return a_length;
}

// ---------------- individual file operation function for CH A
/*********************************************************************
**********************************************************************/
ssize_t TL16C554A_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char rxb_usr[UART_BUFFERSIZE] = {0,};
	return ReadData(a_pBuf,&A_rxb,a_length,rxb_usr);
}

ssize_t TL16C554A_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char txb_usr[UART_BUFFERSIZE] = {0,};
	return WriteData(a_pBuf,a_length,CH_A_OFFSET,txb_usr,exUartBaud[EX_UART0],exUartType[EX_UART0]);
}

// ---------------- individual file operation function for CH B
/*********************************************************************
**********************************************************************/
ssize_t TL16C554B_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char rxb_usr[UART_BUFFERSIZE] = {0,};
	return ReadData(a_pBuf,&B_rxb,a_length,rxb_usr);
}

/*********************************************************************
**********************************************************************/
ssize_t TL16C554B_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char txb_usr[UART_BUFFERSIZE] = {0,};
	return WriteData(a_pBuf,a_length,CH_B_OFFSET,txb_usr,exUartBaud[EX_UART1],exUartType[EX_UART1]);
}

// ---------------- individual file operation function for CH C
/*********************************************************************
**********************************************************************/
ssize_t TL16C554C_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char rxb_usr[UART_BUFFERSIZE] = {0,};
	return ReadData(a_pBuf,&C_rxb,a_length,rxb_usr);
}

/*********************************************************************
**********************************************************************/
ssize_t TL16C554C_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char txb_usr[UART_BUFFERSIZE] = {0,};
	return WriteData(a_pBuf,a_length,CH_C_OFFSET,txb_usr,exUartBaud[EX_UART2],exUartType[EX_UART2]);
}

// ---------------- individual file operation function for CH D
/*********************************************************************
**********************************************************************/
ssize_t TL16C554D_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char rxb_usr[UART_BUFFERSIZE] = {0,};
	return ReadData(a_pBuf,&D_rxb,a_length,rxb_usr);
}

/*********************************************************************
**********************************************************************/
ssize_t TL16C554D_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char txb_usr[UART_BUFFERSIZE] = {0,};
	return WriteData(a_pBuf,a_length,CH_D_OFFSET,txb_usr,exUartBaud[EX_UART3],exUartType[EX_UART3]);
}

// ---------------- individual file operation function for CH E
/*********************************************************************
**********************************************************************/
ssize_t TL16C554E_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char rxb_usr[UART_BUFFERSIZE] ={0,};
	return ReadData(a_pBuf,&E_rxb,a_length,rxb_usr);
}
/*********************************************************************
**********************************************************************/
ssize_t TL16C554E_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char txb_usr[UART_BUFFERSIZE] = {0,};
	return WriteData(a_pBuf,a_length,CH_E_OFFSET,txb_usr,exUartBaud[EX_UART4],exUartType[EX_UART4]);
}

// ---------------- individual file operation function for CH F
/*********************************************************************
**********************************************************************/
ssize_t TL16C554F_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char rxb_usr[UART_BUFFERSIZE] = {0,};
	return ReadData(a_pBuf,&F_rxb,a_length,rxb_usr);
}

/*********************************************************************
**********************************************************************/
ssize_t TL16C554F_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char txb_usr[UART_BUFFERSIZE] = {0,};
	return WriteData(a_pBuf,a_length,CH_F_OFFSET,txb_usr,exUartBaud[EX_UART5],exUartType[EX_UART5]);
}

// ---------------- individual file operation function for CH G
/*********************************************************************
**********************************************************************/
ssize_t TL16C554G_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char rxb_usr[UART_BUFFERSIZE] = {0,};
	return ReadData(a_pBuf,&G_rxb,a_length,rxb_usr);
}

/*********************************************************************
**********************************************************************/
ssize_t TL16C554G_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char txb_usr[UART_BUFFERSIZE] = {0,};
	return WriteData(a_pBuf,a_length,CH_G_OFFSET,txb_usr,exUartBaud[EX_UART6],exUartType[EX_UART6]);
}

// ---------------- individual file operation function for CH H
/*********************************************************************
**********************************************************************/
ssize_t TL16C554H_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char rxb_usr[UART_BUFFERSIZE] = {0,};
	return ReadData(a_pBuf,&H_rxb,a_length,rxb_usr);
}

/*********************************************************************
**********************************************************************/
ssize_t TL16C554H_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char txb_usr[UART_BUFFERSIZE] = {0,};
	return WriteData(a_pBuf,a_length,CH_H_OFFSET,txb_usr,exUartBaud[EX_UART7],exUartType[EX_UART7]);
}

// ---------------- individual file operation function for CH I
/*********************************************************************
**********************************************************************/
ssize_t TL16C554I_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char rxb_usr[UART_BUFFERSIZE] = {0,};
	return ReadData(a_pBuf,&I_rxb,a_length,rxb_usr);
}

/*********************************************************************
**********************************************************************/
ssize_t TL16C554I_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char txb_usr[UART_BUFFERSIZE] = {0,};
	return WriteData(a_pBuf,a_length,CH_I_OFFSET,txb_usr,exUartBaud[EX_UART8],exUartType[EX_UART8]);
}

// ---------------- individual file operation function for CH J
/*********************************************************************
**********************************************************************/
ssize_t TL16C554J_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char rxb_usr[UART_BUFFERSIZE] = {0,};
	return ReadData(a_pBuf,&J_rxb,a_length,rxb_usr);
}

/*********************************************************************
**********************************************************************/
ssize_t TL16C554J_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char txb_usr[UART_BUFFERSIZE] = {0,};
	return WriteData(a_pBuf,a_length,CH_J_OFFSET,txb_usr,exUartBaud[EX_UART9],exUartType[EX_UART9]);
}

// ---------------- individual file operation function for CH K
/*********************************************************************
**********************************************************************/
ssize_t TL16C554K_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char rxb_usr[UART_BUFFERSIZE] = {0,};
	return ReadData(a_pBuf,&K_rxb,a_length,rxb_usr);
}

/*********************************************************************
**********************************************************************/
ssize_t TL16C554K_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char txb_usr[UART_BUFFERSIZE] = {0,};
	return WriteData(a_pBuf,a_length,CH_K_OFFSET,txb_usr,exUartBaud[EX_UART10],exUartType[EX_UART10]);
}

// ---------------- individual file operation function for CH L
/*********************************************************************
**********************************************************************/
ssize_t TL16C554L_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char rxb_usr[UART_BUFFERSIZE] = {0,};
	return ReadData(a_pBuf,&L_rxb,a_length,rxb_usr);
}

/*********************************************************************
**********************************************************************/
ssize_t TL16C554L_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset)
{
	unsigned char txb_usr[UART_BUFFERSIZE] = {0,};
	return WriteData(a_pBuf,a_length,CH_L_OFFSET,txb_usr,exUartBaud[EX_UART11],exUartType[EX_UART11]);
}

//--------------------------------------------------------------------- private 함수.
//--------------------------------------------------------------------- private 함수.
//--------------------------------------------------------------------- private 함수.

/*********************************************************************
**********************************************************************/

static void PutRxData(ubuf_t* bufPtr,unsigned int offset)
{
	unsigned char reg = 0;
	unsigned char dat = 0;

	reg = inb(devBaseAddress + offset + REG_IIR) & 0x0F;

	if( (reg == IIR_RDA) ||(reg == IIR_CTI) )
	{
		dat = readb(devBaseAddress + offset + REG_RBR);
		Ubuf_Rx_Insert(bufPtr, dat);
	}
	else
	{
		dlp("read pass - offset[0x%x]\n",offset);
	}

}
static irqreturn_t ISR_chA_rx(int irq,void *dev_id)
{
	long flags;

	local_irq_save(flags);
	PutRxData(&A_rxb,CH_A_OFFSET);
	local_irq_restore(flags);

	return IRQ_HANDLED;

}

static irqreturn_t ISR_chB_rx(int irq,void *dev_id)
{
	long flags;

	local_irq_save(flags);
	PutRxData(&B_rxb,CH_B_OFFSET);
	local_irq_restore(flags);

	return IRQ_HANDLED;

}

static irqreturn_t ISR_chC_rx(int irq,void *dev_id)
{
	long flags;

	local_irq_save(flags);
	PutRxData(&C_rxb,CH_C_OFFSET);
	local_irq_restore(flags);

	return IRQ_HANDLED;

}


static irqreturn_t ISR_chD_rx(int irq,void *dev_id)
{
	long flags;

	local_irq_save(flags);
	PutRxData(&D_rxb,CH_D_OFFSET);
	local_irq_restore(flags);

	return IRQ_HANDLED;

}

static irqreturn_t ISR_chE_rx(int irq,void *dev_id)
{
	long flags;

	local_irq_save(flags);
	PutRxData(&E_rxb,CH_E_OFFSET);
	local_irq_restore(flags);

	return IRQ_HANDLED;

}

static irqreturn_t ISR_chF_rx(int irq,void *dev_id)
{
	long flags;

	local_irq_save(flags);
	PutRxData(&F_rxb,CH_F_OFFSET);
	local_irq_restore(flags);

	return IRQ_HANDLED;

}

static irqreturn_t ISR_chG_rx(int irq,void *dev_id)
{
	long flags;

	local_irq_save(flags);
	PutRxData(&G_rxb,CH_G_OFFSET);
	local_irq_restore(flags);

	return IRQ_HANDLED;

}


static irqreturn_t ISR_chH_rx(int irq,void *dev_id)
{
	long flags;

	local_irq_save(flags);
	PutRxData(&H_rxb,CH_H_OFFSET);
	local_irq_restore(flags);

	return IRQ_HANDLED;

}
static irqreturn_t ISR_chI_rx(int irq,void *dev_id)
{
	long flags;

	local_irq_save(flags);
	PutRxData(&I_rxb,CH_I_OFFSET);
	local_irq_restore(flags);

	return IRQ_HANDLED;

}
static irqreturn_t ISR_chJ_rx(int irq,void *dev_id)
{
	long flags;

	local_irq_save(flags);
	PutRxData(&J_rxb,CH_J_OFFSET);
	local_irq_restore(flags);

	return IRQ_HANDLED;

}
static irqreturn_t ISR_chK_rx(int irq,void *dev_id)
{
	long flags;

	local_irq_save(flags);
	PutRxData(&K_rxb,CH_K_OFFSET);
	local_irq_restore(flags);

	return IRQ_HANDLED;

}
static irqreturn_t ISR_chL_rx(int irq,void *dev_id)
{
	long flags;

	local_irq_save(flags);
	PutRxData(&L_rxb,CH_L_OFFSET);
	local_irq_restore(flags);

	return IRQ_HANDLED;

}

static unsigned char GetPortFromIrq(int irq)
{
	switch(irq){
		case EINTCS1A:		// ExUART 0 //change irq number
			return EX_UART0;
		case EINTCS1B:		// ExUART 1
			return EX_UART1;
		case EINTCS1C:			// ExUART 2
			return EX_UART2;
		case EINTCS1D:			// ExUART 3
			return EX_UART3;
		case EINTCS2A:			// ExUART 4
			return EX_UART4;
		case EINTCS2B:			// ExUART 5
			return EX_UART5;
		case EINTCS2C:			// ExUART 6
			return EX_UART6;
		case EINTCS2D:			// ExUART 7
			return EX_UART7;
		case EINTCS3A:			// ExUART 8 
			return EX_UART8;
		case EINTCS3B:			// ExUART 9
			return EX_UART9;
		case EINTCS3C:			// ExUART 10
			return EX_UART10;
		case EINTCS3D:			// ExUART 11
			return EX_UART11;
		default:
			printm("## Unknown irq [%d]\n",irq);
			return -1;
	}

}

static int GetOffsetPort(unsigned char port)
{
	switch(port){
		case EX_UART0:
			return CH_A_OFFSET;
		case EX_UART1:
			return CH_B_OFFSET;
		case EX_UART2:
			return CH_C_OFFSET;
		case EX_UART3:
			return CH_D_OFFSET;
		case EX_UART4:
			return CH_E_OFFSET;
		case EX_UART5:
			return CH_F_OFFSET;
		case EX_UART6:
			return CH_G_OFFSET;
		case EX_UART7:
			return CH_H_OFFSET;
		case EX_UART8:
			return CH_I_OFFSET;
		case EX_UART9:
			return CH_J_OFFSET;
		case EX_UART10:
			return CH_K_OFFSET;
		case EX_UART11:
			return CH_L_OFFSET;
		default:
			printm("Wrong serial port[%d]\n",port);
			return 0;
	}

}



/*********************************************************************
	(in interrupt routine when received data)
**********************************************************************/
int Ubuf_Rx_Insert(ubuf_t *ub, unsigned char dat)
{
	if( (ub->head+1) % MAXUARTBUFSIZE == ub->tail )
	{
	}
	else
	{
		ub->lq[ub->head] = dat;

		ub->head = (ub->head + 1) % MAXUARTBUFSIZE;
	}

	return 0;

}


/*********************************************************************
( in App to read data)
**********************************************************************/
int Ubuf_Rx_Pullout(ubuf_t *ub, unsigned char *dat, int *nlen)
{
	int tmp = 0;
	int head = ub->head;					// to avoid sync error by ISR when App use (Rx_Buf)
	
	if( ub->tail < head )
	{
		if( *nlen > ( head - ub->tail ))
		{
			*nlen = head - ub->tail;
		}
		else
		{
		}

		memcpy(&dat[0],&ub->lq[ub->tail], *nlen);

		ub->tail = ( ub->tail + *nlen ) % MAXUARTBUFSIZE;		
	
	}
	else if( ub->tail > head )
	{
		if( *nlen > ( MAXUARTBUFSIZE - ( ub->tail - head ) ) )
		{	
			*nlen = MAXUARTBUFSIZE - ( ub->tail - head );
		}
		else
		{
		}

		if( *nlen  <= ( MAXUARTBUFSIZE  - ub->tail ) )
		{
			memcpy(&dat[0],&ub->lq[ub->tail], *nlen);

			ub->tail = ( ub->tail + *nlen ) % MAXUARTBUFSIZE;				
		}
		else
		{
			tmp = MAXUARTBUFSIZE  - ub->tail ;
			memcpy(&dat[0], &ub->lq[ub->tail], tmp);
			ub->tail = ( ub->tail + tmp ) % MAXUARTBUFSIZE;	
			
			memcpy(&dat[tmp], &ub->lq[ub->tail], *nlen - tmp);
			ub->tail = ( ub->tail + *nlen - tmp ) % MAXUARTBUFSIZE;				
		}		
		
	}
	else
	{
		*nlen = 0;
		return -1;		
	}

	return 0;
}

/*********************************************************************
**********************************************************************/
static int GpioInit(void)
{
	int ret;
	dp("Ex UART DRV : Gpio Init \n");
	// intN
	gpio_direction_output(IMX_GPIO_NR(4,15),1);
	gpio_direction_output(IMX_GPIO_NR(5,21),1);
	gpio_direction_output(IMX_GPIO_NR(4,9),1);
	
	// init value High
	// Always Enable 16C554's Interrupt
	gpio_set_value(IMX_GPIO_NR(4,15),1);
	gpio_set_value(IMX_GPIO_NR(5,21),1);
	gpio_set_value(IMX_GPIO_NR(4,9),1);

	dp("Ex UART DRV : init. Ch A...");
	/* Interrupt Init */
	/* ------------------------------------------------------- Int A */
	/* nTL16C554-INTA	: GPM0*/
	// interrupt disable
	writel(readl(MX6_IO_ADDRESS(GPIO4_IMR)) | (1<<5) , MX6_IO_ADDRESS(GPIO4_IMR));
	
	// Set Ext interrupt
	gpio_request(IMX_GPIO_NR(4,5),"EINTCS1A");
	gpio_direction_input(IMX_GPIO_NR(4,5));

	ret = request_irq(EINTCS1A, ISR_chA_rx,IRQF_DISABLED,ANDUART_DEVNAME, &TL16C554_fops);
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				EINTCS1A,ret);
		return -1;
	}

	// Enable Interrppt 
	writel(readl(MX6_IO_ADDRESS(GPIO4_IMR)) & ~(1<<5) , MX6_IO_ADDRESS(GPIO4_IMR));
	
	dp("Done\n");
	

	dp("Ex UART DRV : init. Ch B...");
	/* ------------------------------------------------------- Int B */
	/* nTL16C554-INTB	: GPM1, EXTINT24 */
	//Disable interrupt
	writel(readl(MX6_IO_ADDRESS(GPIO4_IMR)) | (1<<6) , MX6_IO_ADDRESS(GPIO4_IMR));
	
	gpio_request(IMX_GPIO_NR(4,6),"EINTCS1B");
	gpio_direction_input(IMX_GPIO_NR(4,6));
	ret = request_irq(EINTCS1B, ISR_chB_rx, IRQF_DISABLED, ANDUART_DEVNAME, &TL16C554_fops);
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				EINTCS1B,ret);
		return -1;
	}
	// Enable Interrppt 
	writel(readl(MX6_IO_ADDRESS(GPIO4_IMR)) & ~(1<<6) , MX6_IO_ADDRESS(GPIO4_IMR));
	dp("Done\n");

	dp("Ex UART DRV : init. Ch C...");

	/* ------------------------------------------------------- Int C */
	/* nTL16C554-INTC	: GPN0 ,EXTINT0*/
	// Disable Interrppt 
	writel(readl(MX6_IO_ADDRESS(GPIO4_IMR)) | (1<<7) , MX6_IO_ADDRESS(GPIO4_IMR));
	
	gpio_request(IMX_GPIO_NR(4,7),"EINTCS1C");
	gpio_direction_input(IMX_GPIO_NR(4,7));
	ret = request_irq(EINTCS1C, ISR_chC_rx, IRQF_DISABLED, ANDUART_DEVNAME, &TL16C554_fops);	
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				EINTCS1C,ret);
		return -1;
	}

	// Enable Interrppt 
	//writel(readl(S3C_EINTMASK) & ~(1<<0) , S3C_EINTMASK);
	writel(readl(MX6_IO_ADDRESS(GPIO4_IMR)) & ~(1<<7) , MX6_IO_ADDRESS(GPIO4_IMR));
	dp("Done\n");

	dp("Ex UART DRV : init. Ch D...");

	/* ------------------------------------------------------- Int D */
	/* nTL16C554-INTD	: GPN1 */
	// Disable interrupt
	writel(readl(MX6_IO_ADDRESS(GPIO4_IMR)) | (1<<8) , MX6_IO_ADDRESS(GPIO4_IMR));
	
	gpio_request(IMX_GPIO_NR(4,8),"EINTCS1D");
	gpio_direction_input(IMX_GPIO_NR(4,8));
	ret = request_irq(EINTCS1D, ISR_chD_rx, IRQF_DISABLED, ANDUART_DEVNAME, &TL16C554_fops);	
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				EINTCS1D,ret);
		return -1;
	}

	// Enable Interrppt  
	//writel(readl(S3C_EINTMASK) & ~(1<<1) , S3C_EINTMASK);
	writel(readl(MX6_IO_ADDRESS(GPIO4_IMR)) & ~(1<<8) , MX6_IO_ADDRESS(GPIO4_IMR));
	dp("Done\n");


	/* ------------------------------------------------------- Int E */
	/* nTL16C554-INTE	:  GPN3, EXTINT3 */
	dp("Ex UART DRV : init. Ch E...");
	// Disable Interrppt  */
	writel(readl(MX6_IO_ADDRESS(GPIO4_IMR)) | (1<<11) , MX6_IO_ADDRESS(GPIO4_IMR));
	
	gpio_request(IMX_GPIO_NR(4,11),"EINTCS2A");
	gpio_direction_input(IMX_GPIO_NR(4,11));
	ret = request_irq(EINTCS2A, ISR_chE_rx,IRQF_DISABLED, ANDUART_DEVNAME, &TL16C554_fops);
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				EINTCS2A,ret);
		return -1;
	}

	// Enable Interrppt  */
	writel(readl(MX6_IO_ADDRESS(GPIO4_IMR)) & ~(1<<11) , MX6_IO_ADDRESS(GPIO4_IMR));
	
	dp("Done\n");

	/* ------------------------------------------------------- Int F */
	/* nTL16C554-INTE	:  GPN4, EXTINT4*/
	dp("Ex UART DRV : init. Ch F...");
	// Disable interrupt
	writel(readl(MX6_IO_ADDRESS(GPIO2_IMR)) | (1<<13) , MX6_IO_ADDRESS(GPIO2_IMR));
	
	gpio_request(IMX_GPIO_NR(2,13),"EINTCS2B");
	gpio_direction_input(IMX_GPIO_NR(2,13));

	// EINT4,5
	ret = request_irq(EINTCS2B, ISR_chF_rx,IRQF_DISABLED, ANDUART_DEVNAME, &TL16C554_fops);
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				EINTCS2B,ret);
		return -1;
	}

	// Enable Interrppt 
	writel(readl(MX6_IO_ADDRESS(GPIO2_IMR)) & ~(1<<13) , MX6_IO_ADDRESS(GPIO2_IMR));
	
	dp("Done\n");

	/* ------------------------------------------------------- Int G */
	/* nTL16C554-INTE	:  GPN5, EXTINT5 */
	dp("Ex UART DRV : init. Ch G...");
	// Disable Interrppt  
	writel(readl(MX6_IO_ADDRESS(GPIO5_IMR)) | (1<<19) , MX6_IO_ADDRESS(GPIO5_IMR));
	
	gpio_request(IMX_GPIO_NR(5,19),"EINTCS2C");
	gpio_direction_input(IMX_GPIO_NR(5,19));

	ret = request_irq(EINTCS2C, ISR_chG_rx,IRQF_DISABLED, ANDUART_DEVNAME, &TL16C554_fops);
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				EINTCS2C,ret);
		return -1;
	}

	// Enable Interrppt  
	writel(readl(MX6_IO_ADDRESS(GPIO2_IMR)) & ~(1<<13) , MX6_IO_ADDRESS(GPIO2_IMR));

	dp("Done\n");

	/* ------------------------------------------------------- Int H */
	/* nTL16C554-INTE	:  GPN6, EXTINT6 */
	dp("Ex UART DRV : init. Ch H...");
	// Disable Interrppt  
	writel(readl(MX6_IO_ADDRESS(GPIO5_IMR)) | (1<<20) , MX6_IO_ADDRESS(GPIO5_IMR));
	
	gpio_request(IMX_GPIO_NR(5,20),"EINTCS2D");
	gpio_direction_input(IMX_GPIO_NR(5,20));

	// EINT 6,7
	ret = request_irq(EINTCS2D, ISR_chH_rx,IRQF_DISABLED, ANDUART_DEVNAME, &TL16C554_fops);
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				EINTCS2D,ret);
		return -1;
	}

	// Enable Interrppt 
	writel(readl(MX6_IO_ADDRESS(GPIO5_IMR)) & ~(1<<20) , MX6_IO_ADDRESS(GPIO5_IMR));
	
	dp("Done\n");


	/* ------------------------------------------------------- Init 8 */
	/* GPH0, EXTINT6-0, Ch I*/
	/* GPH1, EXTINT6-1, Ch J*/
	/* GPH2, EXTINT6-2, Ch K*/
	/* GPH3, EXTINT6-3, Ch L*/

	/* ------------------------------------------------------- Int I */
	/* nTL16C554-INTE	:  GPN6, EXTINT6 */
	dp("Ex UART DRV : init. Ch I...");
	// Disable Interrppt  
	writel(readl(MX6_IO_ADDRESS(GPIO2_IMR)) | (1<<11) , MX6_IO_ADDRESS(GPIO2_IMR));
	
	gpio_request(IMX_GPIO_NR(2,11),"EINTCS3A");
	gpio_direction_input(IMX_GPIO_NR(2,11));

	ret = request_irq(EINTCS3A, ISR_chH_rx,IRQF_DISABLED, ANDUART_DEVNAME, &TL16C554_fops);
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				EINTCS3A,ret);
		return -1;
	}

	// Enable Interrppt 
	writel(readl(MX6_IO_ADDRESS(GPIO2_IMR)) & ~(1<<11) , MX6_IO_ADDRESS(GPIO2_IMR));
	
	dp("Done\n");


	/* ------------------------------------------------------- Int J */
	/* nTL16C554-INTE	:  GPN6, EXTINT6 */
	dp("Ex UART DRV : init. Ch J...");
	// Disable Interrppt  
	writel(readl(MX6_IO_ADDRESS(GPIO2_IMR)) | (1<<12) , MX6_IO_ADDRESS(GPIO2_IMR));
	
	gpio_request(IMX_GPIO_NR(2,12),"EINTCS3B");
	gpio_direction_input(IMX_GPIO_NR(2,12));

	// EINT 6,7
	ret = request_irq(EINTCS3B, ISR_chH_rx,IRQF_DISABLED, ANDUART_DEVNAME, &TL16C554_fops);
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				EINTCS3B,ret);
		return -1;
	}

	// Enable Interrppt 
	writel(readl(MX6_IO_ADDRESS(GPIO2_IMR)) & ~(1<<12) , MX6_IO_ADDRESS(GPIO2_IMR));
	
	dp("Done\n");


	/* ------------------------------------------------------- Int K */
	/* nTL16C554-INTE	:  GPN6, EXTINT6 */
	dp("Ex UART DRV : init. Ch K...");
	// Disable Interrppt  
	writel(readl(MX6_IO_ADDRESS(GPIO6_IMR)) | (1<<15) , MX6_IO_ADDRESS(GPIO6_IMR));
	
	gpio_request(IMX_GPIO_NR(6,15),"EINTCS3C");
	gpio_direction_input(IMX_GPIO_NR(6,15));

	// EINT 6,7
	ret = request_irq(EINTCS3C, ISR_chH_rx,IRQF_DISABLED, ANDUART_DEVNAME, &TL16C554_fops);
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				EINTCS3C,ret);
		return -1;
	}

	// Enable Interrppt 
	writel(readl(MX6_IO_ADDRESS(GPIO6_IMR)) & ~(1<<15) , MX6_IO_ADDRESS(GPIO6_IMR));
	
	dp("Done\n");


	/* ------------------------------------------------------- Int L */
	/* nTL16C554-INTE	:  GPN6, EXTINT6 */
	dp("Ex UART DRV : init. Ch L...");
	// Disable Interrppt  
	writel(readl(MX6_IO_ADDRESS(GPIO4_IMR)) | (1<<14) , MX6_IO_ADDRESS(GPIO4_IMR));
	
	gpio_request(IMX_GPIO_NR(4,14),"EINTCS3D");
	gpio_direction_input(IMX_GPIO_NR(4,14));

	// EINT 6,7
	ret = request_irq(EINTCS3D, ISR_chH_rx,IRQF_DISABLED, ANDUART_DEVNAME, &TL16C554_fops);
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				EINTCS3D,ret);
		return -1;
	}

	// Enable Interrppt 
	writel(readl(MX6_IO_ADDRESS(GPIO4_IMR)) & ~(1<<14) , MX6_IO_ADDRESS(GPIO4_IMR));
	
	dp("Done\n");

	return 0;
}


/*********************************************************************
**********************************************************************/

/////////////////////////////////////////////////////////////////////////////////
//*********************************************************
//	init/exit entry
//*********************************************************
int __init TL16C554_Init (void)
{
	int ret = 0;

	char *version = DRIVER_VERSION;

	printk("\n");
	printk("---------------------- Loading ANDUART Module --------------------------\n");

	ret = register_chrdev(ANDUART_MAJOR, ANDUART_DEVNAME, &TL16C554_fops);
	
	if(ret < 0)
	{
		printk("registration failed (%d)\n", ANDUART_MAJOR);
		printk("------------------------------------------------------------------------\n");
		return ret;
	}
	
	printk("TL16C554 Device registered with Major Number = %d\n", ANDUART_MAJOR);
	printk("D/D Version    : %s\n",version);
	printk("Build Time     : %s %s\n", __DATE__, __TIME__);	

	devBaseAddress = (unsigned int )ioremap_nocache(TL16C554DEV_PHY_ADDR, TL16C554_MEM_SIZE); 

	ret = GpioInit();
	if(ret < 0)
	{
		printk("TL16C554 Initialize failed (%d)\n", ANDUART_MAJOR);
		printk("------------------------------------------------------------------------\n");
		return ret;
	}


	printk("TL16C554 Phy Address : 0x%x is mapped to 0x%x\n",TL16C554DEV_PHY_ADDR,devBaseAddress);
	printk("------------------------------------------------------------------------\n");
	return 0;
}

//*********************************************************
void __exit TL16C554_Cleanup (void)
{
	int ret = 0;
	
	printk("\n");
	printk("--------------- Cleanup Module ---------------\n");

	// unregister device
	unregister_chrdev(ANDUART_MAJOR, ANDUART_DEVNAME);
	
	if(ret < 0)		
	{
		printk("UART Device unregistration failed (%d)\n", ret);
	}
	else			
	{
		printk("UART Device unregistered = %d\n", ANDUART_MAJOR);
		printk("----------------------------------------------\n");
	}

	// interrupt  해제.
	free_irq(EINTCS1A,&TL16C554_fops);	// A
	free_irq(EINTCS1B,&TL16C554_fops);	// B
	free_irq(EINTCS1C,&TL16C554_fops);		// C
	free_irq(EINTCS1D,&TL16C554_fops);		// D
	free_irq(EINTCS2A,&TL16C554_fops);		// E 
	free_irq(EINTCS2B,&TL16C554_fops);		// F
	free_irq(EINTCS2C,&TL16C554_fops);		// G
	free_irq(EINTCS2D,&TL16C554_fops);		// H
	free_irq(EINTCS3A,&TL16C554_fops);	// I,J,K,L
	free_irq(EINTCS3B,&TL16C554_fops);	// I,J,K,L
	free_irq(EINTCS3C,&TL16C554_fops);	// I,J,K,L
	free_irq(EINTCS3D,&TL16C554_fops);	// I,J,K,L

	if( devBaseAddress != 0 )
	{
		iounmap((void *)devBaseAddress);
		devBaseAddress = 0;
	}
}


module_init(TL16C554_Init);
module_exit(TL16C554_Cleanup);


MODULE_LICENSE("GPL");					// Get rid of taint message by declaring code as GPL..
MODULE_AUTHOR(DRIVER_AUTHOR);		// Who wrote this module?
MODULE_DESCRIPTION(DRIVER_DESC);   
