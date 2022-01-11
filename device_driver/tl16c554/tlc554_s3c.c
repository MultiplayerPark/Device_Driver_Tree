/************************************************************************
* tl16c554 driver. for imx6(linux-2.6.32)
************************************************************************/
//#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/moduleparam.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/interrupt.h>

#include <asm/mach/map.h>

#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/hardware.h>

#include "adtlc554.h"

#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-mem.h>
#include <asm/arch/regs-s3c6410-clock.h>

//-------------------------------------------------------------------//
#define printm(fmt,args...) printk("[%s,%s,%d]"fmt,__FILE__,__FUNCTION__,__LINE__,## args)

#ifdef DEBUG_ANDUART_DRV 
#define dp(fmt,args...) printk(fmt,## args)
#define dlp(fmt,args...) printk("[%s,%s,%d]"fmt,__FILE__,__FUNCTION__,__LINE__,## args)
#else
#define dp(fmt,args...)
#define dlp(fmt,args...)
#endif
//-------------------------------------------------------------------//

#define WHILE_CNT		10000
#define STRCAT(ABC) #ABC

#define DRIVER_AUTHOR		"Worked by pwk"
#define DRIVER_VERSION		"1.00"
#define DRIVER_DESC			"16C554 Driver Version "DRIVER_VERSION

#define ANDUART_DEVNAME		"ttyTL"

#define ANDUART_MAJOR		204

//#define DEBUG_ANDUART_DRV
//#define INT_FALLEDGE

#define TL16C554DEV_PHY_ADDR	0x38000000 
#define TL16C554_MEM_SIZE		0x00001000

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

#define CH_A_OFFSET		0x100
#define CH_B_OFFSET		0x200
#define CH_C_OFFSET		0x300
#define CH_D_OFFSET		0x400
#define CH_E_OFFSET		0x500
#define CH_F_OFFSET		0x600
#define CH_G_OFFSET		0x700
#define CH_H_OFFSET		0x800
#define CH_I_OFFSET		0x000
#define CH_J_OFFSET		0xA00
#define CH_K_OFFSET		0xE00
#define CH_L_OFFSET		0xF00

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
#define LSR_TEMT		0x40		// Transmitter holding register(THR)와 Transmitter Shift register(TSR)

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
	open	: TL16C554_Open,
};

static int TL16C554x_Open(struct inode* a_pINode, struct file* a_pFile);
static int TL16C554x_Release(struct inode* a_pINode, struct file* a_pFile);
static int TL16C554x_Ioctl(struct inode* a_pINode, struct file* a_pFile, unsigned int a_cmd, unsigned long a_arg);

// file operation entry function - CH A
static ssize_t TL16C554A_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554A_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static struct file_operations TL16C554A_fops = {
	open	: TL16C554x_Open,
	release	: TL16C554x_Release,
	read	: TL16C554A_Read,
	write	: TL16C554A_Write,
	ioctl	: TL16C554x_Ioctl,
};


// file operation entry function - CH B
static ssize_t TL16C554B_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554B_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554B_fops = {
	open	: TL16C554x_Open,
	release	: TL16C554x_Release,
	read	: TL16C554B_Read,
	write	: TL16C554B_Write,
	ioctl	: TL16C554x_Ioctl,
};


// file operation entry function - CH C
static ssize_t TL16C554C_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554C_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554C_fops = {
	open	: TL16C554x_Open,
	release	: TL16C554x_Release,
	read	: TL16C554C_Read,
	write	: TL16C554C_Write,
	ioctl	: TL16C554x_Ioctl,
};


// file operation entry function - CH D
static ssize_t TL16C554D_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554D_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554D_fops = {
	open	: TL16C554x_Open,
	release	: TL16C554x_Release,
	read	: TL16C554D_Read,
	write	: TL16C554D_Write,
	ioctl	: TL16C554x_Ioctl,
};

// file operation entry function - CH E
static ssize_t TL16C554E_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554E_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554E_fops = {
	open	: TL16C554x_Open,
	release	: TL16C554x_Release,
	read	: TL16C554E_Read,
	write	: TL16C554E_Write,
	ioctl	: TL16C554x_Ioctl,
};


// file operation entry function - CH F
static ssize_t TL16C554F_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554F_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554F_fops = {
	open	: TL16C554x_Open,
	release	: TL16C554x_Release,
	read	: TL16C554F_Read,
	write	: TL16C554F_Write,
	ioctl	: TL16C554x_Ioctl,
};


// file operation entry function - CH G
static ssize_t TL16C554G_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554G_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554G_fops = {
	open	: TL16C554x_Open,
	release	: TL16C554x_Release,
	read	: TL16C554G_Read,
	write	: TL16C554G_Write,
	ioctl	: TL16C554x_Ioctl,
};


// file operation entry function - CH H
static ssize_t TL16C554H_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554H_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554H_fops = {
	open	: TL16C554x_Open,
	release	: TL16C554x_Release,
	read	: TL16C554H_Read,
	write	: TL16C554H_Write,
	ioctl	: TL16C554x_Ioctl,
};

#if 1
// file operation entry function - CH I
static ssize_t TL16C554I_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554I_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554I_fops = {
	open	: TL16C554x_Open,
	release	: TL16C554x_Release,
	read	: TL16C554I_Read,
	write	: TL16C554I_Write,
	ioctl	: TL16C554x_Ioctl,
};

// file operation entry function - CH J
static ssize_t TL16C554J_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554J_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554J_fops = {
	open	: TL16C554x_Open,
	release	: TL16C554x_Release,
	read	: TL16C554J_Read,
	write	: TL16C554J_Write,
	ioctl	: TL16C554x_Ioctl,
};


// file operation entry function - CH K
static ssize_t TL16C554K_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554K_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554K_fops = {
	open	: TL16C554x_Open,
	release	: TL16C554x_Release,
	read	: TL16C554K_Read,
	write	: TL16C554K_Write,
	ioctl	: TL16C554x_Ioctl,
};


// file operation entry function - CH L
static ssize_t TL16C554L_Read(struct file* a_pFile, char* a_pBuf, size_t a_length, loff_t* a_pOffset);
static ssize_t TL16C554L_Write(struct file* a_pFile, const char* a_pBuf, size_t a_length, loff_t* a_pOffset);

static struct file_operations TL16C554L_fops = {
	open	: TL16C554x_Open,
	release	: TL16C554x_Release,
	read	: TL16C554L_Read,
	write	: TL16C554L_Write,
	ioctl	: TL16C554x_Ioctl,
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
		{
			a_pFile->f_op = &TL16C554A_fops;
			break;
		}
		case CH_B_MINOR:
		{
			a_pFile->f_op = &TL16C554B_fops;
			break;
		}
		case CH_C_MINOR:
		{
			a_pFile->f_op = &TL16C554C_fops;			
			break;
		}
		case CH_D_MINOR:
		{
			a_pFile->f_op = &TL16C554D_fops;			
			break;
		}
		case CH_E_MINOR:
		{
			a_pFile->f_op = &TL16C554E_fops;			
			break;
		}
		case CH_F_MINOR:
		{
			a_pFile->f_op = &TL16C554F_fops;			
			break;
		}
		case CH_G_MINOR:
		{
			a_pFile->f_op = &TL16C554G_fops;			
			break;
		}
		case CH_H_MINOR:
		{
			a_pFile->f_op = &TL16C554H_fops;
			break;
		}
#if 1
		case CH_I_MINOR:
		{
			a_pFile->f_op = &TL16C554I_fops;
			break;
		}

		case CH_J_MINOR:
		{
			a_pFile->f_op = &TL16C554J_fops;
			break;
		}

		case CH_K_MINOR:
		{
			a_pFile->f_op = &TL16C554K_fops;
			break;
		}

		case CH_L_MINOR:
		{
			a_pFile->f_op = &TL16C554L_fops;
			break;
		}
#endif

		default:
		{
			return -ENXIO;
		}
	}

	if( (a_pFile->f_op != NULL) && (a_pFile->f_op->open != NULL) ) return a_pFile->f_op->open(a_pINode, a_pFile);
	else return -ENXIO;

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
		{
			reVal = 1;
			break;
		}
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
			break;
		}
		case IOCTL_UART_FLUSH:
		{
			// buffer clear	
			reVal = 1;
			break;
		}
		case IOCTL_UART_DIR:
		{
			reVal = 1;
			break;
		}
		default : 
		{
			printm("## ioctl fault - cmd[%d]\n",a_cmd);
			break;
		}
	}

	return reVal;

}

static int ReadData(char* dst, ubuf_t* src, int len, char* temp)
{
	int err = 0;

	Ubuf_Rx_Pullout(src, temp, &len);

	if( len != 0 )
	{
		err = copy_to_user((void *)dst, (void *)temp, len);
	}
	else
	{
		return 0;
	}

	if(err != 0)
	{
		printm("## Ex Uart read - fail copy_to_user\n");
		return -1;
	}
	else
	{
		return len;
	}

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

static unsigned char GetPortFromIrq(int irq)
{
	switch(irq){
		case IRQ_EINT23:		// ExUART 0
			return EX_UART0;
		case IRQ_EINT24:		// ExUART 1
			return EX_UART1;
		case IRQ_EINT0:			// ExUART 2
			return EX_UART2;
		case IRQ_EINT1:			// ExUART 3
			return EX_UART3;
		case IRQ_EINT3:			// ExUART 4
			return EX_UART4;
		case IRQ_EINT4:			// ExUART 5
			return EX_UART5;
		case IRQ_EINT5:			// ExUART 6
			return EX_UART6;
		case IRQ_EINT6:			// ExUART 7
			return EX_UART7;
		case IRQ_RESERVED:		// ExUART 8 ~ (External interrupt group 1 ~ 9)
			{
				int pend = 0;
				
				pend = readl(S3C_SERVICEPEND);
				//dlp("Int group 5,6 pend[0x%x]\n",pend);

				if(pend >> (16 + 0) & 1)
				{
					return EX_UART8;
				}
				else if(pend >> (16 + 1) & 1)
				{
					return EX_UART9;
				}
				else if(pend >> (16 + 2) & 1)
				{
					return EX_UART10;
				}
				else if(pend >> (16 + 3) & 1)
				{
					return EX_UART11;
				}
				else
				{
					printm("## wrong irq pend [%d] - pend[0x%x]\n",irq,pend);
					return -1;
				}
			}
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


static irqreturn_t ISR_chIJKL_rx(int irq,void *dev_id)
{
	long flags;
	unsigned char port = 0;
	unsigned int offset = 0;

	local_irq_save(flags);

	port = GetPortFromIrq(irq);
	offset = GetOffsetPort(port);

	PutRxData(SelRxBufPort(port),offset);

	// clear pending bit
	writel(readl(S3C_SERVICEPEND),S3C_EINT56PEND);
	local_irq_restore(flags);

	return IRQ_HANDLED;

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
	라인 큐 데이터 사출 ( in App to read data)
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
	s3c_gpio_cfgpin(S3C_GPN2,S3C_GPN2_OUTP);
	s3c_gpio_cfgpin(S3C_GPN7,S3C_GPN7_OUTP);
	s3c_gpio_cfgpin(S3C_GPH4,S3C_GPH4_OUTP);

	// init value High
	// Always Enable 16C554's Interrupt
	writel(readl(S3C_GPNDAT) | (1<<2),S3C_GPNDAT);
	writel(readl(S3C_GPNDAT) | (1<<7),S3C_GPNDAT);
	writel(readl(S3C_GPHDAT) | (1<<4),S3C_GPHDAT);

	dp("Ex UART DRV : init. Ch A...");
	/* Interrupt Init */
	/* ------------------------------------------------------- Int A */
	/* nTL16C554-INTA	: GPM0*/
	// interrupt disable
	writel(readl(S3C_EINTMASK) | (1<<23) , S3C_EINTMASK);
	
	// Set Ext interrupt
	s3c_gpio_cfgpin(S3C_GPM0,S3C_GPM0_EXTINT23);
	//s3c_gpio_pullup(S3C_GPM0,1); //pull down

	writel((readl(S3C_EINTCON1) & ~(7<<12)) | (S3C_EXTINT_HILEV << 12), S3C_EINTCON1);

 	// cleared interrupt Pending 
	writel((1<<23) , S3C_EINTPEND);

	ret = request_irq(IRQ_EINT23, ISR_chA_rx,SA_INTERRUPT,ANDUART_DEVNAME, &TL16C554_fops);
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				IRQ_EINT23,ret);
		return -1;
	}

	// Enable Interrppt 
	writel(readl(S3C_EINTMASK) & ~(1<<23) , S3C_EINTMASK);

	dp("Done\n");
	

	dp("Ex UART DRV : init. Ch B...");
	/* ------------------------------------------------------- Int B */
	/* nTL16C554-INTB	: GPM1, EXTINT24 */
	//Disable interrupt
	writel(readl(S3C_EINTMASK) | (1<<24) , S3C_EINTMASK); 

	s3c_gpio_cfgpin(S3C_GPM1,S3C_GPM1_EXTINT24);
	//s3c_gpio_pullup(S3C_GPM1,1); //pull down

	writel((readl(S3C_EINTCON1) & ~(7<<16)) | (S3C_EXTINT_HILEV << 16) , S3C_EINTCON1);

	// cleared interrupt Pending 
	writel((1<<24) , S3C_EINTPEND);

	ret = request_irq(IRQ_EINT24, ISR_chB_rx, SA_INTERRUPT, ANDUART_DEVNAME, &TL16C554_fops);
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				IRQ_EINT24,ret);
		return -1;
	}
	// Enable Interrppt 
	writel(readl(S3C_EINTMASK) & ~(1<<24) , S3C_EINTMASK); 
	dp("Done\n");

	dp("Ex UART DRV : init. Ch C...");

	/* ------------------------------------------------------- Int C */
	/* nTL16C554-INTC	: GPN0 ,EXTINT0*/
	// Disable Interrppt 
	writel(readl(S3C_EINTMASK) | (1<<0) , S3C_EINTMASK);

	s3c_gpio_cfgpin(S3C_GPN0,S3C_GPN0_EXTINT0);
	//s3c_gpio_pullup(S3C_GPN0,1); //pull down

	// Eint 1,0
	writel((readl(S3C_EINTCON0) & ~(7<<0)) | (S3C_EXTINT_HILEV << 0), S3C_EINTCON0);

	// cleared interrupt Pending 
	writel((1<<0) , S3C_EINTPEND);

	ret = request_irq(IRQ_EINT0, ISR_chC_rx, SA_INTERRUPT, ANDUART_DEVNAME, &TL16C554_fops);	
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				IRQ_EINT0,ret);
		return -1;
	}

	// Enable Interrppt 
	writel(readl(S3C_EINTMASK) & ~(1<<0) , S3C_EINTMASK);
	dp("Done\n");

	dp("Ex UART DRV : init. Ch D...");

	/* ------------------------------------------------------- Int D */
	/* nTL16C554-INTD	: GPN1 */
	// Disable interrupt
	writel(readl(S3C_EINTMASK) | (1<<1) , S3C_EINTMASK);

	s3c_gpio_cfgpin(S3C_GPN1,S3C_GPN1_EXTINT1);
	//s3c_gpio_pullup(S3C_GPN1,1); //pull down

	// EINT1,0
	/* writel((readl(S3C_EINTCON0) & ~(7<<0)) | (S3C_EXTINT_HILEV << 0), S3C_EINTCON0); */

	// cleared interrupt Pending 
	writel((1<<1) , S3C_EINTPEND);

	ret = request_irq(IRQ_EINT1, ISR_chD_rx, SA_INTERRUPT, ANDUART_DEVNAME, &TL16C554_fops);	
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				IRQ_EINT1,ret);
		return -1;
	}

	// Enable Interrppt  
	writel(readl(S3C_EINTMASK) & ~(1<<1) , S3C_EINTMASK);

	dp("Done\n");


	/* ------------------------------------------------------- Int E */
	/* nTL16C554-INTE	:  GPN3, EXTINT3 */
	dp("Ex UART DRV : init. Ch E...");
	// Disable Interrppt  */
	writel(readl(S3C_EINTMASK) | (1<<3) , S3C_EINTMASK); 

	s3c_gpio_cfgpin(S3C_GPN3,S3C_GPN3_EXTINT3);
	//s3c_gpio_pullup(S3C_GPN3,1); //pull down

	writel((readl(S3C_EINTCON0) & ~(7<<4)) | (S3C_EXTINT_HILEV << 4) , S3C_EINTCON0);

	// cleared interrupt Pending */
	writel((1<<3) , S3C_EINTPEND);
	
	ret = request_irq(IRQ_EINT3, ISR_chE_rx,SA_INTERRUPT, ANDUART_DEVNAME, &TL16C554_fops);
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				IRQ_EINT3,ret);
		return -1;
	}

	// Enable Interrppt  */
	writel(readl(S3C_EINTMASK) & ~(1<<3) , S3C_EINTMASK); 

	dp("Done\n");

	/* ------------------------------------------------------- Int F */
	/* nTL16C554-INTE	:  GPN4, EXTINT4*/
	dp("Ex UART DRV : init. Ch F...");
	// Disable interrupt
	writel(readl(S3C_EINTMASK) | (1<<4) , S3C_EINTMASK); 

	s3c_gpio_cfgpin(S3C_GPN4,S3C_GPN4_EXTINT4);
	//s3c_gpio_pullup(S3C_GPN4,1); //pull down

	// EINT4,5
	writel((readl(S3C_EINTCON0) & ~(7<<8)) | (S3C_EXTINT_HILEV << 8) , S3C_EINTCON0);

	// cleared interrupt Pending
	writel((1<<4) , S3C_EINTPEND);
	
	ret = request_irq(IRQ_EINT4, ISR_chF_rx,SA_INTERRUPT, ANDUART_DEVNAME, &TL16C554_fops);
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				IRQ_EINT4,ret);
		return -1;
	}

	// Enable Interrppt 
	writel(readl(S3C_EINTMASK) & ~(1<<4) , S3C_EINTMASK); 

	dp("Done\n");

	/* ------------------------------------------------------- Int G */
	/* nTL16C554-INTE	:  GPN5, EXTINT5 */
	dp("Ex UART DRV : init. Ch G...");
	// Disable Interrppt  
	writel(readl(S3C_EINTMASK) | (1<<5) , S3C_EINTMASK); 

	s3c_gpio_cfgpin(S3C_GPN5,S3C_GPN5_EXTINT5);
	//s3c_gpio_pullup(S3C_GPN5,1); //pull down 

	writel((readl(S3C_EINTCON0) & ~(7<<8)) | (S3C_EXTINT_HILEV << 8) , S3C_EINTCON0);

	// cleared interrupt Pending
	writel((1<<5) , S3C_EINTPEND);
	
	ret = request_irq(IRQ_EINT5, ISR_chG_rx,SA_INTERRUPT, ANDUART_DEVNAME, &TL16C554_fops);
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				IRQ_EINT5,ret);
		return -1;
	}

	// Enable Interrppt  
	writel(readl(S3C_EINTMASK) & ~(1<<5) , S3C_EINTMASK); 

	dp("Done\n");

	/* ------------------------------------------------------- Int H */
	/* nTL16C554-INTE	:  GPN6, EXTINT6 */
	dp("Ex UART DRV : init. Ch H...");
	// Disable Interrppt  
	writel(readl(S3C_EINTMASK) | (1<<6) , S3C_EINTMASK); 

	s3c_gpio_cfgpin(S3C_GPN6,S3C_GPN6_EXTINT6);
	//s3c_gpio_pullup(S3C_GPN6,1); //pull down

	// EINT 6,7
	writel((readl(S3C_EINTCON0) & ~(7<<12)) | (S3C_EXTINT_HILEV << 12) , S3C_EINTCON0);

	// cleared interrupt Pending 
	writel((1<<6) , S3C_EINTPEND);
	
	ret = request_irq(IRQ_EINT6, ISR_chH_rx,SA_INTERRUPT, ANDUART_DEVNAME, &TL16C554_fops);
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				IRQ_EINT6,ret);
		return -1;
	}

	// Enable Interrppt 
	writel(readl(S3C_EINTMASK) & ~(1<<6) , S3C_EINTMASK); 

	dp("Done\n");


	/* ------------------------------------------------------- Init 8 */
	/* GPH0, EXTINT6-0, Ch I*/
	/* GPH1, EXTINT6-1, Ch J*/
	/* GPH2, EXTINT6-2, Ch K*/
	/* GPH3, EXTINT6-3, Ch L*/

#if 1
	dp("Ex UART DRV : init. Ch I...");
	// GPH0
	// interrupt disable
	writel(readl(S3C_EINT56MASK) | (1<<(16 + 0)) , S3C_EINT56MASK);

	// Interrupt high level : Ext int group 6 - 0~3
	writel((readl(S3C_EINT56CON) & ~(7<<16)) | (S3C_EXTINT_HILEV << 16), S3C_EINT56CON);

	// Set Ext interrupt
	s3c_gpio_cfgpin(S3C_GPH0,S3C_GPH0_EXT_INT_G6_0);
	//s3c_gpio_pullup(S3C_GPH0,1); //pull down

 	// cleared interrupt Pending 
	writel((1<<(16 + 0)) , S3C_EINT56PEND);
	dp("Done\n");


	dp("Ex UART DRV : init. Ch J...");
	// GPH1
	// interrupt disable
	writel(readl(S3C_EINT56MASK) | (1<<(16 + 1)) , S3C_EINT56MASK);
	//s3c_gpio_pullup(S3C_GPH1,1); //pull down
	// Set Ext interrupt
	s3c_gpio_cfgpin(S3C_GPH1,S3C_GPH1_EXT_INT_G6_1);
 	// cleared interrupt Pending 
	writel((1<<(16 + 1)) , S3C_EINT56PEND);
	dp("Done\n");


	dp("Ex UART DRV : init. Ch K...");
	// GPH2
	// interrupt disable
	writel(readl(S3C_EINT56MASK) | (1<<(16 + 2)) , S3C_EINT56MASK);
	//s3c_gpio_pullup(S3C_GPH2,1); //pull down
	// Set Ext interrupt
	s3c_gpio_cfgpin(S3C_GPH2,S3C_GPH2_EXT_INT_G6_2);
 	// cleared interrupt Pending 
	writel((1<<(16 + 2)) , S3C_EINT56PEND);
	dp("Done\n");

	dp("Ex UART DRV : init. Ch L...");
	// GPH3
	// interrupt disable
	writel(readl(S3C_EINT56MASK) | (1<<(16 + 3)) , S3C_EINT56MASK);
	//s3c_gpio_pullup(S3C_GPH3,1); //pull down
	// Set Ext interrupt
	s3c_gpio_cfgpin(S3C_GPH3,S3C_GPH3_EXT_INT_G6_3);
 	// cleared interrupt Pending 
	writel((1<<(16 + 3)) , S3C_EINT56PEND);

	ret = request_irq(IRQ_RESERVED,ISR_chIJKL_rx,SA_INTERRUPT | SA_SHIRQ ,ANDUART_DEVNAME, &TL16C554_fops);
	//ret = request_irq(IRQ_RESERVED,ISR_chIJKL_rx,SA_INTERRUPT | SA_SHIRQ ,ANDUART_DEVNAME, &TL16C554_fops);
	if (ret != 0)
	{
		printk("%s,%s:interrupt %d request failled. return %d\n",__FILE__,__FUNCTION__,
				IRQ_RESERVED,ret);
		return -1;
	}

	// Enable Interrppt 
	writel(readl(S3C_EINT56MASK) & ~(1<<(16 + 0)) , S3C_EINT56MASK);
	writel(readl(S3C_EINT56MASK) & ~(1<<(16 + 1)) , S3C_EINT56MASK);
	writel(readl(S3C_EINT56MASK) & ~(1<<(16 + 2)) , S3C_EINT56MASK);
	writel(readl(S3C_EINT56MASK) & ~(1<<(16 + 3)) , S3C_EINT56MASK);
	dp("Done\n");
#endif



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



	writel(readl(S3C_SROM_BW) & ~(0xf << 20) , S3C_SROM_BW);
	writel(readl(S3C_SROM_BW) | (0 << 20) | (1 << 22) | (1 << 23) , S3C_SROM_BW);

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
	ret = unregister_chrdev(ANDUART_MAJOR, ANDUART_DEVNAME);
	
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
	free_irq(IRQ_EINT23,&TL16C554_fops);	// A
	free_irq(IRQ_EINT24,&TL16C554_fops);	// B
	free_irq(IRQ_EINT0,&TL16C554_fops);		// C
	free_irq(IRQ_EINT1,&TL16C554_fops);		// D
	free_irq(IRQ_EINT3,&TL16C554_fops);		// E 
	free_irq(IRQ_EINT4,&TL16C554_fops);		// F
	free_irq(IRQ_EINT5,&TL16C554_fops);		// G
	free_irq(IRQ_EINT6,&TL16C554_fops);		// H
	free_irq(IRQ_RESERVED,&TL16C554_fops);	// I,J,K,L

	// memory 해제.
	if( devBaseAddress != 0 )
	{
		iounmap((void *)devBaseAddress);
		/* release_mem_region(TL16C554DEV0_PHY_ADDR,TL16C554_MEM_SIZE); */
		devBaseAddress = 0;
	}
}


module_init(TL16C554_Init);
module_exit(TL16C554_Cleanup);


MODULE_LICENSE("GPL");					// Get rid of taint message by declaring code as GPL..
MODULE_AUTHOR(DRIVER_AUTHOR);		// Who wrote this module?
MODULE_DESCRIPTION(DRIVER_DESC);   
