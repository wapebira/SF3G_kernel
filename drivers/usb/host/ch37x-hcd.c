#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/pci_ids.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <asm/irq.h>
#include <asm/byteorder.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/list.h>

#include <linux/device_state_pm.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include "ch37x.h"


#define DRIVER_VERSION		"09 May 2014"
#define DRIVER_AUTHOR		"WCH GROUP<tech@wch.cn>"
#define DRIVER_DESC			"CH374U USB HOST DRIVER MODULE"

static struct spi_ch37x_hcd_data *g_hcd_data;
static const char hcd_name[] = "ch374_hcd"; 
static const char product_desc[] = "ch374 usb host controller";
#if 0
#define DMBERR(args...) do { printk(KERN_ERR  args); } while (0)
#define DMBMSG(args...) do { printk(KERN_INFO args); } while (0) 
#else
#define DMBERR(args...) do { } while (0)
#define DMBMSG(args...) do { } while (0)
#endif


#define spi_hcd_dbg(ihid, args...)				\
do {								\
/*	if (atomic_read(&ihid->debug_flag))*/			\
	/*	pr_info(args);	*/			\
} while (0)

/* 11-bit counter that wraps around (USB 2.0 Section 8.3.3): */
#define USB_MAX_FRAME_NUMBER	0x7ff
#define USB_MAX_RETRIES		3 /* # of retries before error is reported */

/*
 * Max. # of times we're willing to retransmit a request immediately in
 * resposne to a NAK.  Afterwards, we fall back on trying once a frame.
 */
#define NAK_MAX_FAST_RETRANSMITS	2

#define POWER_BUDGET  500

#define RH_PORT_STATUS_STAYED	0
#define RH_PORT_STATUS_CHANGED	1

#define	CMD_SPI_374READ		0xC0
#define	CMD_SPI_374WRITE	0x80

#define	ERR_USB_UNKNOWN		0xFA
#ifndef	USB_INT_SUCCESS
#define	USB_INT_SUCCESS		0x14
#define	USB_INT_CONNECT		0x15
#define	USB_INT_DISCONNECT	0x16
#define	USB_INT_BUF_OVER	0x17
#define	USB_INT_DISK_ERR	0x1F
#endif
/* ======================================= */
#define	DEF_USB_PID_NULL	0x00
#define	DEF_USB_PID_SOF		0x05
#define	DEF_USB_PID_SETUP	0x0D
#define	DEF_USB_PID_IN		0x09
#define	DEF_USB_PID_OUT		0x01
#define	DEF_USB_PID_ACK		0x02
#define	DEF_USB_PID_NAK		0x0A
#define	DEF_USB_PID_STALL	0x0E
#define	DEF_USB_PID_DATA0	0x03
#define	DEF_USB_PID_DATA1	0x0B
#define	DEF_USB_PID_PRE		0x0C

/*
 * 令牌包数据,用于写0x0D寄存器
 */
#define CH374_TOKEN_SETUP		  M_MK_HOST_PID_ENDP( DEF_USB_PID_SETUP, 0 )
#define CH374_TOKEN_BULK_IN(ep)   M_MK_HOST_PID_ENDP( DEF_USB_PID_IN, ep )
#define CH374_TOKEN_BULK_OUT(ep)  M_MK_HOST_PID_ENDP( DEF_USB_PID_OUT, ep )

#define MAX_FIFO_SIZE	64
/* Port-change mask: */
#define PORT_C_MASK	(USB_PORT_STAT_C_CONNECTION |	\
			  USB_PORT_STAT_C_ENABLE |	\
			  USB_PORT_STAT_C_SUSPEND |	\
			  USB_PORT_STAT_C_OVERCURRENT | \
			  USB_PORT_STAT_C_RESET)

enum pkt_state {
	PKT_STATE_NONE = 0,
	PKT_STATE_SETUP,	/* waiting to send setup packet to ctrl pipe */
	PKT_STATE_TRANSFER,	/* waiting to xfer transfer_buffer */
	PKT_STATE_TERMINATE
};

enum ch374_rh_state {
	CH374_RH_RESET,
	CH374_RH_SUSPENDED,
	CH374_RH_RUNNING
};


enum ch374_pid_state {
	CH374_DEF_USB_PID_SETUP = 0,
	CH374_DEF_USB_PID_OUT,
	CH374_DEF_USB_PID_IN
};

enum {
	ENABLE_IRQ = 0,
	RESET_PORT,
	CHECK_CONNECT,
	CHECK_UNLINK
};

enum {
	UNINTERRUPT = 0,
	INTERRUPT
};

struct ch37x_chip_data {
	atomic_t flag_low_speed;	//if usb 1.0 then set 1
	//int flag_device_status;
	unsigned char endp_num;	//num of hid such as key or mouse
	unsigned char endp_in_addr;
	unsigned char endp_out_addr;
	unsigned char hid_dest_len;
	unsigned char UsbDevEndpSize;	// USB设备的端点0的最大包尺寸 
	unsigned char FlagDeviceStatus;
};

struct xgold_spi_chip {
	u8 poll_mode;	/* 0 for contoller polling mode */
	u8 type;	/* SPI/SSP/Micrwire */
	u8 enable_dma;
	int hcd_irq;
	void (*cs_control)(u32 command);
};


struct spi_ch37x_hcd_data {	
	struct device	*dev;
	struct spi_device	*spi;	
	struct usb_hcd *hcd;
	int irq;
	struct ch37x_chip_data ch37x;
	struct ch374_hcd *ch374_hcd;
	char *rx_buf;
	int rx_len; 
	char *tx_buf;
	int tx_len;
	struct device_pm_platdata *pm_platdata;
	atomic_t debug_flag;
};

struct ch374_buffer {
	u8 data[3];
};

struct ch374_hcd {
	spinlock_t lock;

	struct task_struct *spi_thread;

	enum ch374_rh_state rh_state;
	enum ch374_pid_state pid_state;
	/* lower 16 bits contain port status, upper 16 bits the change mask: */
	struct usb_port_status port_status;
	unsigned short wPortTrigger;

	unsigned active:1;

	struct list_head ep_list;	/* list of EP's with work */
	u16 frame_number;
	int olddevnum;
	/*
	 * URB we're currently processing.  Must not be reset to NULL
	 */
	struct urb *curr_urb;
	struct urb *unlink_urb;
	int unlink_status;

	u8 controltog;	/*setup packet tog*/
	u8 transfertog;	/*transfer packet tog*/
	bool isLastSetup;		/*last setup packet finished flag*/
	struct usb_device *loaded_dev;	/* dev that's loaded into the chip */
	int loaded_epnum;		/* epnum whose toggles are loaded */
	int urb_done;			/* > 0 -> no errors, < 0: errno */
	int interrupt; 
	size_t curr_len;
	unsigned long todo;
	int irq;
	struct spi_device       *spi;
};

struct ch374_ep {
	struct usb_host_endpoint *ep;
	struct list_head ep_list;
	u32 naks;
	u8 retransmit;
	enum pkt_state pkt_state;
	enum pkt_state curr_pkt;
	u8 retries;
//	u8 retransmit;			/* packet needs retransmission */
};

static inline s16
frame_diff(u16 left, u16 right)
{
	return ((unsigned) (left - right)) % (USB_MAX_FRAME_NUMBER + 1);
}

static inline struct ch374_hcd *
hcd_to_ch374(struct usb_hcd *hcd)
{
	return (struct ch374_hcd *) hcd->hcd_priv;
}

static inline struct usb_hcd *
ch374_to_hcd(struct ch374_hcd *ch374_hcd)
{
	return container_of((void *) ch374_hcd, struct usb_hcd, hcd_priv);
}

/*******************************************************************/
/*******************************************************************/
/*******************************************************************/
#define GPBCON          0xE0200040
#define GPBDAT          0xE0200044
#define GPBPUD          0xE0200048 
#define CH_CFG0         0xE1300000
#define CLK_CFG0        0xE1300004
#define MODE_CFG0       0xE1300008
#define CS_REG0         0xE130000C
#define SPI_INT_EN0     0xE1300010
#define SPI_STATUS0     0xE1300014
#define SPI_TX_DATA0    0xE1300018
#define SPI_RX_DATA0    0xE130001C
#define PACKET_CNT_REG0 0xE1300020
#define PENDING_CLR_REG0        0xE1300024
#define SWAP_CFG0               0xE1300028
#define FB_CLK_SEL0             0xE130002C
#define UART3_TX                0xE2900C20

unsigned long gpio_reg_gpb_con;
unsigned long gpio_reg_gpb_dat;
unsigned long gpio_reg_gpb_up;
unsigned long spi_ch_cfg;
unsigned long spi_clk_cfg;
unsigned long spi_mode_cfg;
unsigned long spi_cs_reg;
unsigned long spi_int_enable;
unsigned long spi_reg_sta;
unsigned long spi_reg_tx;
unsigned long spi_reg_rx;
unsigned long spi_reg_packet_cnt;
unsigned long spi_reg_swap_cfg;
unsigned long spi_fb_clk_sel;
unsigned long uart3_reg_tx;


static int Write374Byte(unsigned char mAddr, unsigned char mData ) 
{
	struct spi_ch37x_hcd_data *hcd_data = g_hcd_data;
	/*4 wires spi mode*/
	unsigned char txbuf[3];
	int ret;
	txbuf[0] = mAddr;
	txbuf[1] = CMD_SPI_374WRITE;
	txbuf[2] = mData;
	
	ret = spi_write(hcd_data->spi, txbuf, ARRAY_SIZE(txbuf));
	
	spi_hcd_dbg(hcd_data, "%s:tx:0x%02x,0x%02x,0x%02x\n",__func__, txbuf[0], txbuf[1], txbuf[2]);
	return ret;
}


static unsigned char Read374Byte(unsigned char mAddr) 
{
	struct spi_ch37x_hcd_data *hcd_data = g_hcd_data;
	/*4 wires spi mode*/
	unsigned char txbuf[2];
	unsigned char rxbuf[1];
	int ret;

	txbuf[0] = mAddr;
	txbuf[1] = CMD_SPI_374READ;
	
	ret = spi_write_then_read(hcd_data->spi, txbuf, ARRAY_SIZE(txbuf), rxbuf, ARRAY_SIZE(rxbuf));

	spi_hcd_dbg(hcd_data, "%s:tx:0x%02x,0x%02x, rx:0x%02x\n",__func__, txbuf[0], txbuf[1], rxbuf[0]);

	return rxbuf[0];
}

//static int Read374Block(UINT8 mAddr, UINT8 mLen, PUINT8 mBuf )  
void Read374Block( unsigned char mAddr, unsigned char mLen, unsigned char *mBuf )
{
	int ret = 0;
	int i = 0;
	struct spi_ch37x_hcd_data *hcd_data = g_hcd_data;
	unsigned char txbuf[2];

	txbuf[0] = mAddr;
	txbuf[1] = CMD_SPI_374READ;

	ret = spi_write_then_read(hcd_data->spi, txbuf, ARRAY_SIZE(txbuf), mBuf, mLen);

	spi_hcd_dbg(hcd_data, "%s:mAddr=0x%02x, mLen=%d\n",__func__, mAddr, mLen);

	for(i=0; i<mLen; i++)
		spi_hcd_dbg(hcd_data, "0x%02x,", mBuf[i]);

	spi_hcd_dbg(hcd_data, "\n");
}

//void Write374Block(UINT8 mAddr, UINT8 mLen, PUINT8 mBuf )
void Write374Block( unsigned char mAddr, unsigned char mLen, unsigned char *mBuf )
{
	unsigned char txbuf[256];
	int ret = 0;
	int i = 0;
	struct spi_ch37x_hcd_data *hcd_data = g_hcd_data;

	txbuf[0] = mAddr;
	txbuf[1] = CMD_SPI_374WRITE;
	memcpy(&txbuf[2], mBuf, mLen);
	
	ret = spi_write(hcd_data->spi, txbuf, mLen+2);
	
	//spi_hcd_dbg(hcd_data, "%s:mAddr=0x%02x, mLen=%d\n",__func__, mAddr, mLen);

	for(i=0; i<mLen; i++)
		spi_hcd_dbg(hcd_data, "0x%02x,", mBuf[i]);

	//spi_hcd_dbg(hcd_data, "\n");
	
//	return ret;
}

static void host_set_bus_free( void )
{
	Write374Byte( REG_USB_SETUP, BIT_SETP_HOST_MODE | BIT_SETP_AUTO_SOF ); 
	Write374Byte( REG_HUB_SETUP, Read374Byte( REG_HUB_SETUP )&(~BIT_HUB_DISABLE) );  
}

static void set_host_usb_addr( unsigned char addr )
{
	Write374Byte( REG_USB_ADDR, addr );
}

static void reset_hub_port( void )
{  
	Write374Byte( REG_USB_H_CTRL, 0x00 ); 	
	set_host_usb_addr( 0x00 );

	Write374Byte( REG_HUB_SETUP, (Read374Byte(REG_HUB_SETUP)|BIT_HUB0_RESET) & (~BIT_HUB0_POLAR) );
	mdelay( 20 ); 
	Write374Byte( REG_HUB_SETUP, (Read374Byte(REG_HUB_SETUP) & (~BIT_HUB0_RESET)) ); 

	mdelay( 1 );
	Write374Byte( REG_INTER_FLAG, BIT_IF_USB_PAUSE | BIT_IF_DEV_DETECT | BIT_IF_USB_SUSPEND );
	
}

void change_hub_port_speed(struct usb_hcd *hcd)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	if((ch374_hcd->port_status.wPortStatus & USB_PORT_STAT_LOW_SPEED)  ==  USB_PORT_STAT_LOW_SPEED) {
		Write374Byte(REG_HUB_SETUP, Read374Byte(REG_HUB_SETUP) | BIT_HUB0_POLAR);
		Write374Byte( REG_USB_SETUP, Read374Byte( REG_USB_SETUP ) | BIT_SETP_LOW_SPEED | BIT_SETP_AUTO_SOF ); 
	} 
}

/**
 * start_port_operate
 */
static void ch374_set_address( struct usb_hcd *hcd, struct usb_device *dev, int epnum )
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	if (dev->devnum != ch374_hcd->olddevnum) {
		set_host_usb_addr(dev->devnum);
		ch374_hcd->olddevnum = dev->devnum;
	}

//	change_hub_port_speed(hcd);
}

static void enable_hub_port(void)
{
	/*depends on hub0*/
	Write374Byte( REG_HUB_SETUP, Read374Byte( REG_HUB_SETUP ) | BIT_HUB0_EN );	
}

static void init_ch374_host( void )                                             {

	Write374Byte( REG_SYS_CTRL, 0x08);
	mdelay(50);
	Write374Byte( REG_SYS_CTRL, 0x00);
	mdelay(50);
	Write374Byte( REG_USB_SETUP, 0x00 );
	set_host_usb_addr( 0x00 );			                            
	Write374Byte( REG_USB_H_CTRL, 0x00 );
	Write374Byte( REG_INTER_FLAG, BIT_IF_USB_PAUSE | BIT_IF_INTER_FLAG );  
	Write374Byte( REG_INTER_EN, BIT_IE_TRANSFER | BIT_IE_DEV_DETECT ); 
	Write374Byte( REG_SYS_CTRL, BIT_CTRL_OE_POLAR );  			
	host_set_bus_free( );  

}

static void finish_port_operate( void )
{
	Write374Byte( REG_INTER_FLAG, BIT_IF_USB_PAUSE | BIT_IF_TRANSFER | BIT_IF_USB_SUSPEND | BIT_IF_WAKE_UP );
//	set_host_usb_addr(0);
//	Write374Byte( REG_USB_SETUP, (Read374Byte( REG_USB_SETUP) & (~BIT_SETP_LOW_SPEED)) | BIT_SETP_AUTO_SOF );
}

/**
 * 针对端口0
 */
static void disable_hub_port(struct usb_hcd *hcd)
{	
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	ch374_hcd->port_status.wPortStatus &= ~(USB_PORT_STAT_CONNECTION | USB_PORT_STAT_ENABLE | USB_PORT_STAT_LOW_SPEED | USB_PORT_STAT_HIGH_SPEED);
	ch374_hcd->port_status.wPortChange |= USB_PORT_STAT_C_CONNECTION | USB_PORT_STAT_C_ENABLE;
	ch374_hcd->wPortTrigger = RH_PORT_STATUS_CHANGED;
	Write374Byte( REG_HUB_SETUP, Read374Byte( REG_HUB_SETUP) & 0xF0 );
	
	finish_port_operate();
}


/**
 * HostCtrlTransfer374的写ReqBuf（setup包）
 */
static int ch374_ctrl_setup(struct usb_hcd *hcd, struct urb *urb)
{
	Write374Block( RAM_HOST_TRAN, 8, urb->setup_packet);
	Write374Byte( REG_USB_LENGTH, 8 );
	return CH374_TOKEN_SETUP;
}

/**
 * HostCtrlTransfer374的写ReqBuf（setup包）
 */
static int 
ch374_transfer_in(struct usb_hcd *hcd, struct urb *urb)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	int epnum = usb_pipeendpoint(urb->pipe);

	ch374_hcd->curr_len = 0;

	return CH374_TOKEN_BULK_IN(epnum);
}

static int 
ch374_transfer_out(struct usb_hcd *hcd, struct urb *urb)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	int epnum = usb_pipeendpoint(urb->pipe);
	u32 max_packet;
	void *src;

	src = urb->transfer_buffer + urb->actual_length;

	max_packet = usb_maxpacket(urb->dev, urb->pipe, 1);
	if (max_packet > MAX_FIFO_SIZE){
		ch374_hcd->urb_done = -EMSGSIZE;
		return -EMSGSIZE;
	}
	ch374_hcd->curr_len = min((urb->transfer_buffer_length -
			urb->actual_length), max_packet);
	/**
	 * 向send buffer写数据，写数据长度
	 */
	Write374Block( RAM_HOST_TRAN, ch374_hcd->curr_len, src );
	Write374Byte( REG_USB_LENGTH, ch374_hcd->curr_len );

	return CH374_TOKEN_BULK_OUT(epnum);
}

/*
 * Issue the next host-transfer command.
 * Caller must NOT hold HCD spinlock.
 */
static void
ch374_next_transfer(struct usb_hcd *hcd)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	struct urb *urb = ch374_hcd->curr_urb;
	struct ch374_ep *ch374_ep;
	int cmd = -EINVAL;

	if (!urb){
		DMBERR("ch374_next_transfer:URB NULL\n\n");
		return;	/* nothing to do */
	}

	if (urb->ep->hcpriv){
		ch374_ep = urb->ep->hcpriv;
		DMBERR("ch374_next_transfer:full%d\n\n", ch374_ep->pkt_state);
	} else {
		DMBERR("ch374_next_transfer:NULL\n\n");
		return;
	}

	switch(ch374_ep->pkt_state) {
	case PKT_STATE_SETUP:	
		cmd = ch374_ctrl_setup(hcd, urb);
		ch374_hcd->pid_state = CH374_DEF_USB_PID_SETUP;
		break;

	case PKT_STATE_TRANSFER:
		if (usb_urb_dir_in(urb)) {
			cmd = ch374_transfer_in(hcd, urb);
			ch374_hcd->pid_state = CH374_DEF_USB_PID_IN;
		} else {
			cmd = ch374_transfer_out(hcd, urb);
			ch374_hcd->pid_state = CH374_DEF_USB_PID_OUT;
		}
		break;

	default:
		break;
	}
	if (cmd < 0) {
		DMBERR("ch374_next_transfer:cmd < 0\n\n");
		return;
	}

	Write374Byte( REG_USB_H_PID, cmd ); 
	if (ch374_hcd->pid_state == CH374_DEF_USB_PID_SETUP && ch374_ep->curr_pkt == PKT_STATE_SETUP) {
		Write374Byte( REG_USB_H_CTRL,  BIT_HOST_START );
	} else if (ch374_ep->curr_pkt == PKT_STATE_SETUP) {
		Write374Byte( REG_USB_H_CTRL,  (ch374_hcd->controltog ? ( BIT_HOST_START | BIT_HOST_TRAN_TOG | BIT_HOST_RECV_TOG ) : BIT_HOST_START ) );
	} else if (ch374_ep->curr_pkt == PKT_STATE_TRANSFER) {
		Write374Byte( REG_USB_H_CTRL,  (ch374_hcd->transfertog ? ( BIT_HOST_START | BIT_HOST_TRAN_TOG | BIT_HOST_RECV_TOG ) : BIT_HOST_START ) );
	}
	//udelay(200);
}

/*
 * Find the next URB to process and start its execution.
 */
static int 
ch374_select_and_start_urb(struct usb_hcd *hcd)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	struct urb *urb, *curr_urb = NULL;
	struct ch374_ep *ch374_ep;
	int epnum;
	struct usb_host_endpoint *ep;
	struct list_head *pos;
	unsigned long flags;

	spin_lock_irqsave(&ch374_hcd->lock, flags);
	if (list_empty(&ch374_hcd->ep_list)) {
		spin_unlock_irqrestore(&ch374_hcd->lock,
					       flags);
		return 0;
	}

	list_for_each(pos, &ch374_hcd->ep_list) {
		urb = NULL;
		ch374_ep = container_of(pos, struct ch374_ep, ep_list);
		ep = ch374_ep->ep;

		if (list_empty(&ep->urb_list)) 
			continue;
		urb = list_first_entry(&ep->urb_list, struct urb, urb_list);
		if (urb->unlinked) {
			ch374_hcd->curr_urb = urb;
			ch374_hcd->urb_done = 1;
			spin_unlock_irqrestore(&ch374_hcd->lock,
						       flags);
			return 1;
		}

		/*move current ep to tail*/
		list_move_tail(pos, &ch374_hcd->ep_list);
		curr_urb = urb;
		goto done;
	}
	
done:	
	if (!curr_urb) {
		spin_unlock_irqrestore(&ch374_hcd->lock, flags);
		return 0;
	}
	urb = ch374_hcd->curr_urb = curr_urb;
	epnum = usb_endpoint_num(&curr_urb->ep->desc);
	if (ch374_ep->retransmit) {
		ch374_ep->retransmit = 0;
	} else {	
		if (usb_endpoint_xfer_control(&ep->desc)) {
			/*
			 * See USB 2.0 spec section 8.6.1
			 * Initialization via SETUP Token:
			 */
			usb_settoggle(urb->dev, epnum, 0, 1);
			usb_settoggle(urb->dev, epnum, 1, 1);
			ch374_ep->pkt_state = PKT_STATE_SETUP;
			ch374_ep->curr_pkt = PKT_STATE_SETUP;
			ch374_hcd->controltog = true;
		} else{
			ch374_ep->pkt_state = PKT_STATE_TRANSFER;
			ch374_ep->curr_pkt = PKT_STATE_TRANSFER;
		}
	}
	spin_unlock_irqrestore(&ch374_hcd->lock, flags);

	/*
	 * set address
	 */
	ch374_set_address(hcd, urb->dev, epnum);
	DMBERR("ch374_next_transfer:select_urb epnum = %02x---\n", epnum);
	
	/************************************************/
	ch374_next_transfer(hcd);
	return 1;
}

/*
 * Caller must NOT hold HCD spinlock.
 * 接收数据
 */
 static int
ch374_recv_data_available(struct usb_hcd *hcd, struct urb *curr_urb)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	struct urb *urb = curr_urb;
	size_t remaining, transfer_size;
	u32 max_packet;
	u8 rcvLen;

	rcvLen = Read374Byte( REG_USB_LENGTH );
	
	if (rcvLen > MAX_FIFO_SIZE){
		rcvLen = MAX_FIFO_SIZE;
	}

	if (urb->actual_length >= urb->transfer_buffer_length)
		remaining = 0;
	else 
		remaining = urb->transfer_buffer_length - urb->actual_length;
	DMBERR("ch374_recv_data_available remaining = %d, rcvLen = %d\n\n", remaining, rcvLen);
	transfer_size = rcvLen;
	if (transfer_size > remaining) {
		transfer_size = remaining;
	}
	if (transfer_size > 0) {
		void *dst = urb->transfer_buffer + urb->actual_length;

		Read374Block( RAM_HOST_RECV, transfer_size, dst );
		urb->actual_length += transfer_size;
		ch374_hcd->curr_len = transfer_size;
	}

	if (urb->actual_length >= urb->transfer_buffer_length)
		return 1;	/* read is complete, so we're done */
	/*
	 * USB 2.0 Section 5.3.2 Pipes: packets must be full size
	 * except for last one.
	 */
	max_packet = usb_maxpacket(urb->dev, urb->pipe, 0);
	if (max_packet > MAX_FIFO_SIZE) {
		DMBERR("%s: packet-size of %u too big (limit is %u bytes)",
			__func__, max_packet, MAX_FIFO_SIZE);
		return -EINVAL;
	}

	if (rcvLen == 0 || (rcvLen & ( max_packet - 1 ))) {
		return 1;
	} else {
		return 0;
	}

}

static int
ch374_urb_done(struct usb_hcd *hcd)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	unsigned long flags;
	struct urb *urb;
	int status;

	status = ch374_hcd->urb_done;
	ch374_hcd->urb_done = 0;
	if (status > 0){
		status = 0;
	}
	urb = ch374_hcd->curr_urb;
	if (urb){
		ch374_hcd->curr_urb = NULL;
		spin_lock_irqsave(&ch374_hcd->lock, flags);
		usb_hcd_unlink_urb_from_ep(hcd, urb);
		spin_unlock_irqrestore(&ch374_hcd->lock, flags);
		
		/* must be called without the HCD spinlock: */
		usb_hcd_giveback_urb(hcd, urb, status);
	}
	return 1;
}

static void
ch374_slow_retransmit(struct usb_hcd *hcd)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	struct urb *urb = ch374_hcd->curr_urb;
	struct ch374_ep *ch374_ep;

	ch374_ep = urb->ep->hcpriv;
	ch374_ep->retransmit = 1;
	ch374_hcd->curr_urb = NULL;
}

static void
ch374_handle_error(struct usb_hcd *hcd, u8 result)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	u8 result_code = result & BIT_STAT_DEV_RESP;
	struct urb *urb = ch374_hcd->curr_urb;
	struct ch374_ep *ch374_ep = urb->ep->hcpriv;
	
	switch (result_code) {
	case DEF_USB_PID_ACK:
		return;			/* this shouldn't happen */
	case DEF_USB_PID_STALL:
		ch374_hcd->urb_done = -EPIPE;
		ch374_urb_done(hcd);
		finish_port_operate();
		break;

	case DEF_USB_PID_NAK:
		/*
		 * Device wasn't ready for data or has no data
		 * available: retry the packet again.
		 */
#if 1
		// mdelay(7);
		 if (ch374_ep->naks++ < NAK_MAX_FAST_RETRANSMITS) {
			ch374_next_transfer(hcd);
		 } else
			ch374_slow_retransmit(hcd);
#endif
		DMBERR("DEF_USB_PID_NAK\n\n");
		break;
	default:
		ch374_slow_retransmit(hcd);
		break;
	}

}

/*
 * Caller must NOT hold HCD spinlock.
 */
static int
ch374_transfer_out_done(struct usb_hcd *hcd, struct urb *urb)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	DMBERR("ch374_transfer_out_done\n");
	urb->actual_length += ch374_hcd->curr_len;
	if (urb->actual_length < urb->transfer_buffer_length)
		return 0;
	if (urb->transfer_flags & URB_ZERO_PACKET) {
		/*
		 * Some hardware needs a zero-size packet at the end
		 * of a bulk-out transfer if the last transfer was a
		 * full-sized packet (i.e., such hardware use <
		 * max_packet as an indicator that the end of the
		 * packet has been reached).
		 */
		u32 max_packet = usb_maxpacket(urb->dev, urb->pipe, 1);

		if (ch374_hcd->curr_len == max_packet)
			return 0;
	}
	return 1;
}

static int ch374_handle_pid_status(struct usb_hcd *hcd, int result)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	int retval = result & BIT_STAT_DEV_RESP;	
	switch(ch374_hcd->pid_state) {
	case CH374_DEF_USB_PID_SETUP:
	case CH374_DEF_USB_PID_OUT:
		if (retval == DEF_USB_PID_ACK) {
			return result;
		} else {
			return (retval | 0x20);
		}
		break;
	case CH374_DEF_USB_PID_IN:
		if (M_IS_HOST_IN_DATA(retval)) {
			if (result & BIT_STAT_TOG_MATCH) {
				 /*keep high four bits*/
				 result =  ( (result & BIT_HOST_PID_TOKEN) | DEF_USB_PID_ACK);
				return result;
			}
			
		} else if (retval == DEF_USB_PID_STALL || retval == DEF_USB_PID_NAK) {
			return (retval | 0x20);
		} else  if (!M_IS_HOST_TIMEOUT(retval)){
			return (retval | 0x20);
		}
		break;
	default:
		break;
	}
	return result;
}


static void 
ch374_setpid(struct urb *urb, bool tog)
{
	int epnum = usb_pipeendpoint(urb->pipe);
	Write374Byte( REG_USB_LENGTH, 0 );

	Write374Byte( REG_USB_H_PID, tog ? CH374_TOKEN_BULK_IN(epnum) : CH374_TOKEN_BULK_OUT(epnum));
	Write374Byte( REG_USB_H_CTRL, BIT_HOST_START | BIT_HOST_TRAN_TOG | BIT_HOST_RECV_TOG );	
}


/*
 * Caller must NOT hold HCD spinlock.
 */
 static void 
 ch374_host_transfer_done(struct usb_hcd *hcd)
 {
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	struct urb *urb = ch374_hcd->curr_urb;
	struct ch374_ep *ch374_ep;
	bool tog = false;
	u8 result_code, status;
	int urb_done = 0;
	//USB read PID
	status = Read374Byte( REG_USB_STATUS );

	result_code = ch374_handle_pid_status(hcd, status);	

	result_code = result_code & BIT_STAT_DEV_RESP;
	DMBERR("ch374_host_transfer_done:status = %02x,result_code = %02x\n", status, result_code);	
	if (unlikely(result_code != DEF_USB_PID_ACK)) {
		ch374_handle_error(hcd, result_code);
		DMBERR("ch374_host_transfer_done:ch374_handle_error.\n\n");
		return;
	}

	if (urb->ep->hcpriv == NULL) {
		DMBERR("ch374_host_transfer_done:hcpriv NULL\n\n");
		ch374_hcd->curr_urb = NULL;
		return;
	}
	ch374_ep = urb->ep->hcpriv;

	ch374_ep->naks = 0;
	ch374_ep->retries = 0;

	if (ch374_ep->curr_pkt == PKT_STATE_SETUP && ch374_hcd->isLastSetup == true) {
		ch374_hcd->isLastSetup = false;
		urb_done = 1;
		ch374_hcd->urb_done = urb_done;

		ch374_urb_done(hcd);
		finish_port_operate();
		return;
	}
	
	switch (ch374_ep->pkt_state) {
	case PKT_STATE_SETUP:
		if (urb->transfer_buffer_length > 0) {
			ch374_ep->pkt_state = PKT_STATE_TRANSFER;
		} else {
			if (usb_urb_dir_in(urb)) {
				tog = false;
			} else {
				/*urb dir is out and transfer length = 0*/
				tog = true;
			}
			ch374_hcd->isLastSetup = true;
			urb_done = 1;
		}
		break;

	case PKT_STATE_TRANSFER:
		if (usb_urb_dir_in(urb)) {
			urb_done = ch374_recv_data_available(hcd, urb);
			if (ch374_ep->curr_pkt == PKT_STATE_SETUP) {
				ch374_hcd->controltog  = ch374_hcd->controltog ? false : true;
				if (urb_done) {
					ch374_hcd->isLastSetup = true;
					ch374_hcd->controltog = true;
					tog = false;
				}
			} else {
				ch374_hcd->transfertog = ch374_hcd->transfertog ? false : true;
			}
		} else {
			urb_done = ch374_transfer_out_done(hcd, urb);
			if (ch374_ep->curr_pkt == PKT_STATE_SETUP) {
				ch374_hcd->controltog  = ch374_hcd->controltog ? false : true;
				if (urb_done) {
					ch374_hcd->isLastSetup = true;
					ch374_hcd->controltog = true;
					tog = true;
				}
			} else {
				ch374_hcd->transfertog = ch374_hcd->transfertog ? false : true;
			}
		}

		break;
	default:
		break;	
	}
	
	if (urb_done && ch374_ep->curr_pkt == PKT_STATE_SETUP && ch374_hcd->isLastSetup == true) {
		urb_done = 0;
		if (tog) {
			ch374_hcd->pid_state = CH374_DEF_USB_PID_IN;
		} else {
			ch374_hcd->pid_state = CH374_DEF_USB_PID_OUT;
		}
		ch374_setpid(urb, tog);		
	} else if (urb_done) {
		ch374_hcd->urb_done = urb_done;
		ch374_urb_done(hcd);
		finish_port_operate();
	} else {
		DMBERR("ch374_host_transfer_done:ch374_next_transfer\n\n");
		ch374_next_transfer(hcd);
	}
 }

/*
 * Caller must NOT hold HCD spinlock.
 */
 static void
 ch374_detect_conn(struct usb_hcd *hcd)
 {
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);

	if( Read374Byte( REG_HUB_SETUP ) & BIT_HUB0_ATTACH && (ch374_hcd->port_status.wPortStatus == 0)) {
		DMBERR("\n\n****-----------------connect---------------------***\n\n");
		enable_hub_port();
		ch374_hcd->port_status.wPortStatus |= USB_PORT_STAT_CONNECTION;
		ch374_hcd->port_status.wPortChange |= USB_PORT_STAT_C_CONNECTION;
		ch374_hcd->wPortTrigger = RH_PORT_STATUS_CHANGED;

		if(!(Read374Byte(REG_INTER_FLAG) & BIT_HUB0_DX_IN)) {
			ch374_hcd->port_status.wPortStatus |= USB_PORT_STAT_LOW_SPEED;
			Write374Byte(REG_HUB_SETUP, Read374Byte(REG_HUB_SETUP) | BIT_HUB0_POLAR);
			Write374Byte(REG_USB_SETUP, Read374Byte(REG_USB_SETUP) | BIT_SETP_LOW_SPEED | BIT_SETP_AUTO_SOF);	
		} else {
			ch374_hcd->port_status.wPortStatus &= ~(USB_PORT_STAT_LOW_SPEED); 
		}
		mdelay(100);

	} else if (!(Read374Byte( REG_HUB_SETUP ) & BIT_HUB0_ATTACH) && ch374_hcd->port_status.wPortStatus){
		DMBERR("\n\n****-----------------disconnect---------------------***\n\n");
		Write374Byte(REG_SYS_CTRL, Read374Byte(REG_SYS_CTRL) | BIT_CTRL_RESET_NOW);
		mdelay(50);
		Write374Byte(REG_SYS_CTRL, Read374Byte(REG_SYS_CTRL) &  ~BIT_CTRL_RESET_NOW);
		disable_hub_port(hcd);
		init_ch374_host(); 
		
		ch374_hcd->curr_urb = NULL;
		ch374_hcd->curr_len = 0;
	}
	ch374_hcd->controltog = 0;
	ch374_hcd->transfertog = 0;
	if (test_and_clear_bit(ENABLE_IRQ, &ch374_hcd->todo))
		enable_irq(ch374_hcd->irq);
	Write374Byte( REG_INTER_FLAG, BIT_IF_DEV_DETECT | BIT_IF_USB_PAUSE | BIT_IF_TRANSFER | BIT_IF_USB_SUSPEND | BIT_IF_WAKE_UP );
}
/**
  * check connection
  */
static void
ch374_check_connect(struct usb_hcd *hcd)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	set_bit(CHECK_CONNECT, &ch374_hcd->todo);
	wake_up_process(ch374_hcd->spi_thread);
}

/**
 * interrupt handler
 */
static irqreturn_t
ch374_irq_handler(int irq, void *dev_id)
{
	struct usb_hcd *hcd = dev_id;
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	ch374_hcd->interrupt = INTERRUPT;
	DMBERR("\n\n&&&&&&&&&&&&ch374_irq_handler&&&&&&&&&&&&\n\n");
	if (ch374_hcd->spi_thread &&
	    ch374_hcd->spi_thread->state != TASK_RUNNING)
		wake_up_process(ch374_hcd->spi_thread);

	if (!test_and_set_bit(ENABLE_IRQ, &ch374_hcd->todo))
		disable_irq_nosync(ch374_hcd->irq);
	return IRQ_HANDLED;
}

/* Return zero if no work was performed, 1 otherwise.  */
static int
ch374_handle_irqs(struct usb_hcd *hcd)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	u8 interript;
	int i = 3;

	if (ch374_hcd->interrupt == UNINTERRUPT) {
		return 0;
	} else {
		ch374_hcd->interrupt = UNINTERRUPT;
	}

	//udelay(200);
	interript = Read374Byte(REG_INTER_FLAG);
	while(interript & 0x2){
		interript = Read374Byte(REG_INTER_FLAG);
		if(i-- == 0)
			break;
	}

	DMBERR("*******************interript = %02x*************\n", interript);
	if (interript & BIT_IF_DEV_DETECT) {
		/*
 		 * set 1 to clear BIT_IF_DEV_DETECT  
		 */
		ch374_check_connect(hcd);
	} else if (interript & BIT_IF_TRANSFER) { 
		ch374_host_transfer_done(hcd);
		Write374Byte( REG_INTER_FLAG, BIT_IF_USB_PAUSE | BIT_IF_TRANSFER );
		if (!(Read374Byte(REG_HUB_SETUP) & BIT_HUB0_ATTACH)) {
			ch374_detect_conn(hcd);
		}
	} else {
		Write374Byte( REG_INTER_FLAG, BIT_IF_USB_PAUSE | BIT_IF_INTER_FLAG );
		return 0;
	}

	/*
	 * Now process interrupts that may affect HCD state
	 * other than the end-points:
	 */

	return 1;
}

static int ch374_check_unlink(struct usb_hcd *hcd)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	struct list_head *pos, *upos, *next_upos;
	struct ch374_ep *ch374_ep;
	struct urb *urb;
	struct usb_host_endpoint *ep;

	unsigned long flags;
	int retval = 0;

	spin_lock_irqsave(&ch374_hcd->lock, flags);

	list_for_each(pos, &ch374_hcd->ep_list) {
		ch374_ep = container_of(pos, struct ch374_ep, ep_list);
		ep = ch374_ep->ep;
		list_for_each_safe(upos, next_upos, &ep->urb_list) {
			urb = container_of(upos, struct urb, urb_list);
			if (urb->unlinked) {
				retval = 1;
				usb_hcd_unlink_urb_from_ep(hcd, urb);
				spin_unlock_irqrestore(&ch374_hcd->lock,
						       flags);
				usb_hcd_giveback_urb(hcd, urb, 0);
				spin_lock_irqsave(&ch374_hcd->lock, flags);
			}
		}
	}
	spin_unlock_irqrestore(&ch374_hcd->lock, flags);
	return retval;
}

static int 
ch374_spi_thread(void *dev_id) 
{
	struct usb_hcd *hcd = dev_id;
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	int  i_worked = 1;

	while(!kthread_should_stop()) {
		if (ch374_hcd->rh_state == CH374_RH_RUNNING) {
			break;
		}
		msleep(10000);
	}

	while(!kthread_should_stop()) {
		if (!i_worked){
			set_current_state(TASK_INTERRUPTIBLE);
			if (test_and_clear_bit(ENABLE_IRQ, &ch374_hcd->todo)){
				enable_irq(ch374_hcd->irq);
			}
			schedule();
			__set_current_state(TASK_RUNNING);
		}
		
		i_worked = 0;
		
		if (ch374_handle_irqs(hcd)){
			i_worked = 1;
		} else if (!ch374_hcd->curr_urb){
			if (ch374_hcd->port_status.wPortStatus) {
				i_worked |= ch374_select_and_start_urb(hcd);
			}			
		}

		if (test_and_clear_bit(RESET_PORT, &ch374_hcd->todo)) {
			/* perform a USB bus reset: */
			reset_hub_port();
			enable_hub_port();
			change_hub_port_speed(hcd);
			i_worked = 1;
		}

		if (test_and_clear_bit(CHECK_CONNECT, &ch374_hcd->todo)) {
			ch374_detect_conn(hcd);
			i_worked = 1;
		}

		if (test_and_clear_bit(CHECK_UNLINK, &ch374_hcd->todo)) {
			i_worked |= ch374_check_unlink(hcd);
		}
	
	}
	set_current_state(TASK_RUNNING);
	DMBERR("SPI thread exiting");
	return 0;	
}

static int ch374_get_frame(struct usb_hcd *hcd)
{
	struct ch374_hcd *ch374 = hcd_to_ch374(hcd);
	ch374->frame_number = 0;
	return ch374->frame_number;
}

#ifdef	CONFIG_PM
static int ch374_bus_suspend(struct usb_hcd *hcd)
{
#if 0
	struct spi_ch37x_hcd_data *hcd_data = g_hcd_data;
	struct device_state_pm_state *pm_state;
	int ret = 0;

	/* close hcd power, disable hcd clks */
	pm_state = get_device_pm_state(hcd_data->pm_platdata, PM_STATE_D3);
	ret = device_state_pm_set_state(hcd_data->dev, pm_state);
	if (ret	< 0) {
		dev_err(hcd_data->dev, "pm set state disable failed: %d", ret);
		return -EINVAL;
	}
	pr_info("%s:exit %s\n",__func__, pm_state->name);
#endif
	return 0;
}

static int ch374_bus_resume(struct usb_hcd *hcd)
{
#if 0
	struct spi_ch37x_hcd_data *hcd_data = g_hcd_data;
	struct device_state_pm_state *pm_state;
	int ret = 0;

	/* restore hcd power, disable hcd clks */
	pm_state = get_device_pm_state(hcd_data->pm_platdata, PM_STATE_D0);
	ret = device_state_pm_set_state(hcd_data->dev, pm_state);
	if (ret < 0) {
		dev_err(hcd_data->dev, "pm set state disable failed: %d", ret);
		return -EINVAL;
	}
	pr_info("%s:exit %s\n",__func__, pm_state->name);
#endif
	return 0;
}

#else

#define	ch374_bus_suspend	NULL
#define	ch374_bus_resume	NULL

#endif

static int
ch374_map_urb_for_dma(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags)
{
	return 0;
}

static void
ch374_unmap_urb_for_dma(struct usb_hcd *hcd, struct urb *urb)
{
}

static void 
ch374_endpoint_disable(struct usb_hcd *hcd, struct usb_host_endpoint *hep)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	unsigned long flags;
	spin_lock_irqsave (&ch374_hcd->lock, flags);	
	if(hep->hcpriv) {
		struct ch374_ep *ch374_ep = hep->hcpriv;
		if (!list_empty(&ch374_ep->ep_list))
			list_del(&ch374_ep->ep_list);
		kfree(ch374_ep);
		hep->hcpriv = NULL; 	
	}
	spin_unlock_irqrestore (&ch374_hcd->lock, flags);
	return;
}

static inline void
ch374_hub_descriptor(struct usb_hub_descriptor *desc)
{
	/*
	 * See Table 11-13: Hub Descriptor in USB 2.0 spec.
	 */
	desc->bDescriptorType = 0x29; /* hub descriptor */
	desc->bDescLength = 9;
	desc->bHubContrCurrent = 0;
	// Power switching, device type, overcurrent.
	desc->wHubCharacteristics = 0x12; // 
	desc->bPwrOn2PwrGood = 10; // 20ms
	desc->bNbrPorts = 1;

	// two bitmaps:  ports removable, and legacy PortPwrCtrlMask
	desc->u.hs.DeviceRemovable[0] = 0;
	desc->u.hs.DeviceRemovable[1] = ~0;
}

static int ch374_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	unsigned long flags;
	int retval = 0;
	if (!HC_IS_RUNNING(hcd->state)) return -ESHUTDOWN;
	// Report no status change now, if we are scheduled to be called later
	if (timer_pending(&hcd->rh_timer)) return 0;
	spin_lock_irqsave(&ch374_hcd->lock, flags);	

	if (ch374_hcd->wPortTrigger) {
		buf[0] = 1 + (1<<1);
		retval = 1;
		ch374_hcd->wPortTrigger = RH_PORT_STATUS_STAYED;
	} else {
		buf[0] = 0;
		retval = 0;
	}
	spin_unlock_irqrestore(&ch374_hcd->lock, flags);
	return retval;
}

static int
ch374_reset_port(struct usb_hcd *hcd)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	ch374_hcd->port_status.wPortStatus &= ~(USB_PORT_STAT_RESET);	
	ch374_hcd->port_status.wPortStatus |= USB_PORT_STAT_ENABLE;
	ch374_hcd->port_status.wPortChange |= USB_PORT_STAT_C_RESET;

	set_bit(RESET_PORT, &ch374_hcd->todo);
	wake_up_process(ch374_hcd->spi_thread);
	return 0;
}

static int 
ch374_hub_control(struct usb_hcd *hcd, u16 typeReq, u16 wValue, u16 wIndex, char *buf, u16 wLength)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	int retval = 0;
	unsigned long flags;
	short *wdata_buf = (short *)buf;
	spin_lock_irqsave(&ch374_hcd->lock, flags);
	
	switch (typeReq) 
	{
	case ClearHubFeature:
		switch (wValue) 
		{
		case C_HUB_OVER_CURRENT:
		case C_HUB_LOCAL_POWER:
			break;
		default:
			goto error;
		}
		break;
	case SetHubFeature:
		switch (wValue) 
		{
		case C_HUB_OVER_CURRENT:
		case C_HUB_LOCAL_POWER:
			break;
		default:
			goto error;
		}
		break;
	case GetHubDescriptor:
		ch374_hub_descriptor((struct usb_hub_descriptor *)buf);
		break;
	case GetHubStatus:
		*(__le32 *) buf = cpu_to_le32(0);
		break;
	case GetPortStatus:
		if (!wIndex){
			retval = -EPIPE;
			goto error;
		} 
		*wdata_buf = cpu_to_le16(ch374_hcd->port_status.wPortStatus);
		wdata_buf++;
		*wdata_buf = cpu_to_le16(ch374_hcd->port_status.wPortChange);
		break;
	case ClearPortFeature:
		switch (wValue) {
		case USB_PORT_FEAT_ENABLE:
			if(ch374_hcd->port_status.wPortStatus & USB_PORT_STAT_CONNECTION){
				ch374_hcd->port_status.wPortStatus &= ~USB_PORT_STAT_ENABLE;
			}
			break;
		case USB_PORT_FEAT_C_ENABLE:
			ch374_hcd->port_status.wPortChange &= ~USB_PORT_STAT_C_ENABLE;
			break;
		case USB_PORT_FEAT_SUSPEND:
			break;
		case USB_PORT_FEAT_C_SUSPEND:
			ch374_hcd->port_status.wPortChange &= ~USB_PORT_STAT_C_SUSPEND;
			break;
		case USB_PORT_FEAT_POWER:
			ch374_hcd->port_status.wPortStatus &= ~USB_PORT_STAT_POWER;
			break;
		case USB_PORT_FEAT_C_CONNECTION:
			ch374_hcd->port_status.wPortChange &= ~USB_PORT_STAT_C_CONNECTION;
			break;
		case USB_PORT_FEAT_C_OVER_CURRENT:
			ch374_hcd->port_status.wPortChange &= ~USB_PORT_STAT_C_OVERCURRENT;
			break;
		case USB_PORT_FEAT_C_RESET:
			ch374_hcd->port_status.wPortChange &= ~USB_PORT_STAT_C_RESET;
			break;
		default:
			goto error;
		}
		
	case SetPortFeature:
		switch (wValue) 
		{
		case USB_PORT_FEAT_SUSPEND:
			break;
		case USB_PORT_FEAT_POWER:
			break;
		case USB_PORT_FEAT_RESET:
			/*reset port*/
			ch374_reset_port(hcd);			
			break;
		default:
			goto error;
		}
		break;
			
	default:
		DMBMSG("hub control req%04x v%04x i%04x \n",
			typeReq, wValue, wIndex);
error:	/* "protocol stall" on error */
		retval = -EPIPE;
	}
	spin_unlock_irqrestore(&ch374_hcd->lock, flags);
	return retval;
}

static int 
ch374_urb_enqueue( struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	struct ch374_ep *ch374_ep;
	unsigned long flags;
	int retval = 0;

	DMBERR("ch374_urb_enqueue: get flags.\n");
	spin_lock_irqsave(&ch374_hcd->lock, flags);
	if ((ch374_hcd->port_status.wPortStatus & USB_PORT_STAT_CONNECTION) == USB_PORT_STAT_CONNECTION) {
		ch374_ep = urb->ep->hcpriv;
		if (!ch374_ep) {
			ch374_ep = kzalloc(sizeof(struct ch374_ep), GFP_ATOMIC);
			if (!ch374_ep) {
				retval = -ENOMEM;
				goto out;
			}
			ch374_ep->ep = urb->ep;
			urb->ep->hcpriv = ch374_ep;
			list_add_tail(&ch374_ep->ep_list, &ch374_hcd->ep_list);
		}

		retval = usb_hcd_link_urb_to_ep(hcd, urb);
		if (retval == 0) {
			wake_up_process(ch374_hcd->spi_thread);
		}
	}
	DMBERR("ch374_urb_enqueue:end\n");
out:
	spin_unlock_irqrestore(&ch374_hcd->lock, flags);
	return retval;
}

static int 
ch374_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	unsigned long flags;
	int retval = 0;

	spin_lock_irqsave(&ch374_hcd->lock, flags);
	retval = usb_hcd_check_unlink_urb(hcd, urb, status);
	DMBERR("\n\n-ch374_urb_dequeue: retval:%d\n\n", retval);
	if (retval == 0) {
		set_bit(CHECK_UNLINK, &ch374_hcd->todo);
		wake_up_process(ch374_hcd->spi_thread);
	}
	spin_unlock_irqrestore(&ch374_hcd->lock, flags);
	return 0;
}

static int ch374_start(struct usb_hcd *hcd)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	DMBMSG("ch374_start start\n");
	hcd->state = HC_STATE_RUNNING;
	spin_lock_init(&ch374_hcd->lock);
	ch374_hcd->rh_state = CH374_RH_RUNNING;

	INIT_LIST_HEAD(&ch374_hcd->ep_list);

	hcd->state = HC_STATE_RUNNING;
	
	ch374_hcd->port_status.wPortStatus = 0;
	ch374_hcd->port_status.wPortChange = 0;
	ch374_hcd->wPortTrigger = RH_PORT_STATUS_STAYED;

	ch374_hcd->controltog = 0;
	ch374_hcd->transfertog = 0;

	init_ch374_host(); 
	/**
	 * set interrupt mode
	 */
	Write374Byte( REG_SYS_CTRL, Read374Byte(REG_SYS_CTRL) | 0x20 );  
	
	DMBMSG("ch374_start end,%08x\n",Read374Byte(REG_SYS_CTRL));
	return 0; 
}


static void ch374_stop(struct usb_hcd *hcd)
{
	struct ch374_hcd *ch374_hcd = hcd_to_ch374(hcd);
	unsigned long flags;
	spin_lock_irqsave(&ch374_hcd->lock,flags);
	hcd->state = HC_STATE_HALT;
	spin_unlock_irqrestore(&ch374_hcd->lock,flags);	
	DMBMSG("ch374_stop end\n");
}

static int ch374_reset(struct usb_hcd *hcd)
{
	return 0;
}

#ifdef CONFIG_OF
static struct xgold_spi_chip *spi_ch37x_hcd_parse_dt(struct device *dev)
{
	u32 temp;
	struct xgold_spi_chip *spi_chip_data; 

	spi_chip_data = devm_kzalloc(dev, sizeof(*spi_chip_data), GFP_KERNEL);
	if (!spi_chip_data) {
		dev_err(dev, "memory allocation for spi_chip_data failed\n");
		return ERR_PTR(-ENOMEM);
	}
	
	if (of_property_read_u32(dev->of_node, "poll_mode", &temp)) {
		dev_warn(dev, "fail to get poll_mode, default set 0\n");
		spi_chip_data->poll_mode = 0;
	} else {
		spi_chip_data->poll_mode = temp;
	}

	if (of_property_read_u32(dev->of_node, "type", &temp)) {
		dev_warn(dev, "fail to get type, default set 0\n");
		spi_chip_data->type = 0;
	} else {
		spi_chip_data->type = temp;
	}

	if (of_property_read_u32(dev->of_node, "enable_dma", &temp)) {
		dev_warn(dev, "fail to get enable_dma, default set 0\n");
		spi_chip_data->enable_dma = 0;
	} else {
		spi_chip_data->enable_dma = temp;
	}

	return spi_chip_data;
}
#else
static struct spi_board_info *spi_ch37x_hcd_parse_dt(struct device *dev)
{
	return dev->platform_data;
}
#endif

static struct hc_driver ch374_hc_driver = {
	.description = hcd_name,
	.product_desc = product_desc,
	.hcd_priv_size = sizeof(struct ch374_hcd),
	.flags = HCD_USB11,

	.reset = ch374_reset,
	.start = ch374_start,
	.stop = ch374_stop,

	.urb_enqueue = ch374_urb_enqueue, 
	.urb_dequeue = ch374_urb_dequeue,
	.map_urb_for_dma = ch374_map_urb_for_dma,
	.unmap_urb_for_dma = ch374_unmap_urb_for_dma,
	.endpoint_disable = ch374_endpoint_disable,

	.get_frame_number = ch374_get_frame,

	.hub_status_data = ch374_hub_status_data,
	.hub_control = ch374_hub_control,
	.bus_suspend = ch374_bus_suspend,
	.bus_resume = ch374_bus_resume,
};

static int ch374_probe(struct spi_device *spi)
{
	int retval = -ENOMEM;
	struct ch374_hcd *ch374_hcd;
	struct usb_hcd *hcd = NULL;
	static struct xgold_spi_chip *spi_chip_data;
	struct spi_ch37x_hcd_data *spi_hcd_data;
	struct device_node *np = of_node_get(spi->dev.of_node);
	struct device_state_pm_state *pm_state;
	int ret = 0;

	if(!spi)	
		return -ENOMEM;

	if (!spi_chip_data && np) {
		spi_chip_data = spi_ch37x_hcd_parse_dt(&spi->dev);
		if (IS_ERR(spi_chip_data))
		return -ENOMEM;
	}

	spi_hcd_data = (struct spi_ch37x_hcd_data *)kzalloc(sizeof(struct spi_ch37x_hcd_data), GFP_KERNEL);
        if(!spi_hcd_data){
                dev_err(&spi->dev, "ERR: no memory for spi_ch37x_hcd_data\n");
                return -ENOMEM;
        }
	spi->bits_per_word = 8;	
	spi->controller_data = spi_chip_data;

	spi_hcd_data->spi = spi;
	spi_hcd_data->dev = &spi->dev;

	retval = spi_setup(spi);
	if (retval < 0){
		dev_err(spi_hcd_data->dev, "ERR: fail to setup spi\n");
		return -1;
	}

	spi_hcd_data->irq = spi->irq;

	/* Get power states */
	spi_hcd_data->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(spi_hcd_data->pm_platdata)) {
		dev_err(spi_hcd_data->dev, "could not get platform PM data\n");

		return -EINVAL;
	}

	ret = device_state_pm_set_class(spi_hcd_data->dev,
				spi_hcd_data->pm_platdata->pm_user_name);
	if (IS_ERR_VALUE(ret)) {
		dev_err(spi_hcd_data->dev, "registering to PM class failed\n");
		return -EINVAL;
	}

	ret = device_pm_get_states_handlers(spi_hcd_data->dev, spi_hcd_data->pm_platdata);
	if (IS_ERR_VALUE(ret)) {
		dev_err(spi_hcd_data->dev, "errors while retrieving PM states\n");
		return -EINVAL;
	}

	/*enable hcd clks */
	pm_state = get_device_pm_state(spi_hcd_data->pm_platdata, PM_STATE_D0);
	ret = device_state_pm_set_state(spi_hcd_data->dev, pm_state);
	if (ret < 0) {
		dev_err(spi_hcd_data->dev, "pm set state disable failed: %d", ret);
		return -EINVAL;
	}

	spi_set_drvdata(spi, spi_hcd_data);

	//atomic_set(&spi_hcd_data->debug_flag, 1);
	if (usb_disabled()) 
		return -ENODEV;

	spi_hcd_data->hcd = usb_create_hcd(&ch374_hc_driver, spi_hcd_data->dev, dev_name(spi_hcd_data->dev));

	g_hcd_data = spi_hcd_data;
	retval = usb_add_hcd(spi_hcd_data->hcd, 0, 0);
	if (retval != 0) 
		goto error;

	ch374_hcd = hcd_to_ch374(spi_hcd_data->hcd);
	ch374_hcd->irq = spi->irq;

	spi_hcd_data->ch374_hcd = ch374_hcd;

	INIT_LIST_HEAD(&ch374_hcd->ep_list);

	ch374_hcd->spi_thread = kthread_run(ch374_spi_thread, spi_hcd_data->hcd,
					      "ch374_spi_thread");

	if (ch374_hcd->spi_thread == ERR_PTR(-ENOMEM)) {
		DMBERR(
			"failed to create SPI thread (out of memory)\n");
		goto error;
	}

	retval = request_threaded_irq(spi_hcd_data->irq, ch374_irq_handler, NULL,
			     IRQF_TRIGGER_HIGH | IRQF_ONESHOT , "spi-ch37x-hcd", spi_hcd_data->hcd);
	if (retval < 0) {
		DMBERR("failed to request irq %d\n", spi_hcd_data->irq);
		goto error;
	}
	pr_info("%s: ok, pm name is %s\n",__func__, pm_state->name);
	return 0;
error:
	if (hcd) {
		if (ch374_hcd->spi_thread)
			kthread_stop(ch374_hcd->spi_thread);
		usb_put_hcd(hcd);
	}

	return retval;
}

static int  ch374_remove(struct spi_device *dev)
{
	struct spi_ch37x_hcd_data *hcd_data = spi_get_drvdata(dev);
	struct ch374_hcd *ch374_hcd = hcd_data->ch374_hcd;

	unsigned long flags;

	usb_remove_hcd(hcd_data->hcd);

	spin_lock_irqsave(&ch374_hcd->lock, flags);
	kthread_stop(ch374_hcd->spi_thread);
	spin_unlock_irqrestore(&ch374_hcd->lock, flags);

	free_irq(dev->irq,hcd_data->hcd);

	usb_put_hcd(hcd_data->hcd);
	return 0;
}
#ifdef CONFIG_OF
static const struct of_device_id spi_ch37x_hcd_dt_match[] = {
	{ .compatible = "intel,spi_ch37x_hcd", },
	{},
};
MODULE_DEVICE_TABLE(of, spi_ch37x_hcd_dt_match);

#endif
#ifdef	CONFIG_PM

/* for this device there's no useful distinction between the controller
 * and its root hub, except that the root hub only gets direct PM calls
 * when CONFIG_USB_SUSPEND is enabled.
 */

static int ch374_suspend(struct spi_device *spi, pm_message_t state)
{
#if 1
	struct spi_ch37x_hcd_data *hcd_data = g_hcd_data;//spi_get_drvdata(spi);	
	struct device_state_pm_state *pm_state;
	int ret = 0;

	disable_irq_nosync(hcd_data->irq);
	kthread_stop(hcd_data->ch374_hcd->spi_thread);
	Write374Byte( REG_SYS_CTRL, Read374Byte( REG_SYS_CTRL ) | BIT_CTRL_OSCIL_OFF );  // 时钟振荡器停止振荡,进入睡眠状态

	/* close hcd power, disable hcd clks */
	pm_state = get_device_pm_state(hcd_data->pm_platdata, PM_STATE_D3);
	pr_info("%s: pm name %s\n",__func__, pm_state->name);
	ret = device_state_pm_set_state(hcd_data->dev, pm_state);
	if (ret	< 0) {
		dev_err(hcd_data->dev, "pm set state disable failed: %d", ret);
		return -EINVAL;
	}
	pr_info("%s:exit %s\n",__func__, pm_state->name);
#endif
	return 0;
}

static int ch374_resume(struct spi_device *spi)
{
#if 1
	struct spi_ch37x_hcd_data *hcd_data = g_hcd_data;//spi_get_drvdata(spi);
	struct device_state_pm_state *pm_state;
	int ret = 0;
	
	/* restore hcd power, disable hcd clks */
	pm_state = get_device_pm_state(hcd_data->pm_platdata, PM_STATE_D0);
	ret = device_state_pm_set_state(hcd_data->dev, pm_state);
	if (ret < 0) {
		dev_err(hcd_data->dev, "pm set state disable failed: %d", ret);
		return -EINVAL;
	}
	mdelay(100);
	Write374Byte( REG_SYS_CTRL, Read374Byte(REG_SYS_CTRL ) &~ BIT_CTRL_OSCIL_OFF ); // 打开时钟振荡器，唤醒芯片
	enable_irq(hcd_data->irq);
	pr_info("%s:exit %s\n",__func__, pm_state->name);
#endif
	return 0;
}


#else

#define	ch374_suspend	NULL
#define	ch374_resume	NULL

#endif

static struct spi_driver ch374_driver = {
	.probe =	ch374_probe,
	.remove =	ch374_remove,
	.suspend =	ch374_suspend,
	.resume =	ch374_resume,
	.driver = {
		.name =	"spi_ch37x_hcd",//(char *) hcd_name,
		.owner = THIS_MODULE,
	},
};

static int __init ch374_init(void)
{
	int retval;
	if (usb_disabled())
		return -ENODEV;
	DMBMSG("driver %s, %s\n", hcd_name, DRIVER_VERSION);
	retval = spi_register_driver(&ch374_driver);
	return retval;
}

static void __exit ch374_exit(void)
{
	spi_unregister_driver(&ch374_driver);
}

late_initcall(ch374_init);
module_exit(ch374_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
