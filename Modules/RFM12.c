#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
 
 
#include "RFM12_config.h"
#include "RFM12.h"
 
MODULE_AUTHOR("Schoenlieb");
MODULE_DESCRIPTION("A Char Driver for the RFM12 Radio Transceiver");
MODULE_LICENSE("GPL");



typedef enum _rfm12_state_t {
   RFM12_STATE_NO_CHANGE		= 0,
   RFM12_STATE_CONFIG			= 1,
   RFM12_STATE_SLEEP			= 2,
   RFM12_STATE_IDLE			= 3,
   RFM12_STATE_LISTEN			= 4,
   RFM12_STATE_RECV			= 5,
   RFM12_STATE_RECV_FINISH		= 6,
   RFM12_STATE_SEND_PRE1	    	= 7,
   RFM12_STATE_SEND_PRE2		= 8,
   RFM12_STATE_SEND_PRE3		= 9,
   RFM12_STATE_SEND_SYN1		= 10,
   RFM12_STATE_SEND_SYN2	    	= 11,
   RFM12_STATE_SEND			= 12,
   RFM12_STATE_SEND_TAIL1		= 13,
   RFM12_STATE_SEND_TAIL2		= 14,
   RFM12_STATE_SEND_TAIL3		= 15
} rfm12_state_t;

struct rfm12_spi_message {
   struct spi_message   spi_msg;
   struct spi_transfer  spi_transfers[4];
   rfm12_state_t        spi_finish_state;
   u8                   spi_tx[8], spi_rx[8];
   void*                context;
   u8                   pos;
};

struct rfm12_data {
	u16	 	 	irq;
	int			irq_identifier;

	dev_t			devt;
	spinlock_t		rfm12_lock;
	struct spi_device*	spi;
	struct list_head	device_entry;

	u8                   	open;
	u8			should_release;
	rfm12_state_t        	state;
	u8			group_id, band_id;
	unsigned long        	bytes_recvd, pkts_recvd;
	unsigned long        	bytes_sent, pkts_sent;
	unsigned long        	num_recv_overflows, num_recv_timeouts, num_recv_crc16_fail;
	unsigned long		num_send_underruns, num_send_timeouts;
	u8*                  	in_buf, *in_buf_pos;
	u8*                  	out_buf, *out_buf_pos;
	u8*                  	in_cur_len_pos;
	u8*                  	in_cur_end, *out_cur_end;
	u16			crc16, last_status;
	int                  	in_cur_num_bytes, out_cur_num_bytes;
	struct rfm12_spi_message spi_msgs[NUM_MAX_CONCURRENT_MSG];
	u8                   	free_spi_msgs;
	struct timer_list    	rxtx_watchdog;
	u8                   	rxtx_watchdog_running;
	struct timer_list	retry_sending_timer;
	u8			retry_sending_running;
};


static irqreturn_t input_ISR (int irq, void *data);
static int rfm12_send_generic_async_cmd( uint16_t* cmds,int num_cmds, uint16_t delay_usecs, void (*callback)(void*),rfm12_state_t finish_state);
static int rfm12_finish_sending(struct rfm12_data* rfm12, int success);
static void rfm12_send_spi_completion_handler(void *arg);
static void rfm12_spi_completion_common(struct rfm12_spi_message* msg);


struct spi_device *spi_device;
static struct gpio input[] = { { IRQ_PIN, GPIOF_IN, "INPUT" },};
static int input_irqs[] = { -1 };
static struct gpio leds[] = {
    { ON_LED, GPIOF_OUT_INIT_HIGH, "ON" },
    { RX_LED, GPIOF_OUT_INIT_HIGH, "RX" },
    { TX_LED, GPIOF_OUT_INIT_HIGH, "TX" },
};

struct rfm12_data dev_data;
struct spi_master* spi_master;
struct spi_device* spi_device;



struct spi_transfer
rfm12_make_spi_transfer(uint16_t cmd, u8* tx_buf, u8* rx_buf)
{
   struct spi_transfer tr = {
	  .tx_buf           = tx_buf,
	  .rx_buf           = rx_buf,
	  .len              = 2,
	  .cs_change        = 0,
	  .bits_per_word    = 8,
	  .delay_usecs      = 0,
	  .speed_hz         = SPI_BUS_SPEED, //other value here is 2500000
   };

   tx_buf[0] = (cmd >> 8) & 0xff;
   tx_buf[1] = cmd & 0xff;

   return tr;
}

static int
rfm12_write_tx_byte(struct rfm12_data* rfm12, u8 tx_byte)
{
	uint16_t cmds[2];
	
	cmds[0] = RF_READ_STATUS;
	cmds[1] = RF_TXREG_WRITE | tx_byte;
	
	return rfm12_send_generic_async_cmd(cmds, 2,
		WRITE_TX_WAIT, rfm12_send_spi_completion_handler,
		RFM12_STATE_NO_CHANGE);
}

static irqreturn_t input_ISR (int irq, void *data)
{
	printk(KERN_INFO "IRQ called \n");
	
	struct rfm12_data *rfm12;
	rfm12 = &dev_data;
	
	spin_lock(&rfm12->rfm12_lock);
	
	printk(KERN_INFO "state is = %d \n",rfm12->state);
	

	switch (rfm12->state) {
	  case RFM12_STATE_LISTEN:
	  case RFM12_STATE_RECV:
		  printk(KERN_INFO "No receive state = %d \n",rfm12->state);
//		 (void)rfm12_request_fifo_byte(rfm12);
		 break;
	  case RFM12_STATE_SEND_PRE1:
	  case RFM12_STATE_SEND_PRE2:
	  case RFM12_STATE_SEND_PRE3:
	  case RFM12_STATE_SEND_TAIL1:
	  case RFM12_STATE_SEND_TAIL2:
	     (void)rfm12_write_tx_byte(rfm12, 0xAA);
	     break;
	  case RFM12_STATE_SEND_TAIL3: {
		uint16_t cmd = RF_IDLE_MODE;
	  	(void)rfm12_send_generic_async_cmd(&cmd, 1, 0,
	  		NULL, RFM12_STATE_NO_CHANGE);
	  	(void)rfm12_write_tx_byte(rfm12, 0xAA);
	  	break;
	  }
	  case RFM12_STATE_SEND_SYN1:
		 printk(KERN_INFO "RFM12_STATE_SEND_SYN1\n");
	  	 (void)rfm12_write_tx_byte(rfm12, 0x2D);
	  	 break;
	  case RFM12_STATE_SEND_SYN2:
		  printk(KERN_INFO "RFM12_STATE_SEND_SYN2\n");
	  	 (void)rfm12_write_tx_byte(rfm12, rfm12->group_id);
	  	 break;
	  case RFM12_STATE_SEND:
		printk(KERN_INFO "RFM12_STATE_SEND\n");
	     (void)rfm12_write_tx_byte(rfm12, *rfm12->out_buf_pos++);
	     break;
	  default: {
		  
		  printk(KERN_INFO "irq status is fishy\n");
		 uint16_t cmd = RF_READ_STATUS; 
		 (void)rfm12_send_generic_async_cmd( &cmd, 1,
		 	0, NULL, RFM12_STATE_NO_CHANGE);
		 break;
	  }
   }

   spin_unlock(&rfm12->rfm12_lock);
	
	return IRQ_HANDLED;
}
static void
rfm12_finish_send_or_recv_callback(void *arg)
{
   unsigned long flags;
   struct rfm12_spi_message* spi_msg =
	  (struct rfm12_spi_message*)arg; 
   struct rfm12_data* rfm12 =
	  (struct rfm12_data*)spi_msg->context;
   u8 should_release = 0;
   
   printk(KERN_INFO "rfm12_finish_send_or_recv_callback is called\n");
   
   
   spin_lock_irqsave(&rfm12->rfm12_lock, flags);

   rfm12_spi_completion_common(spi_msg);
 
   if (!(should_release = rfm12->should_release)) {
//	  rfm12_release_when_safe(rfm12);
  // } else {
	//rfm12_begin_sending_or_receiving(rfm12);
	   printk(KERN_INFO "rfm12_begin_sending_or_receiving\n");
   }

   spin_unlock_irqrestore(&rfm12->rfm12_lock, flags);
   
//   if (!should_release)
//      platform_irq_handled(rfm12->irq_identifier);
}


static int
rfm12_finish_send_recv_common(struct rfm12_data* rfm12)
{
	uint16_t cmds[3];
	printk(KERN_INFO "rfm12_finish_send_recv_common is called\n");
	
	
	cmds[0] = RF_READ_STATUS;
	cmds[1] = RF_IDLE_MODE;
	cmds[2] = RF_TXREG_WRITE | 0xAA;
	
	rfm12->state = RFM12_STATE_IDLE;
	
	return rfm12_send_generic_async_cmd( cmds, 3,
		0, rfm12_finish_send_or_recv_callback, RFM12_STATE_NO_CHANGE);
}

static int
rfm12_finish_sending(struct rfm12_data* rfm12, int success)
{
   int err = 0, len = 0;
   
  // rfm12_update_rxtx_watchdog(rfm12, 1);
   printk(KERN_INFO "rfm12_finish_sending is called\n");
      
   if (success) {
	   len = rfm12->out_buf[1] + RF_EXTRA_LEN;
	   
	   memmove(rfm12->out_buf,
	   		   rfm12->out_buf + len,
	   		   DATA_BUF_SIZE - len
	   );
	   
	   rfm12->out_cur_end -= len;
	   rfm12->out_buf_pos = rfm12->out_buf;
	   
	   rfm12->pkts_sent++;
	   rfm12->bytes_sent += len - RF_EXTRA_LEN;
	   
//	   wake_up_interruptible(&rfm12_wait_write);
   }
   
   err = rfm12_finish_send_recv_common(rfm12);
   
   return err;
}

static void
rfm12_send_spi_completion_handler(void *arg)
{
   unsigned long flags;
   uint16_t status, valid_interrupt = 0, packet_finished = 0;
   struct rfm12_spi_message* spi_msg =
	  (struct rfm12_spi_message*)arg; 
   struct rfm12_data* rfm12 =
	  (struct rfm12_data*)spi_msg->context;

	  printk(KERN_INFO "rfm12_send_spi_completion_handler is called\n");
   spin_lock_irqsave(&rfm12->rfm12_lock, flags);

   rfm12_spi_completion_common(spi_msg);

   status = (spi_msg->spi_rx[0] << 8) | spi_msg->spi_rx[1];
   rfm12->last_status = status;

   valid_interrupt =
   		((status & RF_STATUS_BIT_FFIT_RGIT) || (status & RF_STATUS_BIT_FFOV_RGUR));

   if (valid_interrupt && NULL != rfm12->out_buf) {
/*	   if (RETRY_SEND_ON_RGUR && (status & RF_STATUS_BIT_FFOV_RGUR)) {
		   packet_finished = 1;
		   rfm12->num_send_underruns++;
		   (void)rfm12_finish_sending(rfm12, 0);
		   printk(KERN_INFO "valid_interrupt\n");
	   } else {*/
		   printk(KERN_INFO "No valid_interrupt\n");
//		   if (!RETRY_SEND_ON_RGUR && (status & RF_STATUS_BIT_FFOV_RGUR))
		   	   rfm12->num_send_underruns++;
		   
		   switch(rfm12->state) {
			   case RFM12_STATE_SEND_PRE1:
			   	  rfm12->out_buf_pos = rfm12->out_buf;
			   	  rfm12->out_cur_num_bytes = rfm12->out_buf_pos[1] + RF_EXTRA_LEN;
			   case RFM12_STATE_SEND_PRE2:
			   case RFM12_STATE_SEND_PRE3:
			   case RFM12_STATE_SEND_SYN1:
			   case RFM12_STATE_SEND_SYN2:
			   case RFM12_STATE_SEND_TAIL1:
			   case RFM12_STATE_SEND_TAIL2:
			   	  rfm12->state++;
			   	  break;
			   case RFM12_STATE_SEND:
			      rfm12->out_cur_num_bytes--;
			      
			      if (0 == rfm12->out_cur_num_bytes) {
				      rfm12->state = RFM12_STATE_SEND_TAIL1;
			      }
			      
			      break;
			   case RFM12_STATE_SEND_TAIL3:
			   	  packet_finished = 1;
				  printk(KERN_INFO "packet_finished\n");
			   	  (void)rfm12_finish_sending(rfm12, 1);
			   	  break;
			   default:
				   printk(KERN_INFO "should never happen\n");
			      // should never happen
			      packet_finished = 1;
			      (void)rfm12_finish_sending(rfm12, 0);
			      break;
		   }
		   
		 //  if (!packet_finished)
		 	  // rfm12_update_rxtx_watchdog(rfm12, 0);
  // 	   }
   }

   spin_unlock_irqrestore(&rfm12->rfm12_lock, flags);

 //  if (!valid_interrupt || !packet_finished)
 //  	   platform_irq_handled(rfm12->irq_identifier);
}


static int Initialize(uint8_t nodeid, uint8_t freqBand, uint8_t groupid,
		uint8_t txPower, uint8_t airKbps, struct rfm12_data* rfm12) {


	struct spi_transfer tr1, tr2, tr3, tr4, tr5, tr6, tr7, tr8, tr9,
	tr10, tr11, tr12, tr13;
	
	struct spi_message msg;
	u8 tx_buf[26];
	unsigned long wait_millis = 0;
	int err,por_success = 0;
	
	rfm12->state = RFM12_STATE_CONFIG;
	spi_message_init(&msg);
	
	tr1 = rfm12_make_spi_transfer(0x0000,tx_buf,NULL);  
	tr1.cs_change = 1;
	spi_message_add_tail(&tr1, &msg);
	
	tr2 = rfm12_make_spi_transfer(RF_SLEEP_MODE,tx_buf+2,NULL);  
	tr2.cs_change = 1;
	spi_message_add_tail(&tr2, &msg);
	
	tr3 = rfm12_make_spi_transfer(RF_TXREG_WRITE,tx_buf+4,NULL); 
	tr3.cs_change = 1;
	spi_message_add_tail(&tr3, &msg);	
	
	err = spi_sync(rfm12->spi, &msg);

	printk(KERN_INFO "First config of RFM12 set\n");

		// wait until the RFM12B is out of power-up reset.
	for (wait_millis = 0;
			wait_millis < POWERON_RESET_WAIT_MILLIS;
			wait_millis += POWERON_RESET_WAIT_STEP) {
			msleep(POWERON_RESET_WAIT_STEP);
			printk(KERN_INFO "RFM12 POR waited for =  %d\n",wait_millis);

			if (1 == gpio_get_value(input[0].gpio)) {
				printk(KERN_INFO "POR is set RFM12 is ready\n");
				por_success = 1;
				break;
			}

			spi_message_init(&msg);
			tr1 = rfm12_make_spi_transfer(0x0000, tx_buf+0, NULL);
			spi_message_add_tail(&tr1, &msg);
			(void)spi_sync(rfm12->spi, &msg);
	}
	if (0 == por_success) {
		printk(KERN_ERR ": timed-out while waiting for POR to finish.\n");
//		err = -ENODEV;
//	  	goto pError;
	}
	spi_message_init(&msg);
	tr1 = rfm12_make_spi_transfer(0x80C7 | ((freqBand & 0xff) << 4),tx_buf+0,NULL);
	tr1.cs_change = 1;
	spi_message_add_tail(&tr1, &msg);
	
	tr2 = rfm12_make_spi_transfer(0xA640,tx_buf+2,NULL); 
	tr2.cs_change = 1;
	spi_message_add_tail(&tr2, &msg);

   	tr3 = rfm12_make_spi_transfer(0xC600 + airKbps,tx_buf+4,NULL);  
	tr3.cs_change = 1;
	spi_message_add_tail(&tr3, &msg);
   
	tr4 = rfm12_make_spi_transfer(0x94A2,tx_buf+6,NULL);
	tr4.cs_change = 1;
	spi_message_add_tail(&tr4, &msg);
	
	tr5 = rfm12_make_spi_transfer(0xC2AC,tx_buf+8,NULL);
	tr5.cs_change = 1;
	spi_message_add_tail(&tr5, &msg);
	
	
	if (groupid != 0) {
		tr6 = rfm12_make_spi_transfer(0xCA83,tx_buf+10,NULL);               
		tr6.cs_change = 1;
		spi_message_add_tail(&tr6, &msg);

		tr7 = rfm12_make_spi_transfer(0xCE00 | groupid,tx_buf+12,NULL); 
		tr7.cs_change = 1;
		spi_message_add_tail(&tr7, &msg);
	} else {
		tr6 = rfm12_make_spi_transfer(0xCA8B,tx_buf+10,NULL);               
		tr6.cs_change = 1;
		spi_message_add_tail(&tr6, &msg);

		tr7 = rfm12_make_spi_transfer(0xCE2D,tx_buf+12,NULL);               
		tr7.cs_change = 1;
		spi_message_add_tail(&tr7, &msg);
	}
	
	tr8 = rfm12_make_spi_transfer(0xC483,tx_buf+14,NULL);                   
	tr8.cs_change = 1;
	spi_message_add_tail(&tr8, &msg);
	
	tr9 = rfm12_make_spi_transfer(0x9850,tx_buf+16,NULL); // Here a change
	tr9.cs_change = 1;
	spi_message_add_tail(&tr9, &msg);
	
	tr10 = rfm12_make_spi_transfer(0xCC77, tx_buf+18, NULL);
	tr10.cs_change = 1;
	spi_message_add_tail(&tr10, &msg);
	
	tr11 = rfm12_make_spi_transfer(0xE000, tx_buf+20, NULL);
	tr11.cs_change = 1;
	spi_message_add_tail(&tr11, &msg);

	tr12 = rfm12_make_spi_transfer(0xC800, tx_buf+22, NULL);
	tr12.cs_change = 1;
	spi_message_add_tail(&tr12, &msg);

	tr13 = rfm12_make_spi_transfer(0xC049, tx_buf+24, NULL);
	spi_message_add_tail(&tr13, &msg);

	printk(KERN_INFO "Send second config to RFM12\n");
	if (0 == err) {
		spi_message_init(&msg);

		tr1 = rfm12_make_spi_transfer(0x0000, tx_buf+0, NULL);
		spi_message_add_tail(&tr1, &msg);

		err = spi_sync(rfm12->spi, &msg);
		printk(KERN_INFO "No Error RFM12 started\n");
	}
	
	return err;
}


static int rfm12_open(void){
//	unsigned long flags;
	struct rfm12_data* rfm12;
	int err;
	
	rfm12 = & dev_data; //myadd to set internal dev_data
	
	rfm12->state = RFM12_STATE_CONFIG;
	rfm12->spi = spi_device;
	rfm12->open = 0;
	rfm12->free_spi_msgs = 0;
	spin_lock_init(&rfm12->rfm12_lock);
	
//	spin_lock_irqsave(&rfm12->rfm12_lock, flags);
	printk(KERN_INFO "Spin lock active start config\n");

	Initialize(CLIENT_MBED_NODE, RF12_433MHZ, 212,0,0x08,rfm12);
	
	printk(KERN_INFO "config finished\n");
	if (!rfm12->in_buf) {
		rfm12->in_buf = kmalloc(2*DATA_BUF_SIZE, GFP_KERNEL);
		printk(KERN_INFO "Requested RFM buffer\n");
		if (!rfm12->in_buf) {
			printk(KERN_ALERT "ERROR RFM12 couldn create in_buf\n");
			err = -ENOMEM;
		}
		rfm12->out_buf = rfm12->in_buf + DATA_BUF_SIZE;
		rfm12->out_buf_pos = rfm12->out_buf;
		rfm12->in_buf_pos = rfm12->in_buf;
	}
	
	rfm12->open++;	
	if (1 == rfm12->open) {
		 rfm12->bytes_recvd = 0;
		 rfm12->pkts_recvd = 0;
		 rfm12->bytes_sent = 0;
		 rfm12->pkts_sent = 0;
		 rfm12->num_recv_overflows = 0;
		 rfm12->num_recv_timeouts = 0;
		 rfm12->num_recv_crc16_fail = 0;
		 rfm12->num_send_underruns = 0;
		 rfm12->num_send_timeouts = 0;
		 rfm12->in_cur_num_bytes = 0;
		 rfm12->rxtx_watchdog_running = 0;
		 rfm12->retry_sending_running = 0;
		 rfm12->crc16 = 0;
		 rfm12->last_status = 0;
		 rfm12->should_release = 0;
		 rfm12->in_cur_len_pos = rfm12->in_buf;
		 rfm12->in_cur_end = rfm12->in_buf;
		 rfm12->out_cur_end = rfm12->out_buf;
		printk(KERN_INFO "RFM struct set\n");
	}
	
//	spin_unlock_irqrestore(&rfm12->rfm12_lock, flags);
	printk(KERN_INFO "spinlock release RFM12 ready\n");	

	printk(KERN_INFO "RFM12 Receiving would start\n");
	// err = rfm12_start_receiving(rfm12);

	rfm12->state = RFM12_STATE_IDLE;
	return err;
}

static ssize_t read(struct file *file, char __user *buf, size_t count,
		    loff_t *ppos)
{
	printk(KERN_INFO "Read is called !\n");
	return 0;
}


struct spi_transfer rfm12_control_spi_transfer(struct rfm12_spi_message* msg,
   u8 pos, uint16_t cmd)
{   
   return rfm12_make_spi_transfer(cmd,
			   msg->spi_tx + 2*pos,
			   msg->spi_rx + 2*pos);
}

static struct rfm12_spi_message* rfm12_claim_spi_message(void)
{
   u8 i;
   struct rfm12_spi_message* rv = NULL;

   for (i=0; i < NUM_MAX_CONCURRENT_MSG; i++) {
	  if (0 == (dev_data.free_spi_msgs & (1 << i))) {
		 dev_data.free_spi_msgs |= (1 << i);
		 rv = &dev_data.spi_msgs[i];
		 rv->pos = i;
		 rv->context = &dev_data;
			
		 printk(KERN_INFO "created spi msg free = %d",dev_data.free_spi_msgs);
		 break;
	  }
   }

   return rv;
}

static void
rfm12_unclaim_spi_message(struct rfm12_spi_message* spi_msg)
{
 //  struct rfm12_data* rfm12 =
//	  (struct rfm12_data*)spi_msg->context;

   dev_data.free_spi_msgs &= ~(1 << spi_msg->pos);   
}

static void
rfm12_spi_completion_common(struct rfm12_spi_message* msg)
{
   rfm12_unclaim_spi_message(msg);
}


static void
__rfm12_generic_spi_completion_handler(void *arg)
{
   struct rfm12_spi_message* spi_msg =
	  (struct rfm12_spi_message*)arg;
   struct rfm12_data* rfm12 =
	  (struct rfm12_data*)spi_msg->context;
	  
	printk(KERN_INFO "__rfm12_generic_spi_completion_handler is called\n");

	if (RFM12_STATE_NO_CHANGE != spi_msg->spi_finish_state){
		rfm12->state = spi_msg->spi_finish_state;
		printk(KERN_INFO "RFM12_STATE_NO_CHANGE\n"); 
	}
	rfm12_spi_completion_common(spi_msg);
}
static void
rfm12_generic_spi_completion_handler(void *arg)
{
   unsigned long flags;
   struct rfm12_spi_message* spi_msg =
	  (struct rfm12_spi_message*)arg;
   struct rfm12_data* rfm12 =
	  (struct rfm12_data*)spi_msg->context;

   spin_lock_irqsave(&rfm12->rfm12_lock, flags);

   __rfm12_generic_spi_completion_handler(arg);

   spin_unlock_irqrestore(&rfm12->rfm12_lock, flags);
}



static int
rfm12_send_generic_async_cmd( uint16_t* cmds,int num_cmds, uint16_t delay_usecs, void (*callback)(void*),
								 rfm12_state_t finish_state)
{
   int err, i;
   struct rfm12_spi_message* spi_msg;   

   spi_msg = rfm12_claim_spi_message();

   if (NULL == spi_msg){
	  printk(KERN_ALERT "Error could not get a SPI message\n");	
	  return -EBUSY;
   }
   spi_msg->spi_finish_state = finish_state;
   spi_message_init(&spi_msg->spi_msg);
   spi_msg->spi_msg.complete = 
	  (NULL == callback) ? rfm12_generic_spi_completion_handler : callback;
   spi_msg->spi_msg.context = (void*)spi_msg;
   spi_msg->spi_transfers[0] =
   rfm12_control_spi_transfer(spi_msg, 0, cmds[0]);

   if (num_cmds > 1) {
	  for (i=1; i<num_cmds; i++) {
		  printk(KERN_INFO "Number of commands is greater 1\n");
		  spi_msg->spi_transfers[i-1].cs_change = 1;
		  spi_msg->spi_transfers[i-1].delay_usecs = delay_usecs;
		  spi_message_add_tail(&spi_msg->spi_transfers[i-1], &spi_msg->spi_msg);
	
		  spi_msg->spi_transfers[i] =
			   rfm12_control_spi_transfer(spi_msg, i, cmds[i]);
		   spi_message_add_tail(&spi_msg->spi_transfers[i], &spi_msg->spi_msg);
	  }
   } else{
	  printk(KERN_INFO "only sending one command\n");
	  spi_message_add_tail(&spi_msg->spi_transfers[0], &spi_msg->spi_msg);
   }
   
   printk(KERN_ALERT "Now sending async message\n");
   
   err = spi_async(dev_data.spi, &spi_msg->spi_msg);
   if (err)
	  __rfm12_generic_spi_completion_handler((void*)spi_msg);

   return err;
}

static void
rfm12_trysend_completion_handler(void *arg)
{
   unsigned long flags;
   uint16_t status = 0;
   struct rfm12_spi_message* spi_msg =
	  (struct rfm12_spi_message*)arg; 
   struct rfm12_data* rfm12 =
	  (struct rfm12_data*)spi_msg->context;
	  
	  
	printk(KERN_INFO "Spy trysend complete called\n");

   spin_lock_irqsave(&rfm12->rfm12_lock, flags);

   rfm12->retry_sending_running = 0;

   rfm12_spi_completion_common(spi_msg);

   status = (spi_msg->spi_rx[0] << 8) | spi_msg->spi_rx[1];
   
   
   printk(KERN_INFO "status = %d\n",status);

   rfm12->last_status = status;
   
   if (RFM12_STATE_IDLE == rfm12->state &&
       0 == (status & RF_STATUS_BIT_RSSI)) {
	   uint16_t cmd[4];
	   
	   cmd[0] = RF_IDLE_MODE;
	   cmd[1] = RF_READ_STATUS;
	   cmd[2] = RF_RX_FIFO_READ;
	   cmd[3] = RF_XMITTER_ON;
	   
	   rfm12->state = RFM12_STATE_SEND_PRE1;
	   
//	   rfm12_update_rxtx_watchdog(rfm12, 0);
	   
	   printk(KERN_INFO "sending next send start commands\n");
	   
	   rfm12_send_generic_async_cmd( cmd, 4,
	   	   0, NULL, RFM12_STATE_NO_CHANGE);
   } else {
	   printk(KERN_INFO "status is not the correct one try resend\n");
	   printk(KERN_INFO "current no retry done\n");
	   // try again a bit later...
/*	   init_timer(&rfm12->retry_sending_timer);
	   rfm12->retry_sending_timer.expires = jiffies + TRYSEND_RETRY_JIFFIES;
	   rfm12->retry_sending_timer.data = (unsigned long)rfm12;
	   rfm12->retry_sending_timer.function = rfm12_trysend_retry_timer_expired;
	   add_timer(&rfm12->retry_sending_timer);
	   rfm12->retry_sending_running = 1;
*/ 
	   
	}

   spin_unlock_irqrestore(&rfm12->rfm12_lock, flags);
}

// 0 ... nothing to send, can go to listen state
// 1 ... something needs sending, don't go to listen state
static int rfm12_try_sending(void)
{	
	unsigned long flags;
	int retval = 0;
	
//	spin_lock_irqsave(&dev_data.rfm12_lock, flags);
	
	if (NULL != dev_data.out_buf && dev_data.out_cur_end != dev_data.out_buf) {
		uint16_t cmd = RF_READ_STATUS;
		
		printk(KERN_INFO " sending first async message\n");
		
		(void)rfm12_send_generic_async_cmd( &cmd, 1,
			0, rfm12_trysend_completion_handler, RFM12_STATE_NO_CHANGE);
		
		retval = 1;
	}else{
		printk(KERN_ALERT "ERROR sending not started some buf problem \n");	
	}
	
//	spin_unlock_irqrestore(&dev_data.rfm12_lock, flags);
	
	printk(KERN_INFO " finished start sending  \n");
	
	return retval;
}


static void rfm12_begin_sending_or_receiving(void)
{	
	if (RFM12_STATE_IDLE == dev_data.state) {
		/*if (!rfm12_try_sending(rfm12))
			rfm12_start_receiving(rfm12);
			*/
		printk(KERN_INFO " state is idle sending can start \n");
		rfm12_try_sending();
	}else{
		printk(KERN_INFO " RFM12 has the wrange state \n");
		
	}
}


static ssize_t write(struct file *file, const char __user *buf,
					 size_t count, loff_t *ppos)
{
	unsigned long flags;	
	
	printk(KERN_INFO " Write is called \n");
	
	spin_lock_irqsave(&dev_data.rfm12_lock, flags);
	//add other data for sending crc andso 
	dev_data.out_cur_end = &dev_data.out_buf[2];	
	dev_data.out_buf[0] = 'a';
	dev_data.out_buf[1] = 'a';
	dev_data.out_buf[2] = 'a';
	dev_data.out_buf[3] = 'a';
	
	printk(KERN_INFO " prepare start sending data = %c\n",dev_data.out_buf[0]);
	
	rfm12_begin_sending_or_receiving();
	
	printk(KERN_INFO " start sending finished \n");
	
	spin_unlock_irqrestore(&dev_data.rfm12_lock, flags);
	
	return 0;
}


/* Struct with the File Operations*/
static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = read,
	.write = write,
};

static struct miscdevice eud_dev = {
	.minor          = MISC_DYNAMIC_MINOR,
	.name           = "RFM12_RW",
	.fops           = &fops
};

static int init_Gpio(void){

	int ret = 0;
	int err = 0;
	int i; 
	/*Request LEDs*/
	ret = gpio_request_array(leds, ARRAY_SIZE(leds));
	if(ret){
		printk(KERN_ERR "ERROR Unable to request gpio for LEDs  %d",ret);
		return ret;
	}
	/* Turn Off all LEDs*/
	for(i = 0; i < ARRAY_SIZE(leds); i++){
		gpio_set_value(leds[i].gpio,0);
	}

	/*Request IRQ*/
	ret = gpio_request_array(input, ARRAY_SIZE(input));

	if (ret) {
		printk(KERN_ERR "ERROR  Unable to request GPIOs for INT: %d\n", ret);
		goto fail1;
	}

	ret = gpio_to_irq(input[0].gpio);
	if(ret < 0) {
		printk(KERN_ERR "ERROR Unable to request IRQ: %d\n", ret);
		goto fail2;
	}

	input_irqs[0] = ret;
	printk(KERN_INFO "Successfully requested INPUT1 IRQ # %d\n", input_irqs[0]);

	err = request_irq(input_irqs[0], input_ISR, IRQF_TRIGGER_FALLING | IRQF_DISABLED, "gpiomod#input1", (void*)&dev_data);
	
	if (err) {
		printk(KERN_ERR "ERROR Unable to request IRQ: %d\n", err);
		goto fail2;
	}
	printk(KERN_INFO " Request IRQ finished \n");

	gpio_set_value(leds[0].gpio,1);

	return 0;
fail2:
		gpio_free_array(input, ARRAY_SIZE(input));
fail1:
		gpio_free_array(leds, ARRAY_SIZE(leds));
	return ret;
}



static int init_Spi(void){

	int status = 0;
	char buff[64];

	struct device* pdev;
	
	spi_master = spi_busnum_to_master(SPI_BUS);
	if (!spi_master) {
		printk(KERN_ALERT "ERROR spi_busnum_to_master(%d) returned NULL\n",SPI_BUS);
		return -ENODEV;
	}
	    spi_device = spi_alloc_device(spi_master);
    	if (!spi_device) {
		put_device(&spi_master->dev);
		printk(KERN_ALERT "ERROR spi_alloc_device() failed\n");
		return -ENOMEM;
	}
	spi_device->chip_select = SPI_BUS_CS0;
	if (!spi_device) {
		put_device(&spi_master->dev);
		printk(KERN_ALERT "ERROR spi_alloc_device() failed\n");
		return -ENOMEM;
	}
    	snprintf(buff, sizeof(buff), "%s.%u",
	      dev_name(&spi_device->master->dev),
	      spi_device->chip_select);

     	pdev = bus_find_device_by_name(spi_device->dev.bus, NULL, buff);
	if (pdev) {
		/* We are not going to use this spi_device, so free it */
		printk(KERN_INFO "SPI Device already exists !\n");
		spi_dev_put(spi_device);
		/*
		* There is already a device configured for this bus.cs
		* It is okay if it us, otherwise complain and fail.
		*/
		if (pdev->driver && pdev->driver->name &&
			strcmp("RFM12_spi", pdev->driver->name)) {
			printk(KERN_ALERT
			"Driver [%s] already registered for %s\n",
			pdev->driver->name, buff);
			status = -1;
		}
		printk(KERN_INFO "Returning !\n");
		return status;
	}
	printk(KERN_INFO " SPI INIT finished set device parameter finished \n");
	spi_device->max_speed_hz = SPI_BUS_SPEED;
	spi_device->mode = SPI_MODE_0;
	spi_device->bits_per_word = 8;
	spi_device->irq = -1;
	spi_device->controller_state = NULL;
	spi_device->controller_data = NULL;
    	strlcpy(spi_device->modalias, "rfm12", 5);

	status = spi_add_device(spi_device);
	if (status < 0) {
		spi_dev_put(spi_device);
		printk(KERN_ALERT "ERROR spi_add_device() failed: %d\n",status);
	}

	return status;
	
}
/***********Init and Deinit************/
static int __init RFM_init(void)
{
	int status = 0;

	/* Register GPIO and Interrupt)*/
	status = init_Gpio();
	if (status!= 0){
		return status;
	}
    	printk(KERN_INFO " GPIO INIT finished \n");

        /*Reguster SPI Communication*/
	status = init_Spi();
	if(status != 0){
		printk(KERN_ALERT "ERROR SPI initialization failed: %d\n",status);
		return status;
	}
	printk(KERN_INFO " SPI INIT finished \n");

	status = misc_register(&eud_dev);
	if (status!= 0){
		// free irqs
		free_irq(input_irqs[0], NULL);
		/* unregister */
		gpio_free_array(leds,ARRAY_SIZE(leds));
		gpio_free_array(input, ARRAY_SIZE(input));
		printk(KERN_ALERT "ERROR requesting CHAR device : %d\n",status);
		return status;
	}
	
	printk(KERN_INFO " Char device request finished \n");
	/*Initialize the RFM12*/
	rfm12_open();

	printk(KERN_INFO "RFM12 Module Successfully Initialized !\n");
	return 0;		
}

static void __exit RFM_deinit(void)
{
	int i;
	//free spi
	spi_unregister_device(spi_device);

	for(i = 0; i < ARRAY_SIZE(leds); i++){
		gpio_set_value(leds[i].gpio,0);
	}
	//free file structure

	misc_deregister(&eud_dev);
	// free irqs
	
	free_irq(input_irqs[0], NULL);

	/* unregister */
	gpio_free_array(leds,ARRAY_SIZE(leds));
	gpio_free_array(input, ARRAY_SIZE(input));
	
	
	kfree(dev_data.out_buf);
	kfree(dev_data.in_buf);
	
	printk(KERN_INFO "RFM12 Successfully unloaded!\n");
}

module_init(RFM_init);
module_exit(RFM_deinit);
