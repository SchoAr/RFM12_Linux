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

static irqreturn_t input_ISR (int irq, void *data);

typedef enum _rfm12_state_t {
   RFM12_STATE_NO_CHANGE,
   RFM12_STATE_CONFIG,
   RFM12_STATE_SLEEP,
   RFM12_STATE_RECV,
   RFM12_STATE_RECV_FINISH,
   RFM12_STATE_SEND
} rfm12_state_t;

struct rfm12_spi_message {
   struct spi_message   spi_msg;
   struct spi_transfer  spi_transfers[2];
   rfm12_state_t        spi_finish_state;
   u8                   spi_tx[4], spi_rx[4];
   void*                context;
   u8                   pos;
};

struct rfm12_data {
	u16			irq;
	
	dev_t			devt;
	spinlock_t		rfm12_lock;
	struct spi_device*	spi;
	struct list_head	device_entry;
	
	u8                   	open;
	rfm12_state_t       	state;
	u8                   	irq_pin_val;
	unsigned long        	bytes_recvd;
	unsigned long        	bytes_sent;
	unsigned long        	num_overflows;
	u8*                  	in_buf, *in_buf_pos;
	u8*                  	out_buf, *out_buf_pos;
	u8*                  	in_cur_len_pos;
	u8*                  	in_cur_end;
	int                  	in_cur_num_bytes;
	struct rfm12_spi_message spi_msgs[1];
	u8                   	free_spi_msgs;
	struct timer_list    	rxtx_watchdog;
	u8                   	rxtx_watchdog_running;
};

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

static irqreturn_t input_ISR (int irq, void *data)
{
	printk(KERN_INFO "IRQ called \n");
	return IRQ_HANDLED;
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
	unsigned long flags;
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
		rfm12->bytes_sent = 0;
		rfm12->num_overflows = 0;
		rfm12->in_cur_num_bytes = 0;
		rfm12->rxtx_watchdog_running = 0;
		rfm12->in_cur_len_pos = rfm12->in_buf;
		rfm12->in_cur_end = rfm12->in_buf;
		printk(KERN_INFO "RFM struct set\n");
	}
	
//	spin_unlock_irqrestore(&rfm12->rfm12_lock, flags);
	printk(KERN_INFO "spinlock release RFM12 ready\n");	

	printk(KERN_INFO "RFM12 Receiving would start\n");
	// err = rfm12_start_receiving(rfm12);

	return err;
}

static ssize_t read(struct file *file, char __user *buf, size_t count,
		    loff_t *ppos)
{
	printk(KERN_INFO "Read is called !\n");
	return 0;
}

static ssize_t write(struct file *file, const char __user *buf,
					 size_t count, loff_t *ppos)
{
	/*The copieng of the user buffer is done in SendStart*/
//	SendStart(SERVER_MBED_NODE, "a",1, 0,0);
	printk(KERN_INFO " Write is called \n");
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
	
	printk(KERN_INFO "RFM12 Successfully unloaded!\n");
}

module_init(RFM_init);
module_exit(RFM_deinit);
