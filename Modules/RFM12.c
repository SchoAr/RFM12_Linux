#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h> 
#include <linux/spi/spi.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/jiffies.h>
#include "RFM12_config.h"
#include "RFM12.h"

MODULE_AUTHOR("Schoenlieb");
MODULE_DESCRIPTION("A Char Driver for the RFM12 Radio Transceiver");
MODULE_LICENSE("GPL");
/********************/
int ledstatus = 0;
/********************************Prototyping*******************************/
static void SendStart(uint8_t toNodeID, const void* sendBuf, uint8_t sendLen,
		u8 requestACK, u8 sendACK);
uint16_t writeCmd(uint16_t cmd);
uint16_t xfer(uint16_t cmd);
uint16_t crc16_update(uint16_t crc, uint8_t data);
struct spi_transfer rfm12_make_spi_transfer(uint16_t cmd, u8* tx_buf, u8* rx_buf);

static void RFM12_work_handler(struct work_struct *w);
/************RFM12 Variables *************************************************/
u8 useEncryption = 0;
enum {
    TXCRC1, TXCRC2, TXTAIL, TXDONE, TXIDLE, TXRECV, TXPRE1, TXPRE2, TXPRE3, TXSYN1, TXSYN2,
};
volatile uint8_t nodeID;                    // address of this node
volatile uint8_t networkID;                 // network group

volatile uint8_t* Data;
volatile uint8_t* DataLen;

volatile uint8_t rf12_buf[RF_MAX];   	    // recv/xmit buf, including hdr & crc bytes

volatile uint8_t rxfill;                    // number of data bytes in rf12_buf
volatile int8_t rxstate;                    // current transceiver state
volatile uint16_t rf12_crc;                 // running crc value
uint32_t seqNum;                            // encrypted send sequence number
uint32_t cryptKey[4];                       // encryption key to use
long rf12_seq;                              // seq number of encrypted packet (or -1)

static struct workqueue_struct *wq = 0;


struct spi_device *spi_device;

/*
 * Define for The LEDs
 */
static struct gpio leds[] = {
#ifdef ON_LED_ENABLE
    { ON_LED, GPIOF_OUT_INIT_HIGH, "ON" },
#endif
#ifdef RX_LED_ENABLE
    { RX_LED, GPIOF_OUT_INIT_HIGH, "RX" },
#endif
#ifdef TX_LED_ENABLE
    { TX_LED, GPIOF_OUT_INIT_HIGH, "TX" },
#endif  
};

/*Define for the Inputs*/ 
static struct gpio input[] = {
    { IRQ_PIN, GPIOF_IN, "INPUT" },
};
/* Defines for the Interrupt */
static int input_irqs[] = { -1 };

unsigned long time_interrupt = 0;
unsigned long time_work = 0;

static DECLARE_WORK(RFM12_work,RFM12_work_handler);

static void RFM12_work_handler(struct work_struct *w){
        printk("Work Queue is working\n");

	if (rxstate == TXRECV) {
		uint8_t in = xfer(RF_RX_FIFO_READ);

		if (rxfill == 0 && networkID != 0)
			rf12_buf[rxfill++] = networkID;

		rf12_buf[rxfill++] = in;
		rf12_crc = crc16_update(rf12_crc, in);

		if (rxfill >= rf12_len+ 6 || rxfill >= RF_MAX)
			xfer(RF_IDLE_MODE);
	} else {
		uint8_t out;
		if (rxstate < 0) {
			uint8_t pos = 4 + rf12_len + rxstate++;
			out = rf12_buf[pos];
			rf12_crc = crc16_update(rf12_crc, out);
		} else {
			switch (rxstate++) {
				case TXSYN1:
				out = 0x2D;
				break;
				case TXSYN2:
				out = rf12_grp;
				rxstate = - (3 + rf12_len);
				break;
				case TXCRC1:
				out = rf12_crc;
				break;
				case TXCRC2:
				out = rf12_crc >> 8;
				break;
				case TXDONE:
				xfer2(RF_IDLE_MODE); // fall through
				/**xfer out****/
				out = 0xAA;
				xfer(RF_TXREG_WRITE + out);
				break;
				default:
				out = 0xAA;
			}
		}
		/*
		* Here a other xfer is needed which is sending 0x0000 before
		*/
		xfer2(RF_TXREG_WRITE + out);
	}
        printk("Work Queue ended\n");	  
}
unsigned long flags;
static irqreturn_t input_ISR (int irq, void *data)
{
#ifdef DEBUG
        printk(KERN_INFO"IRQ called.\n");
#endif
        time_interrupt = jiffies;
	queue_work(wq, &RFM12_work);
	return IRQ_HANDLED;
}

static int init_Gpio(void){
    
    int ret = 0;
    int i; 
    /*Request LEDs*/
    ret = gpio_request_array(leds, ARRAY_SIZE(leds));
    if(ret){
      printk(KERN_ERR "Unable to request gpio for LEDs  %d",ret);
      return ret;
    }
    /* Turn Off all LEDs*/
    for(i = 0; i < ARRAY_SIZE(leds); i++){
	gpio_set_value(leds[i].gpio,0);
    }
   
    /*Request IRQ*/
    ret = gpio_request_array(input, ARRAY_SIZE(input));
    
    if (ret) {
      printk(KERN_ERR "Unable to request GPIOs for BUTTONs: %d\n", ret);
      goto fail1;
    }
    
    ret = gpio_to_irq(input[0].gpio);
    if(ret < 0) {
      printk(KERN_ERR "Unable to request IRQ: %d\n", ret);
      goto fail2;
    }
    
    input_irqs[0] = ret;
    printk(KERN_INFO "Successfully requested INPUT1 IRQ # %d\n", input_irqs[0]);
    
    ret = request_irq(input_irqs[0], input_ISR, IRQF_TRIGGER_FALLING | IRQF_DISABLED, "gpiomod#input1", NULL);
    if(ret) {
      printk(KERN_ERR "Unable to request IRQ: %d\n", ret);
      goto fail2;
    }
    /*Create Work Quue*/
    wq = create_singlethread_workqueue("RFM12mod");
    if (!wq){
        printk(KERN_ERR "Unable to request Work Queue \n");
        goto fail2;
    }
/***************DEBUG ledstatus  */
    ledstatus = 1;
/*Only when On LED is avaiable it can be set to indicate the driver is working*/ 
#ifdef ON_LED_ENABLE
//   gpio_set_value(leds[0].gpio,1);
#endif     
    return 0;
    
    fail2:
      gpio_free_array(input, ARRAY_SIZE(input));
    fail1:
      gpio_free_array(leds, ARRAY_SIZE(leds));
    
    return ret;
}
/***************SPI Operations************************/
struct spi_master *spi_master;
struct spi_message msg;
spinlock_t spi_lock;
static int RFM12_probe(struct spi_device *spi_devicef)
{
    spi_device = spi_devicef;
    return 0;
}
 static int RFM12_remove(struct spi_device *spi_devicef)
{
    spi_device = NULL;
    return 0;
}
 
static struct spi_driver RFM12_driver = {
    .driver = {
      .name = "RFM12_spi",
      .owner = THIS_MODULE,
      },
      .probe = RFM12_probe,
      .remove = RFM12_remove,
};

char buff[64];
struct device *pdev;
char tx_buff[3];
char rx_buff[3];
int busy;
int i;

struct spi_transfer transfer = {
        .tx_buf         = tx_buff,
	.rx_buf 	= 0,
        .len            = 4,
    };
    
static int init_Spi(void){
  
    int status; 
    status = spi_register_driver(&RFM12_driver);
    if (status < 0) {
	printk(KERN_ALERT "spi_register_driver() failed %d\n", status);
	return status;
    }
     spi_master = spi_busnum_to_master(SPI_BUS);
    if (!spi_master) {
      printk(KERN_ALERT "spi_busnum_to_master(%d) returned NULL\n",SPI_BUS);
      return -1;
    }
    spi_device = spi_alloc_device(spi_master);
    if (!spi_device) {
      put_device(&spi_master->dev);
      printk(KERN_ALERT "spi_alloc_device() failed\n");
      return -1;
    } 
    spi_device->chip_select = SPI_BUS_CS0; 
    /* Check whether this SPI bus.cs is already claimed */
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
    
    spi_device->max_speed_hz = SPI_BUS_SPEED;
    spi_device->mode = SPI_MODE_0;
    spi_device->bits_per_word = 8;
    spi_device->irq = -1;
    spi_device->controller_state = NULL;
    spi_device->controller_data = NULL;
    
    strlcpy(spi_device->modalias, "RFM12_SPI", SPI_NAME_SIZE);
    
    status = spi_add_device(spi_device);
    if (status < 0) {
	spi_dev_put(spi_device);
	printk(KERN_ALERT "spi_add_device() failed: %d\n",status);
    } 
    return status;
}

/*************Send Receive SPi***********/

uint16_t xfer(uint16_t cmd) {
    int err;
    printk(KERN_INFO "--------------XFER called \n"); 
    spi_message_init(&msg);

    tx_buff[0] = cmd >> 8;
    tx_buff[1] = cmd & 0xFF;
    
    transfer.tx_buf = tx_buff;
    transfer.rx_buf = rx_buff;
    transfer.len = 2;
    
    spi_message_add_tail(&transfer, &msg);
    err = spi_sync(spi_device, &msg);
    
    if (err != 0){
	printk(KERN_INFO "Error in xfer function \n"); 
    }
    return  (rx_buff[0]<<8) | rx_buff[1];
}

struct spi_transfer rfm12_make_spi_transfer(uint16_t cmd, u8* tx_buf, u8* rx_buf)
{
    struct spi_transfer tr = {
      .tx_buf = tx_buf,
      .rx_buf = rx_buf,
      .len    = 2,
    };
    tx_buf[0] = (cmd >> 8) & 0xff;
    tx_buf[1] = cmd & 0xff;
    return tr;
}
    
/***************File Operations************************/

static const char *id = "Hello World";
static ssize_t read(struct file *file, char __user *buf, size_t count,
		    loff_t *ppos)
{
	printk(KERN_INFO "Read is called !\n");
	return simple_read_from_buffer(buf, count, ppos, id, strlen(id));
}

static ssize_t write(struct file *file, const char __user *buf,
					 size_t count, loff_t *ppos)
{
	/*The copieng of the user buffer is done in SendStart*/
	SendStart(SERVER_MBED_NODE, buf,count, 0,0);
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

/***********RFM12 Operations****************************************/

void Initialize(uint8_t nodeid, uint8_t freqBand, uint8_t groupid,
		uint8_t txPower, uint8_t airKbps) {
	
	struct spi_transfer tr1, tr2, tr3, tr4, tr5, tr6, tr7, tr8, tr9,
	tr10, tr11, tr12, tr13, tr14,tr15,tr16;
	struct spi_message msg;
	u8 tx_buf[32];
	int err;
    
	nodeID = nodeid;
	networkID = groupid;
	rf12_grp= groupid;

	spi_message_init(&msg);	
	
	tr1 = rfm12_make_spi_transfer(0x0000,tx_buf,NULL);    // initial SPI transfer added to avoid power-up problem
	tr1.cs_change = 1;
	spi_message_add_tail(&tr1, &msg);
	
	tr2 = rfm12_make_spi_transfer(RF_SLEEP_MODE,tx_buf+2,NULL);            // DC (disable clk pin), enable lbd
	tr2.cs_change = 1;
	spi_message_add_tail(&tr2, &msg);		 
	
	tr3 = rfm12_make_spi_transfer(RF_TXREG_WRITE,tx_buf+4,NULL);           // in case we're still in OOK mode
	tr3.cs_change = 1;
	spi_message_add_tail(&tr3, &msg);		  // wait until RFM12B is out of power-up reset, this takes several *seconds*
	
	tr4 = rfm12_make_spi_transfer(0x80C7 | (freqBand << 4),tx_buf+6,NULL); // EL (ena TX), EF (ena RX FIFO), 12.0pF
	tr4.cs_change = 1;
	spi_message_add_tail(&tr4, &msg);
	
	tr5 = rfm12_make_spi_transfer(0xA640,tx_buf+8,NULL); // Frequency is exactly 434/868/915MHz (whatever freqBand is)
	tr5.cs_change = 1;
	spi_message_add_tail(&tr5, &msg);
	
	tr6 = rfm12_make_spi_transfer(0xC600 + airKbps,tx_buf+10,NULL);   //Air transmission baud rate: 0x08= ~38.31Kbps
	tr6.cs_change = 1;
	spi_message_add_tail(&tr6, &msg);
	
	tr7 = rfm12_make_spi_transfer(0x94A2,tx_buf+12,NULL);                   // VDI,FAST,134kHz,0dBm,-91dBm
	tr7.cs_change = 1;
	spi_message_add_tail(&tr7, &msg);
	
	tr8 = rfm12_make_spi_transfer(0xC2AC,tx_buf+14,NULL);                   // AL,!ml,DIG,DQD4
	tr8.cs_change = 1;
	spi_message_add_tail(&tr8, &msg);
	
	if (networkID != 0) {
		tr9 = rfm12_make_spi_transfer(0xCA83,tx_buf+16,NULL);               // FIFO8,2-SYNC,!ff,DR
		tr9.cs_change = 1;
		spi_message_add_tail(&tr9, &msg);
		
		tr10 = rfm12_make_spi_transfer(0xCE00 | networkID,tx_buf+18,NULL);   // SYNC=2DXX
		tr10.cs_change = 1;
		spi_message_add_tail(&tr10, &msg);
	} else {
		tr9 = rfm12_make_spi_transfer(0xCA8B,tx_buf+16,NULL);               // FIFO8,1-SYNC,!ff,DR
		tr9.cs_change = 1;
		spi_message_add_tail(&tr9, &msg);
		
		tr10 = rfm12_make_spi_transfer(0xCE2D,tx_buf+18,NULL);               // SYNC=2D
		tr10.cs_change = 1;
		spi_message_add_tail(&tr10, &msg);
	}

	tr11 = rfm12_make_spi_transfer(0xC483,tx_buf+20,NULL);                   // @PWR,NO RSTRIC,!st,!fi,OE,EN
	tr11.cs_change = 1;
	spi_message_add_tail(&tr11, &msg);
	
	tr12 = rfm12_make_spi_transfer(0x9850 | (txPower > 7 ? 7 : txPower),tx_buf+22,NULL); // !mp,90kHz,MAX OUT
	tr12.cs_change = 1;
	spi_message_add_tail(&tr12, &msg);

	tr13 = rfm12_make_spi_transfer(0xCC77,tx_buf+24,NULL);                   // OB1, OB0, LPX, ddy, DDIT, BW0
	tr13.cs_change = 1;
	spi_message_add_tail(&tr13, &msg);
	
	tr14 = rfm12_make_spi_transfer(0xE000,tx_buf+26,NULL);                   // NOT USE
	tr14.cs_change = 1;
	spi_message_add_tail(&tr14, &msg);
	
	tr15 = rfm12_make_spi_transfer(0xC800,tx_buf+28,NULL);                   // NOT USE
	tr15.cs_change = 1;
	spi_message_add_tail(&tr15, &msg);
	
	tr16 = rfm12_make_spi_transfer(0xC049,tx_buf+30,NULL);                   // 1.66MHz,3.1V
	/*
	* Here no CS  Change cause it is the last message 
	*/
	spi_message_add_tail(&tr16, &msg);
	
	err = spi_sync(spi_device, &msg);
	if (err){
	      printk(KERN_INFO "Error sending second SPI Message !\n");
	}
	rxstate = TXIDLE;
}
uint16_t crc16_update(uint16_t crc, uint8_t data) {
	int i;

	crc ^= data;
	for (i = 0; i < 8; ++i) {
		if (crc & 1)
			crc = (crc >> 1) ^ 0xA001;
		else
			crc = (crc >> 1);
	}
	return crc;
}

static void SendStart_short(uint8_t toNodeID, u8 requestACK, u8 sendACK) {

	rf12_hdr1= toNodeID | (sendACK ? RF12_HDR_ACKCTLMASK : 0);
	rf12_hdr2= nodeID | (requestACK ? RF12_HDR_ACKCTLMASK : 0);

//	if (useEncryption)
//	Encryption(1);

	rf12_crc = ~0;
	rf12_crc = crc16_update(rf12_crc, rf12_grp);
	rxstate = TXPRE1;

	xfer(RF_XMITTER_ON); // bytes will be fed via interrupts
}

static void SendStart(uint8_t toNodeID, const void* sendBuf, uint8_t sendLen,
		u8 requestACK, u8 sendACK) {
	int i;
	rf12_len= sendLen;
	memcpy((void*) rf12_data, sendBuf, sendLen);

#ifdef DEBUG
	printk(KERN_INFO "\nSending message from [%d]; crc:%x,  len: %d, message: ", nodeID, rf12_crc, rf12_len);
	for (i=0; i<rf12_len; i++) {
		printk(KERN_INFO "%c", rf12_data[i]);
	}
#endif
	SendStart_short(toNodeID, requestACK, sendACK);
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
    /*Reguster SPI Communication*/
   status = init_Spi();
    if(status != 0){
      printk(KERN_ALERT "SPI initialization failed: %d\n",status);
      return status;
    }
    /*Initialize the File Operations*/
    status = misc_register(&eud_dev);
    if (status!= 0){
      // free irqs
      free_irq(input_irqs[0], NULL);
      /* unregister */
      gpio_free_array(leds,ARRAY_SIZE(leds));
      gpio_free_array(input, ARRAY_SIZE(input));
      return status; 
    }
    /*Initialize the RFM12*/
    Initialize(CLIENT_MBED_NODE, RF12_433MHZ, 212,0,0x08);

    printk(KERN_INFO "RFM12 Module Successfully Initialized !\n");
    return 0;
}

static void __exit RFM_exit(void)
{
    int i; 
    //free spi
    spi_unregister_device(spi_device);
    spi_unregister_driver(&RFM12_driver);

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
    /*Free Work Queue*/
    if (wq){
//        cancel_work_sync(RFM12_work);
        destroy_workqueue(wq);
    }
    printk(KERN_INFO "RFM12 Successfully unloaded!\n");
}

module_init(RFM_init);
module_exit(RFM_exit);
