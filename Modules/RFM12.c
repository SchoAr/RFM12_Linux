#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h> 
#include <linux/spi/spi.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include "RFM12_config.h"

MODULE_AUTHOR("Schoenlieb");
MODULE_DESCRIPTION("A Char Driver for the RFM12 Radio Transceiver");
MODULE_LICENSE("GPL");
/********************/
int ledstatus = 0;
/********************/
static int queue_spi_write(void);
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
    { INPUTPIN, GPIOF_IN, "INPUT" },
};
/* Defines for the Interrupt */
static int input_irqs[] = { -1 };

static irqreturn_t input_ISR (int irq, void *data)
{
#ifdef DEBUG
  printk(KERN_INFO"Intterupt Occured.\n");
#endif
  
  if(irq == input_irqs[0]) {
      if(ledstatus){
	gpio_set_value(leds[1].gpio,0);
	ledstatus = 0;
      }
      else{
	gpio_set_value(leds[1].gpio,1);
	ledstatus = 1;
      }	 
      	//queue_spi_write();
    }
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
    
    ret = request_irq(input_irqs[0], input_ISR, IRQF_TRIGGER_RISING | IRQF_DISABLED, "gpiomod#input1", NULL);
    if(ret) {
      printk(KERN_ERR "Unable to request IRQ: %d\n", ret);
      goto fail2;
    }
 /***************DEBUG ledstatus  */
    ledstatus = 1;
/*Only when On LED is avaiable it can be set to indicate the driver is working*/ 
#ifdef ON_LED_ENABLE
    gpio_set_value(leds[0].gpio,1);
#endif     
    return 0;
    
    fail2:
      gpio_free_array(input, ARRAY_SIZE(input));
    fail1:
      gpio_free_array(leds, ARRAY_SIZE(leds));
    
    return ret;
}
/***************SPI Operations************************/
#define SPI_BUS 32766
#define SPI_BUS_CS0 0	
#define SPI_BUS_SPEED 200000

struct spi_master *spi_master;
struct spi_device *spi_device;
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
char tx_buff[26];
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
    
    i++;
    tx_buff[0] = i++;
    tx_buff[1] = i++;
    tx_buff[2] = i++;
    tx_buff[3] = i++;

    spi_message_init(&msg);
	
    spi_message_add_tail(&transfer,&msg); 
    spi_sync(spi_device, &msg);
    return status;
}
    
/***************File Operations************************/
static const char *id = "Hello World";

static ssize_t read(struct file *file, char __user *buf, size_t count,
		    loff_t *ppos)
{
	printk(KERN_INFO "Read is called !\n");
	
	queue_spi_write();
	return simple_read_from_buffer(buf, count, ppos, id, strlen(id));
}

static ssize_t write(struct file *file, const char __user *buf,
					 size_t count, loff_t *ppos)
{
	char temp[32] = {};
	
	printk(KERN_INFO "write is called !\n");
	simple_write_to_buffer(temp, sizeof(temp), ppos, buf, count);
	printk(KERN_INFO "write value =  %s \n",temp);
	
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

static void spi_completion_handler(void *arg)
{        
    busy = 0;
    printk(KERN_INFO "spi complete handler called !\n");
} 

static int queue_spi_write(void)
{
    int status;
    unsigned long flags;
    
    spi_message_init(&msg);
    
    /* this gets called when the spi_message completes */
    msg.complete = spi_completion_handler;
    msg.context = NULL;
    
    /* write some toggling bit patterns, doesn't really matter */
     tx_buff[0] = i++;
    tx_buff[1] = i++;
    tx_buff[2] = i++;
    tx_buff[3] = i++;

    
    transfer.tx_buf = tx_buff;
    transfer.rx_buf = NULL;
    transfer.len = 4;
    
    spi_message_add_tail(&transfer, &msg);
    
    printk(KERN_INFO "spi message added tail !\n");
    
    spin_lock_irqsave(&spi_lock, flags);
    printk(KERN_INFO "spi lock is taken !\n");
    
    if (spi_device){
	status = spi_async(spi_device, &msg);
	printk(KERN_INFO "spi async returns =  %d!\n",status);
	
    }else{
	printk(KERN_ALERT "SPI Couldnt do a async send: %d\n",status);
	status = -ENODEV;
    }
    
    spin_unlock_irqrestore(&spi_lock, flags);
    
    printk(KERN_INFO "spi spin lock give !\n");
    
    if (status == 0){
	busy = 1; 
	printk(KERN_INFO "status = 0 busy = 1 !\n");
      
    }
    printk(KERN_INFO "returning spi send !\n");
    return status;
} 

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
    printk(KERN_INFO "RFM12 Successfully unloaded!\n");
}

module_init(RFM_init);
module_exit(RFM_exit);
