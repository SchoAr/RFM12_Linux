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
/*
 **************DEBUG ledstatus 
 */
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
//	int ret;
	printk(KERN_INFO "write is called !\n");
//	char temp[32] = {};

	printk(KERN_INFO "Message = %s",buf);
	
/*	simple_write_to_buffer(temp, sizeof(temp), ppos, buf, count);
	ret =  memcmp(temp, id, strlen(id)) ? -EINVAL : count;
	if(ret){
	    return ret; 
	}
	printk(KERN_INFO "write value =  %s !\n",temp);
*/
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

static int __init RFM_init(void)
{
    int ret; 
    /* Register GPIO and Interrupt)*/
    ret = init_Gpio();
    if (ret!= 0){
      return ret; 
    }
    /*Initialize the File Operations*/
    ret = misc_register(&eud_dev);
    if (ret!= 0){
      // free irqs
      free_irq(input_irqs[0], NULL);
      /* unregister */
      gpio_free_array(leds,ARRAY_SIZE(leds));
      gpio_free_array(input, ARRAY_SIZE(input));
      return ret; 
    }
    
    return 0;
}


static void __exit RFM_exit(void)
{
    int i; 
    
    for(i = 0; i < ARRAY_SIZE(leds); i++){
	gpio_set_value(leds[i].gpio,0);
    } 
    // free irqs
    free_irq(input_irqs[0], NULL);
    /* unregister */
    gpio_free_array(leds,ARRAY_SIZE(leds));
    gpio_free_array(input, ARRAY_SIZE(input));
}

module_init(RFM_init);
module_exit(RFM_exit);
