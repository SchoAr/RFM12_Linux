#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>

MODULE_AUTHOR("Schoenlieb");
MODULE_DESCRIPTION("A Simple Hello World module");
MODULE_LICENSE("GPL");


static struct gpio leds[] = {
    { 15, GPIOF_OUT_INIT_HIGH, "USERLED" },
};

static int __init hello_init(void)
{
    int ret = 0;
    int i; 
    
    ret = gpio_request_array(leds, ARRAY_SIZE(leds));
    
    if(ret){
	printk(KERN_ERR "Unable to request gpio for LEDs  %d",ret);
	return ret;
    }
    
    for(i = 0; i < ARRAY_SIZE(leds); i++){
	gpio_set_value(leds[i].gpio,1);
    }
      
    return ret;
}
static void __exit hello_exit(void)
{
    int i; 
    
    for(i = 0; i < ARRAY_SIZE(leds); i++){
	gpio_set_value(leds[i].gpio,0);
    }
    
    gpio_free_array(leds,ARRAY_SIZE(leds));
}

module_init(hello_init);
module_exit(hello_exit);
