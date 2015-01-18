 
/*
 * LED.c
 *
 * @since : 18.01.2015
 * @Author: SchoAr
 * 
 * @brief : A Encapsulation fir the used LED 
 */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <fcntl.h>
#include <signal.h>
#include <pthread.h>
#include <linux/input.h>

#define DEBUG


void init_Led(){
  
    int valuefd, exportfd, directionfd;
    
#ifdef DEBUG
    printk(KERN_ALERT "INIT_LED IS CALLED\n");
#endif
    
    exportfd = open("/sys/class/gpio/export", O_WRONLY);
    if (exportfd < 0)
    {
        printk("Cannot open GPIO to export it\n");
        exit(1);
    }
    write(exportfd, "15", 4);
    close(exportfd);
#ifdef DEBUG
    printk(KERN_ALERT "Created GPIO\n");
#endif  
    
    directionfd = open("/sys/class/gpio/gpio15/direction", O_RDWR);
    if (directionfd < 0)
    {
        printk("Cannot open GPIO direction it\n");
        exit(1);
    }
 
    write(directionfd, "out", 4);
    close(directionfd);

#ifdef DEBUG
    printk(KERN_ALERT "DIRECTION IS SET\n");
#endif 
    
    valuefd = open("/sys/class/gpio/gpio15/value", O_RDWR);
    if (valuefd < 0)
    {
        printk("Cannot open GPIO value\n");
        exit(1);
    }
    write(valuefd,"1", 2);
    close(valuefd);
#ifdef DEBUG
    printk(KERN_ALERT "VALUE IS SET\n");
#endif
  
}
void toggle_Led(int pin){
  
}
void on_Led(int pin){
  
}
void off_Led(int pin){
  
}
