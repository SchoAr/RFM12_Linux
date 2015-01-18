/*
 * LED	.h
 *
 * @since : 18.01.2015
 * @Author: SchoAr
 * 
 * @brief : A Encapsulation fir the used LED 
 */
#ifndef USERLED_H_
#define USERLED_H_
 

#define RX_LED 9
#define TX_LED 14
#define ON_LED 15

  
void init_Led();
void toggle_Led(int pin);
void on_Led(int pin);
void off_Led(int pin);

#endif /* USERLED_H_ */