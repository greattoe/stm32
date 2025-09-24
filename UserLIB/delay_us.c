#include "delay_us.h"

int delay = 0;
int value = 0;
void delay_us(int us){
value = 3;
delay = us * value;
for(int i=0;i < delay;i++);
}
