#include "gpio_api.h"

//PORT SET Function
int set_fcn(int *GPIO_NUM)
{
	printf("GPIO %i SET \n", GPIO_NUM);
	return 0;
}

//PORT CLEAR Function
int clear_fcn(int *GPIO_NUM)
{   
 	printf("GPIO %i CLEAR \n", GPIO_NUM);
	return 0;
}

struct gpio_api gpio_api =
{
    .set   = set_fcn,
    .clear = clear_fcn
};