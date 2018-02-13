#include <stdio.h>
/* Function Pointer Struct (API) implementation */
struct gpio_api
{
    int (*set)(int *GPIO_NUM);
    int (*clear)(int *GPIO_NUM);
};

extern struct gpio_api gpio_api;

int set_fcn(int *GPIO_NUM);
int clear_fcn(int *GPIO_NUM);