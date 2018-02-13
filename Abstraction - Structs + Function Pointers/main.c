#include <stdio.h>
#include <stdlib.h>
#include "gpio_api.h"

int main(int argc, char *argv[])
{	
	int x = atoi(argv[1]);

	gpio_api.set(x);
	gpio_api.clear(x);

	printf("%p \n", gpio_api.set);

}