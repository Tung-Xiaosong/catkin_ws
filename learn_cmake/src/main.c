#include "hello.h"
#include "calc.h"

int main()
{
	hello();
	
	int ret = calc(3, 5);
	printf("ret = %d\n", ret);
	return 0;
}
