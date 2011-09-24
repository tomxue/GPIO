#include <sys/types.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <memory.h>

//run on N900

#define GPIO_BASE 0x48002000
//GPIO_104 register address, the resigter is 32-bit and the lower 16 bit belong to GPIO_104
//GPIO_104 is also TSC_RST on N900, which is touch panel driver's reset, active low
#define GPIO_OFFSET 0x120

#define INT *(volatile unsigned int*)

void *map_base;
int n,fd;

int main(int argc,char *argv[])
{
    if((fd=open("/dev/mem",O_RDWR | O_SYNC))==-1){
        perror("open error!\n");
        return(-1);
    }

    printf("fd=%d\n",fd);

    map_base = mmap(0,0x200,PROT_READ | PROT_WRITE,MAP_SHARED,fd,GPIO_BASE);
    printf("map_base=%p\n",map_base);
    while(1){
	if(strcmp(argv[1], "high") == 0)
       		{
		INT(map_base+GPIO_OFFSET) = 0x100001c; 	//0x100,0004 or 0x100,001c - set high. mode 4: GPIO. It is also the N900 default setting.
		printf("high - The register value is set to: 0x%x\n", INT(map_base+GPIO_OFFSET));
		}
	else if(strcmp(argv[1], "low") == 0)
		{
		INT(map_base+GPIO_OFFSET) = 0x1000000;	//0x100,0000 - set low
		printf("low - The register value is set to: 0x%x\n", INT(map_base+GPIO_OFFSET));
		}
        usleep(10);
    }

    close(fd);

    munmap(map_base,0xff);
}

