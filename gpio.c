#include <sys/types.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <memory.h>

//run on BB-XM-00 RevC

#define GPIO_BASE 0x48002000
//GPIO_150 register address, the resigter is 32-bit and the lower 16 bit belong to GPIO_150
//GPIO_150 is also LED0_GPIO150
#define GPIO_OFFSET 0x180

#define INT *(volatile unsigned int*)

/*
 * IEN  - Input Enable
 * IDIS - Input Disable
 * PTD  - Pull type Down
 * PTU  - Pull type Up
 * DIS  - Pull type selection is inactive
 * EN   - Pull type selection is active
 * M0   - Mode 0
 */

#define IEN     (1 << 8)

#define IDIS    (0 << 8)
#define PTU     (1 << 4)
#define PTD     (0 << 4)
#define EN      (1 << 3)
#define DIS     (0 << 3)

#define M0      0
#define M1      1
#define M2      2
#define M3      3
#define M4      4
#define M5      5
#define M6      6
#define M7      7 

void *map_base;
int n,fd;
unsigned int padconf;

int main(int argc,char *argv[])
{
    if((fd=open("/dev/mem",O_RDWR | O_SYNC))==-1){
        perror("open error!\n");
        return(-1);
    }

    printf("fd=%d\n",fd);

    map_base = mmap(0,0x200,PROT_READ | PROT_WRITE,MAP_SHARED,fd,GPIO_BASE);
    padconf = INT(map_base+GPIO_OFFSET);
    padconf &= 0xffff0000;
    printf("map_base=%p\n",map_base);
    while(1){
	if(strcmp(argv[1], "high") == 0)
       		{
		padconf |= (EN | PTU | M4);
		INT(map_base+GPIO_OFFSET) = padconf;
		printf("high - The register value is set to: 0x%x\n", padconf);
		}
	else if(strcmp(argv[1], "low") == 0)
		{
		padconf |= (EN | PTD | M4);
		INT(map_base+GPIO_OFFSET) = padconf;
		printf("low - The register value is set to: 0x%x\n", padconf);
		}
        usleep(10);
    }

    close(fd);

    munmap(map_base,0xff);
}

