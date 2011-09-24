#include <sys/types.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <memory.h>

//run on N900

#define GPIO_BASE 0x48002000
//GPIO_97 register address, the resigter is 32-bit and the higher 16 bit belong to GPIO_97(cam_pclk)
//GPIO_97 is also N900's CAM_B_EN, which enables camera_B when in high state
#define GPIO_OFFSET 0x110

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
	if(strcmp(argv[1], "low") == 0)
       		{
		INT(map_base+GPIO_OFFSET) = 0x40000; 	//set low. mode 4: GPIO. It is also the N900 default setting.
		printf("high - The register value is set to: 0x%x\n", INT(map_base+GPIO_OFFSET));
		}
	else if(strcmp(argv[1], "high") == 0)
		{
		INT(map_base+GPIO_OFFSET) = 0x180000;	//set high. mode 4: GPIO. PU enabled and selected.
		printf("low - The register value is set to: 0x%x\n", INT(map_base+GPIO_OFFSET));
		}
        usleep(10);
    }

    close(fd);

    munmap(map_base,0xff);
}

