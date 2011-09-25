#include <sys/types.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <memory.h>

#define GPIO_BASE 0x48002000
//GPIO_150 register address, the resigter is 32-bit and the lower 16 bit belong to GPIO_150
#define GPIO_OFFSET 0x180	

#define INT *(volatile unsigned int*)

void *map_base;
int n,fd,ret;

int main(int argc,char *argv[])
{
    if((fd=open("/dev/mem",O_RDWR | O_SYNC))==-1){
    perror("open error!\n");
    return(-1);
    }

printf("fd=%d\n",fd);   
 
map_base = mmap(0,0x200,PROT_READ | PROT_WRITE,MAP_SHARED,fd,GPIO_BASE);
printf("map_base=%p\n",map_base);
ret=INT(map_base+GPIO_OFFSET);
printf("GPIO register value: %x\n",ret);

    close(fd);   

    munmap(map_base,0xff);
}
