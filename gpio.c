#include <sys/types.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <memory.h>

#define GPIO 0x48002000
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
 
map_base = mmap(0,0x600,PROT_READ | PROT_WRITE,MAP_SHARED,fd,GPIO);
printf("map_base=%p\n",map_base);
    while(1){
	INT(map_base+0x110) = 0xffff0000; //set it high
	usleep(10);
	INT(map_base+0x110) = 0; //set it low
        usleep(20);
	INT(map_base+0x110) = 0; //set it low
        usleep(20);
    }

    close(fd);   

    munmap(map_base,0xff);
}
