#include <sys/types.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <memory.h>

#define GPIO 0x48002110
#define INT *(volatile unsigned int*)

void *map_base;
int n,fd;

int main(int argc,char *argv[])
{
    if((fd=open("/dev/mem",O_RDWR | O_SYNC))==-1){
    perror("open error!\n");
    return(-1);
    }

    map_base = mmap(0,0xff,PROT_READ | PROT_WRITE,MAP_SHARED,fd,GPIO);
    while(1){
	INT(map_base) = 1; //set it high
	sleep(1000);
	INT(map_base) = 0; //set it low
        sleep(1000);
    }

    close(fd);   

    munmap(map_base,0xff);
}
