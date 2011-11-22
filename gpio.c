#include <sys/types.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <memory.h>

//run on N900, to drive XTI_RX(UART1_RX, J5602.7, ETK_D10) GPIO_24
#define GPIO_BASE 			0x48002000
//GPIO_24 register address, the resigter is 32-bit
#define GPIO_24_OFFSET_LOWER 		0x5f0

//General-Purpose Interface Integration Figure, GPIO5: GPIO_[159:128]
#define GPIO24  0x01000000			//GPIO24

#define GPIO1_BASE 		0x48310000	//P3606
#define GPIO1_OE_OFFSET 	0x034		//P3606, Output Data Enable Register
#define GPIO1_DATAOUT_OFFSET	0x03C		//P3606, Data Out register

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

//void below means the pointer points to byte data, if e.g. unsigned int *map_base
//then should be: INT(map_base+GPIO_138_OFFSET_LOWER/4) = padconf;
void *map_base;  
int n,fd,k,i,num_duty,j;
unsigned int padconf;


int main(int argc,char *argv[])
{
    if((fd=open("/dev/mem",O_RDWR | O_SYNC))==-1){
        perror("open error!\n");
        return(-1);
    }

    printf("fd=%d\n",fd);

    //GPIO1: Set the pinmux to select the GPIO signal
    map_base = mmap(0,0x200,PROT_READ | PROT_WRITE,MAP_SHARED,fd,GPIO_BASE);
    printf("GPIO_BASE map_base=%p\n",map_base);
    //GPIO24
    padconf = INT(map_base+GPIO_24_OFFSET_LOWER);
    padconf &= 0xFFFF0000; //[31:16]=GPIO_24  - Clear register bits [31:16]
    padconf |= 0x00000004; //[31:16]=GPIO_24  - Select mux mode 4 for gpio
    INT(map_base+GPIO_24_OFFSET_LOWER) = padconf; 
    printf("GPIO_24_OFFSET_LOWER - The register value is set to: 0x%x = 0d%u\n", padconf,padconf);    
    munmap(map_base,0x200);


    //GPIO1: Set the OE and DATAOUT registers
    map_base = mmap(0,0x40,PROT_READ | PROT_WRITE,MAP_SHARED,fd,GPIO1_BASE);
    printf("GPIO5_BASE map_base=%p\n",map_base);    
    //OE
    padconf = INT(map_base+GPIO1_OE_OFFSET);
    padconf &= ~GPIO24;  // Set GPIO_24 to output
    INT(map_base+GPIO1_OE_OFFSET) = padconf; 
    printf("GPIO5_OE_OFFSET - The register value is set to: 0x%x = 0d%u\n", padconf,padconf);
    //DATAOUT
    padconf = INT(map_base+GPIO1_DATAOUT_OFFSET);
    padconf |=  GPIO24;  //Set GPIO_24 high
    //padconf &= ~GPIO24;  //Set GPIO_24 low
    INT(map_base+GPIO1_DATAOUT_OFFSET) = padconf; 
    printf("GPIO1_DATAOUT_OFFSET - The register value is set to: 0x%x = 0d%u\n", padconf,padconf);

    //Hello world!
    while(1)
	{
	padconf ^=  GPIO24;  // Toggle GPIO_24
	INT(map_base+GPIO1_DATAOUT_OFFSET) = padconf;
	padconf ^=  GPIO24;  // Toggle GPIO_24	
	INT(map_base+GPIO1_DATAOUT_OFFSET) = padconf;
	} 	

    close(fd);
    munmap(map_base,0x40);
}
