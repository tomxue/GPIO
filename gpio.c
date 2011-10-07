#include <sys/types.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <memory.h>

//run on BB-XM-00 RevC, to drive the LEDs via GPIO 149&150. 
//By setting PADCONFS/OE/DATAOUT registers, it is finally done. Great!
//Beagle Board uses a transistor to drive the LED, which is controlled by GPIO; GPIO -> transistor -> LED
//that means GPIO's output really drives the LED

#define GPIO_BASE 			0x48002000
//GPIO_149 register address, the resigter is 32-bit
//GPIO_149 is also LED1_GPIO149
#define GPIO_149_OFFSET_HIGHER 		0x17C	//LED D7
#define GPIO_150_OFFSET_LOWER 		0x180	//LED D6

//P3461,  General-Purpose Interface Integration Figure, GPIO5: GPIO_[159:128]
#define LED1 0x00200000   		// Bit 21, GPIO149
#define LED0 0x00400000   		// Bit 22, GPIO150

#define GPIO5_BASE 		0x49056000	//P3478
#define GPIO5_OE_OFFSET 		0x034		//P3489, Output Data Enable Register
#define GPIO5_DATAOUT_OFFSET	0x03C		//P3490, Data Out register

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
int n,fd,k,i;
unsigned int padconf;

int main(int argc,char *argv[])
{
    if((fd=open("/dev/mem",O_RDWR | O_SYNC))==-1){
        perror("open error!\n");
        return(-1);
    }

    printf("fd=%d\n",fd);

    //GPIO5: Set the pinmux to select the GPIO signal
    map_base = mmap(0,0x200,PROT_READ | PROT_WRITE,MAP_SHARED,fd,GPIO_BASE);
    printf("GPIO_BASE map_base=%p\n",map_base);
    //GPIO149
    padconf = INT(map_base+GPIO_149_OFFSET_HIGHER);
    padconf &= 0x0000FFFF; //[31:16]=GPIO_149  - Clear register bits [31:16]
    padconf |= 0x00040000; //[31:16]=GPIO_149  - Select mux mode 4 for gpio
    INT(map_base+GPIO_149_OFFSET_HIGHER) = padconf; 
    printf("GPIO_149_OFFSET_HIGHER - The register value is set to: 0x%x = 0d%u\n", padconf,padconf);    
    //GPIO150
    padconf = INT(map_base+GPIO_150_OFFSET_LOWER);
    padconf &= 0xFFFF0000; //[15:0] =GPIO_150  - Clear register bits [15:0]
    padconf |= 0x00000004; //[15:0] =GPIO_150  - Select mux mode 4 for gpio
    INT(map_base+GPIO_150_OFFSET_LOWER) = padconf; 
    printf("GPIO_150_OFFSET_LOWER - The register value is set to: 0x%x = 0d%u\n", padconf,padconf);
    munmap(map_base,0x200);


    //GPIO5: Set the OE and DATAOUT registers
    map_base = mmap(0,0x40,PROT_READ | PROT_WRITE,MAP_SHARED,fd,GPIO5_BASE);
    printf("GPIO5_BASE map_base=%p\n",map_base);    
    //OE
    padconf = INT(map_base+GPIO5_OE_OFFSET);
    padconf &= ~(LED1+LED0);  // Set GPIO_149 & GPIO_150 (GPIO 4 bit 2) to output
    INT(map_base+GPIO5_OE_OFFSET) = padconf; 
    printf("GPIO5_OE_OFFSET - The register value is set to: 0x%x = 0d%u\n", padconf,padconf);
    //DATAOUT
    padconf = INT(map_base+GPIO5_DATAOUT_OFFSET);
    padconf |=  LED0;  //Set GPIO_150 high
    padconf &= ~LED1;  //Set GPIO_149 low
    INT(map_base+GPIO5_DATAOUT_OFFSET) = padconf; 
    printf("GPIO5_DATAOUT_OFFSET - The register value is set to: 0x%x = 0d%u\n", padconf,padconf);

    //Hello world!
    for(k=0;k<=200;k++)
	{
    	usleep(200000);
	padconf ^=  LED0;  // Toggle GPIO_150
	INT(map_base+GPIO5_DATAOUT_OFFSET) = padconf;
	padconf ^=  LED1;  // Toggle GPIO_149    	
	INT(map_base+GPIO5_DATAOUT_OFFSET) = padconf;
	} 	

    close(fd);
    munmap(map_base,0x40);
}




