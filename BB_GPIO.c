#include <sys/types.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <memory.h>

#define bool int
#define false 0
#define true 1

// run on BB-XM-00 RevC
// GPIO_144oe: IR_positioning OE pin
// GPIO_145clk/GPT10_PWMEVT: IR_positioning CLK_1V8 pin
// GPIO_146si/GPT11_PWMEVT: IR_positioning SI_1V8 pin
// GPIO_139sw: IR_positioning Analog Switch IN pin
// McSPI4 applied

// By setting PADCONFS/OE/DATAOUT registers, it is finally done. Great!
// Beagle Board uses a transistor to drive the LED, which is controlled by GPIO; GPIO -> transistor -> LED
// that means GPIO's output really drives the LED

#define GPIO_BASE 			0x48002000
//GPIO_144oe register address, the resigter is 32-bit
#define GPIO_139sw_OFFSET 		0x168   // P2437, high 16 bits
#define GPIO_144oe_OFFSET 		0x174   // P2437, low 16 bits
#define GPIO_145clk_OFFSET 		0x174   // P2437, high 16 bits
#define GPIO_146si_OFFSET 		0x178   // P2437, low 16 bits

//P3461,  General-Purpose Interface Integration Figure, GPIO5: GPIO_[159:128]
#define GPIO139sw 0x00000800   		// Bit 21, GPIO149; Bit 16, GPIO144oe; Bit 10, GPIO138
#define GPIO144oe 0x00010000   		// Bit 21, GPIO149; Bit 16, GPIO144oe; Bit 10, GPIO138
#define GPIO145clk 0x00020000   		// Bit 21, GPIO149; Bit 16, GPIO144oe; Bit 10, GPIO138
#define GPIO146si 0x00040000   		// Bit 21, GPIO149; Bit 16, GPIO144oe; Bit 10, GPIO138

#define GPIO5_BASE 		        0x49056000	//P3478
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

//void below means the pointer points to byte data, if e.g. unsigned int *map_base
//then should be: INT(map_base+GPIO_144oe_OFFSET/4) = padconf;
void *map_base;
int n,fd,k,j;
unsigned int padconf;

int OESetHigh(bool setHigh)
{
    if((fd=open("/dev/mem",O_RDWR | O_SYNC))==-1)
    {
        perror("open error!\n");
        return(-1);
    }

    printf("fd=%d\n",fd);

    //GPIO5: Set the pinmux to select the GPIO signal
    map_base = mmap(0,0x200,PROT_READ | PROT_WRITE,MAP_SHARED,fd,GPIO_BASE);
    printf("GPIO_BASE map_base=%p\n",map_base);
    //GPIO144oe
    padconf = INT(map_base+GPIO_144oe_OFFSET);
    padconf &= 0xFFFF0000; //[15:0]=GPIO_144oe  - Clear register bits [15:0]
    padconf |= 0x00000004; //[15:0]=GPIO_144oe  - Select mux mode 4 for gpio
    INT(map_base+GPIO_144oe_OFFSET) = padconf;
    printf("GPIO_144oe_OFFSET - The register value is set to: 0x%x = 0d%u\n", padconf,padconf);
    munmap(map_base,0x200);

    //GPIO5: Set the OE and DATAOUT registers
    map_base = mmap(0,0x40,PROT_READ | PROT_WRITE,MAP_SHARED,fd,GPIO5_BASE);
    printf("GPIO5_BASE map_base=%p\n",map_base);
    //OE
    padconf = INT(map_base+GPIO5_OE_OFFSET);
    padconf &= ~(GPIO144oe);  // Set GPIO_144oe to output
    INT(map_base+GPIO5_OE_OFFSET) = padconf;
    printf("GPIO5_OE_OFFSET - The register value is set to: 0x%x = 0d%u\n", padconf,padconf);
    //DATAOUT
    padconf = INT(map_base+GPIO5_DATAOUT_OFFSET);
    if(setHigh)
        padconf |=  GPIO144oe;  //Set GPIO_144oe high
    else
        padconf &= ~GPIO144oe;    // set GPIO_144oe low
    INT(map_base+GPIO5_DATAOUT_OFFSET) = padconf;
    printf("GPIO5_DATAOUT_OFFSET - The register value is set to: 0x%x = 0d%u\n", padconf,padconf);

    close(fd);
    munmap(map_base,0x40);
}

int DAQStart(bool started)
{
    int SIcount = 0;

    if((fd=open("/dev/mem",O_RDWR | O_SYNC))==-1)
    {
        perror("open error!\n");
        return(-1);
    }

    printf("fd=%d\n",fd);

    //GPIO5: Set the pinmux to select the GPIO signal
    map_base = mmap(0,0x200,PROT_READ | PROT_WRITE,MAP_SHARED,fd,GPIO_BASE);
    printf("GPIO_BASE map_base=%p\n",map_base);
    //GPIO139sw
    padconf = INT(map_base+GPIO_139sw_OFFSET);
    padconf &= 0x0000FFFF; //[31:16]=GPIO_139sw  - Clear register bits [15:0]
    padconf |= 0x00040000; //[31:16]=GPIO_139sw  - Select mux mode 4 for gpio
    INT(map_base+GPIO_139sw_OFFSET) = padconf;
    printf("GPIO_139sw_OFFSET - The register value is set to: 0x%x = 0d%u\n", padconf,padconf);
    //GPIO145clk
    padconf = INT(map_base+GPIO_145clk_OFFSET);
    padconf &= 0x0000FFFF; //[31:16]=GPIO_145clk  - Clear register bits [15:0]
    padconf |= 0x00040000; //[31:16]=GPIO_145clk  - Select mux mode 4 for gpio
    INT(map_base+GPIO_145clk_OFFSET) = padconf;
    printf("GPIO_145clk_OFFSET - The register value is set to: 0x%x = 0d%u\n", padconf,padconf);
    // GPIO146si
    padconf = INT(map_base+GPIO_146si_OFFSET);
    padconf &= 0xFFFF0000; //[15:0]=GPIO_146si  - Clear register bits [15:0]
    padconf |= 0x00000004; //[15:0]=GPIO_146si  - Select mux mode 4 for gpio
    INT(map_base+GPIO_146si_OFFSET) = padconf;
    printf("GPIO_146si_OFFSET - The register value is set to: 0x%x = 0d%u\n", padconf,padconf);

    munmap(map_base,0x200);

    //GPIO5: Set the OE and DATAOUT registers
    map_base = mmap(0,0x40,PROT_READ | PROT_WRITE,MAP_SHARED,fd,GPIO5_BASE);
    printf("GPIO5_BASE map_base=%p\n",map_base);
    //OE
    padconf = INT(map_base+GPIO5_OE_OFFSET);
    padconf &= ~(GPIO139sw+GPIO145clk+GPIO146si);  // Set GPIO_139sw, GPIO_145clk and GPIO_146si to output
    INT(map_base+GPIO5_OE_OFFSET) = padconf;
    printf("GPIO5_OE_OFFSET - The register value is set to: 0x%x = 0d%u\n", padconf,padconf);
    //DATAOUT
    padconf = INT(map_base+GPIO5_DATAOUT_OFFSET);
    while(1)
    {
        padconf &= ~(GPIO145clk+GPIO146si);    // set GPIO139sw, GPIO_145clk and GPIO_146si low
        INT(map_base+GPIO5_DATAOUT_OFFSET) = padconf;

        SIcount++;

        // analog switch: to switch the output of the 2 light sensors
        if(SIcount > 129)
        {
            padconf |=  GPIO139sw;    // Set GPIO139sw high, S2 on, U2(X) output applied
            INT(map_base+GPIO5_DATAOUT_OFFSET) = padconf;
        }
        else
        {
            padconf &= ~GPIO139sw;    // Set GPIO139sw low, S1 on, U1(Y) output applied
            INT(map_base+GPIO5_DATAOUT_OFFSET) = padconf;
        }

        if(SIcount == 258)  // 258 = 129*2
        {
            padconf |=  GPIO146si;    // Set GPIO_146si high
            INT(map_base+GPIO5_DATAOUT_OFFSET) = padconf;
            SIcount = 0;
        }

        padconf |=  GPIO145clk;    // Set GPIO_145clk high
        INT(map_base+GPIO5_DATAOUT_OFFSET) = padconf;
    }
    printf("GPIO5_DATAOUT_OFFSET - The register value is set to: 0x%x = 0d%u\n", padconf,padconf);

    close(fd);
    munmap(map_base,0x40);
}

int main(int argc,char *argv[])
{
    //OESetHigh(false);
    OESetHigh(true);
    DAQStart(true);
}
