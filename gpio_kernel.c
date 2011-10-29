#include <linux/init.h>
#include <linux/module.h>
#include <asm/io.h>

MODULE_LICENSE("Dual BSD/GPL");

//run on N900, to drive the TSC_RST GPIO 104. 
//By setting PADCONFS/OE/DATAOUT registers, it is finally done. Great!
//GPIO_104 register address, the resigter is 32-bit and the lower 16 bit belong to GPIO_104
//GPIO_104 is also TSC_RST on N900, which is touch panel driver's reset, active low

#define GPIO_BASE 			0x48002000
//GPIO_104 register address, the resigter is 32-bit
#define GPIO_104_OFFSET_LOWER 		0x120

//P3589,  General-Purpose Interface Integration Figure, GPIO4: GPIO_[126:96]
#define GPIO4_BASE 		0x49054000	//P3606
#define GPIO4_OE_OFFSET 	0x034		//P3607, Output Data Enable Register
#define GPIO4_DATAOUT_OFFSET	0x03C		//P3607, Data Out register
#define BITVALUE		0x100		//Bit 8, GPIO104
#define LED1			0x00200000   	// Bit 21, GPIO149
#define LED0 			0x00400000   	// Bit 22, GPIO150

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

#define INT *(volatile unsigned int*)

//void below means the pointer points to byte data, if e.g. unsigned int *map_base
//then should be: INT(map_base+GPIO_104_OFFSET_LOWER/4) = padconf;
void *map_base;
int n,fd,k,i,j;
unsigned int padconf;

static int hello_init(void)
{
    printk(KERN_ALERT "Hello, World\n");


    //GPIO4: Set the pinmux to select the GPIO signal
    map_base = ioremap(GPIO_BASE,0x200);
    //GPIO104
    padconf = INT(map_base+GPIO_104_OFFSET_LOWER);
    padconf &= 0x0000FFFF; //[31:16]=GPIO_149  - Clear register bits [31:16]
    padconf |= 0x00040000; //[31:16]=GPIO_149  - Select mux mode 4 for gpio
    INT(map_base+GPIO_104_OFFSET_LOWER) = padconf; 
    iounmap(map_base);


    //GPIO4: Set the OE and DATAOUT registers
    map_base = ioremap(GPIO4_BASE,0x40);
    //OE
    padconf = INT(map_base+GPIO4_OE_OFFSET);
    padconf &= ~(LED1+LED0);  // Set GPIO_149 & GPIO_150 (GPIO 4 bit 2) to output
    INT(map_base+GPIO4_OE_OFFSET) = padconf; 
    //DATAOUT
    padconf = INT(map_base+GPIO4_DATAOUT_OFFSET);
    padconf |=  BITVALUE;  //Set GPIO_104 high
    INT(map_base+GPIO4_DATAOUT_OFFSET) = padconf; 

    //Hello world!
    while(1)	
	{
	//for(j=0;j<=10000;j++){} //TSC_RST active low only for a while, user can not see clearly the effect
	//padconf ^=  BITVALUE;  //Toggle GPIO_104
	padconf &= ~BITVALUE;	 //Sey GPIO_104 low
	INT(map_base+GPIO4_DATAOUT_OFFSET) = padconf;
	} 	

    iounmap(map_base);
    return 0;
}

static int hello_exit(void)
{
     printk(KERN_ALERT "Goodbye, cruel world\n");
}

module_init(hello_init);
module_exit(hello_exit);
