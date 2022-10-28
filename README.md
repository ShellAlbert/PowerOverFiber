# PowerOverFiber
Video Monitor Low Power Consumption Project   
TinyUARTCamera - STM32L496RGT6 LQFP-64   
SerialCamera - STM32L496ZGT6 LQFP-144    

# October 25, 2022   
InfraredCamera - iRay Infrared Camera LVCMOS 14-bits Gray image output.   
STM32L496ZGT6 LQFP-144 + PSRAM 32Mbits   
For single infrared image, the size is 640*512*16bits=5Mbits.    
STM32 FSMC Memory mapping    
Bank1: 4*64Mbyte    
Since we use CE1 in design, so the address range is 0x6000,0000~0x603F,FFFF   

We have IS66WVE2M16EBLL-70BLI-TR PSRAM on board, it has 21 bits address and 16 bits data bus, so 2^21*16bits=32Mbits.     
