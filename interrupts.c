/******************************************************************************/
/*Files to Include                                                            */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#endif

#include <stdint.h>         /* For uint8_t definition */
#include <stdbool.h>        /* For true/false definition */

#include "uart.h"
#include "system.h"


/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/

/* Baseline devices don't have interrupts. Note that some PIC16's 
 * are baseline devices.  Unfortunately the baseline detection macro is 
 * _PIC12 */

//void interrupt ISR(void)
//{
    /* This code stub shows general interrupt handling.  Note that these
    conditional statements are not handled within 3 seperate if blocks.
    Do not use a seperate if block for each interrupt flag to avoid run
    time errors. */

    
     

    // Determine which flag generated the interrupt
   /*if(PIR1bits.RCIF)  // If UART Rx Interrupt
	{
       while(rxdata[i]!='K') {
		if(OERR) // If over run error, then reset the receiver
		{
			CREN = 0;
			CREN = 1;
		}
      
        rxdata[i] = RCREG; // read received
		i++;
       }
   }*/
    
    
    
//}

  
void interrupt ISR(void)
{
   
    // Determine which flag generated the interrupt
   if(PIR1bits.RCIF)        // If UART Rx Interrupt occured
	{
       
       
     
		if(RCSTAbits.OERR) // If over run error, then reset the receiver
		{
			RCSTAbits.CREN = 0;
            __delay_us(10);
			RCSTAbits.CREN = 1;
		}
       
        buffer[cc] = RCREG; // read received charachter
        cc++;
       
   }
  PIR1bits.RCIF = 0;   //reset intnerrupt flag
  
 }

