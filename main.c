/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

//#if defined(__XC)
//    #include <xc.h>         /* XC8 General Include File */
//#elif defined(HI_TECH_C)
//    #include <htc.h>        /* HiTech General Include File */
//#endif
#include <xc.h> 
#include <htc.h> 
#include <stdint.h>        /* For uint8_t definition */
#include <stdbool.h>       /* For true/false definition */
#include <pic16lf1459.h>
#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp */
#include "uart.h"

/******************************************************************************/
/* User Global Variable Declaration                                           */
/******************************************************************************/

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (ECH, External Clock, High Power Mode (4-20 MHz): device clock supplied to CLKIN pins)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF        // Internal/External Switchover Mode (Internal/External Switchover Mode is enabled)
#pragma config FCMEN = OFF       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config CPUDIV = NOCLKDIV// CPU System Clock Selection Bit (CPU system clock divided by 6)
#pragma config USBLSCLK = 48MHz // USB Low SPeed Clock Selection bit (System clock expects 48 MHz, FS/LS USB CLKENs divide-by is set to 8.)
#pragma config PLLMULT = 3x     // PLL Multipler Selection Bit (3x Output Frequency Selected)
#pragma config PLLEN = DISABLED  // PLL Enable Bit (3x or 4x PLL Enabled)
#pragma config STVREN = OFF      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)


void main(void)
{
    /* Configure the oscillator for the device */
    ConfigureOscillator(); //konfiguracija oscilatora
    
    
    char nino1, nino2, nino3, nino4, nino5, nino6, nino7, nino8, nino9, nino10, nino11, nino12, nino13, nino14;
    char nino15, nino16, nino17, nino18, nino19, nino20, nino21, nino22,nino23,nino24,nino25,nino26,nino27,nino28;
    char nino29, nino30, nino31, nino32, nino33, nino34, nino35, nino36, nino37, nino38, nino39, nino40, nino41;
    char nino42, nino43, nino44, nino45, nino46, nino47, nino48, nino49, nino50, nino51, nino52, nino53, nino54;
    char nino55, nino56, nino57, nino58, nino59, nino60, nino61, nino62, nino63, nino64, nino65;
    char nino66, nino67, nino68, nino69, nino70, nino71, nino72, nino73, nino74, nino75, nino76;
    char nino77, nino78, nino79, nino80, nino81, nino82, nino83, nino84, nino85, nino86, nino87;
    char nino88, nino89, nino90, nino91, nino92, nino93, nino94, nino95;
    
    
    ANSELA = 0; //turn off analog function for port A
    ANSELC = 0;
    ANSELB = 0;
 
    // enable BLE module
    TRISAbits.TRISA5 = 0;           //IRQ output, pin to drive pMOS
    LATAbits.LATA5 = 0;             // turn on pMOS, BLE module
    
       //RESETB, Reset, active low at least in 5ms, specified for BLE HM-11
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 1;
    __delay_ms(10); 
 
    //interrupt settings
    INTCONbits.GIE  = 1;            // Enable Global interrupt
    INTCONbits.PEIE = 1;            // Enable Peripheral Interrupts
    PIE1bits.RCIE   = 1;            //Enable  rx interrupts
    PIE1bits.TXIE   = 0;            //disable tx interrupts
    
    
    __delay_ms(250);                //BLE start-up time
    set_uart();                     //UART setup
    __delay_ms(200);
  
     
   uart_write_string("AT");         // test command
  __delay_ms(1000);
  //uart_write_string("ATE0");        //stop echo
   
    
  uart_write_string("AT+RENEW");      // restores factory defaults
  __delay_ms(500); 
  uart_write_string("AT+RESET");      //reboot HM-11
   __delay_ms(500);
  

   uart_write_string("AT+MARJ0x1234"); // Set iBeacon Major number to 0x1234 (hexadecimal) 
   __delay_ms(500);
   uart_write_string("AT+MINO0xFA01"); // Set iBeacon Minor number to 0xFA01 (hexadecimal)
   __delay_ms(500);
   
   uart_write_string("AT+ADVI3");      // Set advertising interval to 5 (546.25 milliseconds)
   __delay_ms(500);
   
   uart_write_string("AT+NAMENINO");    // Module name
   __delay_ms(500);
   
   uart_write_string("AT+IBEA1");       // ebable iBeacon mode
   __delay_ms(500);
  
   //Additionally to save power
   
    uart_write_string("AT+ADTY3");      // HM-11 is non connectable  
     __delay_ms(500);
  
    uart_write_string("AT+DELO2");   // broadcast-only mode to save power
     __delay_ms(500);
   
   uart_write_string("AT+PWRM0");     //enable auto sleep
    __delay_ms(500);
   

   uart_write_string("AT+RESET"); //Reboot  HM-11
   __delay_ms(500);
  
    nino1 = buffer[0];
   nino2 = buffer[1];
   nino3 = buffer[2];
   nino4 = buffer[3];
   nino5 = buffer[4];
   nino6 = buffer[5];
   nino7 = buffer[6];
   nino8 = buffer[7];
   nino9 = buffer[8];
   nino10 = buffer[9];
   nino11 = buffer[10];
   nino12 = buffer[11];
   nino13 = buffer[12];
   nino14 = buffer[13];
   nino15 = buffer[14];
   nino16 = buffer[15];
   nino17 = buffer[16];
   nino18 = buffer[17];
   nino19 = buffer[18];
   nino20 = buffer[19];
   nino21 = buffer[20];
   nino22 = buffer[21];
   nino23 = buffer[22];
   nino24 = buffer[23];
   nino25 = buffer[24];
   nino26 = buffer[25];
   
   nino27 = buffer[26];
   nino28 = buffer[27];
   nino29 = buffer[28];
   nino30 = buffer[29];
   nino31 = buffer[30];
   nino32 = buffer[31];
   nino33 = buffer[32];
   nino34 = buffer[33];
   nino35 = buffer[34];
   nino36 = buffer[35];
   nino37 = buffer[36];
   nino38 = buffer[37];
   nino39 = buffer[38];
   nino40 = buffer[39];
   nino41 = buffer[40];
   nino42 = buffer[41];
   nino43 = buffer[42];
   nino44 = buffer[43];
   nino45 = buffer[44];
   nino46 = buffer[45];
   nino47 = buffer[46];
   nino48 = buffer[47];
   nino49 = buffer[48];
   nino50 = buffer[49];
   nino51 = buffer[50];
   nino52 = buffer[51];
  /*
   nino53 = buffer[52];
   nino54 = buffer[53];
   nino55 = buffer[54];
   nino56 = buffer[55];
   nino57 = buffer[56];
   nino57 = buffer[57];
   nino59 = buffer[58];
   nino60 = buffer[59];
   
   nino61 = buffer[60];
   nino62 = buffer[61];
   nino63 = buffer[62];
   nino64 = buffer[63];
   nino65 = buffer[64];
   nino66 = buffer[65];
   nino67 = buffer[66];
   nino68 = buffer[67];
   nino69 = buffer[68];
   nino70 = buffer[69];
   nino71 = buffer[70];
   nino72 = buffer[71];
   nino73 = buffer[72];
   nino74 = buffer[73];
   nino75 = buffer[74];
   nino76 = buffer[75];
   nino77 = buffer[76];
   nino78 = buffer[77];
  */
  
  
 
   
    //uart_write_string("AT+PWRM0");     
   //__delay_ms(20);
   //uart_write_string("AT+RESET");      // Reboot
   //__delay_ms(500);
   while(1)
   {
   
   }
   
   
   
}