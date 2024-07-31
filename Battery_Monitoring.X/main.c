/**
  Generated main.c file from MPLAB Code Configurator

  @Company
    Microchip Technology Inc.

  @File Name
    main.c

  @Summary
    This is the generated main.c using PIC24 / dsPIC33 / PIC32MM MCUs.

  @Description
    This source file provides main entry point for system initialization and application code development.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.171.4
        Device            :  PIC24FJ512GU410
    The generated drivers are tested against the following:
        Compiler          :  XC16 v2.10
        MPLAB 	          :  MPLAB X v6.05
*/

/*
    (c) 2020 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/
#include "mcc_generated_files/system.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/mccp6_compare.h"
#include "DELAY.h"

#define Block0 		0x20
#define Block1 		0x60
#define SkipNetAdress   0xCC
#define Read            0x69
#define Write           0x6C
#define Copy            0x48
#define Refresh         0x63
#define Start           0xB5
#define Stop            0xBE
#define SenseResistor   0x64  // 64=25mOhm
#define CellCapacity    255

#define RNAOP  0     // Read Net Adress Opcode Set to 33h
#define CTYPE  0     // Charge type is Li-Ion
#define CINI   0x02  // Charge is initiated by Command or Charge Source
#define PMOD   0     // Sleep Mode is Disabled

#define DATADS_WRITE    LATEbits.LATE4      //Ecriture du DS
#define DATADS_READ     PORTEbits.RE4       //Lecture du DS

#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80

uint8_t Presence_DS2784 = 0;
/*
                         Main application
 */

/******************************************************************************/

void Delay(unsigned int msec)
{
   while(msec > 0)
   {
       __delay_ms(1);
      msec--;
    }
}

void Delay_us(unsigned int usec)
{
   while(usec > 0)
   {
       __delay_us(1);
      usec--;
    }
}

uint8_t Reset_1Wire (void)
{
    uint8_t result_reset=1;
    uint8_t new_result_reset=1;
    
    TRISEbits.TRISE4 = 0; 	//On met la pin 3 du port A en etat "sortie"
    DATADS_WRITE = 0;		//On met le port a l'etat bas
    __delay_us(600);	//On attend 600us = TRSTL = 480 a 960us
    
    DATADS_WRITE = 1;	    //On met le port a l'etat haut
        
        // On echantillonne et on retourne le resultat du reset
        TRISEbits.TRISE4 = 1; 	//On met la pin 3 du port A en etat "entree"
    
    result_reset=DATADS_READ&BIT0; //Attention pour lire le port on utilise la commande PORT et non LAT.
    __delay_us(453);//On attend 400us = TPDL = 240us max
    new_result_reset=DATADS_READ&BIT0; //si relie le port
    if (!new_result_reset) result_reset = 1;    //si toujours à 0 après 400us, c'est pas Presence Pulse mais batterie nimh (position basse de la coque qui force 1.2V<VIL du PIC)

    Presence_DS2784 = !result_reset;
    return result_reset;
}



/******************************************************************************/
/* Reset_1Wire                                                                */
/* -------------------------------------------------------------------------- */
/* Reset pour le protocole de communication du 1-Wire                         */
/******************************************************************************/


/******************************************************************************/
/* Write1                                                                     */
/* -------------------------------------------------------------------------- */
/* On ecrit un 1                                                              */
/* tslot (le temps d'ecrire un 0 ou un 1) doit �tre entre 60 et 120us         */
/******************************************************************************/
void Write1 (void) //Ecrire un "1"
{
    TRISEbits.TRISE4 = 0x0; 	//On met la pin 3 du port A en etat "sortie"
    DATADS_WRITE = 0x0;		//On met le port a l'etat bas
    __delay_us(7);			//On patiente 1 us (1.17us observe a l'oscillo avec clock externe de 12MHz)
            DATADS_WRITE = 0x1;		//On met le port a l'etat haut
    __delay_us(58);//On attend 84us = TSLOT = 60 a 120us
        
}

/******************************************************************************/
/* Write0                                                                     */
/* -------------------------------------------------------------------------- */
/* On ecrit un 0                                                              */
/* tslot (le temps d'ecrire un 0 ou un 1) doit �tre entre 60 et 120us         */
/******************************************************************************/
void Write0 (void) //Ecrire un "0"
{
    TRISEbits.TRISE4 = 0x0; 	//On met la pin 3 du port A en etat "sortie"
    DATADS_WRITE = 0x0;		//On met le port a l'etat bas
    __delay_us(58);//On attend 84us = TSLOT = 60 a 120us
                DATADS_WRITE = 0x1;		//On met le port a l'etat haut
    __delay_us(7);			//On attend 5us (8.50 observe a l'oscillo avec clock externe de 12 MHz);
}

/******************************************************************************/
/* WriteByte                                                                  */
/* -------------------------------------------------------------------------- */
/* On ecrit un octet                                                          */
/******************************************************************************/
void WriteByte (uint8_t data)
{
    uint8_t i = 0;
    // On va ecrire un octet bit a bit, soit un 0 soit un 1 suivant la donnee qu'on doit ecrire
    for(i=0; i<8; i++)
    {
        // 0x01,0x02,0x04,0x08,0x10,etc.
        if(data&(0x01<<i)) Write1();
        else Write0();
    }
}

/******************************************************************************/
/* Readx                                                                      */
/* -------------------------------------------------------------------------- */
/* On lit un bit a partir du bus que l'on retourne                            */
/******************************************************************************/
uint8_t Readx (void)
{
    uint8_t result_readx=0;
    TRISEbits.TRISE4 = 0x0; 	//On met la pin 3 du port A en etat "sortie"
    DATADS_WRITE = 0x0;		//On met le port a l'etat bas
    __delay_us(13);//On attend 18us
                
                // On lit le bus :
                TRISEbits.TRISE4 = 0x1; 	//On met la pin 3 du port A en etat "entree"
    result_readx=DATADS_READ&BIT0;
    __delay_us(60);	//On attend 60us
                
                return result_readx;
}

/******************************************************************************/
/* ReadByte                                                                   */
/* -------------------------------------------------------------------------- */
/* On lit un octet a partir du bus que l'on retourne en appelant 8 fois Readx */
/******************************************************************************/
uint8_t ReadByte (void)
{
    uint8_t i = 0;
    uint8_t result_readByte=0;
    for(i=0; i<8; i++)
        result_readByte=result_readByte+(uint8_t)(Readx()<<i);
    
    return result_readByte;
}

void Init_DS2784 (void)
{
    uint8_t pb_init=0;
    if(!Reset_1Wire())              // Ca fait un Reset du DS et verifie sa presence
    {
        WriteByte(SkipNetAdress);   // Adresse SkipNetAdress
        WriteByte(Write);           // code de commande en ecriture
        WriteByte(0x01);            // addresse du Status Register
        WriteByte(0x55);            // Valeur aleatoire
        Presence_DS2784 = 1;
    }
    else
    {
        pb_init = 1;                // =1 si le DS n'est pas present
        Presence_DS2784 = 0;
    }
    
    if (pb_init==0)                 // si le DS est present alors on lis la capacite de la batterie.
    {
    }
    __delay_us(5);
}

uint32_t read_Full (void)
{
    uint8_t Full_MSB = 0;
    uint8_t Full_LSB = 0;
    uint32_t Full = 0;
    
    if(!Reset_1Wire())
    {
        WriteByte(SkipNetAdress); // commande SkipNetAdress
        WriteByte(Read);          // commande de lecture du registre
        WriteByte(0x6A);          // adresse du Full register
        Full_MSB = ReadByte();	  //MSB
        Full_LSB = ReadByte();	  //MSB
        
        Full = Full_LSB;
        Full = Full & 0x00FF;
        Full = (uint32_t) Full_MSB << 8 | Full;
        
    }
    
    return Full;
}

void Send1(){
    DATA_SetHigh();
    __delay_us(4);
    DATA_SetLow();
    __delay_us(1);
}

void Send0(){
    DATA_SetHigh();
    __delay_us(2);
    DATA_SetLow();
    __delay_us(3);
}

void Send_Data(uint32_t data){
    for (uint8_t i = 0; i<24; i++){
        if (data&0x00800000){
            Send1();
        } else {
            Send0();
        }
        data=data<<1;
    }
}

void Change_PWM(uint16_t value){
    
    CCP6RB=value;
}

void Reset(void){ //267us reset
    for (uint8_t i = 0; i<210;i++){
        CCP6RB = 0xA;
    }
}

void Green(void){
    uint32_t color = 0x20;
    Reset();   
    for (uint8_t i = 0; i<6; i++){
        (color&0x20)?(Change_PWM(0x4)):(Change_PWM(0x7));
        color = color<<1;
    }
}

void Red(void){
    uint32_t color = 0x8;
    Reset();
    for (uint8_t i = 0; i<6; i++){
        (color&0x20)?(Change_PWM(0x4)):(Change_PWM(0x7));
        color=color<<1;
    }
}

void Blue(void){
    uint32_t color = 0x3;
    Reset();
    for (uint8_t i = 0; i<6; i++){
        (color&0x20)?(Change_PWM(0x4)):(Change_PWM(0x7));
        color=color<<1;
    }
}

void Purple(void){
    Red();
    Blue();
}

void Yellow(void){
    Red();
    Red();
    Green();
    
}

void Orange(void){
    Red();
    Red();
    Red();
    Red();
    Red();
    Red();
    Green();
}


int main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    MCCP6_COMPARE_Start();

    while (1)
    {
        Test();
        __delay_ms(500);
        
    }

    return 1;
}
/**
 End of File
*/

