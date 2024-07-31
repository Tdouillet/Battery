/******************************************************************************/
/* 		Configuration du DS2784                                       */
/******************************************************************************/
#include <xc.h>
#include "ds2784.h"
#include "DELAY.h"
#include "Variables.h"
#define DATADS_WRITE    LATEbits.LATE4      //Ecriture du DS
#define DATADS_READ     PORTEbits.RE4       //Lecture du DS

uint8_t Presence_DS2784 = 0;
unsigned char CHGTF = 0;  //flag de detection de la fin de charge
unsigned char flag_Comp_Full_Capacity = 0;      //drapeau comparaison Full et ACR et mise a jour ACR


/******************************************************************************/
/* init_ds2770                                                                */
/* -------------------------------------------------------------------------- */
/* Initialisation du DS2770 (CLEAR CSTAT)                                     */
/******************************************************************************/
void Init_DS2784 (void)
{
    char pb_init=0;
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
    Delay_us(5);
}

/******************************************************************************/
/* Reset_1Wire                                                                */
/* -------------------------------------------------------------------------- */
/* Reset pour le protocole de communication du 1-Wire                         */
/******************************************************************************/
char Reset_1Wire (void)
{
    char result_reset=1;
    char new_result_reset=1;
    unsigned char p = 0;
    
    TRISEbits.TRISE4 = 0x0; 	//On met la pin 3 du port A en etat "sortie"
    DATADS_WRITE = 0x0;		//On met le port a l'etat bas
    Delay_us(600);	//On attend 600us = TRSTL = 480 a 960us
    
    DATADS_WRITE = 0x1;	    //On met le port a l'etat haut
    Delay_us(25);	//On attend 25us = TPDH = 15 a 60us
        
        // On echantillonne et on retourne le resultat du reset
        TRISEbits.TRISE4 = 0x1; 	//On met la pin 3 du port A en etat "entree"
    
    result_reset=DATADS_READ&BIT0; //Attention pour lire le port on utilise la commande PORT et non LAT.
    Delay_us(400);//On attend 400us = TPDL = 240us max
    new_result_reset=DATADS_READ&BIT0; //si relie le port
    if (!new_result_reset) result_reset = 1;    //si toujours à 0 après 400us, c'est pas Presence Pulse mais batterie nimh (position basse de la coque qui force 1.2V<VIL du PIC)

    Presence_DS2784 = !result_reset;
    return result_reset;
}

/******************************************************************************/
/* Write1                                                                     */
/* -------------------------------------------------------------------------- */
/* On ecrit un 1                                                              */
/* tslot (le temps d'ecrire un 0 ou un 1) doit �tre entre 60 et 120us         */
/******************************************************************************/
void Write1 (void) //Ecrire un "1"
{
    unsigned char p = 0;
    TRISEbits.TRISE4 = 0x0; 	//On met la pin 3 du port A en etat "sortie"
    DATADS_WRITE = 0x0;		//On met le port a l'etat bas
    Delay_us(1);			//On patiente 1 us (1.17us observe a l'oscillo avec clock externe de 12MHz)
            DATADS_WRITE = 0x1;		//On met le port a l'etat haut
    Delay_us(84);//On attend 84us = TSLOT = 60 a 120us
        
}

/******************************************************************************/
/* Write0                                                                     */
/* -------------------------------------------------------------------------- */
/* On ecrit un 0                                                              */
/* tslot (le temps d'ecrire un 0 ou un 1) doit �tre entre 60 et 120us         */
/******************************************************************************/
void Write0 (void) //Ecrire un "0"
{
    unsigned char p = 0;
    TRISEbits.TRISE4 = 0x0; 	//On met la pin 3 du port A en etat "sortie"
    DATADS_WRITE = 0x0;		//On met le port a l'etat bas
    Delay_us(84);//On attend 84us = TSLOT = 60 a 120us
                DATADS_WRITE = 0x1;		//On met le port a l'etat haut
    Delay_us(5);			//On attend 5us (8.50 observe a l'oscillo avec clock externe de 12 MHz);
}

/******************************************************************************/
/* WriteByte                                                                  */
/* -------------------------------------------------------------------------- */
/* On ecrit un octet                                                          */
/******************************************************************************/
void WriteByte (char data)
{
    char i = 0;
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
char Readx (void)
{
    unsigned char p = 0;
    char result_readx=0;
    TRISEbits.TRISE4 = 0x0; 	//On met la pin 3 du port A en etat "sortie"
    DATADS_WRITE = 0x0;		//On met le port a l'etat bas
    Nop();			//On patiente 1 us (1.4us observe a l'oscillo avec clock externe)
            DATADS_WRITE = 0x1;		//On met le port a l'etat haut
    Delay_us(18);//On attend 18us
                
                // On lit le bus :
                TRISEbits.TRISE4 = 0x1; 	//On met la pin 3 du port A en etat "entree"
    result_readx=DATADS_READ&BIT0;
    Delay_us(60);	//On attend 60us
                
                return result_readx;
}

/******************************************************************************/
/* ReadByte                                                                   */
/* -------------------------------------------------------------------------- */
/* On lit un octet a partir du bus que l'on retourne en appelant 8 fois Readx */
/******************************************************************************/
char ReadByte (void)
{
    char i = 0;
    char result_readByte=0;
    for(i=0; i<8; i++)
        result_readByte=result_readByte+(char)(Readx()<<i);
    
    return result_readByte;
}


/******************************************************************************/
/* DS2784 functions                                                           */
/******************************************************************************/

/******************************************************************************/
/* test active empty                                                          */
/* -------------------------------------------------------------------------- */
/* si la batterie atteind sa valeur active empty, on eteind l'equipement      */
/******************************************************************************/
void test_active_empty(void)
{
    char status_ds=0;
    char AEF=0;
    
    status_ds = read_status();	// lecture du status register
    AEF =(status_ds&0x40)>>6; 	// recuperation du bit AEF
    if(AEF==1)			    // test si AEF=1, c'est que l'on viens d'atteindre la tension minimum de fonctionnement, on eteind alors l'equipement.
    {
        REG_CTRL=0;
    }
}

/******************************************************************************/
/* Drapeau de fin de charge                                                   */
/******************************************************************************/
void test_full_detect(void)
{
    char status_ds=0;
    
    status_ds = read_status();	// lecture du status register
    CHGTF =(status_ds&0x80)>>7; 	// recuperation du bit CHGTF
}

///******************************************************************************/
///* read_status register                                                       */
///* -------------------------------------------------------------------------- */
///* On lit le registre Statut pour avoir des informations sur l'etat de la charge
///* pour avoir des details sur ce registre se reporter a la p.25 de la datasheet du DS2784 rev3/12
///*****************************************************************************/
char read_status (void)
{
    char status_ds = 0;
    if(!Reset_1Wire())
    {
        WriteByte(SkipNetAdress); // commande SkipNetAdress
        WriteByte(Read);          // commande de lecture du registre
        WriteByte(0x01);          // adresse du status register
        status_ds = ReadByte();	  //MSB
    }
    return status_ds;
}

///******************************************************************************/
///* read_RARC lis la capacite restante dans la batterie                        */
///* -------------------------------------------------------------------------- */
///* On lit le registre Statut pour avoir des informations sur l'etat de la charge
///* pour avoir des details sur ce registre se reporter a la p.25 de la datasheet du DS2784 rev3/12
///*****************************************************************************/
char read_RARC (void)
{
    char RARC = 0;
    if(!Reset_1Wire())
    {
        WriteByte(SkipNetAdress); // commande SkipNetAdress
        WriteByte(Read);          // commande de lecture du registre
        WriteByte(0x06);          // adresse du RARC register
        RARC = ReadByte();	  //MSB
    }
    return RARC;
}

/******************************************************************************/
/* read_AS lis la donn?e Age Scalar                                           */
/* -------------------------------------------------------------------------- */
unsigned char read_AS (void)
{
    unsigned char AS = 0;
    if(!Reset_1Wire())
    {
        WriteByte(SkipNetAdress); // commande SkipNetAdress
        WriteByte(Read);          // commande de lecture du registre
        WriteByte(0x14);          // adresse du AS register
        AS = ReadByte();	  //MSB
    }
    return AS;
}

/******************************************************************************/
/* read_Current lis le courant                                           */
/* -------------------------------------------------------------------------- */
short read_Current (void)
{
    char Current_MSB = 0;
    char Current_LSB = 0;
    int Current = 0;
    long Current_Long = 0;
    int Current_Unit = 0;
    //int Current_Dec = 0;
    
    if(!Reset_1Wire())
    {
        WriteByte(SkipNetAdress); // commande SkipNetAdress
        WriteByte(Read);          // commande de lecture du registre
        WriteByte(0x0E);          // adresse du Current register
        Current_MSB = ReadByte();	  //MSB
        Current_LSB = ReadByte();	  //LSB
        
        Current = Current_LSB;
        Current = Current & 0x00FF;
        Current = (int) Current_MSB << 8 | Current;
        
        /* 15625 = 15,625uV
         * 20 = 20mOhms RSNS */
        
        Current_Long = (((long) Current * 15625)/20)/100;
        Current_Unit = (int)(Current_Long/100);
        //Current_Dec = ((int)Current_Long - (Current_Unit * 100));
    }
    
    return Current_Unit; //on ne recupere que les unites, pas besoin d'afficher consommation precise
}

/******************************************************************************/
/* read_Capacity r?cup?re la capacit? restante de la batterie                 */
/* -------------------------------------------------------------------------- */
unsigned long read_Capacity (void)
{
    unsigned char Capacity_MSB = 0;
    unsigned char Capacity_LSB = 0;
    unsigned long Capacity = 0;
    
    if(!Reset_1Wire())
    {
        WriteByte(SkipNetAdress); // commande SkipNetAdress
        WriteByte(Read);          // commande de lecture du registre
        WriteByte(0x10);          // adresse du ACR register
        Capacity_MSB = ReadByte();	  //MSB
        Capacity_LSB = ReadByte();	  //MSB
        
        Capacity = Capacity_LSB;
        Capacity = Capacity & 0x00FF;
        Capacity = (unsigned int) Capacity_MSB << 8 | Capacity;
        
        /* 625 = 6.25uV
         * 20 = 20mOhms RSNS */
        
        //Capacity_Long = (((long) Capacity * 625)/20)/100;
    }
    
    return Capacity;
}

/******************************************************************************/
/* read_Full recupere la capacitite max de la batterie                        */
/* -------------------------------------------------------------------------- */
unsigned long read_Full (void)
{
    unsigned char Full_MSB = 0;
    unsigned char Full_LSB = 0;
    unsigned long Full = 0;
    
    if(!Reset_1Wire())
    {
        WriteByte(SkipNetAdress); // commande SkipNetAdress
        WriteByte(Read);          // commande de lecture du registre
        WriteByte(0x6A);          // adresse du Full register
        Full_MSB = ReadByte();	  //MSB
        Full_LSB = ReadByte();	  //MSB
        
        Full = Full_LSB;
        Full = Full & 0x00FF;
        Full = (unsigned int) Full_MSB << 8 | Full;
        
    }
    
    return Full;
}


/******************************************************************************/
/* comp_Full_Capacity compare capacite max et ACR et corrige ACR si anormal   */
/* -------------------------------------------------------------------------- */
void comp_Full_Capacity (void)
{
    unsigned char Full_MSB = 0;
    unsigned char Full_LSB = 0;
    
    //Full_AS = Capacite max de la batterie compense avec l'Age Scalar : FULL(40) x AS
    //625 = 6.25uV - 20 = 20mOhms RSNS
    //0.78 = voir page 23 datasheet
    Full_AS = (((read_Full() * 625)/20) * read_AS()*78/100)/10000;
    //Capacity = ACR
    BatCapacity = (( read_Capacity() * 625)/20)/100;
    
    //Si ACR est superieur ? la capacite reelle (corrige par Age scalar)
    //alors on corrige l'ACR
    if (BatCapacity > Full_AS){
        
        Full_AS = (Full_AS*20*100)/625;     //Full corrige Age Scalar avec la bonne resolution
        
        Full_LSB = (unsigned char) Full_AS; //on prends que les bits LSB
        Full_AS = Full_AS >> 8;             //on decale les bits MSB vers LSB
        Full_MSB = (unsigned char) Full_AS; //on recupere les bits MSB
        
        //on ecrit dans le registre ACR avec valeur Full corrige par Age Scalar
        if(!Reset_1Wire()){
            WriteByte(SkipNetAdress); // commande SkipNetAdress
            WriteByte(Write);          // commande d'ecriture du registre
            WriteByte(0x10);          // adresse du ACR MSB (0x11 = LSB) register
            WriteByte(Full_MSB);          // 
            WriteByte(Full_LSB);          //
        }
        
        //on copie dans l'EEPROM
        if(!Reset_1Wire()){
            WriteByte(SkipNetAdress); // commande SkipNetAdress
            WriteByte(Copy);          // commande de copy du registre
            WriteByte(0x10);          // adresse du ACR MSB register
        }
        if(!Reset_1Wire()){
            WriteByte(SkipNetAdress); // commande SkipNetAdress
            WriteByte(Copy);          // commande de copy du registre
            WriteByte(0x11);          // adresse du ACR LSB register
        } 
    }
    
    flag_Comp_Full_Capacity = 1;
}

char readTemp(void){
    char Temp_MSB = 0;
    unsigned char Temp_LSB = 0;
    
    if(!Reset_1Wire())
    {
        WriteByte(SkipNetAdress); // commande SkipNetAdress
        WriteByte(Read);          // commande de lecture du registre
        WriteByte(0x0A);          // adresse du Temp register
        Temp_MSB = ReadByte();	  //MSB
        Temp_LSB = ReadByte();	  //MSB
        
    }
    
    return Temp_MSB;
}
/* EOF ************************************************************/
/******************************************************************/
/******************************************************************/
