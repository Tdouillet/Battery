#ifndef __DS2784_H__
#define __DS2784_H__

#include <xc.h>
#include "Variables.h"
//#include "ISD.h"

/******************************************************************/
/** VARIABLES EXTERN  *********************************************/
/******************************************************************/
extern char BattLevel;
extern unsigned char BattFlag;
extern unsigned char BaseBatteryLow_Drp;	// Compteur pour eviter la double annonce du msg base battery is low.
/******************************************************************/
/**      VARIABLES         ****************************************/
/******************************************************************/
char Readx (void);
char ReadByte (void);
char Reset_1Wire (void);
char read_status (void);
char read_RARC (void);
unsigned char read_AS (void);
unsigned long Full_AS = 0;
unsigned long BatCapacity = 0;
char readTemp(void);

/******************************************************************/
/** PROTOTYPES FONCTIONS  *****************************************/
/******************************************************************/

void Init_DS2784 (void);

void Write1 (void);
void Write0 (void);
void WriteByte (char data);

void test_active_empty(void);
void test_full_detect(void);
void comp_Full_Capacity(void);


#endif
