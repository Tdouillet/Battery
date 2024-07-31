/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 !                COPYRIGHTS(c) 2017 BY GLOBALSYS                             !
 !                -------------------------------                             !
 !                                                                            !
 ! This software is copyrighted by and is the sole property of Globalsys.     !
 ! All rights, title, ownership, or other interests in the software remains   !
 ! the property of GLOBALSYS.                                                 !
 !                                                                            !
 ! This software may only be used in accordance with the corresponding        !
 ! license agreement. Any unauthorized use, duplication, transmission,        !
 ! distribution, or disclosure of this software is expressly forbidden.       !
 !                                                                            !
 ! This Copyright notice may not be removed or modified without prior         !
 ! written consent of Globalsys.                                              !
 !                                                                            !
 ! GLOBALSYS reserves the right to modify this software without notice.       !
 !                                                                            !
 !          GLOBALSYS                                                         !
 !          12, Avenue des coquelicots                                        !
 !          94380 BONNEUIL             http://www.globalsys.fr                !
 !          FRANCE                     technical@globalsys.fr                 !
 !                                                                            !
  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

#ifndef VARIABLES_H
#define	VARIABLES_H


//#define BRUNO_A_INVERSE_LES_BOUTONS

////////////////////////////////////////////////////////////////////////////////
// PORT MAP ////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/** SWITCH *********************************************************/
#define ON_OFF_SWITCH                 PORTBbits.RB1 // ON/OFF
#ifdef BRUNO_A_INVERSE_LES_BOUTONS
#define PLUS_SWITCH                   PORTBbits.RB4 // UP
#define MINUS_SWITCH                  PORTBbits.RB2 // DOWN
#else
#define PLUS_SWITCH                   PORTBbits.RB2 // UP
#define MINUS_SWITCH                  PORTBbits.RB4 // DOWN
#endif

//ALIMENTATIONS
#define REG_CTRL        LATBbits.LATB0      //Alimentation 

//BOUTONS
#define PTT             PORTAbits.RA3       //Bouton PTT


//CVM
#define UTX_CVM         LATCbits.LATC6      //Transmission CVM 
#define URX_CVM         PORTCbits.RC7       //Reception CVM
#define RSTN_CVM        LATJbits.LATJ4      //Reset CVM 

//LED RGB
#define LED_BLEU        LATHbits.LATH4      //LED Bleu 
#define LED_ROUGE       LATHbits.LATH6      //LED Rouge     
#define LED_VERT        LATHbits.LATH5      //LED Verte

#define CHARGE_EN       LATJbits.LATJ5      //Enable module de charge     
#define CHARGE_STATUS   PORTJbits.RJ6       //Etat charge (= 1 pas de charge; = 0 charge en cours BQ24040)

#define SQUELCH_EN      LATHbits.LATH7      //Squelch enable
//ISD 
#define SSB_NUV         LATCbits.LATC0      //
#define SCLK_NUV        LATCbits.LATC3      //Clock ISD
#define MISO_NUV        PORTCbits.RC4       //
#define MOSI_NUV        LATCbits.LATC5      //

//DS



////////////////////////////////////////////////////////////////////////////////
// DEFINITION DES VARIABLES INTERNES ET EXTERNES ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//Variables ISOLATION COCKPIT
#define ISOCOCKPIT_ON  1
#define ISOCOCKPIT_OFF 0
#define BUT_MODE_DEBOUNCE_TIMEOUT 10000     //Debounce Bouton Mode
#define BUT_MODE_DELAY_TIMEOUT 10000        //Delai Bouton Mode


enum DECT_REGION{
    USA=0x01,
    EUROPE=0x00,
    JAPAN=0x0A,
    JAPAN5CH=0x0B
};

//Enum pour choix de la diversit? du CVM
enum DIVERSITY_MODES
{
    AntennaDiversity,
    Antenna1Always,
    Antenna2Always
};

enum DECT_MODES{
    FP_IMAGE,
    PP_IMAGE        
} ;

//Variables LED_ETAT
#define CLIGNOTE_LENT               1   //EN ATTENTE
#define FIXE                        2   //EN COMM
#define CLIGNOTE_ABONNEMENT         3   //EN ABONNEMENT
#define CLIGNOTE_CHARGE             4   //
#define ABONNEMENT_ECHEC            5   //
#define ON_LED                      6   //
#define OFF_LED                     7   //
#define CLIGNOTE_COMMUNICATION_BC   8   //CVM ABONNE (CASQUE FP)
#define CLIGNOTE_ABONNEMENT_RR      9   //CVM EN_ABONNEMENT_RR
#define CONFIG                      10  //Configuration du CVM en cours
#define CONFIGURED                  11  //Configuration du CVM termin�e
#define CLIGNOTE_RAPIDE_BLEU        12  //Clignotement rapide
#define CLIGNOTE_RAPIDE             13

//Variables LED_COULEUR
#define ROUGE	1
#define VERT	2
#define ORANGE	3
#define BLEU    4
#define BLANC   5
#define VIOLET  6
#define TURQUOISE 7
#define TEST_DIAG   8

// Variables etat appareil

enum {
    ETAT_APPAREIL_NORMAL,
    ETAT_APPAREIL_CONFIG_USB,
    ETAT_APPAREIL_EN_CHARGE
};

//Variables Etat peut prendre les valeurs suivantes
#define INITIALISATION      0   //�tat d'init (LCD et BKL �teint pdt cette p�riode)
#define DEMARRAGE           1   //etat de demarrage
#define IMAGE_ACQUIS        2   //obtention de l'image du CVM
#define SANS_COM            3   //sans communication
#define SYNCHRONISE         4   
#define ABONNE              5   //audio ouvert
#define D_CONNECT           6
#define CONNECT             7   //abonne
#define NO_PP               8
#define AUDIO_OPEN          9
#define DEMANDE_APPEL       10
#define EN_ABONNEMENT       11  //en cours d'abonnement
#define RELEASE_PP_FROM_FP  12
#define EN_ABONNEMENT_RR    13  //en cours de connexion lorsqu'on est en mode Remote Recognition
#define SCAN_EN_COURS       14  //en cours de scan bases en RR
#define ABONNE_RR           15  //Etat Abonne en mode Remote
#define CVM_STANDBY         16  //Bascule sur le mode DMR. CVM plus utilise
#define CONEI_CFM           17
#define SETUP               18
#define REG_COMPLETE        19
#define CONFIG_UNIT         20  //config du mobile en cours, remplace par Config_Unit_Enable
#define PP2PP_ENTER             21
#define PP2PP_ENTER_FP_MODE     22
#define PP2PP_IDLE              23
#define PP2PP_SETUP_REQ         24
#define WAIT_PP2PP_CONNECT_IND  25
#define PP2PP_CONNECT_REQ       26
#define WAIT_CVM                27
#define PP2PP_EXIT_REQ          28
#define CONFIG_START            29
#define CONFIG_EEPROM_DEFAULT   30
#define CONFIG_DECT_CHANGE      31
#define CONFIG_EEPROM           32
#define CONFIG_DONE             33
#define CHANGE_IMAGE_CVM        34
#define SANS_COM_BASE           35
#define CONNECTED_TO_MOBILE     36

//PP2PP_Mode
#define Disabled 0
#define Master   1
#define Slave    2

#define AC_INPUT   0x01

//Variables Niveaux CVM Mic, Lsr et Sidetone par defaut
#define MIC_GAIN_INIT    2       // MIC_GAIN_INIT valeur d'initialisation du gain micro CVM entre 0 et 4
#define LSR_GAIN_INIT    2       // LSR_GAIN_INIT valeur d'initialisation du gain HP entre 0 et 4
#define SDT_GAIN_INIT    2       // SDT_GAIN_INIT valeur d'initialisation du gain retour d'�coite entre 0 et 4

////////////////////////////////////////////////////////////////////////////////
// DEFINITION DES API CVM //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#define API_PP_REMOTE_RECOGNITION_FP_FOUND_IND      0x5126
#define API_PP_MM_REMOTE_RECOGNITION_LIST_STOP_REQ  0x5127
#define API_PP_MM_REMOTE_RECOGNITION_LIST_REQ       0x5125
#define API_PP_MM_REGISTRATION_SELECTED_REQ         0x510E

#define API_LAS_START_SESSION_REQ                   0x5500
#define API_LAS_START_SESSION_CFM                   0x5501
#define API_LAS_READ_ENTRIES_REQ                    0x5506
#define API_LAS_EDIT_ENTRY_REQ                      0x5508
#define API_LAS_SAVE_ENTRY_REQ                      0x550A
#define API_LAS_SAVE_ENTRY_CFM                      0x550B
#define API_LAS_DELETE_ENTRY_REQ                    0x550C
#define API_LAS_DELETE_ENTRY_CFM                    0x550D
#define API_LAS_DATA_PACKET_IND                     0x5513
        
#define API_PROD_TEST_REQ                           0x4FFE
#define API_PROD_TEST_CFM                           0x4FFF
#define API_IMAGE_INFO_REQ                          0x5800
#define API_IMAGE_INFO_CFM                          0x5801
#define API_FP_MM_SET_REGISTRATION_MODE_REQ         0x4105
#define API_PP_MM_REGISTRATION_AUTO_REQ             0x510B
#define API_PP_MM_STOP_PROTOCOL_REQ                 0x510A
#define API_PP_AUDIO_CLOSE_REQ                      0x5324
#define API_CC_RELEASE_REQ                          0x5014
#define API_CC_RELEASE_IND                          0x5016
#define API_PP_MM_REGISTRATION_STOP_REQ             0x510F
#define API_HAL_READ_REQ                            0x5904
#define API_HAL_READ_CFM                            0x5905
#define API_HAL_WRITE_REQ                           0x5906
#define API_HAL_WRITE_CFM                           0x5907

#define API_PP_MM_READ_RSSI_REQ                     0x5104
#define API_PP_MM_READ_RSSI_CFM                     0x5105
    
#define API_IMAGE_ACTIVATE_REQ                      0x5802
#define API_IMAGE_ACTIVATE_CFM                      0x5803
#define API_PP_RESET_IND                            0x5203
#define API_PP_RESET_REQ                            0x5202
#define API_FP_RESET_IND                            0x4001
#define API_FP_RESET_REQ                            0x4000
    
/*  
#define RSS_SUCCESS                                 0x00
#define RSS_NO_DEVICE                               0x09
#define RSS_BAD_ADDRESS                             0x03
#define RSS_BAD_LENGTH                              0x07
*/  
    
#define API_PP_MM_LOCKED_IND                        0x5108
#define API_PP_MM_UNLOCKED_IND                      0x5109
#define API_PP_MM_REGISTRATION_COMPLETE_IND         0x5110
#define API_PP_MM_LOCK_REQ                          0x5106
#define API_PP_MM_LOCKED_REQ                        0x5107
#define API_CC_GET_CONEI_REQ                        0x5036
#define API_CC_GET_CONEI_CFM                        0x5037
#define API_CC_CONNECT_CFM                          0x500D
#define API_CC_CONNECT_IND                          0x5004
#define API_CC_SETUP_REQ                            0x5000
#define API_CC_REJECT_IND                           0x5018
#define API_PP_AUDIO_OPEN_REQ                       0x5323
#define API_PP_AUDIO_SET_VOLUME_REQ                 0x5303
#define API_PP_AUDIO_UNMUTE_REQ                     0x5301
#define API_PP_MM_REGISTRATION_FAILED_IND           0x5111
#define API_CC_RELEASE_CFM                          0x5015
#define API_PP_AUDIO_MUTE_REQ                       0x5300
#define API_CC_SETUP_ACK_IND                        0x5001
#define API_PP_MM_UNITDATA_IND                      0x5119
#define API_PP_AUDIO_START_TONE_REQ                 0x5306
#define API_PP_AUDIO_STOP_TONE_REQ                  0x5307
//#define API_PP_AUDIO_SET_VOLUME_REQ                 0x5303
#define API_PP_AUDIO_START_TONE_EXT_REQ             0x5328
#define API_PP_AUDIO_START_TONE_CFM                 0x5314
#define API_CC_INFO_IND                             0x5031
#define API_CC_INFO_REQ                             0x5032

    
#define API_FP_MM_START_PROTOCOL_REQ                0x410D
#define API_FP_MM_REGISTRATION_COMPLETE_IND         0x4107
#define API_FP_AUDIO_INTERNAL_CODEC_SETUP_REQ       0x4216
#define API_FP_AUDIO_INTERNAL_CODEC_SETUP_CFM       0x4217
#define API_FP_AUDIO_SET_HW_SRC_REQ                 0x4218
#define API_FP_AUDIO_SET_HW_SRC_CFM                 0x4219
#define API_FP_SET_AUDIO_FORMAT_REQ                 0x4210
#define API_FP_SET_AUDIO_FORMAT_CFM                 0x4211
#define API_FP_AUDIO_SET_CODEC_GAIN_REQ             0x421A
#define API_FP_AUDIO_SET_CODEC_GAIN_CFM             0x421B
#define API_FP_CC_SETUP_IND                         0x4400
#define API_FP_CC_SETUP_RES                         0x4401
#define API_FP_CC_SETUP_ACK_REQ                     0x4423
#define API_FP_CC_SETUP_ACK_CFM                     0x4424
#define API_FP_CC_CONNECT_REQ                       0x4409
#define API_FP_CC_CONNECT_CFM                       0x440A
#define API_FP_CC_RELEASE_IND                       0x440F
#define API_FP_MM_GET_REGISTRATION_COUNT_REQ        0x4100
#define API_FP_MM_GET_REGISTRATION_COUNT_CFM        0x4101
#define API_FP_RESET_IND                            0x4001
#define API_FP_INIT_PCM_REQ                         0x4200
#define API_FP_INIT_PCM_CFM                         0x4201
#define API_FP_MAC_NO_EMISSION_MODE_DISABLE_REQ     0x4601
#define API_FP_AUDIO_UNMUTE_REQ                     0x4214
#define API_FP_CC_REJECT_IND                        0x4413
#define API_FP_CC_RELEASE_RES                       0x4410
#define API_FP_CC_RELEASE_REQ                       0x4411
#define API_FP_CC_RELEASE_CFM                       0x4412
#define API_FP_MM_STOP_PROTOCOL_REQ                 0x410B
#define API_FP_MM_UNITDATA_REQ                      0x4180
#define API_FP_AUDIO_ENABLE_EC_REQ                  0x421C
#define API_FP_AUDIO_ENABLE_EC_CFM                  0x421D
#define API_PP2PP_ENTER_REQ                         0x4920
#define API_PP2PP_ENTER_CFM                         0x4921
#define API_PP2PP_EXIT_REQ                          0x4922
#define API_PP2PP_EXIT_CFM                          0x4923
#define API_PP2PP_ENTER_FP_MODE_REQ                 0x4924
#define API_PP2PP_ENTER_FP_MODE_CFM                 0x4925
#define API_PP2PP_SETUP_REQ                         0x4926
#define API_PP2PP_CONNECT_IND                       0x4927
#define API_PP2PP_SETUP_IND                         0x4928
#define API_PP2PP_CONNECT_REQ                       0x4929
#define API_PP2PP_CONNECT_CFM                       0x492A
#define API_PP2PP_RELEASE_REQ                       0x492B
#define API_PP2PP_RELEASE_CFM                       0x492C
#define API_PP2PP_RELEASE_IND                       0x492D


    
#define API_FP_MM_GET_REGISTRATION_COUNT_REQ        0x4100
#define API_FP_MM_GET_REGISTRATION_COUNT_CFM        0x4101
    
#define API_FP_MM_HANDSET_PRESENT_IND               0x4108
//#define API_FP_MM_HANDSET_DETACH_IND                0x410E
    
#define API_FP_MM_DELETE_REGISTRATION_REQ           0x4102
#define API_FP_MM_DELETE_REGISTRATION_CFM           0x4103
    
#define API_FP_CC_INFO_REQ                          0x442E
#define API_FP_AUDIO_MUTE_REQ                       0x4212
#define API_FP_AUDIO_MUTE_CFM                       0x4213
#define API_FP_AUDIO_UNMUTE_CFM                     0x4215
    
#define API_FP_CC_INFO_IND                          0x442D
#define API_PP_GET_FW_VERSION_REQ                   0x5200
#define API_FP_GET_FW_VERSION_REQ                   0x4002
#define API_PP_GET_FW_VERSION_CFM                   0x5201
#define API_FP_GET_FW_VERSION_CFM                   0x4003

#define API_FP_RESET_IND                            0x4001

////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Gestion de la batterie Li-Ion DS2784 avec Data-One-Wire /////////////////////
////////////////////////////////////////////////////////////////////////////////
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

///******************************************************************************/
///* MASQUES
///*----------------------------------------------------------------------------*/
///* BIT0 -> 0000 0001                                                          */
///* BIT1 -> 0000 0010                                                          */
///* BIT2 -> 0000 0100                                                          */
///* BIT3 -> 0000 1000                                                          */
///* BIT4 -> 0001 0000                                                          */
///* BIT5 -> 0010 0000                                                          */
///* BIT6 -> 0100 0000                                                          */
///* BIT7 -> 1000 0000                                                          */
///*----------------------------------------------------------------------------*/
#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80

////////////////////////////////////////////////////////////////////////////////
// DEFINITION DES VARIABLES DE L'ISD ///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/*
// Variables des messages de voix pour l'ISD
#define		Power_ON_message				0x03
#define		Power_OFF_message				0x0A
#define		Low_Battery_message				0x04
#define		Lost_Communication				0x06
#define 	Communication_Established		0x07

#define		Low_Battery_Base_message		0x05
#define		Too_Far_From_Base				0x08
#define		Base_Not_Removed				0x09
*/


//Timer appui bouton
#define BTN_LONG_PRESS_TIMEOUT    20

#endif	/* VARIABLES_H */
