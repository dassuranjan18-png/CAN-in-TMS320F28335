/*
###########################################################################
 *  Version | DD MM YYYY | Who  | Description of Changes
 * =========|============|======|===============================================
 * RAM 1.0  | 16/05/2024 | S.D  | Code for CAN Communication(SDO[Enable,Disable] & PDO) to run Epos4 50/5 in PPM mode.
 *
 *
 *
 *
###########################################################################

*/
//
// Included Files
//
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File


//#### external function prototypes
    extern void InitSysCtrl(void);
    extern void InitPieVectTable(void);
    extern void InitPieCtrl(void);

    extern void InitECanaGpio(void);
    extern void InitECana(void);

    void Gpio_select(void);
    void MBXwrA(void);                 // This function initializes all 32 MBOXes of CAN-A

    interrupt void eCAN1INT_ISR(void);	//	Prototype for CAN Mailbox ISR

    struct ECAN_REGS ECanaShadow;

//variables of ECANa
unsigned short int Allow_Communication = 0 , Start_position=0 , Enable=0, Disable=0, Position_Actual_Value=0, NMT_OPERATIONAL=0, Velocity_update=0, Motor_Parameter=0;
unsigned short int MIV = 0, DigitalInputStates = 0, EPOS_DI1_State = 0;
long int Target_position=0 , Profile_velocity=0,Current_Position=0,Statusword=0,Mode=0,Current_Actual_Value=0,Torque_Actual_Value=0,PWM=0;

main()
{


//struct ECAN_REGS ECanaShadow;

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP280x_SysCtrl.c file.
   InitSysCtrl();
   Gpio_select();                      // GPIO Configuration
/* Initialize the CAN module */

    InitECanaGpio();
    InitECana();
    EALLOW;

/* Zero out (or initialize) the entire MBX RAM area */

    MBXwrA();

/* Write to the MSGID field 0f CAN-A - MBX number is written as its MSGID */
    ECanaMboxes.MBOX20.MSGID.bit.STDMSGID = 0x281;	//TXPDO2 COB-ID;MAPPING-BYTE[0-3]-Current Actual Value,BYTE[4-5]-Torque Actual Value.
    ECanaMboxes.MBOX21.MSGID.bit.STDMSGID = 0x181;	//TXPDO1 COB-ID;MAPPING-BYTE[0-3]-Position Actual Value,BYTE[4-5]-Statusword,BYTE[6]-Modes Of Operation Display.
    ECanaMboxes.MBOX22.MSGID.bit.STDMSGID = 0x301;	//RXPDO2 COB-ID;MAPPING-BYTE[0]-Homing Method,BYTE[1-2]-Controlword,BYTE[3]-Modes Of Operation,BYTE[4-7]-Target Position.
    ECanaMboxes.MBOX23.MSGID.bit.STDMSGID = 0x601;	//SDO COB-ID
    ECanaMboxes.MBOX24.MSGID.bit.STDMSGID = 0x201;	//RXPDO1 COB-ID;MAPPING-BYTE[0-3]-Profile Velocity.
    ECanaMboxes.MBOX25.MSGID.bit.STDMSGID = 0x0;	//NMT COB-ID
    ECanaMboxes.MBOX26.MSGID.bit.STDMSGID = 0x080;	//SYNC COB-ID
    ECanaMboxes.MBOX27.MSGID.bit.STDMSGID = 0x381;	//TXPDO3 COB-ID;MAPPING-BYTE[0-1]-Digital inputs logic state


/* Configure CAN-A Mailboxes Direction */
    ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;
    ECanaShadow.CANMD.bit.MD20 = 1;
    ECanaShadow.CANMD.bit.MD21 = 1;
    ECanaShadow.CANMD.bit.MD27 = 1;
    ECanaShadow.CANMD.bit.MD22 = 0;
    ECanaShadow.CANMD.bit.MD23 = 0;
    ECanaShadow.CANMD.bit.MD24 = 0;
    ECanaShadow.CANMD.bit.MD25 = 0;
    ECanaShadow.CANMD.bit.MD26 = 0;
    ECanaRegs.CANMD.all = ECanaShadow.CANMD.all;


/* Enable Mailboxes */
    ECanaShadow.CANME.all = ECanaRegs.CANME.all;
    ECanaShadow.CANME.bit.ME20 = 1;
    ECanaShadow.CANME.bit.ME21 = 1;
    ECanaShadow.CANME.bit.ME22 = 1;
    ECanaShadow.CANME.bit.ME23 = 1;
    ECanaShadow.CANME.bit.ME24 = 1;
    ECanaShadow.CANME.bit.ME25 = 1;
    ECanaShadow.CANME.bit.ME26 = 1;
    ECanaShadow.CANME.bit.ME27 = 1;
    ECanaRegs.CANME.all = ECanaShadow.CANME.all;


    ECanaMboxes.MBOX23.MSGCTRL.bit.DLC = 8; // For SDO to ENABLE the EPOS4
    ECanaMboxes.MBOX25.MSGCTRL.bit.DLC = 2; // For NMT-Operational
    ECanaMboxes.MBOX26.MSGCTRL.bit.DLC = 1; // For SYNC
    ECanaMboxes.MBOX22.MSGCTRL.bit.DLC = 8; // For RXPDO2
    ECanaMboxes.MBOX24.MSGCTRL.bit.DLC = 8; // For RXPDO1

    /* Write to DBO field in Master Control reg for eCAN A module*/
    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.DBO = 1;                      // Data is stored or read starting with the least significant byte of the CANMDL register
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;


    DINT;
    IER = 0x0000;
    IFR = 0x0000;

    ECanaShadow.CANMIL.all = 0;
    ECanaShadow.CANMIL.bit.MIL27 = 1;
    ECanaRegs.CANMIL.all = ECanaShadow.CANMIL.all; 	// Receive Mailbox 27 to generate interrupt on Line1

    ECanaShadow.CANMIM.all = 0;
    ECanaShadow.CANMIM.bit.MIM27 = 1;
    ECanaRegs.CANMIM.all = ECanaShadow.CANMIM.all;	// Mailbox interrupt is enabled for Receive Mailbox 27

    ECanaShadow.CANGIM.all = 0;
    ECanaShadow.CANGIM.bit.I1EN = 1;
    ECanaRegs.CANGIM.all = ECanaShadow.CANGIM.all;	// Enabling the ECAN1INT line

    InitPieCtrl();      // basic setup of PIE table; from DSP2833x_PieCtrl.c
    InitPieVectTable(); // default ISR's in PIE

    PieVectTable.ECAN1INTA = &eCAN1INT_ISR;		// Changing the ISR addresses from default to user defined ISRs

    PieCtrlRegs.PIEIER9.bit.INTx6 = 1; 			// Enables PIE:INT9 line to drive a interrupt pulse into the CPU through ECAN1_INTA

    PieCtrlRegs.PIEACK.bit.ACK9 = 1;			// Enables PIE to drive a pulse into the CPU

    IER |= 0x0100;                      		// Enabling of INT9 IER (Bit:8)

    EDIS;


    Target_position=0;//Default position
    Profile_velocity=1000; //Default speed

    // SDO Frame bytes data except the data byte[4-7]
            ECanaMboxes.MBOX23.MDL.byte.BYTE0 = 0x22; //CCS
            ECanaMboxes.MBOX23.MDL.byte.BYTE1 = 0x40; //Index[low byte]
            ECanaMboxes.MBOX23.MDL.byte.BYTE2 = 0x60; //Index[high byte]
            ECanaMboxes.MBOX23.MDL.byte.BYTE3 = 0x00; //Sub index

    // NMT - Operational frame
            ECanaMboxes.MBOX25.MDL.byte.BYTE0 = 0x01;//CS(Start remote node)
            ECanaMboxes.MBOX25.MDL.byte.BYTE1 = 0x00;//Node-Id,0X00 for all node

    //RXPDO2 - Homing method,Modes of operation
            ECanaMboxes.MBOX22.MDL.byte.BYTE0 = 0x00;//Homing method
            ECanaMboxes.MBOX22.MDL.byte.BYTE3 = 0x01;//Modes of operation(PPM)



    //  For no failed communication at start. A disable frame is sent as the first frame over CAN bus.
            ECanaMboxes.MBOX23.MDH.byte.BYTE4=  0x06;
            ECanaShadow.CANTRS.all = 0;
            ECanaShadow.CANTRS.bit.TRS23 = 1;               // Set TRS for mailbox under test: Transmission request
            ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;
            do
            {
               ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
            }
            while (ECanaShadow.CANTA.bit.TA23 == 0);        // Wait for TA25 bit to be set: Transmission acknowledged for mailbox 25

            ECanaShadow.CANTA.all = 0;
            ECanaShadow.CANTA.bit.TA23 = 1;                 // Clear TA25 bit
            ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;


    EINT;		// Enable all interrupts


    while(1)
        {


        /* Transmission of data from Master to Slave */




        if(Allow_Communication == 1)
        {

            if (NMT_OPERATIONAL == 1)// NMT-OPERATIONAL for start PDO Communication
            {
               ECanaShadow.CANTRS.all = 0;
               ECanaShadow.CANTRS.bit.TRS25 = 1;               // Set TRS for mailbox under test: Transmission request
               ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;
               do
               {
                 ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
               }
               while (ECanaShadow.CANTA.bit.TA25 == 0);        // Wait for TA25 bit to be set: Transmission acknowledged for mailbox 25

               ECanaShadow.CANTA.all = 0;
               ECanaShadow.CANTA.bit.TA25 = 1;                 // Clear TA25 bit
               ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;

               NMT_OPERATIONAL = 0;

            }// End of NMT-OPERATIONAL





            if (Enable == 1)//  Enable for Motor power up through SDO
            {
              ECanaMboxes.MBOX23.MDH.byte.BYTE4=  0x06;//SDO Data byte for disable

              ECanaShadow.CANTRS.all = 0;
              ECanaShadow.CANTRS.bit.TRS23 = 1;               // Set TRS for mailbox under test: Transmission request
              ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;
              do
              {
                ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
              }
              while (ECanaShadow.CANTA.bit.TA23 == 0);        // Wait for TA23 bit to be set: Transmission acknowledged for mailbox 23

              ECanaShadow.CANTA.all = 0;
              ECanaShadow.CANTA.bit.TA23 = 1;                 // Clear TA23 bit
              ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;

              ECanaMboxes.MBOX23.MDH.byte.BYTE4=  0x0F;//SDO Data byte for enable

              ECanaShadow.CANTRS.all = 0;
              ECanaShadow.CANTRS.bit.TRS23 = 1;               // Set TRS for mailbox under test: Transmission request
              ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;
              do
              {
                ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
              }
              while (ECanaShadow.CANTA.bit.TA23 == 0);        // Wait for TA23 bit to be set: Transmission acknowledged for mailbox 23

              ECanaShadow.CANTA.all = 0;
              ECanaShadow.CANTA.bit.TA23 = 1;                 // Clear TA23 bit
              ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;

              Enable=0;

            }// End of  Enable

            if (Velocity_update == 1)//  Profile_velocity update through RXPDO1
            {
               ECanaMboxes.MBOX24.MDL.all= Profile_velocity;//Data byte for Profile_velocity update

               ECanaShadow.CANTRS.all = 0;
               ECanaShadow.CANTRS.bit.TRS24 = 1;               // Set TRS for mailbox under test: Transmission request
               ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;
               do
               {
                 ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
               }
               while (ECanaShadow.CANTA.bit.TA24 == 0);        // Wait for TA25 bit to be set: Transmission acknowledged for mailbox 25

               ECanaShadow.CANTA.all = 0;
               ECanaShadow.CANTA.bit.TA24 = 1;                 // Clear TA25 bit
               ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;

               Velocity_update = 0;

            }// End of Profile_velocity update


            if (Start_position == 1)//Start positioning through RXPDO2
            {
              ECanaMboxes.MBOX22.MDL.byte.BYTE1 = 0x5F;              //Data for Type of Position(relative)
              ECanaMboxes.MBOX22.MDH.all= Target_position;           //4 bytes of data for Target Position

              ECanaShadow.CANTRS.all = 0;
              ECanaShadow.CANTRS.bit.TRS22 = 1;               // Set TRS for mailbox under test: Transmission request
              ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;
              do
              {
                ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
              }
              while (ECanaShadow.CANTA.bit.TA22 == 0);        // Wait for TA22 bit to be set: Transmission acknowledged for mailbox 22

              ECanaShadow.CANTA.all = 0;
              ECanaShadow.CANTA.bit.TA22 = 1;                 // Clear TA22 bit
              ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;


              ECanaMboxes.MBOX22.MDL.byte.BYTE1 = 0x0F;              //Data for Start Positioning

              ECanaShadow.CANTRS.all = 0;
              ECanaShadow.CANTRS.bit.TRS22 = 1;               // Set TRS for mailbox under test: Transmission request
              ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;
              do
              {
                ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
              }
              while (ECanaShadow.CANTA.bit.TA22 == 0);        // Wait for TA22 bit to be set: Transmission acknowledged for mailbox 24

              ECanaShadow.CANTA.all = 0;
              ECanaShadow.CANTA.bit.TA22 = 1;                 // Clear TA22 bit
              ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;


              Start_position = 0;
             }// End of Start_position


           if (Disable == 1)//Disable for Motor power down through SDO
           {

               ECanaMboxes.MBOX23.MDH.byte.BYTE4=  0x06;      //Disable Message

               ECanaShadow.CANTRS.all = 0;
               ECanaShadow.CANTRS.bit.TRS23 = 1;               // Set TRS for mailbox under test: Transmission request
               ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;
               do
               {
                  ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
               }
               while (ECanaShadow.CANTA.bit.TA23 == 0);        // Wait for TA23 bit to be set: Transmission acknowledged for mailbox 25

               ECanaShadow.CANTA.all = 0;
               ECanaShadow.CANTA.bit.TA23 = 1;                 // Clear TA23 bit
               ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;

               Disable=0;
           }// End of Disable


           if (Position_Actual_Value == 1) //Taking position actual value through TXPDO1 with SYNC object
           {
               ECanaMboxes.MBOX26.MDL.byte.BYTE0=  0x00; // SYNC Data

               ECanaShadow.CANTRS.all = 0;
               ECanaShadow.CANTRS.bit.TRS26 = 1;               // Set TRS for mailbox under test: Transmission request
               ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;
               do
               {
                 ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
               }
               while (ECanaShadow.CANTA.bit.TA26 == 0);        // Wait for TA27 bit to be set: Transmission acknowledged for mailbox 27

               ECanaShadow.CANTA.all = 0;
               ECanaShadow.CANTA.bit.TA26 = 1;                 // Clear TA27 bit
               ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;



               ECanaShadow.CANRMP.all = ECanaRegs.CANRMP.all;
               do
               {
                  ECanaShadow.CANRMP.all = ECanaRegs.CANRMP.all;
               }
               while (ECanaShadow.CANRMP.bit.RMP21 == 0);      // Wait for RMP21 bit to be set: Message received in mailbox 21

               Current_Position = ECanaMboxes.MBOX21.MDL.all;
               Statusword = ECanaMboxes.MBOX21.MDH.word.LOW_WORD;
               Mode = ECanaMboxes.MBOX21.MDH.byte.BYTE6;

               ECanaShadow.CANRMP.all = 0;
               ECanaShadow.CANRMP.bit.RMP21 = 1;               // Clear RMP21 bit afer reading the data
               ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;

               Position_Actual_Value=0;
           }// End of Position_Actual_Value

           if (Motor_Parameter == 1) //Taking Motor parameter through TXPDO2 with SYNC object
           {
               ECanaMboxes.MBOX26.MDL.byte.BYTE0=  0x00; // SYNC Data

               ECanaShadow.CANTRS.all = 0;
               ECanaShadow.CANTRS.bit.TRS26 = 1;               // Set TRS for mailbox under test: Transmission request
               ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;
               do
               {
                 ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
               }
               while (ECanaShadow.CANTA.bit.TA26 == 0);        // Wait for TA27 bit to be set: Transmission acknowledged for mailbox 27

               ECanaShadow.CANTA.all = 0;
               ECanaShadow.CANTA.bit.TA26 = 1;                 // Clear TA27 bit
               ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;



               ECanaShadow.CANRMP.all = ECanaRegs.CANRMP.all;
               do
               {
                  ECanaShadow.CANRMP.all = ECanaRegs.CANRMP.all;
               }
               while (ECanaShadow.CANRMP.bit.RMP20 == 0);      // Wait for RMP21 bit to be set: Message received in mailbox 21

               Current_Actual_Value = ECanaMboxes.MBOX20.MDL.all;
               Torque_Actual_Value = ECanaMboxes.MBOX20.MDH.word.LOW_WORD;
               PWM = ECanaMboxes.MBOX20.MDH.word.HI_WORD;

               ECanaShadow.CANRMP.all = 0;
               ECanaShadow.CANRMP.bit.RMP20 = 1;               // Clear RMP21 bit afer reading the data
               ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;

               Motor_Parameter=0;
           }// End of Position_Actual_Value



        } //End of Allow_Communication

       }//End of while loop

}//End of Main



void Gpio_select(void)
    {
        EALLOW;

            GpioCtrlRegs.GPAMUX1.all        =   0x0000;                         //  GPIO15 ... GPIO0 = General Puropse I/O
            GpioCtrlRegs.GPAMUX2.all        =   0x0000;                         //  GPIO31 ... GPIO16 = General Purpose I/O

            GpioCtrlRegs.GPAPUD.all         =   0xFFFFFFFF;                     //  Internal Pull Up Disabled for GPIO0-31 (GPIO0-11 are disabled by default)
            GpioDataRegs.GPACLEAR.all       =   0xFFFFFFFF;                     //  Latch is Cleared
            GpioCtrlRegs.GPADIR.all         =   0xFFFFFFFF;                     //  GPIOs from 0-31 declared as Output, This doesn't affect pins defined as Peripherals

            GpioCtrlRegs.GPAMUX1.bit.GPIO0      =   0;                          // GPIO 0 as Digital port to send digital signal to EPOS4
            GpioCtrlRegs.GPADIR.bit.GPIO0       =   1;                          // GPIO 0 as output
            GpioCtrlRegs.GPAPUD.bit.GPIO0       =   1;                          // GPIO 0 Pullup disabled
            GpioCtrlRegs.GPAMUX1.bit.GPIO9		=	0;							// GPIO 9 as Digital port to glow LED1 on recpetion of EPOS4 digital input state as high
            GpioCtrlRegs.GPADIR.bit.GPIO9		=	1;							// GPIO 9 as output for LED1
            GpioCtrlRegs.GPAPUD.bit.GPIO9		=	1;							// GPIO 9 Pullup disabled

            GpioCtrlRegs.GPBMUX1.all        =   0x0000;                         //  GPIO47 ... GPIO32 = General Purpose I/O
            GpioCtrlRegs.GPBMUX2.all        =   0x0000;                         //  GPIO63 ... GPIO48 = General Purpose I/O
            GpioCtrlRegs.GPBPUD.all         =   0xFFFFFFFF;                     //  Internal Pull Up Disabled for GPIO32-63
            GpioDataRegs.GPBCLEAR.all       =   0xFFFFFFFF;                     //  Latch is Cleared
            GpioCtrlRegs.GPBDIR.all         =   0xFFFFFFFF;                     //  GPIOs from 32-63 declared as Output, This doesn't affect pins defined as Peripherals


            GpioCtrlRegs.GPCMUX1.all        =   0x0000;                         //  GPIO79 ... GPIO64 = General Purpose I/O
            GpioCtrlRegs.GPCMUX2.all        =   0x0000;                         //  GPIO87 ... GPIO80 = General Purpose I/O
            GpioCtrlRegs.GPCPUD.all         =   0x00FFFFFF;                     //  Internal Pull Up Disabled for GPIO64-87
            GpioDataRegs.GPCCLEAR.all       =   0x00FFFFFF;                     //  Latch is Cleared
            GpioCtrlRegs.GPCDIR.all         =   0X00FFFFFF;                     //  GPIOs from 64-87 declared as Output, This doesn't affect pins defined as Peripherals

            GpioCtrlRegs.GPCDIR.bit.GPIO84  =   0;                              //  84-87 declared as Input and it is connected to SW1
            GpioCtrlRegs.GPCDIR.bit.GPIO85  =   0;
            GpioCtrlRegs.GPCDIR.bit.GPIO86  =   0;
            GpioCtrlRegs.GPCDIR.bit.GPIO87  =   0;
            // Input Qualification is not applicable for Port C GPIOs.

        EDIS;
    }



/* Zero-out the MBX RAM of CAN-A */

void MBXwrA()
    {
    int j;
    volatile struct MBOX *Mailbox  = (void *) 0x6100;

        for(j=0; j<32; j++)
        {
            Mailbox->MSGID.all = 0;
            Mailbox->MSGCTRL.all = 0;
            Mailbox->MDH.all = 0;
            Mailbox->MDL.all = 0;
            Mailbox = Mailbox + 1;

        }
    }


interrupt void eCAN1INT_ISR(void)
{

	MIV = ECanaRegs.CANGIF1.bit.MIV1;

	DigitalInputStates = ECanaMboxes.MBOX27.MDL.word.LOW_WORD;		// To read the 16 bit information on digital input states coming from EPOS4

	EPOS_DI1_State = DigitalInputStates & 0x0001;

	if (EPOS_DI1_State == 1)
	{
		GpioDataRegs.GPASET.bit.GPIO9 = 1;
	}
	else
	{
		GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
	}

	ECanaShadow.CANRMP.all = 0;
    ECanaShadow.CANRMP.bit.RMP27 = 1;               // Clear RMP27 bit afer reading the data
    ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;

    PieCtrlRegs.PIEACK.bit.ACK9 = 1;			// Enables PIE to drive a pulse into the CPU

    IER |= 0x0100;                      		// Enabling of INT9 IER (Bit:8)

    EINT;										// Enable global interrupt

}







