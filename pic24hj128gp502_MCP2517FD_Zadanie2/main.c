/*****************************************************************************
  FileName:        main.c
  Processor:       PIC24HJ128GP502
  Compiler:        XC16 ver 1.30
  IDE :            MPLABX-IDE
  Created by:      http://strefapic.blogspot.com
 ******************************************************************************/

#include "xc.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h> /*definicje typu uint8_t itp*/
#include <string.h>
#include "ustaw_zegar.h" /*tutaj m.in ustawione FCY*/
#include <libpic30.h>
#include <p24HJ128GP502.h> /*dostep do delay-i,musi byc po zaincludowaniu ustaw_zegar.h*/
#include "drv_canfdspi_register.h"

#define MAX_DATA_BYTES 64
#define SPI_DEFAULT_BUFFER_LENGTH 96
/*instrukcje do dzialania z kontrolerem CAN*/
#define cINSTRUCTION_RESET	0x00
#define cINSTRUCTION_WRITE  0x02
#define cINSTRUCTION_READ	0x03
/*wybrany rejestr do testu zapisu i odczytu*/
#define cREGADDR_CiFLTCON0   0x1D0 /*adres rejestru kontrolera CAN do Zadania 1*/
/*miganie dioda LED - */
#define LED1_TOG PORTA ^= (1<<_PORTA_RA1_POSITION) /*zmienia stan bitu na przeciwny*/

void MCP2517FD_TEST_REGISTER_ACCESS(void) ;
void DRV_CANFDSPI_WriteByteArray(uint16_t address,uint8_t *txd, uint16_t nBytes);
void DRV_CANFDSPI_ReadByteArray(uint16_t address, uint8_t *rxd, uint16_t nBytes);
void DRV_CANFDSPI_WriteByte(uint16_t address, uint8_t txd);
void DRV_CANFDSPI_ReadByte(uint16_t address, uint8_t *rxd);
void DRV_CANFDSPI_WriteWord(uint16_t address , uint32_t txd);
void DRV_SPI_TransferData(uint16_t spiTransferSize);
void DRV_CANFDSPI_Reset(void);
void SPI_CS_DESELECT(void);
void DRV_CANFDSPI_Config(void);
void SPI_CS_SELECT(void);
void DRV_CANFDSPI_OscillatorControlSet(CAN_OSC_CTRL ctrl);
void DRV_CANFDSPI_OscillatorControlObjectReset(CAN_OSC_CTRL* ctrl);
void DRV_CANFDSPI_ConfigureObjectReset(CAN_CONFIG* config);
void DRV_CANFDSPI_Configure(CAN_CONFIG* config);
void DRV_CANFDSPI_GpioModeConfigure(GPIO_PIN_MODE gpio0, GPIO_PIN_MODE gpio1);
void DRV_CANFDSPI_BitTimeConfigureNominal20MHz(CAN_BITTIME_SETUP bitTime);
void DRV_CANFDSPI_BitTimeConfigureNominal40MHz(CAN_BITTIME_SETUP bitTime);
void DRV_CANFDSPI_BitTimeConfigureData20MHz(CAN_BITTIME_SETUP bitTime, CAN_SSP_MODE sspMode);
void DRV_CANFDSPI_BitTimeConfigureData40MHz(CAN_BITTIME_SETUP bitTime, CAN_SSP_MODE sspMode);
void DRV_CANFDSPI_BitTimeConfigure(CAN_BITTIME_SETUP bitTime, CAN_SSP_MODE sspMode, CAN_SYSCLK_SPEED clk);
void DRV_CANFDSPI_TransmitChannelConfigureObjectReset(CAN_TX_FIFO_CONFIG* config);
void DRV_CANFDSPI_TransmitChannelConfigure(CAN_FIFO_CHANNEL channel, CAN_TX_FIFO_CONFIG* config);
void DRV_CANFDSPI_ReceiveChannelConfigure(CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_CONFIG* config);
void DRV_CANFDSPI_EccEnable(void);
void DRV_CANFDSPI_RamInit(uint8_t d);
void DRV_CANFDSPI_OperationModeSelect(CAN_OPERATION_MODE opMode);


void config_DMA0_SPI1(uint16_t Size);
void config_DMA1_SPI1(uint16_t Size);
void config_SPI_MASTER(void);
// SPI Transmit buffer DMA
uint8_t BuforTX[SPI_DEFAULT_BUFFER_LENGTH] __attribute__((space(dma)));
// SPI Receive buffer DMA
uint8_t BuforRX[SPI_DEFAULT_BUFFER_LENGTH] __attribute__((space(dma)));
/*Bufory pomocnicze*/
uint8_t rxd[MAX_DATA_BYTES]; /*bufor pomocniczy do ktorego przepisujemy dane z DMA*/
uint8_t txd[MAX_DATA_BYTES]; /*bufor pomocniczy z ktorego przepisujemy dane do DMA*/

REG_CiFLTCON_BYTE foo ; /*tworzymy nowa Unia, do formowania zawartosci (1 bajtu) rejestru CiFLTCON*/
 
int main(void) {
    ustaw_zegar(); /*odpalamy zegar wewnetrzny na ok 40MHz*/
    __delay_ms(50); /*stabilizacja napiec*/
    /*
     * wylaczamy ADC , wszystkie piny chcemy miec cyfrowe
     * pojedynczo piny analogowe wylaczamy w rejestrze AD1PCFGL 
     * Po resecie procka piny oznaczone ANx sa w trybie analogowych wejsc.
     */
    PMD1bits.AD1MD = 1; /*wylaczamy ADC*/
    /* 
     * ustawiamy wszystkie piny analogowe (oznacznone ANx) jako cyfrowe
     * do zmiany mamy piny AN0-AN5 i AN9-AN12 co daje hex na 16 bitach = 0x1E3F
     */
    AD1PCFGL = 0x1E3F;
    
    TRISAbits.TRISA1 = 0 ; /*RA1 jako wyjscie tu mamy LED*/
    TRISBbits.TRISB7 = 0 ; /*RB7 jako wyjscie tu mamy CS*/
    /*remaping pinow na potrzeby SPI
    SDO --> pin 11
    SDI --> pin 14
    SCK --> pin 15
    */
    RPOR2bits.RP4R = 7;     /*inaczej _RP4R = 7*/
    RPINR20bits.SDI1R = 5;  /*inaczej _SDI1R = 5*/
    RPOR3bits.RP6R = 8;     /*inaczej _RP6R = 8*/
   
    /*SPI init with DMA*/
    config_SPI_MASTER();
    DRV_CANFDSPI_Reset();
    
//    /*tworzymy pojednyczy bajt rejestru CiFLTCON , test zapisu/odczytu Zadanie 1*/   
//            foo.bF.BufferPointer = 0b01100 ;
//            foo.bF.Enable = 1 ;
//            /*foo.byte = 0b10001100  / dec --> 140;*/
      DRV_CANFDSPI_Config(); /*Zadanie 2 : konfiguracja kontrolera CAN*/ 
      /*od tego momentu kontroler CAN jest gotowy do transmisji/odbioru danych*/
      
      while (1) {

        /*Petla glowna programu*/
        //MCP2517FD_TEST_REGISTER_ACCESS(); /*odpal test transferu danych Zadanie 1*/ 
        //__delay_ms(1000) ;
    }

}


void DRV_CANFDSPI_WriteByteArray(uint16_t address, uint8_t *txd, uint16_t nBytes)
{
    uint16_t i;
    uint16_t spiTransferSize = nBytes + 2;
    
    // Compose command
    BuforTX[0] = (uint8_t) ((cINSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF));
    BuforTX[1] = (uint8_t) (address & 0xFF);

    // Add data
    for (i = 2; i < spiTransferSize; i++) {
        BuforTX[i] = txd[i - 2];
    }
/*tu odpalamy kanal DMA z danymi*/
    DRV_SPI_TransferData(spiTransferSize);

}

void DRV_CANFDSPI_ReadByteArray(uint16_t address, uint8_t *rxd, uint16_t nBytes)
{
    uint16_t i;
    uint16_t spiTransferSize = nBytes + 2;
    
    // Compose command
    BuforTX[0] = (uint8_t) ((cINSTRUCTION_READ << 4) + ((address >> 8) & 0xF));
    BuforTX[1] = (uint8_t) (address & 0xFF);

    // Clear data
    for (i = 2; i < spiTransferSize; i++) {
        BuforTX[i] = 0;
    }

    DRV_SPI_TransferData(spiTransferSize);

    // Update data 
    for (i = 0; i < nBytes; i++) {
        rxd[i] = BuforRX[i + 2];
    }

    }
/*wysyla bajt pod wskazany adres rejestru*/
void DRV_CANFDSPI_WriteByte(uint16_t address, uint8_t txd)
{
    uint16_t spiTransferSize = 3;
    
    // Compose command
    BuforTX[0] = (uint8_t) ((cINSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF));
    BuforTX[1] = (uint8_t) (address & 0xFF);
    BuforTX[2] = txd;

    DRV_SPI_TransferData(spiTransferSize);

  }

void DRV_CANFDSPI_ReadByte(uint16_t address, uint8_t *rxd)
{
    uint16_t spiTransferSize = 3;
    
    // Compose command
    BuforTX[0] = (uint8_t) ((cINSTRUCTION_READ << 4) + ((address >> 8) & 0xF));
    BuforTX[1] = (uint8_t) (address & 0xFF);
    BuforTX[2] = 0;

    DRV_SPI_TransferData(spiTransferSize);

    // Update data
    *rxd = BuforRX[2];

}

void DRV_CANFDSPI_WriteWord(uint16_t address , uint32_t txd)
{
    uint8_t i;
    uint16_t spiTransferSize = 6;
    
    // Compose command
    BuforTX[0] = (uint8_t) ((cINSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF));
    BuforTX[1] = (uint8_t) (address & 0xFF);

    // Split word into 4 bytes and add them to buffer
    for (i = 0; i < 4; i++) {
        BuforTX[i + 2] = (uint8_t) ((txd >> (i * 8)) & 0xFF);
    }

    DRV_SPI_TransferData(spiTransferSize);
    
}

void MCP2517FD_TEST_REGISTER_ACCESS(void) {
/*dane wpisujemy i odczytujemy z rejestru CiFLTCON0*/  
    /*rejestr sklada sie z 4 bajtów = 32bity*/
            uint8_t length , i;
            
            // wypelniamy bufory danymi*/
            for (length = 0; length < 4; length++) {
                    txd[length] = foo.byte ; /*powielamy 4 razy bajt bo rejestr ma 32bity */
                    rxd[length] = 0xff; /*wypelniamy wartoscia*/
                }
                
            /*Write data to registers CiFLTCON0*/
             DRV_CANFDSPI_WriteByteArray(cREGADDR_CiFLTCON0, txd, length);
 __delay_ms(500) ; /*opoznimy zapis i odczyt rejestru aby lepiej bylo widac na analizatorze stanow logicznych*/
            /*Read data back from registers CiFLTCON0*/
             DRV_CANFDSPI_ReadByteArray(cREGADDR_CiFLTCON0, rxd, length);
 
             PORTAbits.RA1 = 0; /*zgas LED*/
             /*sprawdzamy poprawnosc danych zapisanych do kontrolera CAN i odebranych*/
             _Bool good = false ;
             for(i=0 ; i < length; i++ ) {
                 good = txd[i] == rxd[i] ;
                 if(!good) PORTAbits.RA1 = 1; /*zapal LED jesli brak zgodnosci danych nadanych i odebranych*/
             } 
            }
            


void DRV_SPI_TransferData(uint16_t spiTransferSize)
{
    
    /*Assert CS*/
    SPI_CS_SELECT(); /*ustaw 0 na pinie CS*/

        /*Send Data from DMA*/
        config_DMA1_SPI1(spiTransferSize);          /*Enable DMA1 Channel*/	
        config_DMA0_SPI1(spiTransferSize);          /*Enable DMA0 Channel*/
        DMA0REQbits.FORCE = 1;                      /*Manual Transfer Start*/
        while(DMA0REQbits.FORCE==1) {};    
        /*Uwaga czekamy az kanal odbiorczy DMA zostanie zamkniety*/
        while(DMA1CONbits.CHEN != 0) {} /*czekaj az kanal odbiorczy DMA1 po ostatniej transmisji  zostanie zamkniety/zwolniony*/
        /*od tego momentu dane odebrane z kontrolera CAN powinny byc dostepne w BuforRX[]*/        
    /*De-assert CS jest w przerwaniu odbiorczym DMA1*/
    
}

void DRV_CANFDSPI_Reset(void)
{
    uint16_t spiTransferSize = 2;
    
    /*Compose command*/
    BuforTX[0] = (uint8_t) (cINSTRUCTION_RESET << 4);
    BuforTX[1] = 0;

    DRV_SPI_TransferData(spiTransferSize);

    
}

void config_DMA0_SPI1(uint16_t Size){
/*=============================================================================
 Konfiguracja DMA kanal 0 do transmisji SPI w trybie One_Shot (bez powtorzenia)
===============================================================================
 DMA0 configuration
 Direction: Read from DMA RAM and write to SPI buffer
 AMODE: Register Indirect with Post-Increment mode
 MODE: OneShot, Ping-Pong Disabled*/
 
/* Rejestr DMA0CON
 * bit15    -0 chen/chanel --> disable
 * bit14    -1 size --> Byte    
 * bit13    -1 dir --> Read from DMA RAM address write to peripheral address
 * bit12    -0 half --> Initiate block transfer complete interrupt when all of the data has been moved
 * bit11    -0 nullw --> Normal operation
 * bit10-6  -Unimplemented raed as 0
 * bit5-4   -00 amode  --> Register Indirect with Post_Incerement mode
 * bit3-2   -Unimplemented read as 0
 * bit1-0   -01 mode --> OneShot, Ping-Pong modes disabled*/
DMA0CON = 0x6001 ;/*0x6001 - wartosc wynika z ustawienia bitow jak wyzej*/
DMA0CNT = Size - 1;/*ustal ile znaków do przeslania max 1024 bajty*/
/*IRQ Select Register,wskazujemy SPI1*/
DMA0REQ = 0x000A ; /*SPI1*/
/*Peripheral Adress Register*/
DMA0PAD =  (volatile unsigned int)&SPI1BUF ; /*rzutowanie typu i pobranie adresu rejestru SPI1BUF*/
/*wewnetrzna konstrukcja/funkcja kompilatora*/
DMA0STA = __builtin_dmaoffset(BuforTX) ;/*taka jest konstrukcja wskazania na bufor z danymi*/

IFS0bits.DMA0IF = 0 ; /*clear DMA Interrupt Flag */
IEC0bits.DMA0IE = 1 ; /*enable DMA Interrupt */

/*Wazne :kanal DMA moze byc otwarty dopiero po wpisaniu danych do rejestru DMASTA i DMAxCNT*/
DMA0CONbits.CHEN  = 1; /*Canal DMA0 enable*/

  /*po zakonczonym transferze automatycznie kanal DMA zostaje wylaczony, zmienia
   *sie wartosc rejestru DMAxCON na 16-tym bicie z "1" na "0". Aby ponownie odpalic transfer
   *nalezy wlaczyc ten bit DMAxCON.bits.CHEN=1 i odpalic transfer DMA0REQbits.FORCE = 1
   */
}

void config_DMA1_SPI1(uint16_t Size)
{
/*=============================================================================
 Konfiguracja DMA kanal 1 do odbioru SPI w trybie One_Shot (bez powtorzenia)
===============================================================================
 DMA1 configuration
 Direction: Read from SPI buffer and write to DMA RAM
 AMODE: Register Indirect with Post-Increment mode
 MODE: OneShot, Ping-Pong Disabled*/
    
 /* Rejestr DMA1CON
 * bit15    -0 chen/chanel --> disable
 * bit14    -1 size --> Byte    
 * bit13    -0 dir --> Read from Peripheral address, write to DMA RAM address 
 * bit12    -0 half --> Initiate block transfer complete interrupt when all of the data has been moved
 * bit11    -0 nullw --> Normal operation
 * bit10-6  -Unimplemented raed as 0
 * bit5-4   -00 amode  --> Register Indirect with Post_Incerement mode
 * bit3-2   -Unimplemented read as 0
 * bit1-0   -01 mode --> OneShot, Ping-Pong modes disabled*/
    DMA1CON = 0x4001 ;          /*0x4001 - wartosc wynika z ustawienia bitow jak wyzej*/							
	DMA1CNT = Size - 1 ;/*ustal ile znaków do odebrania max 1024 bajty*/					
	DMA1REQ = 0x000A; /*podpinamy peryferium SPI do DMA*/					
	DMA1PAD = (volatile unsigned int) &SPI1BUF; /*rzutowanie typu i pobranie adresu rejestru SPI1BUF*/
	DMA1STA= __builtin_dmaoffset(BuforRX); /*taka jest konstrukcja wskazania na bufor z danymi*/
	IFS0bits.DMA1IF  = 0;			/*Clear DMA interrupt*/
	IEC0bits.DMA1IE  = 1;			/*Enable DMA interrupt*/
	DMA1CONbits.CHEN = 1;			/*Enable DMA Channel*/		
	
}

/*konfiguracja SPI dla Mastera*/
void config_SPI_MASTER(void) {
     
IFS0bits.SPI1IF = 0;                    /*Clear the Interrupt Flag*/
IEC0bits.SPI1IE = 0;                    /*Disable the Interrupt*/
/*Set clock SPI on SCK, 40 MHz / (4*8) = 1,250 MHz*/
SPI1CON1bits.PPRE = 0b10;             /*Set Primary Prescaler 4:1*/
SPI1CON1bits.SPRE = 0b000;            /*Set Secondary Prescaler 8:1*/
SPI1CON1bits.CKE = 1 ;                  /*UWAGA ten tryb jest potrzebny do MCP2517FD*/
SPI1CON1bits.MODE16 = 0;                /*Communication is word-wide (8 bits)*/
SPI1CON1bits.MSTEN = 1;                 /*Master Mode Enabled*/
SPI1STATbits.SPIEN = 1;                 /*Enable SPI Module*/
IFS0bits.SPI1IF = 0;                    /*Clear the Interrupt Flag*/
IEC0bits.SPI1IE = 1;                    /*Enable the Interrupt SPI*/
}

void DRV_CANFDSPI_OscillatorControlSet(CAN_OSC_CTRL ctrl)
{
    REG_OSC osc;
    osc.word = 0;

    osc.bF.PllEnable = ctrl.PllEnable;
    osc.bF.OscDisable = ctrl.OscDisable;
    osc.bF.SCLKDIV = ctrl.SclkDivide;
    osc.bF.CLKODIV = ctrl.ClkOutDivide;

    // Write byte
    DRV_CANFDSPI_WriteByte(cREGADDR_OSC, osc.byte[0]);

    }

void DRV_CANFDSPI_OscillatorControlObjectReset(CAN_OSC_CTRL* ctrl)
{
    REG_OSC osc;
    osc.word = mcp2517ControlResetValues[0];

    ctrl->PllEnable = osc.bF.PllEnable;
    ctrl->OscDisable = osc.bF.OscDisable;
    ctrl->SclkDivide = osc.bF.SCLKDIV;
    ctrl->ClkOutDivide = osc.bF.CLKODIV;
 
}


/*konfiguracja kontrolera CAN*/
void DRV_CANFDSPI_Config(void){
    
  /*reset device*/
   DRV_CANFDSPI_Reset();
  
   /*Oscillator config*/
   CAN_OSC_CTRL oscCtrl ;
   DRV_CANFDSPI_OscillatorControlObjectReset(&oscCtrl);
   oscCtrl.ClkOutDivide = OSC_CLKO_DIV10;
   DRV_CANFDSPI_OscillatorControlSet(oscCtrl);
            
   /*Input/Output configuration : use nINT0 and nINT1*/
   DRV_CANFDSPI_GpioModeConfigure(GPIO_MODE_INT,GPIO_MODE_INT);
   
   /*CAN configuration : disable ISO_CRC, disable TEF, disable TXQ*/
   CAN_CONFIG config ;
   DRV_CANFDSPI_ConfigureObjectReset(&config);
   config.IsoCrcEnable = 0;
   config.StoreInTEF = 0;
   config.TXQEnable = 0;
   DRV_CANFDSPI_Configure(&config);
   
   /*Bit Time configuration : 500K/2M , 80% sample point*/
   DRV_CANFDSPI_BitTimeConfigure(CAN_500K_2M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_20M);
   
  /*FIFO 1: Transmit FIFO; 5 messages, 64 byte maximum payload, low priority*/
    CAN_TX_FIFO_CONFIG txfConfig;
    DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txfConfig);
    txfConfig.FifoSize = 4;
    txfConfig.PayLoadSize = CAN_PLSIZE_64;
    txfConfig.TxPriority = 0;
    DRV_CANFDSPI_TransmitChannelConfigure(CAN_FIFO_CH1, &txfConfig);

    /*FIFO 2: Receive FIFO; 16 messages, 64 byte maximum payload, time stamping enabled*/
    CAN_RX_FIFO_CONFIG rxfConfig;
    rxfConfig.FifoSize = 15;
    rxfConfig.PayLoadSize = CAN_PLSIZE_64;
    rxfConfig.RxTimeStampEnable = 1;
    DRV_CANFDSPI_ReceiveChannelConfigure(CAN_FIFO_CH2, &rxfConfig);
    
    /*Double Check RAM Usage: 2040 Bytes out of a maximum of 2048 Bytes -> OK.*/
    /*Enable ECC*/
    DRV_CANFDSPI_EccEnable();

    /*Initialize RAM*/
    DRV_CANFDSPI_RamInit(0xff);

    /*Configuration Done: Select Normal Mode*/
    DRV_CANFDSPI_OperationModeSelect(CAN_NORMAL_MODE);
    
}

// Section: GPIO

void DRV_CANFDSPI_GpioModeConfigure(GPIO_PIN_MODE gpio0, GPIO_PIN_MODE gpio1)
{
    uint16_t a = 0;

    // Read
    a = cREGADDR_IOCON + 3;
    REG_IOCON iocon;
    iocon.word = 0;

    DRV_CANFDSPI_ReadByte(a, &iocon.byte[3]);
    
    // Modify
    iocon.bF.PinMode0 = gpio0;
    iocon.bF.PinMode1 = gpio1;

    // Write
    DRV_CANFDSPI_WriteByte(a, iocon.byte[3]);
        
}

// Section: Configuration

void DRV_CANFDSPI_Configure(CAN_CONFIG* config)
{
    REG_CiCON ciCon;
    
    ciCon.word = canControlResetValues[cREGADDR_CiCON / 4];

    ciCon.bF.DNetFilterCount = config->DNetFilterCount;
    ciCon.bF.IsoCrcEnable = config->IsoCrcEnable;
    ciCon.bF.ProtocolExceptionEventDisable = config->ProtocolExpectionEventDisable;
    ciCon.bF.WakeUpFilterEnable = config->WakeUpFilterEnable;
    ciCon.bF.WakeUpFilterTime = config->WakeUpFilterTime;
    ciCon.bF.BitRateSwitchDisable = config->BitRateSwitchDisable;
    ciCon.bF.RestrictReTxAttempts = config->RestrictReTxAttempts;
    ciCon.bF.EsiInGatewayMode = config->EsiInGatewayMode;
    ciCon.bF.SystemErrorToListenOnly = config->SystemErrorToListenOnly;
    ciCon.bF.StoreInTEF = config->StoreInTEF;
    ciCon.bF.TXQEnable = config->TXQEnable;
    ciCon.bF.TxBandWidthSharing = config->TxBandWidthSharing;

   DRV_CANFDSPI_WriteWord(cREGADDR_CiCON, ciCon.word);
     
}

void DRV_CANFDSPI_ConfigureObjectReset(CAN_CONFIG* config)
{
    REG_CiCON ciCon;
    ciCon.word = canControlResetValues[cREGADDR_CiCON / 4];

    config->DNetFilterCount = ciCon.bF.DNetFilterCount;
    config->IsoCrcEnable = ciCon.bF.IsoCrcEnable;
    config->ProtocolExpectionEventDisable = ciCon.bF.ProtocolExceptionEventDisable;
    config->WakeUpFilterEnable = ciCon.bF.WakeUpFilterEnable;
    config->WakeUpFilterTime = ciCon.bF.WakeUpFilterTime;
    config->BitRateSwitchDisable = ciCon.bF.BitRateSwitchDisable;
    config->RestrictReTxAttempts = ciCon.bF.RestrictReTxAttempts;
    config->EsiInGatewayMode = ciCon.bF.EsiInGatewayMode;
    config->SystemErrorToListenOnly = ciCon.bF.SystemErrorToListenOnly;
    config->StoreInTEF = ciCon.bF.StoreInTEF;
    config->TXQEnable = ciCon.bF.TXQEnable;
    config->TxBandWidthSharing = ciCon.bF.TxBandWidthSharing;

 }

void DRV_CANFDSPI_BitTimeConfigure(CAN_BITTIME_SETUP bitTime, CAN_SSP_MODE sspMode, CAN_SYSCLK_SPEED clk)
{
    // Decode clk
    switch (clk) {
        case CAN_SYSCLK_40M:
            DRV_CANFDSPI_BitTimeConfigureNominal40MHz(bitTime);
            DRV_CANFDSPI_BitTimeConfigureData40MHz(bitTime, sspMode);
            break;
        case CAN_SYSCLK_20M:
            DRV_CANFDSPI_BitTimeConfigureNominal20MHz(bitTime);
            DRV_CANFDSPI_BitTimeConfigureData20MHz(bitTime, sspMode);
            break;
                 }
  
}

void DRV_CANFDSPI_BitTimeConfigureNominal20MHz(CAN_BITTIME_SETUP bitTime)
{
    REG_CiNBTCFG ciNbtcfg;

    ciNbtcfg.word = canControlResetValues[cREGADDR_CiNBTCFG / 4];

    // Arbitration Bit rate
    switch (bitTime) {
            // All 500K
        case CAN_500K_1M:
        case CAN_500K_2M:
        case CAN_500K_3M:
        case CAN_500K_4M:
        case CAN_500K_5M:
        case CAN_500K_6M7:
        case CAN_500K_8M:
        case CAN_500K_10M:
            ciNbtcfg.bF.BRP = 0;
            ciNbtcfg.bF.TSEG1 = 30;
            ciNbtcfg.bF.TSEG2 = 7;
            ciNbtcfg.bF.SJW = 7;
            break;

            // All 250K
        case CAN_250K_500K:
        case CAN_250K_833K:
        case CAN_250K_1M:
        case CAN_250K_1M5:
        case CAN_250K_2M:
        case CAN_250K_3M:
        case CAN_250K_4M:
            ciNbtcfg.bF.BRP = 0;
            ciNbtcfg.bF.TSEG1 = 62;
            ciNbtcfg.bF.TSEG2 = 15;
            ciNbtcfg.bF.SJW = 15;
            break;

        case CAN_1000K_4M:
        case CAN_1000K_8M:
            ciNbtcfg.bF.BRP = 0;
            ciNbtcfg.bF.TSEG1 = 14;
            ciNbtcfg.bF.TSEG2 = 3;
            ciNbtcfg.bF.SJW = 3;
            break;

        case CAN_125K_500K:
            ciNbtcfg.bF.BRP = 0;
            ciNbtcfg.bF.TSEG1 = 126;
            ciNbtcfg.bF.TSEG2 = 31;
            ciNbtcfg.bF.SJW = 31;
            break;
        
    }

    // Write Bit time registers
    DRV_CANFDSPI_WriteWord(cREGADDR_CiNBTCFG, ciNbtcfg.word);
      
}

void DRV_CANFDSPI_BitTimeConfigureNominal40MHz(CAN_BITTIME_SETUP bitTime)
{
   REG_CiNBTCFG ciNbtcfg;

    ciNbtcfg.word = canControlResetValues[cREGADDR_CiNBTCFG / 4];

    // Arbitration Bit rate
    switch (bitTime) {
            // All 500K
        case CAN_500K_1M:
        case CAN_500K_2M:
        case CAN_500K_3M:
        case CAN_500K_4M:
        case CAN_500K_5M:
        case CAN_500K_6M7:
        case CAN_500K_8M:
        case CAN_500K_10M:
            ciNbtcfg.bF.BRP = 0;
            ciNbtcfg.bF.TSEG1 = 62;
            ciNbtcfg.bF.TSEG2 = 15;
            ciNbtcfg.bF.SJW = 15;
            break;

            // All 250K
        case CAN_250K_500K:
        case CAN_250K_833K:
        case CAN_250K_1M:
        case CAN_250K_1M5:
        case CAN_250K_2M:
        case CAN_250K_3M:
        case CAN_250K_4M:
            ciNbtcfg.bF.BRP = 0;
            ciNbtcfg.bF.TSEG1 = 126;
            ciNbtcfg.bF.TSEG2 = 31;
            ciNbtcfg.bF.SJW = 31;
            break;

        case CAN_1000K_4M:
        case CAN_1000K_8M:
            ciNbtcfg.bF.BRP = 0;
            ciNbtcfg.bF.TSEG1 = 30;
            ciNbtcfg.bF.TSEG2 = 7;
            ciNbtcfg.bF.SJW = 7;
            break;

        case CAN_125K_500K:
            ciNbtcfg.bF.BRP = 0;
            ciNbtcfg.bF.TSEG1 = 254;
            ciNbtcfg.bF.TSEG2 = 63;
            ciNbtcfg.bF.SJW = 63;
            break;
    }

    // Write Bit time registers
    DRV_CANFDSPI_WriteWord(cREGADDR_CiNBTCFG, ciNbtcfg.word);

}

void DRV_CANFDSPI_BitTimeConfigureData20MHz(CAN_BITTIME_SETUP bitTime, CAN_SSP_MODE sspMode)
{
    REG_CiDBTCFG ciDbtcfg;
    REG_CiTDC ciTdc;
    //    sspMode;

    ciDbtcfg.word = canControlResetValues[cREGADDR_CiDBTCFG / 4];
    ciTdc.word = 0;

    // Configure Bit time and sample point
    ciTdc.bF.TDCMode = CAN_SSP_MODE_AUTO;
    uint32_t tdcValue = 0;

    // Data Bit rate and SSP
    switch (bitTime) {
        case CAN_500K_1M:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 14;
            ciDbtcfg.bF.TSEG2 = 3;
            ciDbtcfg.bF.SJW = 3;
            // SSP
            ciTdc.bF.TDCOffset = 15;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_3M:
        case CAN_500K_2M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 6;
            ciDbtcfg.bF.TSEG2 = 1;
            ciDbtcfg.bF.SJW = 1;
            // SSP
            ciTdc.bF.TDCOffset = 7;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_4M:
        case CAN_1000K_4M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 2;
            ciDbtcfg.bF.TSEG2 = 0;
            ciDbtcfg.bF.SJW = 0;
            // SSP
            ciTdc.bF.TDCOffset = 3;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_5M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 1;
            ciDbtcfg.bF.TSEG2 = 0;
            ciDbtcfg.bF.SJW = 0;
            // SSP
            ciTdc.bF.TDCOffset = 2;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_6M7:
        case CAN_500K_8M:
        case CAN_500K_10M:
        case CAN_1000K_8M:
            //qDebug("Data Bitrate not feasible with this clock!");
            break;

        case CAN_250K_500K:
        case CAN_125K_500K:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 30;
            ciDbtcfg.bF.TSEG2 = 7;
            ciDbtcfg.bF.SJW = 7;
            // SSP
            ciTdc.bF.TDCOffset = 31;
            ciTdc.bF.TDCValue = tdcValue;
            ciTdc.bF.TDCMode = CAN_SSP_MODE_OFF;
            break;
        case CAN_250K_833K:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 17;
            ciDbtcfg.bF.TSEG2 = 4;
            ciDbtcfg.bF.SJW = 4;
            // SSP
            ciTdc.bF.TDCOffset = 18;
            ciTdc.bF.TDCValue = tdcValue;
            ciTdc.bF.TDCMode = CAN_SSP_MODE_OFF;
            break;
        case CAN_250K_1M:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 14;
            ciDbtcfg.bF.TSEG2 = 3;
            ciDbtcfg.bF.SJW = 3;
            // SSP
            ciTdc.bF.TDCOffset = 15;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_250K_1M5:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 8;
            ciDbtcfg.bF.TSEG2 = 2;
            ciDbtcfg.bF.SJW = 2;
            // SSP
            ciTdc.bF.TDCOffset = 9;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_250K_2M:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 6;
            ciDbtcfg.bF.TSEG2 = 1;
            ciDbtcfg.bF.SJW = 1;
            // SSP
            ciTdc.bF.TDCOffset = 7;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_250K_3M:
            //qDebug("Data Bitrate not feasible with this clock!");
            break;
        case CAN_250K_4M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 2;
            ciDbtcfg.bF.TSEG2 = 0;
            ciDbtcfg.bF.SJW = 0;
            // SSP
            ciTdc.bF.TDCOffset = 3;
            ciTdc.bF.TDCValue = tdcValue;
            break;
     
    }

    // Write Bit time registers
    DRV_CANFDSPI_WriteWord(cREGADDR_CiDBTCFG, ciDbtcfg.word);
    
    DRV_CANFDSPI_WriteWord(cREGADDR_CiTDC, ciTdc.word);
}

void DRV_CANFDSPI_BitTimeConfigureData40MHz(CAN_BITTIME_SETUP bitTime, CAN_SSP_MODE sspMode)
{
    REG_CiDBTCFG ciDbtcfg;
    REG_CiTDC ciTdc;
    //    sspMode;

    ciDbtcfg.word = canControlResetValues[cREGADDR_CiDBTCFG / 4];
    ciTdc.word = 0;

    // Configure Bit time and sample point
    ciTdc.bF.TDCMode = CAN_SSP_MODE_AUTO;
    uint32_t tdcValue = 0;

    // Data Bit rate and SSP
    switch (bitTime) {
        case CAN_500K_1M:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 30;
            ciDbtcfg.bF.TSEG2 = 7;
            ciDbtcfg.bF.SJW = 7;
            // SSP
            ciTdc.bF.TDCOffset = 31;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_2M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 14;
            ciDbtcfg.bF.TSEG2 = 3;
            ciDbtcfg.bF.SJW = 3;
            // SSP
            ciTdc.bF.TDCOffset = 15;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_3M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 8;
            ciDbtcfg.bF.TSEG2 = 2;
            ciDbtcfg.bF.SJW = 2;
            // SSP
            ciTdc.bF.TDCOffset = 9;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_4M:
        case CAN_1000K_4M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 6;
            ciDbtcfg.bF.TSEG2 = 1;
            ciDbtcfg.bF.SJW = 1;
            // SSP
            ciTdc.bF.TDCOffset = 7;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_5M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 4;
            ciDbtcfg.bF.TSEG2 = 1;
            ciDbtcfg.bF.SJW = 1;
            // SSP
            ciTdc.bF.TDCOffset = 5;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_6M7:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 3;
            ciDbtcfg.bF.TSEG2 = 0;
            ciDbtcfg.bF.SJW = 0;
            // SSP
            ciTdc.bF.TDCOffset = 4;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_8M:
        case CAN_1000K_8M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 2;
            ciDbtcfg.bF.TSEG2 = 0;
            ciDbtcfg.bF.SJW = 0;
            // SSP
            ciTdc.bF.TDCOffset = 3;
            ciTdc.bF.TDCValue = 1;
            break;
        case CAN_500K_10M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 1;
            ciDbtcfg.bF.TSEG2 = 0;
            ciDbtcfg.bF.SJW = 0;
            // SSP
            ciTdc.bF.TDCOffset = 2;
            ciTdc.bF.TDCValue = 0;
            break;

        case CAN_250K_500K:
        case CAN_125K_500K:
            ciDbtcfg.bF.BRP = 1;
            ciDbtcfg.bF.TSEG1 = 30;
            ciDbtcfg.bF.TSEG2 = 7;
            ciDbtcfg.bF.SJW = 7;
            // SSP
            ciTdc.bF.TDCOffset = 31;
            ciTdc.bF.TDCValue = tdcValue;
            ciTdc.bF.TDCMode = CAN_SSP_MODE_OFF;
            break;
        case CAN_250K_833K:
            ciDbtcfg.bF.BRP = 1;
            ciDbtcfg.bF.TSEG1 = 17;
            ciDbtcfg.bF.TSEG2 = 4;
            ciDbtcfg.bF.SJW = 4;
            // SSP
            ciTdc.bF.TDCOffset = 18;
            ciTdc.bF.TDCValue = tdcValue;
            ciTdc.bF.TDCMode = CAN_SSP_MODE_OFF;
            break;
        case CAN_250K_1M:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 30;
            ciDbtcfg.bF.TSEG2 = 7;
            ciDbtcfg.bF.SJW = 7;
            // SSP
            ciTdc.bF.TDCOffset = 31;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_250K_1M5:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 18;
            ciDbtcfg.bF.TSEG2 = 5;
            ciDbtcfg.bF.SJW = 5;
            // SSP
            ciTdc.bF.TDCOffset = 19;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_250K_2M:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 14;
            ciDbtcfg.bF.TSEG2 = 3;
            ciDbtcfg.bF.SJW = 3;
            // SSP
            ciTdc.bF.TDCOffset = 15;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_250K_3M:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 8;
            ciDbtcfg.bF.TSEG2 = 2;
            ciDbtcfg.bF.SJW = 2;
            // SSP
            ciTdc.bF.TDCOffset = 9;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_250K_4M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 6;
            ciDbtcfg.bF.TSEG2 = 1;
            ciDbtcfg.bF.SJW = 1;
            // SSP
            ciTdc.bF.TDCOffset = 7;
            ciTdc.bF.TDCValue = tdcValue;
            break;
    
    }

    // Write Bit time registers
    DRV_CANFDSPI_WriteWord(cREGADDR_CiDBTCFG, ciDbtcfg.word);
   
    DRV_CANFDSPI_WriteWord(cREGADDR_CiTDC, ciTdc.word);
      
    
}

void DRV_CANFDSPI_TransmitChannelConfigureObjectReset(CAN_TX_FIFO_CONFIG* config)
{
    REG_CiFIFOCON ciFifoCon;
    ciFifoCon.word = canControlResetValues[cREGADDR_CiFIFOCON / 4];

    config->RTREnable = ciFifoCon.txBF.RTREnable;
    config->TxPriority = ciFifoCon.txBF.TxPriority;
    config->TxAttempts = ciFifoCon.txBF.TxAttempts;
    config->FifoSize = ciFifoCon.txBF.FifoSize;
    config->PayLoadSize = ciFifoCon.txBF.PayLoadSize;
  }

// Section: CAN Transmit

void DRV_CANFDSPI_TransmitChannelConfigure(CAN_FIFO_CHANNEL channel, CAN_TX_FIFO_CONFIG* config)
{
    uint16_t a = 0;

    // Setup FIFO
    REG_CiFIFOCON ciFifoCon;
    ciFifoCon.word = canControlResetValues[cREGADDR_CiFIFOCON / 4];
    ciFifoCon.txBF.TxEnable = 1;
    ciFifoCon.txBF.FifoSize = config->FifoSize;
    ciFifoCon.txBF.PayLoadSize = config->PayLoadSize;
    ciFifoCon.txBF.TxAttempts = config->TxAttempts;
    ciFifoCon.txBF.TxPriority = config->TxPriority;
    ciFifoCon.txBF.RTREnable = config->RTREnable;

    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);

    DRV_CANFDSPI_WriteWord(a, ciFifoCon.word);
   
}

void DRV_CANFDSPI_ReceiveChannelConfigure(CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_CONFIG* config)
{
    uint16_t a = 0;

    // Setup FIFO
    REG_CiFIFOCON ciFifoCon;
    ciFifoCon.word = canControlResetValues[cREGADDR_CiFIFOCON / 4];

    ciFifoCon.rxBF.TxEnable = 0;
    ciFifoCon.rxBF.FifoSize = config->FifoSize;
    ciFifoCon.rxBF.PayLoadSize = config->PayLoadSize;
    ciFifoCon.rxBF.RxTimeStampEnable = config->RxTimeStampEnable;

    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);

   DRV_CANFDSPI_WriteWord(a, ciFifoCon.word);

  }

void DRV_CANFDSPI_EccEnable(void)
{
    uint8_t d = 0;

    // Read
    DRV_CANFDSPI_ReadByte(cREGADDR_ECCCON, &d);
    
    // Modify
    d |= 0x01;

    // Write
    DRV_CANFDSPI_WriteByte(cREGADDR_ECCCON, d);
       
}

void DRV_CANFDSPI_RamInit(uint8_t d)
{
    uint8_t txd[SPI_DEFAULT_BUFFER_LENGTH];
    uint32_t k;
    
    // Prepare data
    for (k = 0; k < SPI_DEFAULT_BUFFER_LENGTH; k++) {
        txd[k] = d;
    }

    uint16_t a = cRAMADDR_START;

    for (k = 0; k < (cRAM_SIZE / SPI_DEFAULT_BUFFER_LENGTH); k++) {
        DRV_CANFDSPI_WriteByteArray(a, txd, SPI_DEFAULT_BUFFER_LENGTH);
       
        a += SPI_DEFAULT_BUFFER_LENGTH;
    }

}

void DRV_CANFDSPI_OperationModeSelect(CAN_OPERATION_MODE opMode)
{
    uint8_t d = 0;
   
    // Read
    DRV_CANFDSPI_ReadByte(cREGADDR_CiCON + 3, &d);
  
    // Modify
    d &= ~0x07;
    d |= opMode;

    // Write
    DRV_CANFDSPI_WriteByte(cREGADDR_CiCON + 3, d);
    
}

/*=============================================================================
Interrupt Service Routines.
=============================================================================*/

void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
        
    IFS0bits.DMA0IF = 0;                /*Clear the DMA0 Interrupt Flag*/

}

void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void)
{
      /*De-assert CS*/
      SPI_CS_DESELECT();        /*ustaw 1 na pinie CS*/
      
      IFS0bits.DMA1IF = 0;      /*Clear the DMA1 Interrupt Flag*/

} 

void __attribute__((interrupt, no_auto_psv)) _SPI1Interrupt(void)
{
      //LED1_TOG ;
      IFS0bits.SPI1IF = 0;		/*Clear the DMA0 Interrupt Flag*/

}

void SPI_CS_SELECT(void){
    
    /*tu zasterowac pinem CS --> 0*/
    PORTBbits.RB7 = 0 ; //wyjscie RB7 stan niski
}

void SPI_CS_DESELECT(void){
    
    /*tu zasterowac pinem CS --> 1*/
    PORTBbits.RB7 = 1 ; //wyjscie RB7 stan wysoki
}

