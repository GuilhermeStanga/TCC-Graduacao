/*
 * Implementação do aquisitor de dados e protocolo Modbus RTU
 * através da biblioteca FreeModbus, efetuando comunicação com o ScadaBR.
 * Este código contém funções e arquivos da biblioteca FreeModbus, cuja documentaçao pode ser
 * encontrada no site: <http://www.freemodbus.org/> e <http://www.freemodbus.org/api/index.html>.
 *
 *0° - 0.1V - 0
 *50°- 3.6V - 1023
 *
 */
//p2.0 pra controle do max485
//OBS: Deixar um led para indicar que esta ligado p2.3
//p2.4 para dht11
//p1.0 led indicar recebimento

//adc[1] - p1.6 - B1
//adc[0] - p1.7 - B0

#include <msp430.h>
#include "config.h"
/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

int habilitaLeitura = 1;

#if SENSOR_TH
#include "DHT11_LIB/DHT11_LIB.h"

unsigned char RH_byte1;
unsigned char RH_byte2;
unsigned char T_byte1;
unsigned char T_byte2;
unsigned char checksum;
unsigned char Packet[5];
unsigned char volatile TOUT;			//REQUIRED for library

//Tratamento de interrupcoes
#pragma vector = TIMER1_A0_VECTOR
__interrupt void CCR0_ISR(void){
		TOUT=1;
		CLR (TA1CCTL0, CCIFG);
}

void config_timer_A1(void)
{
	/*fonte de clock: SMCLK
	 * fator divisão: 4
	 * modo up
	 */
	TA1CCR0 = 50000; // inicializa timer para contar em 5Hz
	TA1CTL = TASSEL_2 + ID_2 + MC_1 + TACLR;
}

#endif

/* ----------------------- Definições de Registradores------------------------------------------*/
#if SENSOR_TH
#define REG_HOLDING_START 0    //No ScadaBR, é o campo offset.
#define REG_HOLDING_NREGS 5    //No ScadaBR, é o campo número de registradores para teste de leitura.
#else
#define REG_HOLDING_START 0
#define REG_HOLDING_NREGS 3
#endif
/* ----------------------- Variáveis Estáticas ---------------------------------*/
static USHORT   usRegHoldingStart = REG_HOLDING_START;
static USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];

//Definição de funções de inicialização
void config_clock(void);		//Configura Clock
void config_portas(void);		//Configura entradas e saídas
void ini_ADC10(void);			//configuracoes do ad
void ler_adc(void);
unsigned int adc[7] = {0}; // vetor de amostras do ad
//adc2->bit5
//adc1->bit6
//adc0->bit7

/* ----------------------- Função Principal ---------------------------------*/

void main(void)
{
	volatile eMBErrorCode    eStatus;
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	config_clock();
	config_portas();
	ini_ADC10();
	#if SENSOR_TH	//se este escravo nao possuir o DHT11, as funcoes do mesmo nao sao compiladas
	config_timer_A1();
	#endif

	if( ( eStatus = eMBInit( MB_RTU, (UCHAR)SLAVEADDRESS, (UCHAR)PORT, (ULONG)BAUDRATE, (eMBParity)PARIDADE ) ) != MB_ENOERR )
	{
	}
	/* Habilita o protocolo Modbus RTU */
	else if( ( eStatus = eMBEnable(  ) ) != MB_ENOERR )
	{
	}
	else
	{
	    _enable_interrupt();             //Habilita interrupcoes
		for(;;)
		{
			( void )eMBPoll(  );
			habilitaLeitura = 1; //variavel para controle de execucao
		}
	}
}

/* ----------------------- Implementação das Funções ---------------------------------*/

//Função de Registrador Holding (03)- Modbus RTU
eMBErrorCode eMBRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
	eMBErrorCode eStatus = MB_ENOERR;
	int iRegIndex;
	if( ( usAddress >= REG_HOLDING_START ) && ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS + 1 ) )
	{
		iRegIndex = ( int )( usAddress - usRegHoldingStart )-1;
		switch ( eMode )
		{
		//Se for de leitura:
		case MB_REG_READ: //se for para leitura de registradores
			while( usNRegs > 0 )
			{
			    if(habilitaLeitura) //para executar somente uma vez,
			    {
                    P1OUT &= ~(BIT0); //para indicar inicio da leitura de sensores
                    #if  SENSOR_TH
                    TA1CCTL0 = CCIE;
                    read_Packet(Packet);
                    RH_byte1 =	Packet[0];
                    RH_byte2 =	Packet[1];
                    T_byte1 =	Packet[2];
                    T_byte2 =	Packet[3];
                    checksum =	Packet[4];
                    if (check_Checksum(Packet))
                        SET (P1OUT, 0x40);
                    SET (TA1CTL, TACLR);
                    SET (TA1CTL, 0x10);
                    TA1CCR0 = 50000;
                    CLR(TA1CCTL0,CCIE);//desabilitar as interrupcoes
                    usRegHoldingBuf[2] = RH_byte1;
                    usRegHoldingBuf[3] = T_byte1;
                    #endif
                    ler_adc();
                    usRegHoldingBuf[0] = adc[0] * 0.0489;
                    usRegHoldingBuf[1] = adc[1] * 0.0489;
                    P1OUT |= (BIT0); //para indicar termino do tratamento de sensores
                                    //pisca led
                    habilitaLeitura = 0;
				}
				*pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
				*pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
				iRegIndex++;
				usNRegs--;
			}
			break;
		case MB_REG_WRITE:
			while( usNRegs > 0 )
			{
				usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
				usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
				iRegIndex++;
				usNRegs--;
			}
		}
	}
	else
	{
		eStatus = MB_ENOREG;
	}
	return eStatus;
}

//Configuracao do clock
void config_clock(void)
{
	WDTCTL =  WDTPW + WDTHOLD;	// Para o WDT
	DCOCTL =  CALDCO_1MHZ;   // DCO com freq. calibrada de 1 MHz
	BCSCTL1 = CALBC1_1MHZ;
	//Se modificar o valor do clock, tambem alterar a funcao do port xMBPortSerialInit o valor para divisao
}
//Configuracao das portas
void config_portas(void)
{
	P1DIR = BIT0 + BIT1 + BIT2 +BIT3 + BIT4 + BIT5 + BIT6 + BIT7;
	P1SEL = BIT1 + BIT2;   // Conecta pinos 3 e 4 nos sinais UCA0TXD/UCA0RXD
	P1SEL2 = BIT1 + BIT2;
	P1IFG = 0; // limpa flag de interrupcao da P1.
	P2DIR |= BIT0+BIT1+BIT2+BIT3+BIT4+BIT5+BIT6+BIT7;   //toda a porta P2 como saida
	P2OUT = 0;
}

//Configuracao do ad
void ini_ADC10(void){
/* clock: MCLK
 * realiza amostragem a partir de canal 7
 * sequencia de canais sem repeticao
 * interrupcao por software
 */
	ADC10CTL1 =  INCH_7 + CONSEQ_1 + ADC10SSEL_2 + SHS_0;
	ADC10CTL0 = ADC10SHT_2  + MSC+ ADC10ON + ADC10IE;
	ADC10DTC1 = 8; //tamanho do bloco a ser transferido
	ADC10AE0 = BIT6 + BIT7;   // Habilita os canais para ad
	ADC10CTL0 |= ENC +ADC10SC ;  // Habilita convesao (gera borda de subida)
}
//Leitura do ad
void ler_adc(void)
{
    ADC10CTL0 &= ~ENC; //desabilita conversao
    while (ADC10CTL1 & BUSY); // espera se o ad esta ocupado
    ADC10SA = (unsigned int)adc; //define um vetor para copiar os dados da conversao
    ADC10CTL0 |= ENC + ADC10SC;// habilita amostragem e conversao
}
//Tratamento de interrupcoes do ADC
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
	ADC10CTL0 &= ~ADC10IFG;//limpa flag de interrupcao
	ADC10SA = (short)&adc[0];//vetor para enviar a conversao
  __bic_SR_register_on_exit(CPUOFF);// Clear CPUOFF bit from 0(SR)
}
