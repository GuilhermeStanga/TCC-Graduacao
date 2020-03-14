/*
 * FreeModbus Libary: MSP430 Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.3 2006/11/19 03:57:49 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

//extern void USCI_A0_graceInit(void);

/* ----------------------- Defines ------------------------------------------*/
#define U0_CHAR                 ( 0x10 )        /* Data 0:7-bits / 1:8-bits */

#define DEBUG_PERFORMANCE       ( 1 )

#if DEBUG_PERFORMANCE == 0
#define DEBUG_PIN_RX            ( 0 )
#define DEBUG_PIN_TX            ( 1 )
#define DEBUG_PORT_DIR          ( P1DIR )
#define DEBUG_PORT_OUT          ( P1OUT )
#define DEBUG_INIT( )           \
  do \
  { \
    DEBUG_PORT_DIR |= ( 1 << DEBUG_PIN_RX ) | ( 1 << DEBUG_PIN_TX ); \
    DEBUG_PORT_OUT &= ~( ( 1 << DEBUG_PIN_RX ) | ( 1 << DEBUG_PIN_TX ) ); \
  } while( 0 );
#define DEBUG_TOGGLE_RX( ) DEBUG_PORT_OUT ^= ( 1 << DEBUG_PIN_RX )
#define DEBUG_TOGGLE_TX( ) DEBUG_PORT_OUT ^= ( 1 << DEBUG_PIN_TX )

#else

#define DEBUG_INIT( )
#define DEBUG_TOGGLE_RX( )
#define DEBUG_TOGGLE_TX( )
#endif

/* ----------------------- Static variables ---------------------------------*/
UCHAR           ucGIEWasEnabled = FALSE;
UCHAR           ucCriticalNesting = 0x00;

/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    ENTER_CRITICAL_SECTION(  );
    if( xRxEnable )
    {
    	//IFG2 |= UCA0RXIFG;
    	IE2 |= UCA0RXIE;
    }
    else
    {
        IE2 &= ~UCA0RXIE;
    }
    if( xTxEnable )
    {
        //pino de controle RS485 em nível alto
    	P2OUT |= BIT0;
    	IE2 |= UCA0TXIE;
        IFG2 |= UCA0TXIFG;
    }
    else
    {
        IE2 &= ~UCA0TXIE;
        //pino de controle RS485 em nível baixo
        while (UCA0STAT & UCBUSY);
        P2OUT &=~BIT0;//alteracao 06/04
    }
    EXIT_CRITICAL_SECTION(  );
}

BOOL
xMBPortSerialInit( UCHAR ucPort, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    BOOL            bInitialized = TRUE;
    /* um bug encontrado era o ulBaudRate vir sempre com valor errado, apos mudar a otimizacao de compilacao resolveu */
    USHORT          UxBR = ( USHORT ) ( /*SMCLK*/1000000UL / ulBaudRate ); // o clock foi configurado para 1M em config_clock
    /* initialize Config for the MSP430 USCI_A0 */
    /* Disable USCI */
    UCA0CTL1 |= UCSWRST;
    UCA0CTL0 = UCMODE_0;
    switch ( eParity )
        {
            case MB_PAR_NONE:
                break;
            case MB_PAR_ODD:
                UCA0CTL0 |= UCPEN;          //habilita paridade
                break;
            case MB_PAR_EVEN:
                UCA0CTL0 |= UCPEN + UCPAR;  //habilita paridade e bit par
            break;
        }
        switch ( ucDataBits )
        {
            case 8:
                break;
            case 7:
                UCA0CTL0 |=UC7BIT;
                break;
            default:
                bInitialized = FALSE;
        }
        UCA0CTL1 = UCSSEL1 + UCSWRST;
        /* Configure USART0 Baudrate Registers. */
        UCA0BR0 = ( UxBR & 0xFF );
        UCA0BR1 = ( UxBR >> 8 );
        UCA0MCTL = UCBRS1 + UCBRS2;
        //UC0IE |= UCA0RXIE + UCA0TXIE; // Enable USCI_A0 RX and TX interrupt
        /* Enable USCI */
        UCA0CTL1 &= ~UCSWRST;

        IFG2 &= ~UCA0TXIFG;  // Limpa flag de int. de TX, pois o bit eh setado apos reset
        IE2 |= UCA0TXIE + UCA0RXIE; // Habilitando a geracao de int. para RX e TX

        return bInitialized;

#if 0
    //TODO: as configuracoes de comunicacao serial ficaram chumbadas pois os componentes tem clock distinto, fica esta parte como futura melhoria

    BOOL    bInitialized = TRUE;

    UCA0CTL1 = UCSSEL1 + UCSWRST;  // Fonte clock: SMCLK ~ 4 MHz | Interface em estado de reset
    //UCA0CTL0 |= UCPEN ;	//habilita paridade //alterado 06/04
    //UCA0BR0 = 0xA0;  // Para 9600 bps, conforme Tabela 15-4 do User Guide
    //UCA0BR1 = 0x01;
    UCA0BR0 = 0x68;
    UCA0BR1 = 0x00;
    UCA0MCTL = UCBRS1 + UCBRS2;  // UCBRF = 0; UCBRS = 6

    UCA0CTL1 &= ~UCSWRST; // Coloca a interface no modo normal

    IFG2 &= ~UCA0TXIFG;  // Limpa flag de int. de TX, pois o bit eh setado apos reset

    IE2 |= UCA0TXIE + UCA0RXIE; // Habilitando a geracao de int. para RX e TX
    return bInitialized;
#endif
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
	UCA0TXBUF = ucByte;
	while(!(IFG2 & UCA0TXIFG));
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    if (IFG2 & UCA0RXIFG)
    {
    	*pucByte = UCA0RXBUF;
    	return TRUE;
    }else
    {
    	return FALSE;
    }


}


#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR_HOOK(void)
{
    DEBUG_TOGGLE_RX( );
    pxMBFrameCBByteReceived(  );
}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR_HOOK(void)
{
    DEBUG_TOGGLE_TX( );
    pxMBFrameCBTransmitterEmpty(  );
}



void EnterCriticalSection( void )
{
    if( ucCriticalNesting == 0 )
    {
    	_DINT( );
    }
    ucCriticalNesting++;
}

void ExitCriticalSection( void )
{
	ucCriticalNesting--;
    if( ucCriticalNesting == 0 )
    {
        _EINT(  );
    }
}

