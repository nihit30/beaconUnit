

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include <strings.h>

#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define ORANGE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) // on-board blue LED

#define GREEN_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
//pbs
#define PUSHBUTTON1  (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) //
#define PUSHBUTTON2  (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4))) //
#define PUSHBUTTON3  (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4))) //
#define CHECKSUM 63002
#define DATALEN 11
bool dataReceived = false;

uint8_t seq_count=0;
bool syncDetected = false;
uint8_t data_len;
uint32_t sum;
uint8_t buffer[20];
uint8_t data[12];
uint8_t id;
uint8_t data_pos = 0;
uint32_t time_constant = 300000;
uint32_t delta_t;
bool data_r = false;
//char sync_seq[6]={0x41,0x42,0x43,0x44,0x45};
volatile uint8_t sync_seq[7] = {0x41,0x42,0x43,0x44,0xC0,0x00};
volatile static uint8_t i = 0;
volatile uint8_t msgLength = 0;
volatile uint8_t msgIndex = 0;
uint32_t x1, y1, x2, y2, x3, y3;

uint16_t checkSum = 0;

void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A to F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOC;




    // Configure port F
    GPIO_PORTF_DIR_R = 0x04;  // bits 2 is outputs for blue led
    GPIO_PORTF_DR2R_R = 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x04;  // enable bit 2

    // Configure port F
    GPIO_PORTA_DIR_R = 0x04;  // bits 2 is outputs for blue led
    GPIO_PORTA_DR2R_R = 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R = 0x04;  // enable bit 2



    //port E for pushbuttons
    //Configure Port C for push buttons
    GPIO_PORTE_DIR_R = 0x00;
    GPIO_PORTE_DEN_R = 0x0E;  // enable pushbuttons
    GPIO_PORTE_PUR_R = 0x0E;  // enable internal pull-up for push button

   // GPIO_PORTC_DIR_R |= 0x80;  // bits 2 is outputs for blue led
    //GPIO_PORTD _DR2R_R = 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    // Configure UART1 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1; // turn-on UART0, leave other uarts in same status
    GPIO_PORTC_DEN_R |= 0x30;                         // default, added for clarity
    GPIO_PORTC_AFSEL_R |= 0x30;                       // default, added for clarity
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;

    //uart1
    UART1_CTL_R = 0;                 // turn-off UART0 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                 // use system clock (40 MHz)
    UART1_IBRD_R = 2083; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART1_FBRD_R = 21;                               // round(fract(r)*64)=45
    UART1_LCRH_R = UART_LCRH_WLEN_8; // configure for 8N1 w/o FIFO
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
    UART1_IM_R = UART_IM_RXIM;                       // turn-on RX interrupt
    NVIC_EN0_R |= 1 << (INT_UART1-16);               // turn-on interrupt 21 (UART0)


    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                         // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                       // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                 // use system clock (40 MHz)
    UART0_IBRD_R = 2083; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 21;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
   // NVIC_EN0_R |= 1 << (INT_UART0 - 16);         // turn-on interrupt 21 (UART0)




     SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
     GPIO_PORTB_DEN_R |= 0xC0;
     GPIO_PORTB_AFSEL_R |= 0xC0;
     GPIO_PORTB_PCTL_R = GPIO_PCTL_PB6_M0PWM0;

     PWM0_0_CTL_R = 0;
     PWM0_0_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO|PWM_0_GENA_ACTLOAD_M;
     PWM0_0_LOAD_R = 0x674;
     PWM0_0_CMPA_R = 0x33A;
     PWM0_0_CTL_R |= 1;
     PWM0_ENABLE_R |= PWM_ENABLE_PWM0EN;
}

void waitMicrosecond(uint32_t us)
{
                                                // Approx clocks per us
  __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
  __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
  __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
  __asm("             NOP");                  // 5
  __asm("             B    WMS_LOOP1");       // 5*3
  __asm("WMS_DONE1:   SUB  R0, #1");          // 1
  __asm("             CBZ  R0, WMS_DONE0");   // 1
  __asm("             B    WMS_LOOP0");       // 1*3
  __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
    putcUart0(str[i]);
}
char getcUart0()
{
     while (UART0_FR_R & UART_FR_RXFE);
     return UART0_DR_R & 0xFF;
}

char getcUart1()
{
     while (UART1_FR_R & UART_FR_RXFE);
     return UART1_DR_R & 0xFF;
}

void sumWords(void* data, uint16_t size_in_bytes)
{
    uint8_t* pData = (uint8_t*)data;
    uint16_t i;
    uint8_t phase = 1;

    uint16_t data_temp;
    for (i = 0; i < size_in_bytes; i++)
    {
        if (phase)
        {
            data_temp = *pData;
            sum += data_temp << 8;
        }
        else
          sum += *pData;
        phase = 1 - phase;
        pData++;
    }
}

// Completes 1's compliment addition by folding carries back uint8_to field
uint16_t getChecksum()
{
    uint16_t result;
    // this is based on rfc1071
    while ((sum >> 16) > 0)
        sum = (sum & 0xFFFF) + (sum >> 16);
    result = sum & 0xFFFF;
    return ~result;
}


char in;
static uint8_t syncIndex=0;
volatile uint8_t inbuf[10];
bool clearInbuf = false;
uint8_t packetReceived[20];
uint8_t indx;

void Uart1Isr()
{

    in = UART1_DR_R;

    if (in == sync_seq[syncIndex])
    {
        syncIndex++;
        if (syncIndex == 5)
        {
            msgLength = getcUart1();
            for (indx = 0; indx <= msgLength; indx++)
            {
                packetReceived[indx] = getcUart1();
            }
            ORANGE_LED ^= 1;
            syncDetected = true;
        }

     }

    else
    {
        syncIndex = 0;
    }
}

// for id
void readPbs()
{
    id = (PUSHBUTTON1<<2)|(PUSHBUTTON2<<1)|(PUSHBUTTON3);
}



void SumWords(void* Sdata, uint16_t size_in_bytes)
{
    uint8_t* pData = (uint8_t*)Sdata;
    uint16_t i;
    uint8_t phase = 1;
    uint16_t data_temp;
    for (i = 0; i < size_in_bytes; i++)
    {
        if (phase)
        {
            data_temp = *pData;
            sum += data_temp << 8;
        }
        else
          sum += *pData;
        phase = 1 - phase;
        pData++;
    }
}

void decodeCoordinates()
{
    x1 = ((((data[0] & 0xFF)-1) << 8) + ((data[1] & 0xFF)-1));

    y1 = ((((data[2] & 0xFF)-1) << 8) + ((data[3] & 0xFF)-1));

    x2 = ((((data[4] & 0xFF)-1) << 8) + ((data[5] & 0xFF)-1));

    y2 = ((((data[6] & 0xFF)-1) << 8) + ((data[7] & 0xFF)-1));

    x3 = ((((data[8] & 0xFF)-1) << 8) + ((data[9] & 0xFF)-1));

    y3 = ((((data[10] & 0xFF)-1) << 8) + ((data[11] & 0xFF)-1));
}
uint8_t incoming;
uint8_t coordinates[15];

void initIncomingData(void* packet)
{
    uint8_t k;
    for(k = 0; k < (msgLength - 1); k++)
    {
        data[k] = packetReceived[k];
    }


}


int main(void)
 {

    initHw();
    BLUE_LED = 1;
    waitMicrosecond(5000);
    BLUE_LED = 0;




    while(1)
    {

        if(syncDetected)
        {
            syncDetected = false;
            UART1_IM_R &= ~UART_IM_RXIM;
            initIncomingData(packetReceived);
            SumWords(&data, DATALEN);
            checkSum = getChecksum();
            if(checkSum == CHECKSUM)
            {
                checkSum = 0;
                sum = 0;
                decodeCoordinates();
                BLUE_LED ^= 1;
            }
            UART1_IM_R |= UART_IM_RXIM;

        }

     }

}
