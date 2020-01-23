#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_adc.h"
#include "inc/hw_timer.h"
#include "inc/hw_gpio.h"
#include "inc/hw_udma.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/fpu.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"
#include "utils/uartstdio.h"

#pragma DATA_ALIGN(dmaControlTable, 1024)
uint8_t dmaControlTable[1024];

#define ADC_BUFFER_SIZE 2

static uint16_t g_adcBuf[ADC_BUFFER_SIZE];
static uint32_t g_uDMAErrCount;

bool g_flagAdc0;
bool g_flagAdc1;

void initPeripherals(void)
{
    ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOD);
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOD);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));

    ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOK);
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOK);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));

    ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_ADC0);
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

    ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_ADC1);
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_ADC1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC1));

    ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_UART2);
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_UART2);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_UART2));

    ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_UDMA);
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_UDMA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_UDMA));
}

void config_UART2(uint32_t baudrate, uint32_t cpuClock)
{
    ROM_GPIOPinConfigure(GPIO_PD4_U2RX);
    ROM_GPIOPinConfigure(GPIO_PD5_U2TX);
    ROM_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    ROM_UARTClockSourceSet(UART2_BASE, UART_CLOCK_SYSTEM);

    UARTStdioConfig(2, baudrate, cpuClock);
}

void config_ADC0DMA(void)
{
    ROM_GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    ROM_ADCReferenceSet(ADC0_BASE, ADC_REF_INT);

    ROM_ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_ALWAYS, 0);

    ROM_ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH16);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH17 | ADC_CTL_IE | ADC_CTL_END);

    //(320MHz PLL/2) / 5 = 32MHz = 2MSPS (there's an issue with the documentation, the PLL is divided by 2, hence 320MHz/2)
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, 10);

    //enable oversampling
    ROM_ADCHardwareOversampleConfigure(ADC0_BASE, 64);

    //set the phase delay, so the two ADCs can sample the same signal alternately, the value is experimental
    //and you should check with an logic analyser or oscilloscope to see if they're separated by ~180°
    //this case is 0 for being the reference
    ROM_ADCPhaseDelaySet(ADC0_BASE, ADC_PHASE_0);

    //enable ADC and ADC DMA
    ROM_ADCSequenceEnable(ADC0_BASE, 1);
    ROM_ADCSequenceDMAEnable(ADC0_BASE, 1);

    //configure the uDMA
    ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC1, UDMA_ATTR_ALL);
    ROM_uDMAChannelAttributeEnable(UDMA_CHANNEL_ADC1, UDMA_ATTR_USEBURST);

    ROM_uDMAChannelControlSet(UDMA_CHANNEL_ADC1 | UDMA_PRI_SELECT,
                              UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 |
                              UDMA_ARB_1);
    ROM_uDMAChannelControlSet(UDMA_CHANNEL_ADC1 | UDMA_ALT_SELECT,
                              UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 |
                              UDMA_ARB_1);

    ROM_uDMAChannelTransferSet(UDMA_CHANNEL_ADC1 | UDMA_PRI_SELECT,
                               UDMA_MODE_PINGPONG,
                               (void *)(ADC0_BASE + ADC_O_SSFIFO1),
                               g_adcBuf, ADC_BUFFER_SIZE);
    ROM_uDMAChannelTransferSet(UDMA_CHANNEL_ADC1 | UDMA_ALT_SELECT,
                               UDMA_MODE_PINGPONG,
                               (void *)(ADC0_BASE + ADC_O_SSFIFO1),
                               g_adcBuf, ADC_BUFFER_SIZE);

    ROM_uDMAChannelEnable(UDMA_CHANNEL_ADC1);

    //enable the interrupts (for this case is ADCIntEnableEx instead of ADCIntEnable)
    ROM_ADCIntEnableEx(ADC0_BASE, ADC_INT_DMA_SS1);
    ROM_IntEnable(INT_ADC0SS1);
}

void config_ADC1DMA(void)
{
    ROM_GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    ROM_ADCReferenceSet(ADC1_BASE, ADC_REF_INT);

    ROM_ADCSequenceConfigure(ADC1_BASE, 1, ADC_TRIGGER_ALWAYS, 0);

    ROM_ADCSequenceStepConfigure(ADC1_BASE, 1, 0, ADC_CTL_CH16);
    ROM_ADCSequenceStepConfigure(ADC1_BASE, 1, 1, ADC_CTL_CH17 | ADC_CTL_IE | ADC_CTL_END);

    //(320MHz PLL/2) / 5 = 32MHz = 2MSPS (there's an issue with the documentation, the PLL is divided by 2, hence 320MHz/2)
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, 10);

    //enable oversampling
    ROM_ADCHardwareOversampleConfigure(ADC1_BASE, 64);

    //set the phase delay, so the two ADCs can sample the same signal alternately, the value is experimental
    //and you should check with an logic analyser or oscilloscope to see if they're separated by ~180°
    ROM_ADCPhaseDelaySet(ADC1_BASE, ADC_PHASE_270);

    //enable ADC and DMA
    ROM_ADCSequenceEnable(ADC1_BASE, 1);
    ROM_ADCSequenceDMAEnable(ADC1_BASE, 1);

    //assign channel 25 to ADC1 (since the channel defaults to SSI1TX)
    ROM_uDMAChannelAssign(UDMA_CH25_ADC1_1);

    //configures the uDMA (secondary peripheral assignments)
    ROM_uDMAChannelAttributeDisable(UDMA_SEC_CHANNEL_ADC11, UDMA_ATTR_ALL);
    ROM_uDMAChannelAttributeEnable(UDMA_SEC_CHANNEL_ADC11, UDMA_ATTR_USEBURST);

    ROM_uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC11 | UDMA_PRI_SELECT,
                              UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 |
                              UDMA_ARB_1);
    ROM_uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC11 | UDMA_ALT_SELECT,
                              UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 |
                              UDMA_ARB_1);

    ROM_uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC11 | UDMA_PRI_SELECT,
                               UDMA_MODE_PINGPONG,
                               (void *)(ADC1_BASE + ADC_O_SSFIFO1),
                               g_adcBuf, ADC_BUFFER_SIZE);
    ROM_uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC11 | UDMA_ALT_SELECT,
                               UDMA_MODE_PINGPONG,
                               (void *)(ADC1_BASE + ADC_O_SSFIFO1),
                               g_adcBuf, ADC_BUFFER_SIZE);

    ROM_uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC11);

    //enable the interrupts (for this case is ADCIntEnableEx instead of ADCIntEnable)
    ROM_ADCIntEnableEx(ADC1_BASE, ADC_INT_DMA_SS1);
    ROM_IntEnable(INT_ADC1SS1);
}

int main(void)
{
    //config CPU to run at 120MHz with 320MHz VCO
    uint32_t g_cpuFrequency = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
            SYSCTL_USE_PLL | SYSCTL_CFG_VCO_320), 120000000);
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();

    //disable interrupts to safely configure peripherals
    ROM_IntMasterDisable();

    //enable peripherals
    initPeripherals();

    //configs UART 2 (Launchpad jumpers must be in CAN position, otherwhise you'll need to rewrite this function to UART0)
    config_UART2(921600, g_cpuFrequency);

    //set PA output pins to change state after every interrupt from the ADC
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    //clears screen and reset the cursor
    UARTprintf("\033[2J\033[H");

    //enable udma error interrupt
    ROM_IntEnable(INT_UDMAERR);

    //enable udma
    ROM_uDMAEnable();

    ROM_uDMAControlBaseSet(dmaControlTable);

    //configs ADC
    config_ADC0DMA();
    config_ADC1DMA();

    //now that every periph is safely configured, we can enable interrupts
    ROM_IntMasterEnable();

    for(;;)
    {
        UARTprintf("ADC Reading: %u %u Error Count: %u\n", g_adcBuf[0], g_adcBuf[1], g_uDMAErrCount);

        ROM_SysCtlDelay((g_cpuFrequency/3)*0.1);
    }
}

void ADC0SS1IntHandler(void)
{
    uint32_t ui32Mode;

    //clear ADC interrupt caused by DMA
    ADCIntClearEx(ADC0_BASE, ADC_INT_DMA_SS1);

    //checks udma primary control structure current mode
    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_ADC1 | UDMA_PRI_SELECT);

    //if udma primary control structure is done transfering data, re-enable it
    if(ui32Mode == UDMA_MODE_STOP)
        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_ADC1 | UDMA_PRI_SELECT,
                                   UDMA_MODE_PINGPONG,
                                   (void *)(ADC0_BASE + ADC_O_SSFIFO1),
                                   g_adcBuf, ADC_BUFFER_SIZE);

    //checks udma secondary control structure current mode
    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_ADC1 | UDMA_ALT_SELECT);

    //if udma secondary control structure is done transfering data, re-enable it
    if(ui32Mode == UDMA_MODE_STOP)
        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_ADC1 | UDMA_ALT_SELECT,
                                   UDMA_MODE_PINGPONG,
                                   (void *)(ADC0_BASE + ADC_O_SSFIFO1),
                                   g_adcBuf, ADC_BUFFER_SIZE);

    //toggle pin PA5 to analyse it with an oscilloscope or logic analyser the ADC current speed
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, g_flagAdc0 ? GPIO_PIN_4 : 0x00);
    g_flagAdc0 = !g_flagAdc0;
}

void ADC1SS1IntHandler(void)
{
    uint32_t ui32Mode;

    //clear ADC interrupt caused by DMA
    ADCIntClearEx(ADC1_BASE, ADC_INT_DMA_SS1);

    //checks udma primary control structure current mode
    ui32Mode = ROM_uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC11 | UDMA_PRI_SELECT);

    //if udma primary control structure is done transfering data, re-enable it
    if(ui32Mode == UDMA_MODE_STOP)
        ROM_uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC11 | UDMA_PRI_SELECT,
                                   UDMA_MODE_PINGPONG,
                                   (void *)(ADC1_BASE + ADC_O_SSFIFO1),
                                   g_adcBuf, ADC_BUFFER_SIZE);

    //checks udma secondary control structure current mode
    ui32Mode = ROM_uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC11 | UDMA_ALT_SELECT);

    //if udma secondary control structure is done transfering data, re-enable it
    if(ui32Mode == UDMA_MODE_STOP)
        ROM_uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC11 | UDMA_ALT_SELECT,
                                   UDMA_MODE_PINGPONG,
                                   (void *)(ADC1_BASE + ADC_O_SSFIFO1),
                                   g_adcBuf, ADC_BUFFER_SIZE);

    //toggle pin PA5 to analyse it with an oscilloscope or logic analyser the ADC current speed
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, g_flagAdc1 ? GPIO_PIN_5 : 0x00);
    g_flagAdc1 = !g_flagAdc1;
}

void uDMAErrorHandler(void)
{
    uint32_t ui32Status;

    ui32Status = ROM_uDMAErrorStatusGet();

    if(ui32Status)
    {
        ROM_uDMAErrorStatusClear();
        g_uDMAErrCount++;
    }
}
