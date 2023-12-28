#include "MKL46Z4.h"
#include "board.h"
#include "pin_mux.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"

#define GREEN_LED_PIN (1U << 5)
#define RED_LED_PIN (1U << 29)
#define SW1_PIN (1U << 3)
#define SW2_PIN (1U << 12)

volatile int32_t msTicks = 0; // Interval counter in ms
volatile int32_t sw2Pressed = 0; // Counter for SW2 presses
volatile int32_t sw1Pressed = 0;
void InitLed() {
      SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    PORTE->PCR[29] = PORT_PCR_MUX(1u);
    PTE->PDDR |= RED_LED_PIN;
    PTE->PSOR |= RED_LED_PIN;

    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    PORTD->PCR[5] = PORT_PCR_MUX(1u);
    PTD->PDDR |= GREEN_LED_PIN;
    PTD->PSOR |= GREEN_LED_PIN;
}

void InitSwitches() {
	    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

   PORTC->PCR[3] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA);
    PTC->PDDR &= ~SW1_PIN;

    PORTC->PCR[12] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA);
    PTC->PDDR &= ~SW2_PIN;

    NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}

void PORTC_PORTD_IRQHandler(void) {
    if (!(PTC->PDIR & SW1_PIN)) {
        sw1Pressed++;
        sw1Pressed = sw1Pressed % 2;
        msTicks = 0;
     
        PORTC->PCR[3] |= PORT_PCR_ISF_MASK;
    }
    if (!(PTC->PDIR & SW2_PIN)) {
        sw2Pressed++;
        sw2Pressed = sw2Pressed % 2;
        msTicks = 0;
        PORTC->PCR[12] |= PORT_PCR_ISF_MASK;
    }
}


void init_systick_interrupt() {
   SysTick->LOAD = SystemCoreClock / 1000; //configured the SysTick to count in 1ms/* Select Core Clock & Enable SysTick & Enable Interrupt */
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |SysTick_CTRL_TICKINT_Msk |SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void) { // SysTick interrupt Handler
    msTicks++; // Increment counter
}

void Delay(uint32_t TICK) {
    uint32_t startTicks = msTicks; 
    while ((msTicks - startTicks) < TICK);
}

int main(void) {
	BOARD_InitPins();
		BOARD_BootClockRUN();
		BOARD_InitDebugConsole();
			
			SegLCD_Init();
		
    SystemInit();
    InitLed();
    InitSwitches();
    init_systick_interrupt();
		SegLCD_DisplayDecimal(0000);
    while (1) {
        if( (sw1Pressed == 1) && (sw2Pressed == 0) && (msTicks < 4000))
				{
					  PTD->PCOR |= GREEN_LED_PIN; // Turn on green LED
						SegLCD_DisplayDecimal(0001);
				}
				else if( (sw1Pressed == 1) && (sw2Pressed == 0) && (msTicks > 4000))
				{
					            PTD->PSOR |= GREEN_LED_PIN; // Turn off green LED
           PTE->PCOR |= RED_LED_PIN; // Turn on red LED
					SegLCD_DisplayDecimal(1111);
				} else if ((sw1Pressed == 1) && (sw2Pressed == 1 ))
				{
					 PTD->PCOR |= GREEN_LED_PIN; // Turn on green LED
            PTE->PSOR |= RED_LED_PIN; // Turn off red LED
					SegLCD_DisplayDecimal(0001);
				}else  
				{
					PTE->PSOR |= RED_LED_PIN; 
					 PTD->PSOR |= GREEN_LED_PIN; 
						SegLCD_DisplayDecimal(0000);
				}
			}
		}