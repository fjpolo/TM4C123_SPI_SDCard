/**
 ******************************************************************************
 * @file      startup_tm4c.c
 * @author    Coocox
 * @version   V1.0
 * @date      12/23/2009
 * @brief     tm4c Devices Startup code.
 *            This module performs:
 *                - Set the initial SP
 *                - Set the vector table entries with the exceptions ISR address
 *                - Initialize data and bss
 *                - Setup the microcontroller system.
 *                - Call the application's entry point.
 *            After Reset the Cortex-M4F processor is in Thread mode,
 *            priority is Privileged, and the Stack is set to Main.
 *******************************************************************************
 */

#include <stdint.h>
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"


/*----------Stack Configuration-----------------------------------------------*/
#define STACK_SIZE       0x00000200      /*!< The Stack size suggest using even number     */
__attribute__ ((section(".co_stack")))
unsigned long pulStack[STACK_SIZE];


/*----------Macro definition--------------------------------------------------*/
#define WEAK __attribute__ ((weak))


/*----------Declaration of the default fault handlers-------------------------*/
/* System exception vector handler */
__attribute__ ((used))
void WEAK  Reset_Handler(void);
void WEAK  NMI_Handler(void);
void WEAK  HardFault_Handler(void);
void WEAK  MemManage_Handler(void);
void WEAK  BusFault_Handler(void);
void WEAK  UsageFault_Handler(void);
void WEAK  SVC_Handler(void);
void WEAK  DebugMon_Handler(void);
void WEAK  PendSV_Handler(void);
void WEAK  SysTick_Handler(void);
void WEAK  GPIOPortA_IRQHandler(void);
void WEAK  GPIOPortB_IRQHandler(void);
void WEAK  GPIOPortC_IRQHandler(void);
void WEAK  GPIOPortD_IRQHandler(void);
void WEAK  GPIOPortE_IRQHandler(void);
void WEAK  UART0_IRQHandler(void);
void WEAK  UART1_IRQHandler(void);
void WEAK  SSI0_IRQHandler(void);
void WEAK  I2C0_IRQHandler(void);
void WEAK  PWMFault_IRQHandler(void);
void WEAK  PWMGen0_IRQHandler(void);
void WEAK  PWMGen1_IRQHandler(void);
void WEAK  PWMGen2_IRQHandler(void);
void WEAK  QEI0_IRQHandler(void);
void WEAK  ADCSeq0_IRQHandler(void);
void WEAK  ADCSeq1_IRQHandler(void);
void WEAK  ADCSeq2_IRQHandler(void);
void WEAK  ADCSeq3_IRQHandler(void);
void WEAK  Watchdog_IRQHandler(void);
void WEAK  Timer0A_IRQHandler(void);
void WEAK  Timer0B_IRQHandler(void);
void WEAK  Timer1A_IRQHandler(void);
void WEAK  Timer1B_IRQHandler(void);
void WEAK  Timer2A_IRQHandler(void);
void WEAK  Timer2B_IRQHandler(void);
void WEAK  Comp0_IRQHandler(void);
void WEAK  Comp1_IRQHandler(void);
void WEAK  Comp2_IRQHandler(void);
void WEAK  SysCtrl_IRQHandler(void);
void WEAK  FlashCtrl_IRQHandler(void);
void WEAK  GPIOPortF_IRQHandler(void);
void WEAK  GPIOPortG_IRQHandler(void);
void WEAK  GPIOPortH_IRQHandler(void);
void WEAK  UART2_IRQHandler(void);
void WEAK  SSI1_IRQHandler(void);
void WEAK  Timer3A_IRQHandler(void);
void WEAK  Timer3B_IRQHandler(void);
void WEAK  I2C1_IRQHandler(void);
void WEAK  QEI1_IRQHandler(void);
void WEAK  CAN0_IRQHandler(void);
void WEAK  CAN1_IRQHandler(void);
void WEAK  CAN2_IRQHandler(void);
void WEAK  Hibernate_IRQHandler(void);
void WEAK  USB0_IRQHandler(void);
void WEAK  PWMGen3_IRQHandler(void);
void WEAK  uDMA_IRQHandler(void);
void WEAK  uDMAErr_IRQHandler(void);
void WEAK  ADC1Seq0_IRQHandler(void);
void WEAK  ADC1Seq1_IRQHandler(void);
void WEAK  ADC1Seq2_IRQHandler(void);
void WEAK  ADC1Seq3_IRQHandler(void);
void WEAK  GPIOPortJ_IRQHandler(void);
void WEAK  GPIOPortK_IRQHandler(void);
void WEAK  GPIOPortL_IRQHandler(void);
void WEAK  SSI2_IRQHandler(void);
void WEAK  SSI3_IRQHandler(void);
void WEAK  UART3_IRQHandler(void);
void WEAK  UART4_IRQHandler(void);
void WEAK  UART5_IRQHandler(void);
void WEAK  UART6_IRQHandler(void);
void WEAK  UART7_IRQHandler(void);
void WEAK  I2C2_IRQHandler(void);
void WEAK  I2C3_IRQHandler(void);
void WEAK  Timer4A_IRQHandler(void);
void WEAK  Timer4B_IRQHandler(void);
void WEAK  Timer5A_IRQHandler(void);
void WEAK  Timer5B_IRQHandler(void);
void WEAK  WTimer0A_IRQHandler(void);
void WEAK  WTimer0B_IRQHandler(void);
void WEAK  WTimer1A_IRQHandler(void);
void WEAK  WTimer1B_IRQHandler(void);
void WEAK  WTimer2A_IRQHandler(void);
void WEAK  WTimer2B_IRQHandler(void);
void WEAK  WTimer3A_IRQHandler(void);
void WEAK  WTimer3B_IRQHandler(void);
void WEAK  WTimer4A_IRQHandler(void);
void WEAK  WTimer4B_IRQHandler(void);
void WEAK  WTimer5A_IRQHandler(void);
void WEAK  WTimer5B_IRQHandler(void);
void WEAK  FPU_IRQHandler(void);
void WEAK  PECI0_IRQHandler(void);
void WEAK  LPC0_IRQHandler(void);
void WEAK  I2C4_IRQHandler(void);
void WEAK  I2C5_IRQHandler(void);
void WEAK  GPIOPortM_IRQHandler(void);
void WEAK  GPIOPortN_IRQHandler(void);
void WEAK  QEI2_IRQHandler(void);
void WEAK  Fan0_IRQHandler(void);
void WEAK  GPIOPortP0_IRQHandler(void);
void WEAK  GPIOPortP1_IRQHandler(void);
void WEAK  GPIOPortP2_IRQHandler(void);
void WEAK  GPIOPortP3_IRQHandler(void);
void WEAK  GPIOPortP4_IRQHandler(void);
void WEAK  GPIOPortP5_IRQHandler(void);
void WEAK  GPIOPortP6_IRQHandler(void);
void WEAK  GPIOPortP7_IRQHandler(void);
void WEAK  GPIOPortQ0_IRQHandler(void);
void WEAK  GPIOPortQ1_IRQHandler(void);
void WEAK  GPIOPortQ2_IRQHandler(void);
void WEAK  GPIOPortQ3_IRQHandler(void);
void WEAK  GPIOPortQ4_IRQHandler(void);
void WEAK  GPIOPortQ5_IRQHandler(void);
void WEAK  GPIOPortQ6_IRQHandler(void);
void WEAK  GPIOPortQ7_IRQHandler(void);
void WEAK  GPIOPortR_IRQHandler(void);
void WEAK  GPIOPortS_IRQHandler(void);
void WEAK  PWM1Gen0_IRQHandler(void);
void WEAK  PWM1Gen1_IRQHandler(void);
void WEAK  PWM1Gen2_IRQHandler(void);
void WEAK  PWM1Gen3_IRQHandler(void);
void WEAK  PWM1Fault_IRQHandler(void);


/*----------Symbols defined in linker script----------------------------------*/
extern unsigned long _sidata;    /*!< Start address for the initialization
                                      values of the .data section.            */
extern unsigned long _sdata;     /*!< Start address for the .data section     */
extern unsigned long _edata;     /*!< End address for the .data section       */
extern unsigned long _sbss;      /*!< Start address for the .bss section      */
extern unsigned long _ebss;      /*!< End address for the .bss section        */
extern void _eram;               /*!< End address for ram                     */


/*----------Function prototypes-----------------------------------------------*/
extern int main(void);           /*!< The entry point for the application.    */
extern void SystemInit(void);    /*!< Setup the microcontroller system(CMSIS) */
void Default_Reset_Handler(void);   /*!< Default reset handler                */
static void Default_Handler(void);  /*!< Default exception handler            */


/**
  *@brief The minimal vector table for a Cortex M4F.  Note that the proper constructs
  *       must be placed on this to ensure that it ends up at physical address
  *       0x00000000.
  */
__attribute__ ((used,section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{
  /*----------Core Exceptions------------------------------------------------ */
  (void *)&pulStack[STACK_SIZE],       /*!< The initial stack pointer         */
  Reset_Handler,                       /*!< The reset handler                 */
  NMI_Handler,                         /*!< The NMI handler                   */
  HardFault_Handler,                   /*!< The hard fault handler            */
  MemManage_Handler,                   /*!< The MPU fault handler             */
  BusFault_Handler,                    /*!< The bus fault handler             */
  UsageFault_Handler,                  /*!< The usage fault handler           */
  0,0,0,0,                             /*!< Reserved                          */
  SVC_Handler,                         /*!< SVCall handler                    */
  DebugMon_Handler,                    /*!< Debug monitor handler             */
  0,                                   /*!< Reserved                          */
  PendSV_Handler,                      /*!< The PendSV handler                */
  SysTick_Handler,                     /*!< The SysTick handler               */

  /*----------External Exceptions---------------------------------------------*/
  GPIOPortA_IRQHandler,                /*!<   0: GPIO Port A                   */
  GPIOPortB_IRQHandler,                /*!<   1: GPIO Port B                   */
  GPIOPortC_IRQHandler,                /*!<   2: GPIO Port C                   */
  GPIOPortD_IRQHandler,                /*!<   3: GPIO Port D                   */
  GPIOPortE_IRQHandler,                /*!<   4: GPIO Port E                   */
  UART0_IRQHandler,                    /*!<   5: UART0 Rx and Tx               */
  UART1_IRQHandler,                    /*!<   6: UART1 Rx and Tx               */
  SSI0_IRQHandler,                     /*!<   7: SSI0 Rx and Tx                */
  I2C0_IRQHandler,                     /*!<   8: I2C0 Master and Slave         */
  PWMFault_IRQHandler,                 /*!<   9: PWM Fault                     */
  PWMGen0_IRQHandler,                  /*!<  10: PWM Generator 0               */
  PWMGen1_IRQHandler,                  /*!<  11: PWM Generator 1               */
  PWMGen2_IRQHandler,                  /*!<  12: PWM Generator 2               */
  QEI0_IRQHandler,                     /*!<  13: Quadrature Encoder 0          */
  ADCSeq0_IRQHandler,                  /*!<  14: ADC Sequence 0                */
  ADCSeq1_IRQHandler,                  /*!<  15: ADC Sequence 1                */
  ADCSeq2_IRQHandler,                  /*!<  16: ADC Sequence 2                */
  ADCSeq3_IRQHandler,                  /*!<  17: ADC Sequence 3                */
  Watchdog_IRQHandler,                 /*!<  18: Watchdog timer                */
  Timer0A_IRQHandler,                  /*!<  19: Timer 0 subtimer A            */
  Timer0B_IRQHandler,                  /*!<  20: Timer 0 subtimer B            */
  Timer1A_IRQHandler,                  /*!<  21: Timer 1 subtimer A            */
  Timer1B_IRQHandler,                  /*!<  22: Timer 1 subtimer B            */
  Timer2A_IRQHandler,                  /*!<  23: Timer 2 subtimer A            */
  Timer2B_IRQHandler,                  /*!<  24: Timer 2 subtimer B            */
  Comp0_IRQHandler,                    /*!<  25: Analog Comparator 0           */
  Comp1_IRQHandler,                    /*!<  26: Analog Comparator 1           */
  Comp2_IRQHandler,                    /*!<  27: Analog Comparator 2           */
  SysCtrl_IRQHandler,                  /*!<  28: System Control (PLL, OSC, BO) */
  FlashCtrl_IRQHandler,                /*!<  29: FLASH Control                 */
  GPIOPortF_IRQHandler,                /*!<  30: GPIO Port F                   */
  GPIOPortG_IRQHandler,                /*!<  31: GPIO Port G                   */
  GPIOPortH_IRQHandler,                /*!<  32: GPIO Port H                   */
  UART2_IRQHandler,                    /*!<  33: UART2 Rx and Tx               */
  SSI1_IRQHandler,                     /*!<  34: SSI1 Rx and Tx                */
  Timer3A_IRQHandler,                  /*!<  35: Timer 3 subtimer A            */
  Timer3B_IRQHandler,                  /*!<  36: Timer 3 subtimer B            */
  I2C1_IRQHandler,                     /*!<  37: I2C1 Master and Slave         */
  QEI1_IRQHandler,                     /*!<  38: Quadrature Encoder 1          */
  CAN0_IRQHandler,                     /*!<  39: CAN0                          */
  CAN1_IRQHandler,                     /*!<  40: CAN1                          */
  CAN2_IRQHandler,                     /*!<  41: CAN2                          */
  0,                                   /*!<  42: Reserved                      */
  Hibernate_IRQHandler,                /*!<  43: Hibernate                     */
  USB0_IRQHandler,                     /*!<  44: USB0                          */
  PWMGen3_IRQHandler,                  /*!<  45: PWM Generator 3               */
  uDMA_IRQHandler,                     /*!<  46: uDMA Software Transfer        */
  uDMAErr_IRQHandler,                  /*!<  47: uDMA Error                    */
  ADC1Seq0_IRQHandler,                 /*!<  48: ADC1 Sequence 0               */
  ADC1Seq1_IRQHandler,                 /*!<  49: ADC1 Sequence 1               */
  ADC1Seq2_IRQHandler,                 /*!<  50: ADC1 Sequence 2               */
  ADC1Seq3_IRQHandler,                 /*!<  51: ADC1 Sequence 3               */
  0,0,                                 /*!<  52-53: Reserved                   */
  GPIOPortJ_IRQHandler,                /*!<  54: GPIO Port J                   */
  GPIOPortK_IRQHandler,                /*!<  55: GPIO Port K                   */
  GPIOPortL_IRQHandler,                /*!<  56: GPIO Port L                   */
  SSI2_IRQHandler,                     /*!<  57: SSI2 Rx and Tx                */
  SSI3_IRQHandler,                     /*!<  58: SSI3 Rx and Tx                */
  UART3_IRQHandler,                    /*!<  59: UART3 Rx and Tx               */
  UART4_IRQHandler,                    /*!<  60: UART4 Rx and Tx               */
  UART5_IRQHandler,                    /*!<  61: UART5 Rx and Tx               */
  UART6_IRQHandler,                    /*!<  62: UART6 Rx and Tx               */
  UART7_IRQHandler,                    /*!<  63: UART7 Rx and Tx               */
  0,0,0,0,                             /*!<  64-67: Reserved                   */
  I2C2_IRQHandler,                     /*!<  68: I2C2 Master and Slave         */
  I2C3_IRQHandler,                     /*!<  69: I2C3 Master and Slave         */
  Timer4A_IRQHandler,                  /*!<  70: Timer 4 subtimer A            */
  Timer4B_IRQHandler,                  /*!<  71: Timer 4 subtimer B            */
  0,0,0,0,0,0,0,0,0,0,                 /*!<  72-81: Reserved                   */
  0,0,0,0,0,0,0,0,0,0,                 /*!<  82-91: Reserved                   */
  Timer5A_IRQHandler,                  /*!<  92: Timer 5 subtimer A            */
  Timer5B_IRQHandler,                  /*!<  93: Timer 5 subtimer B            */
  WTimer0A_IRQHandler,                 /*!<  94: Wide Timer 0 subtimer A       */
  WTimer0B_IRQHandler,                 /*!<  95: Wide Timer 0 subtimer B       */
  WTimer1A_IRQHandler,                 /*!<  96: Wide Timer 1 subtimer A       */
  WTimer1B_IRQHandler,                 /*!<  97: Wide Timer 1 subtimer B       */
  WTimer2A_IRQHandler,                 /*!<  98: Wide Timer 2 subtimer A       */
  WTimer2B_IRQHandler,                 /*!<  99: Wide Timer 2 subtimer B       */
  WTimer3A_IRQHandler,                 /*!< 100: Wide Timer 3 subtimer A       */
  WTimer3B_IRQHandler,                 /*!< 101: Wide Timer 3 subtimer B       */
  WTimer4A_IRQHandler,                 /*!< 102: Wide Timer 4 subtimer A       */
  WTimer4B_IRQHandler,                 /*!< 103: Wide Timer 4 subtimer B       */
  WTimer5A_IRQHandler,                 /*!< 104: Wide Timer 5 subtimer A       */
  WTimer5B_IRQHandler,                 /*!< 105: Wide Timer 5 subtimer B       */
  FPU_IRQHandler,                      /*!< 106: FPU                           */
  PECI0_IRQHandler,                    /*!< 107: PECI 0                        */
  LPC0_IRQHandler,                     /*!< 108: LPC 0                         */
  I2C4_IRQHandler,                     /*!< 109: I2C4 Master and Slave         */
  I2C5_IRQHandler,                     /*!< 110: I2C5 Master and Slave         */
  GPIOPortM_IRQHandler,                /*!< 111: GPIO Port M                   */
  GPIOPortN_IRQHandler,                /*!< 112: GPIO Port N                   */
  QEI2_IRQHandler,                     /*!< 113: Quadrature Encoder 1          */
  Fan0_IRQHandler,                     /*!< 114: Fan 0                         */
  0,                                   /*!< 115: Reserved                      */
  GPIOPortP0_IRQHandler,               /*!< 116: GPIO Port P0                  */
  GPIOPortP1_IRQHandler,               /*!< 117: GPIO Port P1                  */
  GPIOPortP2_IRQHandler,               /*!< 118: GPIO Port P2                  */
  GPIOPortP3_IRQHandler,               /*!< 119: GPIO Port P3                  */
  GPIOPortP4_IRQHandler,               /*!< 120: GPIO Port P4                  */
  GPIOPortP5_IRQHandler,               /*!< 117: GPIO Port P5                  */
  GPIOPortP6_IRQHandler,               /*!< 118: GPIO Port P6                  */
  GPIOPortP7_IRQHandler,               /*!< 119: GPIO Port P7                  */
  GPIOPortQ0_IRQHandler,               /*!< 120: GPIO Port Q0                  */
  GPIOPortQ1_IRQHandler,               /*!< 121: GPIO Port Q1                  */
  GPIOPortQ2_IRQHandler,               /*!< 122: GPIO Port Q2                  */
  GPIOPortQ3_IRQHandler,               /*!< 123: GPIO Port Q3                  */
  GPIOPortQ4_IRQHandler,               /*!< 124: GPIO Port Q4                  */
  GPIOPortQ5_IRQHandler,               /*!< 125: GPIO Port Q5                  */
  GPIOPortQ6_IRQHandler,               /*!< 126: GPIO Port Q6                  */
  GPIOPortQ7_IRQHandler,               /*!< 127: GPIO Port Q7                  */
  GPIOPortR_IRQHandler,                /*!< 127: GPIO Port R                   */
  GPIOPortS_IRQHandler,                /*!< 127: GPIO Port S                   */
  PWM1Gen0_IRQHandler,                 /*!< 128: PWM 1 Generator 0             */
  PWM1Gen1_IRQHandler,                 /*!< 129: PWM 1 Generator 1             */
  PWM1Gen2_IRQHandler,                 /*!< 130: PWM 1 Generator 2             */
  PWM1Gen3_IRQHandler,                 /*!< 131: PWM 1 Generator 3             */
  PWM1Fault_IRQHandler,                /*!< 132: PWM 1 Fault                   */
};


/**
  * @brief  This is the code that gets called when the processor first
  *         starts execution following a reset event. Only the absolutely
  *         necessary set is performed, after which the application
  *         supplied main() routine is called.
  * @param  None
  * @retval None
  */
void Default_Reset_Handler(void)
{
  /* Initialize data and bss */
  unsigned long *pulSrc, *pulDest;

  /* Copy the data segment initializers from flash to SRAM */
  pulSrc = &_sidata;

  for(pulDest = &_sdata; pulDest < &_edata; )
  {
    *(pulDest++) = *(pulSrc++);
  }

  /* Zero fill the bss segment.  This is done with inline assembly since this
     will clear the value of pulDest if it is not kept in a register. */
  __asm("  ldr     r0, =_sbss\n"
        "  ldr     r1, =_ebss\n"
        "  mov     r2, #0\n"
        "  .thumb_func\n"
        "zero_loop:\n"
        "    cmp     r0, r1\n"
        "    it      lt\n"
        "    strlt   r2, [r0], #4\n"
        "    blt     zero_loop");

  /* Enable the floating-point unit.  This must be done here to handle the
     case where main() uses floating-point and the function prologue saves
     floating-point registers (which will fault if floating-point is not
     enabled).  Any configuration of the floating-point unit using DriverLib
     APIs must be done here prior to the floating-point unit being enabled.

     Note that this does not use DriverLib since it might not be included in
     this project. */
  HWREG(NVIC_CPAC) = ((HWREG(NVIC_CPAC) &
                       ~(NVIC_CPAC_CP10_M | NVIC_CPAC_CP11_M)) |
                      NVIC_CPAC_CP10_FULL | NVIC_CPAC_CP11_FULL);


  /* Call the application's entry point.*/
  main();
}


/**
  *@brief Provide weak aliases for each Exception handler to the Default_Handler.
  *       As they are weak aliases, any function with the same name will override
  *       this definition.
  */

#pragma weak Reset_Handler = Default_Reset_Handler
#pragma weak NMI_Handler = Default_Handler
#pragma weak HardFault_Handler = Default_Handler
#pragma weak MemManage_Handler = Default_Handler
#pragma weak BusFault_Handler = Default_Handler
#pragma weak UsageFault_Handler = Default_Handler
#pragma weak SVC_Handler = Default_Handler
#pragma weak DebugMon_Handler = Default_Handler
#pragma weak PendSV_Handler = Default_Handler
#pragma weak SysTick_Handler = Default_Handler
#pragma weak GPIOPortA_IRQHandler = Default_Handler
#pragma weak GPIOPortB_IRQHandler = Default_Handler
#pragma weak GPIOPortC_IRQHandler = Default_Handler
#pragma weak GPIOPortD_IRQHandler = Default_Handler
#pragma weak GPIOPortE_IRQHandler = Default_Handler
#pragma weak UART0_IRQHandler = Default_Handler
#pragma weak UART1_IRQHandler = Default_Handler
#pragma weak SSI0_IRQHandler = Default_Handler
#pragma weak I2C0_IRQHandler = Default_Handler
#pragma weak PWMFault_IRQHandler = Default_Handler
#pragma weak PWMGen0_IRQHandler = Default_Handler
#pragma weak PWMGen1_IRQHandler = Default_Handler
#pragma weak PWMGen2_IRQHandler = Default_Handler
#pragma weak QEI0_IRQHandler = Default_Handler
#pragma weak ADCSeq0_IRQHandler = Default_Handler
#pragma weak ADCSeq1_IRQHandler = Default_Handler
#pragma weak ADCSeq2_IRQHandler = Default_Handler
#pragma weak ADCSeq3_IRQHandler = Default_Handler
#pragma weak Watchdog_IRQHandler = Default_Handler
#pragma weak Timer0A_IRQHandler = Default_Handler
#pragma weak Timer0B_IRQHandler = Default_Handler
#pragma weak Timer1A_IRQHandler = Default_Handler
#pragma weak Timer1B_IRQHandler = Default_Handler
#pragma weak Timer2A_IRQHandler = Default_Handler
#pragma weak Timer2B_IRQHandler = Default_Handler
#pragma weak Comp0_IRQHandler = Default_Handler
#pragma weak Comp1_IRQHandler = Default_Handler
#pragma weak Comp2_IRQHandler = Default_Handler
#pragma weak SysCtrl_IRQHandler = Default_Handler
#pragma weak FlashCtrl_IRQHandler = Default_Handler
#pragma weak GPIOPortF_IRQHandler = Default_Handler
#pragma weak GPIOPortG_IRQHandler = Default_Handler
#pragma weak GPIOPortH_IRQHandler = Default_Handler
#pragma weak UART2_IRQHandler = Default_Handler
#pragma weak SSI1_IRQHandler = Default_Handler
#pragma weak Timer3A_IRQHandler = Default_Handler
#pragma weak Timer3B_IRQHandler = Default_Handler
#pragma weak I2C1_IRQHandler = Default_Handler
#pragma weak QEI1_IRQHandler = Default_Handler
#pragma weak CAN0_IRQHandler = Default_Handler
#pragma weak CAN1_IRQHandler = Default_Handler
#pragma weak CAN2_IRQHandler = Default_Handler
#pragma weak Hibernate_IRQHandler = Default_Handler
#pragma weak USB0_IRQHandler = Default_Handler
#pragma weak PWMGen3_IRQHandler = Default_Handler
#pragma weak uDMA_IRQHandler = Default_Handler
#pragma weak uDMAErr_IRQHandler = Default_Handler
#pragma weak  ADC1Seq0_IRQHandler = Default_Handler
#pragma weak  ADC1Seq1_IRQHandler = Default_Handler
#pragma weak  ADC1Seq2_IRQHandler = Default_Handler
#pragma weak  ADC1Seq3_IRQHandler = Default_Handler
#pragma weak  GPIOPortJ_IRQHandler = Default_Handler
#pragma weak  GPIOPortK_IRQHandler = Default_Handler
#pragma weak  GPIOPortL_IRQHandler = Default_Handler
#pragma weak  SSI2_IRQHandler = Default_Handler
#pragma weak  SSI3_IRQHandler = Default_Handler
#pragma weak  UART3_IRQHandler = Default_Handler
#pragma weak  UART4_IRQHandler = Default_Handler
#pragma weak  UART5_IRQHandler = Default_Handler
#pragma weak  UART6_IRQHandler = Default_Handler
#pragma weak  UART7_IRQHandler = Default_Handler
#pragma weak  I2C2_IRQHandler = Default_Handler
#pragma weak  I2C3_IRQHandler = Default_Handler
#pragma weak  Timer4A_IRQHandler = Default_Handler
#pragma weak  Timer4B_IRQHandler = Default_Handler
#pragma weak  Timer5A_IRQHandler = Default_Handler
#pragma weak  Timer5B_IRQHandler = Default_Handler
#pragma weak  WTimer0A_IRQHandler = Default_Handler
#pragma weak  WTimer0B_IRQHandler = Default_Handler
#pragma weak  WTimer1A_IRQHandler = Default_Handler
#pragma weak  WTimer1B_IRQHandler = Default_Handler
#pragma weak  WTimer2A_IRQHandler = Default_Handler
#pragma weak  WTimer2B_IRQHandler = Default_Handler
#pragma weak  WTimer3A_IRQHandler = Default_Handler
#pragma weak  WTimer3B_IRQHandler = Default_Handler
#pragma weak  WTimer4A_IRQHandler = Default_Handler
#pragma weak  WTimer4B_IRQHandler = Default_Handler
#pragma weak  WTimer5A_IRQHandler = Default_Handler
#pragma weak  WTimer5B_IRQHandler = Default_Handler
#pragma weak  FPU_IRQHandler = Default_Handler
#pragma weak  PECI0_IRQHandler = Default_Handler
#pragma weak  LPC0_IRQHandler = Default_Handler
#pragma weak  I2C4_IRQHandler = Default_Handler
#pragma weak  I2C5_IRQHandler = Default_Handler
#pragma weak  GPIOPortM_IRQHandler = Default_Handler
#pragma weak  GPIOPortN_IRQHandler = Default_Handler
#pragma weak  QEI2_IRQHandler = Default_Handler
#pragma weak  Fan0_IRQHandler = Default_Handler
#pragma weak  GPIOPortP0_IRQHandler = Default_Handler
#pragma weak  GPIOPortP1_IRQHandler = Default_Handler
#pragma weak  GPIOPortP2_IRQHandler = Default_Handler
#pragma weak  GPIOPortP3_IRQHandler = Default_Handler
#pragma weak  GPIOPortP4_IRQHandler = Default_Handler
#pragma weak  GPIOPortP5_IRQHandler = Default_Handler
#pragma weak  GPIOPortP6_IRQHandler = Default_Handler
#pragma weak  GPIOPortP7_IRQHandler = Default_Handler
#pragma weak  GPIOPortQ0_IRQHandler = Default_Handler
#pragma weak  GPIOPortQ1_IRQHandler = Default_Handler
#pragma weak  GPIOPortQ2_IRQHandler = Default_Handler
#pragma weak  GPIOPortQ3_IRQHandler = Default_Handler
#pragma weak  GPIOPortQ4_IRQHandler = Default_Handler
#pragma weak  GPIOPortQ5_IRQHandler = Default_Handler
#pragma weak  GPIOPortQ6_IRQHandler = Default_Handler
#pragma weak  GPIOPortQ7_IRQHandler = Default_Handler
#pragma weak  GPIOPortR_IRQHandler = Default_Handler
#pragma weak  GPIOPortS_IRQHandler = Default_Handler
#pragma weak  PWM1Gen0_IRQHandler = Default_Handler
#pragma weak  PWM1Gen1_IRQHandler = Default_Handler
#pragma weak  PWM1Gen2_IRQHandler = Default_Handler
#pragma weak  PWM1Gen3_IRQHandler = Default_Handler
#pragma weak  PWM1Fault_IRQHandler = Default_Handler


/**
  * @brief  This is the code that gets called when the processor receives an
  *         unexpected interrupt.  This simply enters an infinite loop,
  *         preserving the system state for examination by a debugger.
  * @param  None
  * @retval None
  */
static void Default_Handler(void)
{
  /* Go into an infinite loop. */
  while (1)
  {
  }
}

/*********************** (C) COPYRIGHT 2009 Coocox ************END OF FILE*****/
