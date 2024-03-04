#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_adc.h"
#include "fsl_clock.h"
#include "fsl_power.h"
#include "fsl_swm.h"
#include "fsl_swm_connections.h"
#include "fsl_sctimer.h"
#include "fsl_mrt.h"
#include <LPC824.h>
#include <math.h>
#include "xprintf.h"
#include <stdio.h>
#include <stdlib.h>

#define CORE_CLOCK   30000000U  // Set CPU Core clock frequency (Hz)
#define ADC_CHANNEL1 0U  // Channel 1 will be used in this example.
#define Rate 3300U/4095U
#define ADC_CLOCK_DIVIDER 1U // See Fig 52. ADC clocking in Ref Manual.
#define DelayTime 10U // 10ms delay to sample 100hz.
#define GREEN 27U

void ADC_Configuration(adc_result_info_t * ADCResultStruct1);
void clock_init(void);
void uart_putch (uint8_t character);

void SysTick_DelayTicks(uint32_t n);

volatile uint32_t SystickCounter;

volatile uint32_t mrtClock;				// MRT counter
mrt_config_t mrtConfig;   				// Struct for configuring the MRT
uint32_t mrt_count_val;

uint32_t event1;

volatile int ReferenceSignal;

int main(void) {

  float e;
  float i;
  float u;
  float e_prev = 0.0;
  float i_prev = 0.0;

  float Kp = 15.0;
  float Ki = 60.0;
  float Kd = 0.05;
  
  uint16_t pwmDuty;
  int32_t result;
  float yt;
  uint32_t frequency = 0U;
  adc_result_info_t ADCResultStruct1;

  sctimer_config_t sctimerConfig; // Creating sctimerConfig to configure sctimer

	sctimer_pwm_signal_param_t pwmParam1; // pwmParam struct is used to configure pwm signal 
  
  BOARD_InitPins();
  
  CLOCK_EnableClock(kCLOCK_Uart0);    // Enable UART0 clock
  CLOCK_SetClkDivider(kCLOCK_DivUsartClk, 1U); // Set UART0 clock divider
  BOARD_InitDebugConsole();

  xdev_out(uart_putch); // Set the hardware interface function for xprintf
 
  CLOCK_EnableClock(kCLOCK_Adc);      // Enable ADC clock
  
  POWER_DisablePD(kPDRUNCFG_PD_ADC0); // Power on ADC0
    
  // Hardware calibration is required after each chip reset.
  // See: Sec. 21.3.4 Hardware self-calibration
  frequency = CLOCK_GetFreq(kCLOCK_Irc);

  if (true == ADC_DoSelfCalibration(ADC0, frequency)) {
    PRINTF("ADC Calibration Done.\r\n");
  } else {
    PRINTF("ADC Calibration Failed.\r\n");
  }
  

  ADC_Configuration(&ADCResultStruct1);    // Configure ADC and operation mode.


  CLOCK_EnableClock(kCLOCK_Sct); //Enabling sctimer clock

	int sctimerClock = CORE_CLOCK;
	
	SCTIMER_GetDefaultConfig(&sctimerConfig); // This here reads current configuration of sctimer in order to initialize clock with current config

	SCTIMER_Init(SCT0, &sctimerConfig); // Initializing clock
	
	pwmParam1.output           = kSCTIMER_Out_1; 
	pwmParam1.level            = kSCTIMER_HighTrue;
	pwmParam1.dutyCyclePercent = 20;

	/*pwm parameters are initialized. We will use sctimer out 1. HighTrue means pwm will be manipulated with high duty cycle. duty cycle will manipulate the brightness*/

	SCTIMER_SetupPwm(SCT0,                      
			&pwmParam1,                 
			kSCTIMER_CenterAlignedPwm, 
			100000U,                    
			sctimerClock,              
			&event1); // Setup pwm function is used to get pwm signals

  SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_U); 	

	CLOCK_EnableClock(kCLOCK_Swm);    
	SWM_SetMovablePinSelect(SWM0, kSWM_SCT_OUT1, GREEN);

  CLOCK_DisableClock(kCLOCK_Swm); 

  	CLOCK_EnableClock(kCLOCK_Mrt); 
	MRT_GetDefaultConfig(&mrtConfig);
	MRT_Init(MRT0, &mrtConfig); 
	MRT_SetupChannelMode(MRT0, kMRT_Channel_0, kMRT_RepeatMode);
	mrtClock = CLOCK_GetFreq(kCLOCK_CoreSysClk);
	mrt_count_val = mrtClock * 2;
	MRT_StartTimer(MRT0, kMRT_Channel_0, mrt_count_val);
  	MRT_EnableInterrupts(MRT0, kMRT_Channel_0, kMRT_TimerInterruptEnable);
	EnableIRQ(MRT0_IRQn);

  if (SysTick_Config(SystemCoreClock / 1000U)){
    while (1) { } // This would be an error condition.
  }

  PRINTF("Configuration Done.\r\n\n");

  while (1) {

    SysTick_DelayTicks(DelayTime);
    ADC_DoSoftwareTriggerConvSeqA(ADC0); // Start conversion.
    // Keep polling the ADC to check if the conversion is complete:
    while (!ADC_GetChannelConversionResult(ADC0, ADC_CHANNEL1, &ADCResultStruct1))
      {  }


    yt = ((float) (((int16_t) ADCResultStruct1.result ) - 2047 )) * (2.0/2048);

    e = ReferenceSignal - yt;
    i = i_prev + (Ki*0.01/2) * (e + e_prev);

    
    if (i > 25)
    {
      i = 25;
    }
    else if (i < -25)
    {
      i = -25;
    }
    
    u = Kp*e + (Kd/0.01) * (e - e_prev) + i; 
    
    e_prev = e;
    i_prev = i;

    result = (int16_t)(yt*100);

    if (u > 25){
      u = 25.0;
    }
    else if (u < -25){
      u = -25.0;
    }


    pwmDuty = (uint16_t)((((u)*150/25)+150 )/3);  // Mapping (-25, 25) to (0, 100)

    //xprintf("%d,%d, %d, %d\n",ReferenceSignal, (int16_t)(yt), (int16_t)(u), pwmDuty);
	xprintf("%d",ReferenceSignal);

    SCTIMER_UpdatePwmDutycycle(SCT0, kSCTIMER_Out_1, pwmDuty, event1);
  }  

} 


// Configure and initialize the ADC
void ADC_Configuration(adc_result_info_t * ADCResultStruct1) {

  adc_config_t adcConfigStruct;
  adc_conv_seq_config_t adcConvSeqConfigStruct;
  
  adcConfigStruct.clockDividerNumber = ADC_CLOCK_DIVIDER; // Defined above.
  adcConfigStruct.enableLowPowerMode = false;
  // See Sec. 21.6.11 A/D trim register (voltage mode):
  adcConfigStruct.voltageRange = kADC_HighVoltageRange;
  
  ADC_Init(ADC0, &adcConfigStruct); // Initialize ADC0 with this structure.
  
  // Insert this channel in Sequence A, and set conversion properties:
  // See Sec: 21.6.2 A/D Conversion Sequence A Control Register
  adcConvSeqConfigStruct.channelMask = (1U << ADC_CHANNEL1); 

  adcConvSeqConfigStruct.triggerMask      = 0U;
  adcConvSeqConfigStruct.triggerPolarity  = kADC_TriggerPolarityPositiveEdge;
  adcConvSeqConfigStruct.enableSingleStep = false;
  adcConvSeqConfigStruct.enableSyncBypass = false;
  adcConvSeqConfigStruct.interruptMode    = kADC_InterruptForEachSequence;
  
  // Initialize the ADC0 with the sequence defined above:
  ADC_SetConvSeqAConfig(ADC0, &adcConvSeqConfigStruct);
  
  ADC_EnableConvSeqA(ADC0, true); // Enable the conversion sequence A.
  
  // Make the first ADC conversion so that
  //  the result register has a sensible initial value.
  
  ADC_DoSoftwareTriggerConvSeqA(ADC0);
  while (!ADC_GetChannelConversionResult(ADC0, ADC_CHANNEL1, ADCResultStruct1)){}
  
}

void clock_init(void) {    // Set up the clock source

  // Set up IRC
  POWER_DisablePD(kPDRUNCFG_PD_IRC_OUT);        // Turn ON IRC OUT
  POWER_DisablePD(kPDRUNCFG_PD_IRC);            // Turn ON IRC
  //POWER_DisablePD(kPDRUNCFG_PD_SYSOSC);       // In Alakart SYSOSC is not used.
  CLOCK_Select(kSYSPLL_From_Irc);               // Connect IRC to PLL input.
  clock_sys_pll_t config;
  config.src = kCLOCK_SysPllSrcIrc;             // Select PLL source as IRC. 
  config.targetFreq = CORE_CLOCK*2;             // set pll target freq
  CLOCK_InitSystemPll(&config);                 // set parameters
  CLOCK_SetMainClkSrc(kCLOCK_MainClkSrcSysPll); // Select PLL as main clock source.
  CLOCK_Select(kCLKOUT_From_MainClk);               // select IRC for CLKOUT
  CLOCK_SetCoreSysClkDiv(2U);
  SystemCoreClockUpdate ();
}

void SysTick_Handler(void){
    if (SystickCounter != 0U) {

      SystickCounter--;
    }
}


void SysTick_DelayTicks(uint32_t n){
    SystickCounter = n;
    while (SystickCounter != 0U) {  }
}

void uart_putch (uint8_t character){
  // Check if transmission has ended. See: 13.6.3 USART Status register:
  //  while ((USART0_STAT& 0b0100)==0);
  while ((USART0->STAT& 0b0100)==0);
  //  USART0_TXDAT=character;
  USART0->TXDAT=character;

}

void MRT0_IRQHandler(void){
	
  	ReferenceSignal = (ReferenceSignal == 0)*1 + (ReferenceSignal == 1)*0;
	MRT_ClearStatusFlags(MRT0, kMRT_Channel_0, kMRT_TimerInterruptFlag);		
}
