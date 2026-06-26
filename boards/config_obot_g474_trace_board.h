#pragma once
#include "../peripheral/usb.h"
#include "../usb_communication.h"
#include "../peripheral/stm32g4/hrpwm.h"
#include "../util.h"
#include "../peripheral/stm32g4/pin_config.h"
#include "../peripheral/stm32g4/uart.h"
#include "../peripheral/protocol.h"
#include "../peripheral/stm32g4/flash.h"
#include "../peripheral/stm32g4/rtc.h"
#include "../driver.h"
#include "../task.h"
#include "../interrupts.h"
#include "param_obot_g474_trace.h"

#include "../communication.h"



const Param * const param = (const Param * const) 0x8060000;
const Calibration * const calibration = (const Calibration * const) 0x8070000;
extern const char * const name = param->name;

using PWM = HRPWM;
#include "../led.h"
#include "../fast_loop.h"
#include "config_trace.h"

using Communication = USBCommunication;
using Driver = DriverBase;

#include "../main_loop.h"
#include "../actuator.h"
#include "../system.h"
#include "pin_config_obot_g474_trace.h"
#include "../peripheral/stm32g4/temp_sensor.h"
#include "../messages.h"

extern "C" void SystemClock_Config();
void pin_config_obot_g474_trace(const BoardRev&);

extern "C" void board_init() {
    const BoardRev board_rev = get_board_rev();
    SystemClock_Config();
    pin_config_obot_g474_trace(board_rev);
}

template <typename TraceConfig>
struct TraceBoard {
    static_assert(((double) CPU_FREQUENCY_HZ * 8 / 2) / pwm_frequency < 65535);    // check pwm frequency
    Driver drv;
    TempSensor temp_sensor;
    Flash flash(*FLASH);

    const BoardRev board_rev = get_board_rev();



    HRPWM motor_pwm = {pwm_frequency, *HRTIM1, 3, 5, 4, false, 50, 1000, 1000};
    USB1 usb;
    FastLoop<pwm_frequency> fast_loop = {motor_pwm, motor_encoder, param->fast_loop_param, *calibration, &I_A_DR, &I_B_DR, &I_C_DR, &V_BUS_DR};


    LED led = {const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(get_board_pins(board_rev).led_tim_r)), 
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(get_board_pins(board_rev).led_tim_g)),
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(get_board_pins(board_rev).led_tim_b)),
               config::main_loop_frequency};
};



using MainLoopConfig = TraceConfig::MainLoopConfig;

constexpr TraceParam<MainLoopConfig> active_param;
using System = SystemBase<Actuator<FastLoop<config::pwm_frequency>, MainLoop<MainLoopConfig, active_param.main_loop>>>;

template<>
Communication System::communication_ = {config::usb};



void usb_interrupt() {
    config::usb.interrupt();
}
namespace config {
    MainLoop<MainLoopConfig, active_param.main_loop> main_loop = {fast_loop, System::communication_, led, output_encoder, torque_sensor, drv, param->main_loop_param, *calibration};
};

template<>
decltype(System::actuator_) System::actuator_ = {config::fast_loop, config::main_loop, param->startup_param, *calibration};

float v3v3 = 3.3;

void config_init();

extern uint32_t _eccmram[];

void system_init() {

    System::api.add_api_variable("3v3", new APIFloat(&v3v3));
    System::api.add_api_variable("Tmicro", new APICallbackFloat([]{ return config::temp_sensor.get_value(); },
        [](float f){ config::temp_sensor.set_value(f); }));

    for (auto regs : std::vector<ADC_TypeDef*>{ADC1, ADC2, ADC3, ADC4, ADC5}) {
        regs->CR = ADC_CR_ADVREGEN;
        ns_delay(20000);
        regs->CR |= ADC_CR_ADCAL;
        while(regs->CR & ADC_CR_ADCAL);
        ns_delay(100);
        regs->CR |= ADC_CR_ADCALDIF;
        regs->CR |= ADC_CR_ADCAL;
        while(regs->CR & ADC_CR_ADCAL);
        ns_delay(100);

        regs->ISR = ADC_ISR_ADRDY;
        regs->CR |= ADC_CR_ADEN;
        while(!(regs->ISR & ADC_ISR_ADRDY));
    }

    ADC1->CR |= ADC_CR_JADSTART;
    while(ADC1->CR & ADC_CR_JADSTART);

    v3v3 =  *((uint16_t *) (0x1FFF75AA)) * 3.0 / V_REF_DR;
    System::log("3v3: " + std::to_string(v3v3));

    ADC1->GCOMP = v3v3*4096;
    ADC1->CFGR2 |= ADC_CFGR2_GCOMP;
    ADC1->CR |= ADC_CR_ADSTART;
    ADC2->CR |= ADC_CR_JADSTART;
    ADC5->CR |= ADC_CR_JADSTART | ADC_CR_ADSTART;
    ADC5->IER |= ADC_IER_JEOSIE;
    ADC4->CR |= ADC_CR_JADSTART | ADC_CR_ADSTART;
    ADC3->CR |= ADC_CR_JADSTART | ADC_CR_ADSTART;

    config_init();

    config::main_loop.init();

//          regs_.sTimerxRegs[ch].TIMxCR |= HRTIM_TIMCR_PREEN | HRTIM_TIMCR_TRSTU | HRTIM_TIMCR_CONT;

    NVIC_SetPriority(HRTIM1_Master_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
    NVIC_EnableIRQ(HRTIM1_Master_IRQn);
    HRTIM1->sMasterRegs.MDIER |= HRTIM_MDIER_MCMP1IE; // interrupt on MCMP1
   
    HRTIM1->sMasterRegs.MCMP1R = 400;
    static_assert(config::main_loop_frequency > CPU_FREQUENCY_HZ/4/65536, "Main loop frequency too low");
    HRTIM1->sMasterRegs.MPER = CPU_FREQUENCY_HZ/4/config::main_loop_frequency;
    HRTIM1->sMasterRegs.MCR = 0 << HRTIM_MCR_SYNC_SRC_Pos | 2 << HRTIM_MCR_SYNC_OUT_Pos | HRTIM_MCR_CONT | HRTIM_MCR_PREEN | HRTIM_MCR_MREPU | 7 << HRTIM_MCR_CK_PSC_Pos; // CPU_FREQUENCY * 32 / 2^7 = 42.5 MHz
    config::usb.connect();

    HRTIM1->sMasterRegs.MCR |= HRTIM_MCR_MCEN + HRTIM_MCR_TACEN + HRTIM_MCR_TDCEN + HRTIM_MCR_TECEN + HRTIM_MCR_TFCEN; // start high res timer, also triggers TIM1
}

float T = 0;

void config_maintenance();
void system_maintenance() {
    round_robin_logger.log_data(USB_ERROR_COUNT_INDEX, config::usb.error_count_);    // maybe latch driver fault until reset
    config_maintenance();
}

Task<> main_maintenance_async(CycleScheduler &sched) {
    while (1) {
        co_await sched.async_delay_us(100'000);
        ADC1->CR |= ADC_CR_JADSTART;
        while(ADC1->CR & ADC_CR_JADSTART) {
            co_await sched.yield();
        }
        T = config::temp_sensor.read();
        round_robin_logger.log_data(MICROCONTROLLER_TEMPERATURE_INDEX, T);
        v3v3 =  *((uint16_t *) (0x1FFF75AA)) * 3.0 * ADC1->GCOMP / 4096.0 / ADC1->JDR2;
        round_robin_logger.log_data(VOLTAGE_3V3_INDEX, v3v3);
        if (T > 100) {
            config::main_loop.status_.error.microcontroller_temperature = 1;
        }
    }
}

void setup_sleep() {
    NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
    NVIC_DisableIRQ(ADC5_IRQn);
    config::drv.disable();
    NVIC_SetPriority(USB_LP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 1));
    NVIC_EnableIRQ(RTC_WKUP_IRQn);
    MASK_SET(RCC->CFGR, RCC_CFGR_SW, 2); // HSE is system clock source
    RTC->SCR = RTC_SCR_CWUTF;
}

void finish_sleep() {
    MASK_SET(RCC->CFGR, RCC_CFGR_SW, 3); // PLL is system clock source
    if (!param->main_loop_param.safe_mode_driver_disable) {
        config::drv.enable();
    }
    NVIC_DisableIRQ(RTC_WKUP_IRQn);
    NVIC_SetPriority(USB_LP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    NVIC_EnableIRQ(ADC5_IRQn);
}

extern "C" {

__attribute__((section (".ccmram"))) void USB_LP_IRQHandler()
{
  CommHandler<usb_interrupt>();
}

__attribute__((section (".ccmram"))) void TIM1_CC_IRQHandler()
{
  SystemLoopHandler<System::system_loop>();
  TIM1->SR = 0;
  asm("dsb");
}

__attribute__((section (".ccmram"))) void ADC5_IRQHandler()
{
  FastLoopHandler<System::fast_loop_interrupt>();
  ADC5->ISR = ADC_ISR_JEOS;
  asm("dsb");
}

__attribute__((section (".ccmram"))) void HRTIM1_Master_IRQHandler()
{
  MainLoopHandler<System::main_loop_interrupt>();
  HRTIM1->sMasterRegs.MICR = HRTIM_MICR_MCMP1;
  asm("dsb");
}

void system_run() {
    System::run();
}
} // extern "C"