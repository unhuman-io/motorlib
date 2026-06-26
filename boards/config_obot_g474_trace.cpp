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

#ifdef SCOPE_DEBUG
#define SET_SCOPE_PIN(X,x) GPIO##X->BSRR = 1 << x
#define CLEAR_SCOPE_PIN(X,x) GPIO##X->BSRR = 1 << (16 + x)
#define TOGGLE_SCOPE_PIN(X,x) GPIO##X->ODR ^= 1 << x
#else
#define SET_SCOPE_PIN(X,x)
#define CLEAR_SCOPE_PIN(X,x)
#define TOGGLE_SCOPE_PIN(X,x)
#endif

#include "../communication.h"

#define COMMS_USB   1
#define COMMS_CAN   2

#ifndef COMMS
  #error "COMMS should be defined"
#endif

#if (COMMS != COMMS_USB) && (COMMS != COMMS_CAN)
  #error "Invalid COMMS value"
#endif

const Param * const param = (const Param * const) 0x8060000;
const Calibration * const calibration = (const Calibration * const) 0x8070000;
extern const char * const name = param->name;
namespace config {
    const uint32_t system_loop_frequency =  1000;
};

using PWM = HRPWM;

#if COMMS == COMMS_USB
    using Communication = USBCommunication;
#endif

#if (COMMS == COMMS_CAN)
    #include "../communication/can_communication.h"
    #include "../peripheral/stm32g4/can.h"
    using Communication = CANCommunication<CAN>;
#endif

using Driver = DriverBase;


#ifndef GPIO_OUT
#define GPIO_OUT (reinterpret_cast<volatile gpio_bits*>(&GPIOA->ODR)->bit1)
#endif

#ifndef GPIO_IN
#define GPIO_IN ((GPIOA->IDR & (1 << 2)) ? 1 : 0)
#endif

#include "../led.h"
#include "../fast_loop.h"
#include "../main_loop.h"
#include "../actuator.h"
#include "../system.h"
#include "pin_config_obot_g474_trace.h"
#include "../peripheral/stm32g4/temp_sensor.h"
#include "../temperature_sensor.h"
#include "../peripheral/stm32g4/i2c_dma.h"
#include "../peripheral/stm32g4/spi_dma.h"
#include "../peripheral/stm32g4/max31875.h"
#include "../peripheral/stm32g4/max31889.h"
#include "../mb85rc64.h"
#include "../messages.h"

extern "C" void SystemClock_Config();
void pin_config_obot_g474_trace(const BoardRev&);

extern "C" void board_init() {
    const BoardRev board_rev = get_board_rev();
    SystemClock_Config();
    pin_config_obot_g474_trace(board_rev);
#ifdef SCOPE_DEBUG
    GPIO_SETL(C, 0, GPIO_MODE::OUTPUT, GPIO_SPEED::HIGH, 0); // main loop scope
    GPIO_SETL(C, 1, GPIO_MODE::OUTPUT, GPIO_SPEED::HIGH, 0); // fast loop scope
    GPIO_SETL(C, 2, GPIO_MODE::OUTPUT, GPIO_SPEED::HIGH, 0); // usb int scope
    GPIO_SETL(C, 4, GPIO_MODE::OUTPUT, GPIO_SPEED::HIGH, 0); // main() scope
    GPIO_SETL(A, 0, GPIO_MODE::OUTPUT, GPIO_SPEED::HIGH, 0); // system loop scope
#endif
#if COMMS == COMMS_CAN
    RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
    GPIO_SETL(B, 5, GPIO_MODE::ALT_FUN, GPIO_SPEED::MEDIUM, 9); // can2 rx
    GPIO_SETL(B, 6, GPIO_MODE::ALT_FUN, GPIO_SPEED::MEDIUM, 9); // can2 tx
#endif
}

namespace config {
    static_assert(((double) CPU_FREQUENCY_HZ * 8 / 2) / pwm_frequency < 65535);    // check pwm frequency
    Driver drv;
    TempSensor temp_sensor;
    Flash flash(*FLASH);

    const BoardRev board_rev = get_board_rev();


#if COMMS == COMMS_CAN
    CAN can(CAN::CAN2);
#endif


    HRPWM motor_pwm = {pwm_frequency, *HRTIM1, 3, 5, 4, false, 50, 1000, 1000};
    USB1 usb;
    FastLoop<pwm_frequency> fast_loop = {motor_pwm, motor_encoder, param->fast_loop_param, *calibration, &I_A_DR, &I_B_DR, &I_C_DR, &V_BUS_DR};


    LED led = {const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(get_board_pins(board_rev).led_tim_r)), 
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(get_board_pins(board_rev).led_tim_g)),
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(get_board_pins(board_rev).led_tim_b)),
               config::main_loop_frequency};
    volatile uint32_t &V5V_DR = V_BUS_DR;
};

#include "../config/main_loop_config.h"
using MainLoopConfig = MainLoopConfigDefault;

constexpr TraceParam<MainLoopConfig> active_param;
using System = SystemBase<Actuator<FastLoop<config::pwm_frequency>, MainLoop<MainLoopConfig, active_param.main_loop>>>;

#if COMMS == COMMS_USB
template<>
Communication System::communication_ = {config::usb};
#endif

#if (COMMS == COMMS_CAN)
template<>
Communication System::communication_(config::can, param->can_id);
#endif

void usb_interrupt() {
    config::usb.interrupt();
}
namespace config {
    MainLoop<MainLoopConfig, active_param.main_loop> main_loop = {fast_loop, System::communication_, led, output_encoder, torque_sensor, drv, param->main_loop_param, *calibration};
};

template<>
decltype(System::actuator_) System::actuator_ = {config::fast_loop, config::main_loop, param->startup_param, *calibration};

float v3v3 = 3.3;

// has_5V,i5V,i48V_sense
float v5v, i5v, i48v;

int32_t index_mod = 0;

uint32_t init_failure = 0;

void config_init();

extern uint32_t _eccmram[];

void system_init() {

#if COMMS == COMMS_CAN
    System::api.add_api_variable("can_send_decimation", new APICallbackUint16([]{ return System::communication_.get_send_decimation(); },
        [](uint16_t u){ System::communication_.set_send_decimation(u); }));
#endif

    if (config::motor_encoder.init()) {
        System::log("Motor encoder init success");
    } else {
        System::log("Motor encoder init failure");
        init_failure |= 1;
    }
    if (config::output_encoder.init()) {
        System::log("Output encoder init success");
    } else {
        System::log("Output encoder init failure");
        init_failure |= 1;
    }
    if (config::torque_sensor.init()) {
        System::log("torque sensor init success");
    } else {
        System::log("torque sensor init failure");
        init_failure |= 1;
    }

    System::api.add_api_variable("3v3", new APIFloat(&v3v3));
    System::api.add_api_variable("Tmicro", new APICallbackFloat([]{ return config::temp_sensor.get_value(); },
        [](float f){ config::temp_sensor.set_value(f); }));
    System::api.add_api_variable("index_mod", new APIInt32(&index_mod));
    System::api.add_api_variable("pwm_mult", new APICallbackUint8([](){return config::motor_pwm.get_frequency_multiplier();}, [](uint8_t mult){ config::motor_pwm.set_frequency_multiplier(mult);}));
    System::api.add_api_variable("A1", new const APICallbackUint32([](){ return A1_DR; }));
    System::api.add_api_variable("A2", new const APICallbackUint32([](){ return A2_DR; }));
    System::api.add_api_variable("A3", new const APICallbackUint32([](){ return A3_DR; }));
    System::api.add_api_variable("IA0", new const APIUint32(&ADC3->DR));
    System::api.add_api_variable("IB0", new const APIUint32(&ADC4->DR));
    System::api.add_api_variable("IC0", new const APIUint32(&ADC5->DR));
    System::api.add_api_variable("IA", new const APIUint32(&ADC3->JDR1));
    System::api.add_api_variable("IB", new const APIUint32(&ADC4->JDR1));
    System::api.add_api_variable("IC", new const APIUint32(&ADC5->JDR1));
    System::api.add_api_variable("usb_err", new APIUint32(&config::usb.error_count_));
    System::api.add_api_variable("usb_reset_count", new APIUint32(&config::usb.reset_count_));
    System::api.add_api_variable("hsi48_trim", new const APICallbackInt8([](){ return (int8_t) (((CRS->CR & CRS_CR_TRIM) >> CRS_CR_TRIM_Pos) - 64); }));
    System::api.add_api_variable("shutdown", new const APICallback([](){
        // requires power cycle to return 
        setup_sleep();
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
        PWR->CR1 |= 0b100 << PWR_CR1_LPMS_Pos;
        __WFI();
        return std::string();
    }));
    System::api.add_api_variable("deadtime", new APICallbackUint16([](){ 
        return config::motor_pwm.deadtime_ns_; }, [](uint16_t u) {config::motor_pwm.set_deadtime(u); }));
        System::api.add_api_variable("idelay", new APICallbackUint16([](){ 
            return config::motor_pwm.get_current_sample_delay(); }, [](uint16_t u) {config::motor_pwm.set_current_sample_delay(u); }));

    System::api.add_api_variable("mcmp", new APIUint32(&HRTIM1->sMasterRegs.MCMP1R));
    System::api.add_api_variable("t1cmp", new APIUint32(&TIM1->CCR1));

    System::api.add_api_variable("flash_cal", new const APICallback([]{
        System::set_one_time_api_timeout_us(100 * 1000);
        void * adr = &_eccmram; // End of ccmram is an empty ram space. The linker script ensures that there is enough 
                                // space for the calibration to reside here temporarily
        Calibration *cal = (Calibration *) adr;
        std::memcpy(adr, calibration, sizeof(Calibration));
        cal->motor_encoder_bias = System::actuator_.startup_motor_bias_;
        cal->torque_sensor.bias = config::main_loop.torque_sensor_bias_;
        cal->torque_sensor.gain = config::main_loop.torque_sensor_.gain_;
        //cal->joint_encoder_bias;
        cal->output_encoder_bias = config::main_loop.output_encoder_bias_;
        if (std::isfinite(config::fast_loop.motor_index_electrical_offset_measured_)) {
            cal->motor_encoder_index_electrical_offset_pos = config::fast_loop.motor_index_electrical_offset_measured_;
        }
        config::flash.write((uint32_t) calibration, (uint32_t*) cal, sizeof(Calibration));
        return std::string("ok");
    }));

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
    System::log("obias: " +  std::to_string(calibration->output_encoder_bias));
    System::log("tbias: " + std::to_string(calibration->torque_sensor.bias));
    System::log("tgain: " + std::to_string(calibration->torque_sensor.gain));
    System::log("offset: " + std::to_string(calibration->motor_encoder_index_electrical_offset_pos));
    System::log("mbias: " + std::to_string(calibration->motor_encoder_bias));

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
MedianFilter<> board_temperature_filter;
MedianFilter<> microcontroller_temperature_filter;
MedianFilter<> mosfet_temperature_filter;
MedianFilter<> mosfet2_temperature_filter;

void config_maintenance();
void system_maintenance() {
    static bool driver_fault = false;
    if (config::drv.is_enabled() && !(config::main_loop.mode_ == DAMPED)) {
        config::fast_loop.zero_current_sensors(I_A0_DR, I_B0_DR, I_C0_DR);
    }   


    round_robin_logger.log_data(MOTOR_POWER_INDEX, config::main_loop.status_.fast_loop.power);
    if (!(GPIOC->IDR & 1<<14)) {
        driver_fault = true;
    } else if (param->main_loop_param.no_latch_driver_fault) {
        driver_fault = false;
    }

    round_robin_logger.log_data(BUS_VOLTAGE_INDEX, config::main_loop.status_.fast_loop.vbus);
    round_robin_logger.log_data(USB_ERROR_COUNT_INDEX, config::usb.error_count_);
    config::main_loop.status_.error.driver_fault |= driver_fault;    // maybe latch driver fault until reset
    index_mod = config::motor_encoder.index_error(param->fast_loop_param.motor_encoder.cpr);
    config_maintenance();
    // unclearable init failure fault
    config::main_loop.status_.error.init_failure |= init_failure;
}

Task<> main_maintenance_async(CycleScheduler &sched) {
    while (1) {
        co_await sched.async_delay_us(100'000);
        ADC1->CR |= ADC_CR_JADSTART;
        while(ADC1->CR & ADC_CR_JADSTART) {
            co_await sched.yield();
        }
        T = microcontroller_temperature_filter.update(config::temp_sensor.read());
        round_robin_logger.log_data(MICROCONTROLLER_TEMPERATURE_INDEX, T);
        v3v3 =  *((uint16_t *) (0x1FFF75AA)) * 3.0 * ADC1->GCOMP / 4096.0 / ADC1->JDR2;
        round_robin_logger.log_data(VOLTAGE_3V3_INDEX, v3v3);
        if (T > 100) {
            config::main_loop.status_.error.microcontroller_temperature = 1;
        }

        float Tboard = 0;

        round_robin_logger.log_data(BOARD_TEMPERATURE_INDEX, Tboard);
        if (Tboard > 120 || Tboard < -40) {
            config::main_loop.status_.error.board_temperature = 1;
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