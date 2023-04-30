#include "../peripheral/usb.h"
#include "../usb_communication.h"
#include "../peripheral/stm32g4/hrpwm.h"
#include "../util.h"
#include "../peripheral/stm32g4/pin_config.h"
#include "../peripheral/stm32g4/drv8323s.h"

#ifndef PWM
//using PWM = HRPWM;
#endif
using Communication = USBCommunication;
using Driver = DRV8323S;
volatile uint32_t * const cpu_clock = &DWT->CYCCNT;
uint16_t drv_regs_error = 0;

#ifndef GPIO_OUT
#define GPIO_OUT (reinterpret_cast<volatile gpio_bits*>(&GPIOA->ODR)->bit1)
#endif

#ifndef GPIO_IN
#define GPIO_IN ((GPIOA->IDR & (1 << 2)) ? 1 : 0)
#endif

#include "../led.h"
#include "../controller/position_controller.h"
#include "../controller/torque_controller.h"
#include "../controller/impedance_controller.h"
#include "../controller/velocity_controller.h"
#include "../controller/state_controller.h"
#include "../controller/joint_position_controller.h"
#include "../fast_loop.h"
#include "../main_loop.h"
#include "../actuator.h"
#include "../system.h"
#include "pin_config_obot_g474_motor.h"
#include "../peripheral/stm32g4/temp_sensor.h"


#if defined(R3) || defined(R4) || defined(MR0) || defined(MR0P)
#ifndef BROKEN_MAX31875
#define HAS_MAX31875
#include "../peripheral/stm32g4/max31875.h"
#endif
#endif

#if defined(MR1)
#define HAS_MAX31889
#include "../peripheral/stm32g4/max31889.h"
#endif

#if defined(R4) || defined (MR0P) || defined (MR0) || defined(MR1)
#define HAS_BMI270
#endif

#if defined(MR0) || defined (MR0P)
#define HAS_BRIDGE_THERMISTORS
#endif

namespace config {
    static_assert(((double) CPU_FREQUENCY_HZ * 8 / 2) / pwm_frequency < 65535);    // check pwm frequency
#ifdef SPI1_REINIT_CALLBACK
    DRV8323S drv(*SPI1, spi1_dma.register_operation_, spi1_reinit_callback);
#else
    DRV8323S drv(*SPI1);
#endif
    TempSensor temp_sensor;
    I2C i2c1(*I2C1, 400);
#ifdef HAS_MAX31875
    MAX31875 board_temperature(i2c1);
#endif
#ifdef HAS_MAX31889
    MAX31889 board_temperature(i2c1);
#endif
#ifdef HAS_BRIDGE_THERMISTORS
    NTC temp_bridge(TSENSE);
    NTC temp_bridge2(TSENSE2);
#endif
  //  HRPWM motor_pwm = {pwm_frequency, *HRTIM1, 3, 5, 4, false, 200, 1000, 0};
    USB1 usb;
    FastLoop fast_loop = {(int32_t) pwm_frequency, motor_pwm, motor_encoder, param->fast_loop_param, &I_A_DR, &I_B_DR, &I_C_DR, &V_BUS_DR};
    LED led = {const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM_R)), 
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM_G)),
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM_B))};
    PositionController position_controller = {(float) (1.0/main_loop_frequency)};
    TorqueController torque_controller = {(float) (1.0/main_loop_frequency)};
    ImpedanceController impedance_controller = {(float) (1.0/main_loop_frequency)};
    VelocityController velocity_controller = {(float) (1.0/main_loop_frequency)};
    StateController state_controller = {(float) (1.0/main_loop_frequency)};
    JointPositionController joint_position_controller(1.0/main_loop_frequency);
    MainLoop main_loop = {fast_loop, position_controller, torque_controller, impedance_controller, velocity_controller, state_controller, joint_position_controller, System::communication_, led, output_encoder, torque_sensor, drv, param->main_loop_param};
};

Communication System::communication_ = {config::usb};
void usb_interrupt() {
    config::usb.interrupt();
}
Actuator System::actuator_ = {config::fast_loop, config::main_loop, param->startup_param};

float v3v3 = 3.3;

int32_t index_mod = 0;

void config_init();

void system_init() {
    if (config::motor_encoder.init()) {
        System::log("Motor encoder init success");
    } else {
        System::log("Motor encoder init failure");
    }
    if (config::output_encoder.init()) {
        System::log("Output encoder init success");
    } else {
        System::log("Output encoder init failure");
    }
    if (drv_regs_error) {
        System::log("drv configure failure");
    } else {
        System::log("drv configure success");
    }
    if (config::torque_sensor.init()) {
        System::log("torque sensor init success");
    } else {
        System::log("torque sensor init failure");
    }

    System::api.add_api_variable("3v3", new APIFloat(&v3v3));
    std::function<float()> get_t = std::bind(&TempSensor::get_value, &config::temp_sensor);
    std::function<void(float)> set_t = std::bind(&TempSensor::set_value, &config::temp_sensor, std::placeholders::_1);
    System::api.add_api_variable("T", new APICallbackFloat(get_t, set_t));
#if defined(HAS_MAX31875) || defined(HAS_MAX31889)
    System::api.add_api_variable("Tboard", new const APICallbackFloat([](){ return config::board_temperature.get_temperature(); }));
#endif
#ifdef HAS_BRIDGE_THERMISTORS
    System::api.add_api_variable("Tbridge", new const APICallbackFloat([](){ return config::temp_bridge.read(); }));
    System::api.add_api_variable("Tbridge2", new const APICallbackFloat([](){ return config::temp_bridge2.read(); }));
#endif
    System::api.add_api_variable("index_mod", new APIInt32(&index_mod));
    System::api.add_api_variable("pwm_mult", new APICallbackUint8([](){return config::motor_pwm.get_frequency_multiplier();}, [](uint8_t mult){ config::motor_pwm.set_frequency_multiplier(mult);}));
    System::api.add_api_variable("drv_err", new const APICallbackUint32([](){ return config::drv.get_drv_status(); }));
    System::api.add_api_variable("drv_reset", new const APICallback([](){ return config::drv.drv_reset(); }));
    System::api.add_api_variable("A1", new const APICallbackFloat([](){ return A1_DR; }));
    System::api.add_api_variable("A2", new const APICallbackFloat([](){ return A2_DR; }));
    System::api.add_api_variable("A3", new const APICallbackFloat([](){ return A3_DR; }));
    System::api.add_api_variable("shutdown", new const APICallback([](){
        // requires power cycle to return 
        setup_sleep();
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
        PWR->CR1 |= 0b100 << PWR_CR1_LPMS_Pos;
        __WFI();
        return "";
    }));
    System::api.add_api_variable("deadtime", new APICallbackUint16([](){ 
        return config::motor_pwm.deadtime_ns_; }, [](uint16_t u) {config::motor_pwm.set_deadtime(u); }));

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
    ADC5->CR |= ADC_CR_JADSTART;
    ADC5->IER |= ADC_IER_JEOCIE;
    ADC4->CR |= ADC_CR_JADSTART;
    ADC3->CR |= ADC_CR_JADSTART;

    config_init();

    TIM1->CR1 = TIM_CR1_CEN; // start main loop interrupt
    config::usb.connect();
    HRTIM1->sMasterRegs.MCR = HRTIM_MCR_TDCEN + HRTIM_MCR_TECEN + HRTIM_MCR_TFCEN; // start high res timer
}

FrequencyLimiter temp_rate = {10};
float T = 0;

void config_maintenance();
void system_maintenance() {
    static bool driver_fault = false;
    if (temp_rate.run()) {
        ADC1->CR |= ADC_CR_JADSTART;
        while(ADC1->CR & ADC_CR_JADSTART);
        T = config::temp_sensor.read();
        v3v3 =  *((uint16_t *) (0x1FFF75AA)) * 3.0 * ADC1->GCOMP / 4096.0 / ADC1->JDR2;
        round_robin_logger.log_data(VOLTAGE_3V3_INDEX, v3v3);
        if (T > 100) {
            config::main_loop.status_.error.microcontroller_temperature = 1;
        }
#if defined(HAS_MAX31875) || defined(HAS_MAX31889)
        config::board_temperature.read();
        round_robin_logger.log_data(BOARD_TEMPERATURE_INDEX, config::board_temperature.get_temperature());
        if (config::board_temperature.get_temperature() > 100) {
            config::main_loop.status_.error.board_temperature = 1;
        }
#endif
#ifdef HAS_BRIDGE_THERMISTORS
        config::temp_bridge.read();
        round_robin_logger.log_data(MOSFET_TEMPERATURE_INDEX, config::temp_bridge.get_temperature());
        if (config::temp_bridge.get_temperature() > 150) {
            config::main_loop.status_.error.board_temperature = 1;
        }
        round_robin_logger.log_data(MOSFET2_TEMPERATURE_INDEX, config::temp_bridge2.get_temperature());
        config::temp_bridge2.read();
        if (config::temp_bridge2.get_temperature() > 150) {
            config::main_loop.status_.error.board_temperature = 1;
        }
#endif
    }   
    
    float bus_current = config::main_loop.status_.power/config::main_loop.status_.fast_loop.vbus;
    round_robin_logger.log_data(BUS_CURRENT_INDEX, bus_current);
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
    config::drv.enable();
    NVIC_DisableIRQ(RTC_WKUP_IRQn);
    NVIC_SetPriority(USB_LP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    NVIC_EnableIRQ(ADC5_IRQn);
}


#include "../../motorlib/system.cpp"
