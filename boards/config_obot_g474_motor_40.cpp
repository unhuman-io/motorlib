#include "../peripheral/usb.h"
#include "../usb_communication.h"
#include "../peripheral/stm32g4/hrpwm.h"
#include "../util.h"
#include "../driver_mps.h"

const Param * const param = (const Param * const) 0x8060000;
const Calibration * const calibration = (const Calibration * const) 0x8070000;
extern const char * const name = param->name;

using PWM = HRPWM;
using Communication = USBCommunication;
using Driver = DriverMPS;

#include "../led.h"
#include "../controller/position_controller.h"
#include "../controller/torque_controller.h"
#include "../controller/impedance_controller.h"
#include "../controller/velocity_controller.h"
#include "../controller/state_controller.h"
#include "../controller/joint_position_controller.h"
#include "../controller/admittance_controller.h"
#include "../fast_loop.h"
#include "../main_loop.h"
#include "../actuator.h"
#include "../system.h"
#include "pin_config_obot_g474_motor_40.h"
#include "../peripheral/stm32g4/temp_sensor.h"
#include "../peripheral/stm32g4/max31875.h"
#include "../peripheral/stm32_serial.h"

extern "C" void SystemClock_Config();
void pin_config_obot_g474_motor_40();

extern "C" void board_init() {
    init_serial_number();
    SystemClock_Config();
    pin_config_obot_g474_motor_40();
}

namespace config {
    static_assert(((double) CPU_FREQUENCY_HZ * 8 / 2) / pwm_frequency < 65535);    // check pwm frequency
    TempSensor temp_sensor;
    I2C_DMA i2c1(*I2C1, *DMA1_Channel7, *DMA1_Channel8, 1000);
    MAX31875 board_temperature(i2c1);
    DriverMPS driver;

    HRPWM motor_pwm(pwm_frequency, *HRTIM1, 3, 5, 4, true, 50, 1000, 0);
    USB1 usb;
    FastLoop fast_loop = {(int32_t) pwm_frequency, motor_pwm, motor_encoder, param->fast_loop_param, *calibration, &I_A_DR, &I_B_DR, &I_C_DR, &V_BUS_DR};
    LED led = {const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM_R)), 
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM_G)),
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM_B))};
    PositionController position_controller = {(float) (1.0/main_loop_frequency)};
    TorqueController torque_controller = {(float) (1.0/main_loop_frequency)};
    ImpedanceController impedance_controller = {(float) (1.0/main_loop_frequency)};
    VelocityController velocity_controller = {(float) (1.0/main_loop_frequency)};
    StateController state_controller = {(float) (1.0/main_loop_frequency)};
    JointPositionController joint_position_controller(1.0/main_loop_frequency);
    AdmittanceController admittance_controller = {1.0/main_loop_frequency};
    MainLoop main_loop(main_loop_frequency, fast_loop, position_controller, torque_controller, impedance_controller, velocity_controller, state_controller, joint_position_controller, admittance_controller, System::communication_, led, output_encoder, torque_sensor, driver, param->main_loop_param, *calibration);
};

Communication System::communication_ = {config::usb};
void usb_interrupt() {
    config::usb.interrupt();
}
Actuator System::actuator_ = {config::fast_loop, config::main_loop, param->startup_param, *calibration};

float v3v3 = 3.3;

int32_t index_mod = 0;

void config_init();

void system_init() {
    DMAMUX1_Channel6->CCR =  DMA_REQUEST_I2C1_TX;
    DMAMUX1_Channel7->CCR =  DMA_REQUEST_I2C1_RX;
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
    if (config::torque_sensor.init()) {
        System::log("Torque sensor init success");
    } else {
        System::log("Torque sensor init failure");
    }

    System::api.add_api_variable("3v3", new APIFloat(&v3v3));
    std::function<float()> get_t = std::bind(&TempSensor::get_value, &config::temp_sensor);
    std::function<void(float)> set_t = std::bind(&TempSensor::set_value, &config::temp_sensor, std::placeholders::_1);
    System::api.add_api_variable("T", new APICallbackFloat(get_t, set_t));
    System::api.add_api_variable("Tboard", new const APICallbackFloat([](){ return config::board_temperature.get_temperature(); }));
    System::api.add_api_variable("index_mod", new APIInt32(&index_mod));
    System::api.add_api_variable("drv_reset", new const APICallback([](){ return config::driver.reset(); }));
    System::api.add_api_variable("usb_err", new APIUint32(&config::usb.error_count_));
    System::api.add_api_variable("usb_reset_count", new APIUint32(&config::usb.reset_count_));
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
        if (T > 100) {
            config::main_loop.status_.error.microcontroller_temperature = 1;
        }
        config::board_temperature.read();
        if (config::board_temperature.get_temperature() > 100) {
            config::main_loop.status_.error.board_temperature = 1;
        }
    }
    if (!(GPIOC->IDR & 1<<14)) {
        driver_fault = true;
    }
    config::main_loop.status_.error.driver_fault = driver_fault;    // latch driver fault until reset
    index_mod = config::motor_encoder.index_error(param->fast_loop_param.motor_encoder.cpr);
    config_maintenance();
}

void setup_sleep() {
    NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
    NVIC_DisableIRQ(ADC5_IRQn);
    config::driver.disable();
    NVIC_SetPriority(USB_LP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 1));
    NVIC_EnableIRQ(RTC_WKUP_IRQn);
    MASK_SET(RCC->CFGR, RCC_CFGR_SW, 2); // HSE is system clock source
    RTC->SCR = RTC_SCR_CWUTF;
}

void finish_sleep() {
    MASK_SET(RCC->CFGR, RCC_CFGR_SW, 3); // PLL is system clock source
    config::driver.enable();
    NVIC_DisableIRQ(RTC_WKUP_IRQn);
    NVIC_SetPriority(USB_LP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    NVIC_EnableIRQ(ADC5_IRQn);
}

#include "../../motorlib/system.cpp"
