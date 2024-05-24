#include "../peripheral/usb.h"
#include "../usb_communication.h"
#include "../peripheral/stm32g4/hrpwm.h"
#include "../util.h"
#include "../driver.h"

const Param * const param = (const Param * const) 0x8060000;
const Calibration * const calibration = (const Calibration * const) 0x8070000;
extern const char * const name = param->name;
namespace config {
    const uint32_t system_loop_frequency =  1000;
};

using Driver = DriverBase;
using PWM = HRPWM;
using Communication = USBCommunication;
uint16_t drv_regs_error = 0;  

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
#include "pin_config_obot_g474_osa.h"
#include "../peripheral/stm32g4/temp_sensor.h"
#include "../peripheral/stm32g4/max31875.h"
#include "../peripheral/stm32_serial.h"

extern "C" void SystemClock_Config();
void pin_config_obot_g474_osa();

extern "C" void board_init() {
    init_serial_number();
    SystemClock_Config();
    pin_config_obot_g474_osa();
}

namespace config {
    static_assert(((double) CPU_FREQUENCY_HZ * 8 / 2) / pwm_frequency < 65535);    // check pwm frequency
    TempSensor temp_sensor;
    Driver driver;
    I2C_DMA i2c1(*I2C1, *DMA1_Channel7, *DMA1_Channel8, 1000);
    MAX31875 i2c_temp_sensor(i2c1);
    HRPWM motor_pwm = {pwm_frequency, *HRTIM1, 4, 5, 3, true, 200, 1000, 0};
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
    MainLoop main_loop = {main_loop_frequency, fast_loop, position_controller, torque_controller, impedance_controller, velocity_controller, state_controller, joint_position_controller, admittance_controller, System::communication_, led, output_encoder, torque_sensor, driver, param->main_loop_param, *calibration};
};

Communication System::communication_ = {config::usb};
void usb_interrupt() {
    config::usb.interrupt();
}
Actuator System::actuator_ = {config::fast_loop, config::main_loop, param->startup_param, *calibration};

float v_ref = 3.0;
float t_i2c = 0;

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
    config::torque_sensor.init();

    System::api.add_api_variable("vref", new APIFloat(&v_ref));
    std::function<float()> get_t = std::bind(&TempSensor::get_value, &config::temp_sensor);
    std::function<void(float)> set_t = std::bind(&TempSensor::set_value, &config::temp_sensor, std::placeholders::_1);
    System::api.add_api_variable("Tmicro", new APICallbackFloat(get_t, set_t));
    System::api.add_api_variable("Tdrv", new const APIFloat(&t_i2c));
    System::api.add_api_variable("drv_err", new const APICallbackUint32([](){return is_mps_driver_faulted();}));
    System::api.add_api_variable("drv_enable", new APICallbackUint8(mps_driver_enable_status, mps_driver_enable));
    System::api.add_api_variable("vam", new const APICallbackFloat([]() { return (33.0+2.0)/2.0 * 3.0/4096 * V_A_DR; }));
    System::api.add_api_variable("vbm", new const APICallbackFloat([]() { return (33.0+2.0)/2.0 * 3.0/4096 * V_B_DR; }));
    System::api.add_api_variable("vcm", new const APICallbackFloat([]() { return (33.0+2.0)/2.0 * 3.0/4096 * V_C_DR; }));
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

        regs->ISR = ADC_ISR_ADRDY;
        regs->CR |= ADC_CR_ADEN;
        while(!(regs->ISR & ADC_ISR_ADRDY));
    }

    ADC1->CR |= ADC_CR_JADSTART;
    while(ADC1->CR & ADC_CR_JADSTART);

    v_ref =  *((uint16_t *) (0x1FFF75AA)) * 3.0 / V_REF_DR;
    System::log("v_ref: " + std::to_string(v_ref));
    System::log("obias: " +  std::to_string(calibration->output_encoder_bias));
    System::log("tbias: " + std::to_string(calibration->torque_sensor.bias));
    System::log("tgain: " + std::to_string(calibration->torque_sensor.gain));
    System::log("offset: " + std::to_string(calibration->motor_encoder_index_electrical_offset_pos));
    System::log("mbias: " + std::to_string(calibration->motor_encoder_bias));

    ADC1->GCOMP = 3.0*4096;
    ADC1->CFGR2 |= ADC_CFGR2_GCOMP;
    ADC1->CR |= ADC_CR_ADSTART;
    ADC2->CR |= ADC_CR_JADSTART;
    ADC5->CR |= ADC_CR_JADSTART;
    ADC5->IER |= ADC_IER_JEOCIE;
    ADC4->CR |= ADC_CR_JADSTART;
    ADC3->CR |= ADC_CR_JADSTART;

    config_init();

    config::main_loop.init();

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

FrequencyLimiter temp_rate = {10};

void config_maintenance();
void system_maintenance() {
    if (temp_rate.run()) {
        ADC1->CR |= ADC_CR_JADSTART;
        while(ADC1->CR & ADC_CR_JADSTART);
        config::temp_sensor.read();
        t_i2c = config::i2c_temp_sensor.read();
        v_ref =  *((uint16_t *) (0x1FFF75AA)) * 3.0 * ADC1->GCOMP / 4096.0 / ADC1->JDR2;
    }
    config::main_loop.status_.error.driver_fault = is_mps_driver_faulted();
    
    config_maintenance();
}
void main_maintenance() {}

#include "../../motorlib/system.cpp"
