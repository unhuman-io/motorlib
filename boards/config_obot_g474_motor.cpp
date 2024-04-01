#include "../peripheral/usb.h"
#include "../usb_communication.h"
#include "../peripheral/stm32g4/hrpwm.h"
#include "../util.h"
#include "../peripheral/stm32g4/pin_config.h"
#include "../peripheral/stm32g4/drv8323s.h"
#include "../peripheral/stm32g4/uart.h"
#include "../peripheral/protocol.h"

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
#include <protocol_parser.h>
#include "../peripheral/stm32g4/spi_slave_figure.h"
#include "../spi_communication_obot.h"

#define COMMS_USB   1
#define COMMS_SPI   2
#define COMMS_UART  3

#ifndef COMMS
  #error "COMMS should be defined"
#endif

#if (COMMS != COMMS_USB) && (COMMS != COMMS_SPI) && (COMMS != COMMS_UART)
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

#if (COMMS == COMMS_SPI)
    #include <protocol_parser.h>
    using Communication = SPICommunication;
#endif

#if (COMMS == COMMS_UART)
#ifdef COMMS_UART_OBOT
    #include <protocol_parser.h>
    #include "../uart_communication_obot.h"
#else
    #include "../uart_communication_protocol.h"
    using UARTCommunicationProtocol = UARTRawProtocol<>; 
    #include "../uart_communication.h"
#endif
    using Communication = UARTCommunication;
#endif
using Driver = DRV8323S;
uint16_t drv_regs_error = 0;

#ifndef GPIO_OUT
#define GPIO_OUT (reinterpret_cast<volatile gpio_bits*>(&GPIOA->ODR)->bit1)
#endif

#ifndef GPIO_IN
#define GPIO_IN ((GPIOA->IDR & (1 << 2)) ? 1 : 0)
#endif

#include "../led.h"
#ifndef POSITION_CONTROLLER_OVERRIDE
#include "../controller/position_controller.h"
#endif
#ifndef TORQUE_CONTROLLER_OVERRIDE
#include "../controller/torque_controller.h"
#endif
#ifndef IMPEDANCE_CONTROLLER_OVERRIDE
#include "../controller/impedance_controller.h"
#endif
#ifndef VELOCITY_CONTROLLER_OVERRIDE
#include "../controller/velocity_controller.h"
#endif
#ifndef STATE_CONTROLLER_OVERRIDE
#include "../controller/state_controller.h"
#endif
#ifndef JOINT_POSITION_CONTROLLER_OVERRIDE
#include "../controller/joint_position_controller.h"
#endif
#ifndef ADMITTANCE_CONTROLLER_OVERRIDE
#include "../controller/admittance_controller.h"
#endif
#include "../fast_loop.h"
#include "../main_loop.h"
#include "../actuator.h"
#include "../system.h"
#include "pin_config_obot_g474_motor.h"
#include "../peripheral/stm32g4/temp_sensor.h"
#include "../temperature_sensor.h"
#include "../peripheral/stm32g4/i2c_dma.h"
#include "../peripheral/stm32g4/spi_dma.h"
#include "../bmi270.h"
#include "../peripheral/stm32g4/max31875.h"
#include "../peripheral/stm32g4/max31889.h"
#include "../mb85rc64.h"
#include "../messages.h"

extern "C" void SystemClock_Config();
void pin_config_obot_g474_motor(const BoardRev&);

extern "C" void board_init() {
    const BoardRev board_rev = get_board_rev();
    SystemClock_Config();
    pin_config_obot_g474_motor(board_rev);
#ifdef SCOPE_DEBUG
    GPIO_SETL(C, 0, GPIO_MODE::OUTPUT, GPIO_SPEED::HIGH, 0); // main loop scope
    GPIO_SETL(C, 1, GPIO_MODE::OUTPUT, GPIO_SPEED::HIGH, 0); // fast loop scope
    GPIO_SETL(C, 2, GPIO_MODE::OUTPUT, GPIO_SPEED::HIGH, 0); // usb int scope
    GPIO_SETL(C, 4, GPIO_MODE::OUTPUT, GPIO_SPEED::HIGH, 0); // main() scope
    GPIO_SETL(A, 0, GPIO_MODE::OUTPUT, GPIO_SPEED::HIGH, 0); // system loop scope
#endif
}


namespace config {
    static_assert(((double) CPU_FREQUENCY_HZ * 8 / 2) / pwm_frequency < 65535);    // check pwm frequency
#ifdef SPI1_REINIT_CALLBACK
    DRV8323S drv(*SPI1, spi1_dma.register_operation_, spi1_reinit_callback);
#else
    DRV8323S drv(*SPI1);
#endif
    TempSensor temp_sensor;
    I2C_DMA i2c1(*I2C1, *DMA1_Channel7, *DMA1_Channel8, 400);
    
    // has_max31875
    MAX31875 board_temperature_max31875(i2c1);

    // has_max31889
    MAX31889 board_temperature_max31889(i2c1);

    // has_bridge_thermistors
    NTC temp_bridge(TSENSE);
    NTC temp_bridge2(TSENSE2);

    // has_bmi270
    GPIO imu_cs(*GPIOC, 4, GPIO::OUTPUT);
    SPIDMA spi1_dma_bmi270(*SPI1, imu_cs, *DMA1_Channel3, *DMA1_Channel4, 40, 40, drv.register_operation_,
        SPI_CR1_MSTR | (4 << SPI_CR1_BR_Pos) | SPI_CR1_SSI | SPI_CR1_SSM);    // baud = clock/32
    BMI270 imu(spi1_dma_bmi270);

    // has_mb85rc64
    MB85RC64 mb85rc64(i2c1, 4);

    const BoardRev board_rev = get_board_rev();

#if COMMS == COMMS_SPI
 SpiSlaveFigure spi({
      .spi          = SPI1,
      .gpioPort     = GPIOA,
      .gpioPinSs    = 4U,
      .gpioPinSck   = 5U,
      .gpioPinMosi  = 7U,
      .gpioPinMiso  = 6U,

      .gpioAlternateFunction = 5U,

      .gpioRccEnableRegister = &RCC->AHB2ENR,
      .gpioRccEnableBit      = RCC_AHB2ENR_GPIOAEN_Pos,
      .spiRccEnableRegister = &RCC->APB2ENR,
      .spiRccEnableBit      = RCC_APB2ENR_SPI1EN_Pos,
      .spiRccResetRegister  = &RCC->APB2RSTR,
      .spiRccResetBit       = RCC_APB2RSTR_SPI1RST_Pos,

      .rxDma            = DMA2,
      .rxDmaIfcrCgif    = DMA_IFCR_CGIF1,
      .rxDmaChannel     = DMA2_Channel1,
      .rxDmaMuxChannel  = DMAMUX1_Channel8,
      .rxDmaMuxId       = 10U,
      .rxDmaIrqN        = DMA2_Channel1_IRQn,
      .rxDmaIrqPriority = 1U,

      .txDma            = DMA2,
      .txDmaIfcrCgif    = DMA_IFCR_CGIF2,
      .txDmaChannel     = DMA2_Channel2,
      .txDmaMuxChannel  = DMAMUX1_Channel9,
      .txDmaMuxId       = 11U,
      .txDmaIrqN        = DMA2_Channel2_IRQn,
      .txDmaIrqPriority = 5U,
    });
#endif // COMMS_SPI

#if COMMS == COMMS_UART
#if COMMS_UART_NUMBER == 2
    Uart uart({
      .usart        = USART2,
      .gpioPort     = GPIOA,
      .gpioPinTx    = 2U,
      .gpioPinRx    = 3U,

      .gpioAlternateFunction = 7U,

      .gpioRccEnableRegister = &RCC->AHB2ENR,
      .gpioRccEnableBit      = RCC_AHB2ENR_GPIOAEN_Pos,
      .uartRccEnableRegister  = &RCC->APB1ENR1,
      .uartRccEnableBit       = RCC_APB1ENR1_USART2EN_Pos,
      .uartRccResetRegister   = &RCC->APB1RSTR1,
      .uartRccResetBit        = RCC_APB1RSTR1_USART2RST_Pos,

      .uartIrqN               = USART2_IRQn,

      .rxDma            = DMA2,
      .rxDmaIfcrCgif    = DMA_IFCR_CGIF3,
      .rxDmaChannel     = DMA2_Channel3,
      .rxDmaMuxChannel  = DMAMUX1_Channel10,
      .rxDmaMuxId       = 26U,
      .rxDmaIrqN        = DMA2_Channel3_IRQn,

      .txDma            = DMA2,
      .txDmaIfcrCgif    = DMA_IFCR_CGIF4,
      .txDmaChannel     = DMA2_Channel4,
      .txDmaMuxChannel  = DMAMUX1_Channel11,
      .txDmaMuxId       = 27U,
      .txDmaIrqN        = DMA2_Channel4_IRQn,

      .irqPriority = 2U,

      .brrValue         = (uint32_t)((CPU_FREQUENCY_HZ + COMMS_UART_BAUDRATE/2)/ COMMS_UART_BAUDRATE)   // rounding
    });
#else // default usart1
    Uart uart({
      .usart        = USART1,
      .gpioPort     = GPIOA,
      .gpioPinTx    = 9U,
      .gpioPinRx    = 10U,

      .gpioAlternateFunction = 7U,

      .gpioRccEnableRegister = &RCC->AHB2ENR,
      .gpioRccEnableBit      = RCC_AHB2ENR_GPIOAEN_Pos,
      .uartRccEnableRegister  = &RCC->APB2ENR,
      .uartRccEnableBit       = RCC_APB2ENR_USART1EN_Pos,
      .uartRccResetRegister   = &RCC->APB2RSTR,
      .uartRccResetBit        = RCC_APB2RSTR_USART1RST_Pos,

      .uartIrqN               = USART1_IRQn,

      .rxDma            = DMA2,
      .rxDmaIfcrCgif    = DMA_IFCR_CGIF3,
      .rxDmaChannel     = DMA2_Channel3,
      .rxDmaMuxChannel  = DMAMUX1_Channel10,
      .rxDmaMuxId       = 24U,
      .rxDmaIrqN        = DMA2_Channel3_IRQn,

      .txDma            = DMA2,
      .txDmaIfcrCgif    = DMA_IFCR_CGIF4,
      .txDmaChannel     = DMA2_Channel4,
      .txDmaMuxChannel  = DMAMUX1_Channel11,
      .txDmaMuxId       = 25U,
      .txDmaIrqN        = DMA2_Channel4_IRQn,

      .irqPriority = 2U,

      .brrValue         = (uint32_t)((CPU_FREQUENCY_HZ + COMMS_UART_BAUDRATE/2)/ COMMS_UART_BAUDRATE)   // rounding
    });
#endif // COMMS_UART_NUMBER
#ifdef COMMS_UART_OBOT
    figure::ProtocolParser uart_protocol(config::uart.rx_buffer_, RX_BUFFER_SIZE);
#else
    UARTCommunicationProtocol uart_protocol; 
#endif
#endif // COMMS_UART



#if COMMS == COMMS_SPI
    figure::ProtocolParser spi_protocol(config::spi.rx_buffer_, RX_BUFFER_SIZE);
#endif

    HRPWM motor_pwm = {pwm_frequency, *HRTIM1, 3, 5, 4, false, 50, 1000, 1000};
    USB1 usb;
    FastLoop fast_loop = {(int32_t) pwm_frequency, motor_pwm, motor_encoder, param->fast_loop_param, *calibration, &I_A_DR, &I_B_DR, &I_C_DR, &V_BUS_DR};


    LED led = {const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(get_board_pins(board_rev).led_tim_r)), 
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(get_board_pins(board_rev).led_tim_g)),
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(get_board_pins(board_rev).led_tim_b))};
    volatile uint32_t &V5V_DR = *get_board_pins(board_rev).v5v_dr;
#ifndef POSITION_CONTROLLER_OVERRIDE
    PositionController position_controller = {(float) (1.0/main_loop_frequency)};
#endif
#ifndef TORQUE_CONTROLLER_OVERRIDE
    TorqueController torque_controller = {(float) (1.0/main_loop_frequency)};
#endif
#ifndef IMPEDANCE_CONTROLLER_OVERRIDE
    ImpedanceController impedance_controller = {(float) (1.0/main_loop_frequency)};
#endif
#ifndef VELOCITY_CONTROLLER_OVERRIDE
    VelocityController velocity_controller = {(float) (1.0/main_loop_frequency)};
#endif
#ifndef STATE_CONTROLLER_OVERRIDE
    StateController state_controller = {(float) (1.0/main_loop_frequency)};
#endif
#ifndef JOINT_POSITION_CONTROLLER_OVERRIDE
    JointPositionController joint_position_controller(1.0/main_loop_frequency);
#endif
#ifndef ADMITTANCE_CONTROLLER_OVERRIDE
    AdmittanceController admittance_controller = {1.0/main_loop_frequency};
#endif
    MainLoop main_loop = {main_loop_frequency, fast_loop, position_controller, torque_controller, impedance_controller, velocity_controller, state_controller, joint_position_controller, admittance_controller, System::communication_, led, output_encoder, torque_sensor, drv, param->main_loop_param, *calibration};
};

#if COMMS == COMMS_USB
Communication System::communication_ = {config::usb};
#endif

#if (COMMS == COMMS_SPI)
Communication System::communication_(config::spi, config::spi_protocol);
#endif

#if (COMMS == COMMS_UART)
Communication System::communication_(config::uart, config::uart_protocol);
extern "C" void PendSV_Handler(void) {
  SET_SCOPE_PIN(C,2);
  System::communication_.parse();
  CLEAR_SCOPE_PIN(C,2);
}
#endif

void usb_interrupt() {
    config::usb.interrupt();
}

Actuator System::actuator_ = {config::fast_loop, config::main_loop, param->startup_param, *calibration};

float v3v3 = 3.3;

// has_5V,i5V,i48V_sense
float v5v, i5v, i48v;

// has_mb85rc64
uint32_t total_uptime_start;
uint32_t total_uptime;

int32_t index_mod = 0;

uint32_t init_failure = 0;

void config_init();

void system_init() {

#if COMMS == COMMS_UART
    config::uart.init();
#endif

    DMAMUX1_Channel6->CCR =  DMA_REQUEST_I2C1_TX;
    DMAMUX1_Channel7->CCR =  DMA_REQUEST_I2C1_RX;
    DMAMUX1_Channel2->CCR =  DMA_REQUEST_SPI1_TX;
    DMAMUX1_Channel3->CCR =  DMA_REQUEST_SPI1_RX;
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
    if (drv_regs_error) {
        System::log("drv configure failure");
        init_failure |= 1;
    } else {
        System::log("drv configure success");
    }
    if (config::torque_sensor.init()) {
        System::log("torque sensor init success");
    } else {
        System::log("torque sensor init failure");
        init_failure |= 1;
    }
    if (config::board_rev.has_bmi270) {
        config::imu.init();
    }

    config::drv.set_debug_variables(System::api);

    System::api.add_api_variable("3v3", new APIFloat(&v3v3));
    std::function<float()> get_t = std::bind(&TempSensor::get_value, &config::temp_sensor);
    std::function<void(float)> set_t = std::bind(&TempSensor::set_value, &config::temp_sensor, std::placeholders::_1);
    System::api.add_api_variable("T", new APICallbackFloat(get_t, set_t));
    if (config::board_rev.has_max31875) {
        System::api.add_api_variable("Tboard", new const APICallbackFloat([](){ return config::board_temperature_max31875.get_temperature(); }));
    } else if (config::board_rev.has_max31889) {
        System::api.add_api_variable("Tboard", new const APICallbackFloat([](){ return config::board_temperature_max31889.get_temperature(); }));
    }
    
    if (config::board_rev.has_bridge_thermistors) {
        System::api.add_api_variable("Tbridge", new const APICallbackFloat([](){ return config::temp_bridge.read(); }));
        System::api.add_api_variable("Tbridge2", new const APICallbackFloat([](){ return config::temp_bridge2.read(); }));
    }
    System::api.add_api_variable("index_mod", new APIInt32(&index_mod));
    System::api.add_api_variable("pwm_mult", new APICallbackUint8([](){return config::motor_pwm.get_frequency_multiplier();}, [](uint8_t mult){ config::motor_pwm.set_frequency_multiplier(mult);}));
    System::api.add_api_variable("drv_err", new const APICallbackUint32([](){ return config::drv.get_drv_status(); }));
    System::api.add_api_variable("drv_reset", new const APICallback([](){ return config::drv.drv_reset(); }));
    System::api.add_api_variable("A1", new const APICallbackFloat([](){ return A1_DR; }));
    System::api.add_api_variable("A2", new const APICallbackFloat([](){ return A2_DR; }));
    System::api.add_api_variable("A3", new const APICallbackFloat([](){ return A3_DR; }));
    System::api.add_api_variable("IA0", new const APIUint32(&ADC3->DR));
    System::api.add_api_variable("IB0", new const APIUint32(&ADC4->DR));
    System::api.add_api_variable("IC0", new const APIUint32(&ADC5->DR));
    System::api.add_api_variable("IA", new const APIUint32(&ADC3->JDR1));
    System::api.add_api_variable("IB", new const APIUint32(&ADC4->JDR1));
    System::api.add_api_variable("IC", new const APIUint32(&ADC5->JDR1));
    System::api.add_api_variable("usb_err", new APIUint32(&config::usb.error_count_));
    System::api.add_api_variable("usb_reset_count", new APIUint32(&config::usb.reset_count_));
    System::api.add_api_variable("hsi48_trim", new const APICallbackInt8([](){ return (int8_t) ((CRS->CR & CRS_CR_TRIM) >> CRS_CR_TRIM_Pos) - 64; }));
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

    if (config::board_rev.has_bmi270) {
        System::api.add_api_variable("imu_read", new const APICallback([](){ config::imu.read_with_restore(); return config::imu.get_string(); }));
        System::api.add_api_variable("ax", new const APICallbackFloat([](){ return config::imu.data_.acc_x*8./pow(2,15); }));
        System::api.add_api_variable("ay", new const APICallbackFloat([](){ return config::imu.data_.acc_y*8./pow(2,15); }));
        System::api.add_api_variable("az", new const APICallbackFloat([](){ return config::imu.data_.acc_z*8./pow(2,15); }));
        System::api.add_api_variable("gx", new const APICallbackFloat([](){ return config::imu.data_.gyr_x*2000.*M_PI/180/pow(2,15); }));
        System::api.add_api_variable("gy", new const APICallbackFloat([](){ return config::imu.data_.gyr_y*2000.*M_PI/180/pow(2,15); }));
        System::api.add_api_variable("gz", new const APICallbackFloat([](){ return config::imu.data_.gyr_z*2000.*M_PI/180/pow(2,15); }));
    }

    if (config::board_rev.has_5V_sense) {
        System::api.add_api_variable("5V", new const APIFloat(&v5v));
    }
    if (config::board_rev.has_I5V_sense) {
        System::api.add_api_variable("i5V", new const APIFloat(&i5v));
    }
    if (config::board_rev.has_I48V_sense) {
        System::api.add_api_variable("i48V", new const APIFloat(&i48v));
    }

    if (config::board_rev.has_bmi270) {
        config::mb85rc64.init();
        config::mb85rc64.read_block(0, &total_uptime_start);
        logger.log_printf("total_uptime_start: %u", total_uptime_start);
        {
            std::string s = "startup at " + std::to_string(total_uptime_start) + "\n";
            config::mb85rc64.write_log((uint8_t*) s.c_str(), s.size());
        }
        System::api.add_api_variable("total_uptime", new const APIUint32(&total_uptime));
        System::api.add_api_variable("fram_log", new APICallback([](){
            config::i2c1.init(1000);
            std::string s = config::mb85rc64.get_log();
            config::i2c1.init(400);
            return s;
        }, [](std::string s){
            config::i2c1.init(1000);
            config::mb85rc64.write_log((uint8_t *) s.c_str(), s.size());
            config::i2c1.init(400);
        }));
    }

    System::api.add_api_variable("mcmp", new APIUint32(&HRTIM1->sMasterRegs.MCMP1R));
    System::api.add_api_variable("t1cmp", new APIUint32(&TIM1->CCR1));

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
    System::log("tbias: " + std::to_string(calibration->torque_sensor_bias));
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
    HRTIM1->sMasterRegs.MDIER = HRTIM_MDIER_MCMP1IE; // interrupt on MCMP1
   
    HRTIM1->sMasterRegs.MCMP1R = 400;
    static_assert(config::main_loop_frequency > CPU_FREQUENCY_HZ/4/65536, "Main loop frequency too low");
    HRTIM1->sMasterRegs.MPER = CPU_FREQUENCY_HZ/4/config::main_loop_frequency;
    HRTIM1->sMasterRegs.MCR = 0 << HRTIM_MCR_SYNC_SRC_Pos | 2 << HRTIM_MCR_SYNC_OUT_Pos | HRTIM_MCR_CONT | HRTIM_MCR_PREEN | HRTIM_MCR_MREPU | 7 << HRTIM_MCR_CK_PSC_Pos; // CPU_FREQUENCY * 32 / 2^7 = 42.5 MHz
    config::usb.connect();

    HRTIM1->sMasterRegs.MCR |= HRTIM_MCR_MCEN + HRTIM_MCR_TACEN + HRTIM_MCR_TDCEN + HRTIM_MCR_TECEN + HRTIM_MCR_TFCEN; // start high res timer, also triggers TIM1
}

FrequencyLimiter temp_rate = {10};
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
    
    float bus_current = config::main_loop.status_.power/config::main_loop.status_.fast_loop.vbus;
    if (!config::board_rev.has_I48V_sense) {
        round_robin_logger.log_data(BUS_CURRENT_INDEX, bus_current);
    }
    round_robin_logger.log_data(MOTOR_POWER_INDEX, config::main_loop.status_.fast_loop.power);
    if (!(GPIOC->IDR & 1<<14)) {
        driver_fault = true;
    } else if (param->main_loop_param.no_latch_driver_fault) {
        driver_fault = false;
    }

    if (config::board_rev.has_5V_sense) {
        v5v = (float) config::V5V_DR/4096*v3v3*2;
        round_robin_logger.log_data(VOLTAGE_5V_INDEX, v5v);
    }
    if (config::board_rev.has_I5V_sense) {
        i5v = (float) I5V/4096*v3v3;
        round_robin_logger.log_data(CURRENT_5V_INDEX, i5v);
    }
    if (config::board_rev.has_I48V_sense) {
        i48v = -((float) I_BUS_DR-2048)/4096*v3v3/20/.0005;
        round_robin_logger.log_data(BUS_CURRENT_INDEX, i48v);
    }
    round_robin_logger.log_data(BUS_VOLTAGE_INDEX, config::main_loop.status_.fast_loop.vbus);
    round_robin_logger.log_data(USB_ERROR_COUNT_INDEX, config::usb.error_count_);
    config::main_loop.status_.error.driver_fault |= driver_fault;    // maybe latch driver fault until reset
    index_mod = config::motor_encoder.index_error(param->fast_loop_param.motor_encoder.cpr);
    config_maintenance();
    // unclearable init failure fault
    config::main_loop.status_.error.init_failure |= init_failure;
}

void main_maintenance() {
    if (temp_rate.run()) {
        ADC1->CR |= ADC_CR_JADSTART;
        while(ADC1->CR & ADC_CR_JADSTART);
        T = microcontroller_temperature_filter.update(config::temp_sensor.read());
        round_robin_logger.log_data(MICROCONTROLLER_TEMPERATURE_INDEX, T);
        v3v3 =  *((uint16_t *) (0x1FFF75AA)) * 3.0 * ADC1->GCOMP / 4096.0 / ADC1->JDR2;
        round_robin_logger.log_data(VOLTAGE_3V3_INDEX, v3v3);
        if (T > 100) {
            config::main_loop.status_.error.microcontroller_temperature = 1;
        }

        float Tboard = 0;
        if (config::board_rev.has_max31875) {
            Tboard = board_temperature_filter.update(config::board_temperature_max31875.read());
        } else if (config::board_rev.has_max31889) {
            Tboard = board_temperature_filter.update(config::board_temperature_max31889.read());
        }
        round_robin_logger.log_data(BOARD_TEMPERATURE_INDEX, Tboard);
        if (Tboard > 120) {
            config::main_loop.status_.error.board_temperature = 1;
        }

        if (config::board_rev.has_bridge_thermistors) {
            float Tmosfet = mosfet_temperature_filter.update(config::temp_bridge.read());
            round_robin_logger.log_data(MOSFET_TEMPERATURE_INDEX, Tmosfet);
            if (Tmosfet > 150) {
                config::main_loop.status_.error.board_temperature = 1;
            }
            float Tmosfet2 = mosfet_temperature_filter.update(config::temp_bridge2.read());
            round_robin_logger.log_data(MOSFET2_TEMPERATURE_INDEX, Tmosfet2);
            config::temp_bridge2.read();
            if (Tmosfet2 > 150) {
                config::main_loop.status_.error.board_temperature = 1;
            }
        }
        if (config::board_rev.has_mb85rc64) {
            static bool last_fault = false;
            config::i2c1.init(1000);
            total_uptime = total_uptime_start + get_uptime();
            config::mb85rc64.write_block(0, total_uptime);
            config::mb85rc64.next_block();

            if (config::main_loop.status_.error.fault && !last_fault) {
                char s[100];
                std::sprintf(s, "fault detected, error: %08lx\n", config::main_loop.status_.error.all);
                config::mb85rc64.write_log((uint8_t *) s, std::strlen(s));
            }
            last_fault = config::main_loop.status_.error.fault;

            config::i2c1.init(400);
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


#include "../../motorlib/system.cpp"
