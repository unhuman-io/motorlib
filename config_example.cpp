#include "usb_communication.h"
#include "st_device.h"
#include "system.h"
#include "qep_encoder.h"
#include "peripheral/stm32g4/hrpwm.h"
#include "peripheral/stm32g4/spi_encoder.h"
#include "peripheral/stm32g4/ams_encoder.h"
#include "hall.h"
#include "fast_loop.h"
#include "main_loop.h"
#include "led.h"
#include "peripheral/usb.h"
#include "actuator.h"
#include "../Inc/main.h"

typedef FastLoop<HRPWM, QEPEncoder> FastLoopConfig;
typedef MainLoop<FastLoopConfig> MainLoopConfig;
typedef Actuator<FastLoopConfig, MainLoopConfig> ActuatorConfig;
typedef System<ActuatorConfig, USB1> SystemConfig;

template<>
USB1 SystemConfig ::usb_ = {};

static struct {
    uint32_t pwm_frequency = (double) CPU_FREQUENCY_HZ * 32.0 / (hrperiod);
    uint32_t main_loop_frequency = (double) CPU_FREQUENCY_HZ/(main_loop_period);
    QEPEncoder motor_encoder = {*TIM5};
    // GPIO motor_encoder_cs = {*GPIOA, 15, GPIO::OUTPUT};
    // SPIEncoder motor_encoder = {*SPI3, motor_encoder_cs};
    GPIO hall_a = {*GPIOC, 0, GPIO::INPUT};
    GPIO hall_b = {*GPIOC, 1, GPIO::INPUT};
    GPIO hall_c = {*GPIOC, 2, GPIO::INPUT};
    HallEncoder output_encoder = {hall_a, hall_b, hall_c};
    //AMSEncoder motor_encoder = {*SPI3, motor_encoder_cs};
    GPIO enable = {*GPIOC, 11, GPIO::OUTPUT};
    HRPWM motor_pwm = {pwm_frequency, *HRTIM1, 3, 5, 4};
    FastLoopConfig fast_loop = {(float) (1.0/pwm_frequency), motor_pwm, motor_encoder};
    LED led = {const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM4->CCR1)), 
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM4->CCR2)),
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM4->CCR3))};
    PIDDeadbandController controller = {(float) (1.0/main_loop_frequency)};
    USBCommunication communication = {SystemConfig::usb_};
    MainLoopConfig main_loop = {fast_loop, controller, communication, led, output_encoder};
} config_items;

template<>
ActuatorConfig SystemConfig::actuator_ = {config_items.fast_loop, config_items.main_loop};

#include "system.cpp"
