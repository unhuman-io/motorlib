module;

#include "../peripheral/stm32g4/temp_sensor.h"
#include "../peripheral/usb.h"
#include "../peripheral/stm32g4/hrpwm.h"
#include "../led.h"
#include "../driver.h"
#include "../usb_communication.h"

export import pin_config_obot_g474_trace;

export module config_obot_g474_tracec;

export class TraceBoard {
 public:
    TraceBoard(const uint32_t &pwm_frequency1, const uint32_t &main_loop_frequency1) :
        pwm_frequency(pwm_frequency1),
        main_loop_frequency(main_loop_frequency1) {
        const BoardRev board_rev = get_board_rev();
        //SystemClock_Config();
        pin_config_obot_g474_trace(board_rev);
    }

    using PWM = HRPWM;
    using Driver = DriverBase;
    using Communication = USBCommunication;
    using LED = LED;

    //static_assert(((double) CPU_FREQUENCY_HZ * 8 / 2) / pwm_frequency < 65535);    // check pwm frequency

    TempSensor temp_sensor;
    // Flash flash(*FLASH);

    const BoardRev board_rev = get_board_rev();
    Driver drv;

    HRPWM motor_pwm = {pwm_frequency, *HRTIM1, 3, 5, 4, false, 50, 1000, 1000};
    USB1 usb;
    Communication communication{usb};


    LED led = {const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(get_board_pins(board_rev).led_tim_r)), 
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(get_board_pins(board_rev).led_tim_g)),
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(get_board_pins(board_rev).led_tim_b))};



    const uint32_t pwm_frequency;
    const uint32_t main_loop_frequency;
};
