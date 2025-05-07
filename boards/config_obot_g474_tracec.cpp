module;

#include "../peripheral/stm32g4/temp_sensor.h"
#include "../peripheral/usb.h"

import pin_config_obot_g474_trace;

export module config_obot_g474_tracec;

export class TraceBoard {
 public:
    TraceBoard() {
        const BoardRev board_rev = get_board_rev();
        //SystemClock_Config();
        pin_config_obot_g474_trace(board_rev);
    }

    //static_assert(((double) CPU_FREQUENCY_HZ * 8 / 2) / pwm_frequency < 65535);    // check pwm frequency

    TempSensor temp_sensor;
    // Flash flash(*FLASH);

    // const BoardRev board_rev = get_board_rev();

    // HRPWM motor_pwm = {pwm_frequency, *HRTIM1, 3, 5, 4, false, 50, 1000, 1000};
    USB1 usb;
    // FastLoop fast_loop = {(int32_t) pwm_frequency, motor_pwm, motor_encoder, param->fast_loop_param, *calibration, &I_A_DR, &I_B_DR, &I_C_DR, &V_BUS_DR};


    // LED led = {const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(get_board_pins(board_rev).led_tim_r)), 
    //            const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(get_board_pins(board_rev).led_tim_g)),
    //            const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(get_board_pins(board_rev).led_tim_b))};
};
