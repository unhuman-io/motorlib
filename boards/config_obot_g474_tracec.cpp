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
    using LED = TriColorLED;

    //static_assert(((double) CPU_FREQUENCY_HZ * 8 / 2) / pwm_frequency < 65535);    // check pwm frequency

    TempSensor temp_sensor;
    // Flash flash(*FLASH);

    const BoardRev board_rev = get_board_rev();
    Driver drv;

    HRPWM motor_pwm = {pwm_frequency, *HRTIM1, 3, 5, 4, false, 50, 1000, 1000};
    USB1 usb;
    Communication communication{usb};


    LED led   {const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(get_board_pins(board_rev).led_tim_r)), 
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(get_board_pins(board_rev).led_tim_g)),
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(get_board_pins(board_rev).led_tim_b))};



    const uint32_t pwm_frequency;
    const uint32_t main_loop_frequency;
};

export class BoardFun {
  public:
    void setup_sleep() {
        NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
        NVIC_DisableIRQ(ADC5_IRQn);
    //  config::drv.disable();
        NVIC_SetPriority(USB_LP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 1));
        NVIC_EnableIRQ(RTC_WKUP_IRQn);
        MASK_SET(RCC->CFGR, RCC_CFGR_SW, 2); // HSE is system clock source
        RTC->SCR = RTC_SCR_CWUTF;
    }

    void finish_sleep() {
        MASK_SET(RCC->CFGR, RCC_CFGR_SW, 3); // PLL is system clock source
        // if (!param->main_loop_param.safe_mode_driver_disable) {
        //     config::drv.enable();
        // }
        NVIC_DisableIRQ(RTC_WKUP_IRQn);
        NVIC_SetPriority(USB_LP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
        NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
        NVIC_EnableIRQ(ADC5_IRQn);
    }
};
