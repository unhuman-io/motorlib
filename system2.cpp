module;

#include "parameter_api.h"
#include "logger.h"
#include "round_robin_logger.h"
#include "otp.h"
#include "peripheral/stm32_serial.h"


// extern uint32_t t_exec_fastloop;
// extern uint32_t t_exec_mainloop;
// extern uint32_t t_period_fastloop;
// extern uint32_t t_period_mainloop;

// void system_maintenance();
// void main_maintenance();

#ifndef TOGGLE_SCOPE_PIN
#define TOGGLE_SCOPE_PIN(X,x)
#endif

export module system2;

export template<typename Communication, typename Actuator, typename RTClock>
class System {
 public:
    static void run() {
        {
            char t[18];
            RTClock::power_on_date_time(t);
            logger.log(t);
        }
    //     // check parameter version
    //    // if (OBOT_HASH != std::string(param->obot_hash)) {
    //   //      logger.log_printf("param version error, firmware: %s, param: %s", OBOT_HASH, param->obot_hash);
    //    //     actuator_.main_loop_.led_.set_color(LED::RED);
    //     //    actuator_.main_loop_.led_.set_mode(LED::BLINKING);
    //         while(1) {
    //             go_to_bootloader = 0xB007;
    //             NVIC_SystemReset();
    //         }
    //     } else {
    //         logger.log_printf("param version match: %s", OBOT_HASH);
    //     }

        actuator_.start();

        log("finished startup");

        //uint32_t cpu_frequency = CPU_FREQUENCY_HZ;
       
        uint32_t t_start = get_clock();
        uint32_t api_timeout_us = 10000;
        current_api_timeout_us_ = api_timeout_us;
        while(1) {
            TOGGLE_SCOPE_PIN(C,4);
            count_++;
            if (communication_.send_string_active() && get_clock() - t_start > US_TO_CPU(current_api_timeout_us_)) {
                communication_.cancel_send_string();
                current_api_timeout_us_ = api_timeout_us;
            }
            char *s = System::get_string();
            if (s[0] != 0) {
                auto response = api.parse_string(s);
                current_api_timeout_us_ = api_timeout_us;
                communication_.send_string(response.c_str(), response.length());
                t_start = get_clock();
            }
            //main_maintenance();
        }
    }
    static void set_one_time_api_timeout_us(uint32_t us) {
        communication_.send_one_time_api_timeout_request(us);
        current_api_timeout_us_ = US_TO_CPU(us);
    }
    static void main_loop_interrupt() {
        actuator_.main_loop_update();
    }
    static void fast_loop_interrupt() {
        actuator_.fast_loop_update();
    }
    static void system_loop() {
        //system_maintenance();
        actuator_.maintenance();
        round_robin_logger.log_data(UPTIME_INDEX, get_uptime());
    }
    static void log(std::string str) {
        logger.log(str);
    }
    static std::string get_log() {
        return logger.get_log();
    }

    static char *get_string() {
        static char buf[65];
        communication_.receive_string(buf);
        return buf;
    }

    static Communication communication_;
    static Actuator actuator_;
    static ParameterAPI api;
    static uint32_t count_;
    static uint32_t current_api_timeout_us_;
};

extern "C" {
void system_init();
void system_run();
void main_loop_interrupt();
void fast_loop_interrupt();
void system_loop_interrupt();
void usb_interrupt();
}


#include <string>
#include <fcntl.h>
#include <cstring>

// extern "C" void system_run() {
//     //System::run();
// }

// extern "C" void main_loop_interrupt() {
//  //   System::main_loop_interrupt();
// }

// extern "C" void fast_loop_interrupt() {
//   //  System::fast_loop_interrupt();
// }

// extern "C" void system_log(std::string s) {
//  //   System::log(s);
// }

// extern "C" void system_loop_interrupt() {
//   //  System::system_loop();
// }




//RoundRobinLogger round_robin_logger;
// uint32_t System::count_ = 0;
// ParameterAPI System::api = {};
// uint32_t System::current_api_timeout_us_ = 0;

// send printf and other stdout/err to the logger
extern "C" void _write(int fd, const char *buf, size_t count) {
    logger.log(std::string_view(buf, count));
}


// Necessary for _write maybe?
extern "C" int _fstat (int fd, struct stat * st) 
{
  memset (st, 0, sizeof (* st));
  st->st_mode = S_IFCHR;
  setbuf(stdout, NULL); // Disable buffering for stdout
  setbuf(stderr, NULL);
  return (0);
}

extern "C" int _isatty (int fd) 
{
  return (1);
}

extern "C" caddr_t _sbrk (int incr) 
{
  static char * heap;
         char * prev_heap;

  if (heap == NULL) {
    heap = (char *)&_end;
  }
  
  prev_heap = heap;

  if ((heap + incr) > (char *)(&_estack - (uint32_t) &_Min_Stack_Size)) {
    errno = ENOMEM;
    logger.log("Heap overflow");
    return (caddr_t) -1;
  }
  
  heap += incr;

  return (caddr_t) prev_heap;
}
