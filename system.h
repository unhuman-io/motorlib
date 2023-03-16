#ifndef UNHUMAN_MOTORLIB_SYSTEM_H_
#define UNHUMAN_MOTORLIB_SYSTEM_H_

#ifdef __cplusplus
#include "actuator.h"
#include "parameter_api.h"

extern uint32_t t_exec_fastloop;
extern uint32_t t_exec_mainloop;
extern uint32_t t_period_fastloop;
extern uint32_t t_period_mainloop;

void system_maintenance();

class System {
 public:
  static void run();
  static void main_loop_interrupt();
  static void fast_loop_interrupt();
  static void log(std::string str);
  static std::string get_log();
  static char *get_string();

  static Communication communication_;
  static Actuator actuator_;
  static ParameterAPI api;
  static uint32_t count_;
};

extern "C" {
#endif  // __cplusplus

void system_init();
void system_run();
void main_loop_interrupt();
void fast_loop_interrupt();
void usb_interrupt();

#ifdef __cplusplus
}
#endif

#endif  // UNHUMAN_MOTORLIB_SYSTEM_H_
