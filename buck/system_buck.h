#ifndef UNHUMAN_MOTORLIB_BUCK_SYSTEM_BUCK_H_
#define UNHUMAN_MOTORLIB_BUCK_SYSTEM_BUCK_H_

#ifdef __cplusplus
template<typename FastLoop, typename MainLoop, typename USB>
class SystemBuck {
 public:
    static void run() {
        while(1) {
            //asm("WFI"); // wait for interrupt - sleep
        }
    }
    static void main_loop_interrupt() {
        actuator_.main_loop_.update();
    }
    static void fast_loop_interrupt() {
        actuator_.fast_loop_.update();
    }
    static void usb_interrupt() {
        usb_.interrupt();
    }
 private:
    static FastLoop fast_loop_;
    static MainLoop main_loop_;
    static USB usb_;
};

extern "C" {
#endif // __cplusplus

void system_run();
void main_loop_interrupt();
void fast_loop_interrupt();
void usb_interrupt();

#ifdef __cplusplus
}
#endif

#endif  // UNHUMAN_MOTORLIB_BUCK_SYSTEM_BUCK_H_
