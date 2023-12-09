#define INTERRUPT_PROFILE_GLOBALS(loop) uint32_t t_exec_##loop __attribute__((used));\
                                        uint32_t t_period_##loop __attribute__((used));\
                                        uint32_t loop##_count __attribute__((used)) = 0;
#define INTERRUPT_PROFILE_START static uint32_t last_start = 0; \
                                      uint32_t t_start = get_clock();
#define INTERRUPT_PROFILE_END(loop) t_exec_##loop = get_clock()-t_start; \
                                      t_period_##loop = t_start - last_start; \
                                      loop##_count += t_exec_##loop; \
                                      last_start = t_start;