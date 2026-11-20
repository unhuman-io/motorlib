#pragma once

#include "interrupt_profiler.h"

template<auto func>
void CommHandler() {
  static uint32_t last_start = 0;
  asm("":::"memory");
  Profiler<false> p;
  p.start(last_start);
  func();
  p.end(active_stats.load()->comint_stats);
}

template<auto func>
void FastLoopHandler() {
  static uint32_t last_start = 0;
  asm("":::"memory");
  Profiler<false> p;
  p.start(last_start);
  func();
  p.end(active_stats.load()->fastloop_stats);
}

template<auto func>
void MainLoopHandler() {
  static uint32_t last_start = 0;
  asm("":::"memory");
  Profiler<false> p;
  p.start(last_start);
  func();
  p.end(active_stats.load()->mainloop_stats);
}

template<auto func>
void SystemLoopHandler() {
  static uint32_t last_start = 0;
  asm("":::"memory");
  Profiler<false> p;
  p.start(last_start);
  func();
  p.end(active_stats.load()->systemloop_stats);
}
