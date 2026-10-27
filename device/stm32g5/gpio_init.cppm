module;
#include <string_view>
#include <cstdint>
#include <concepts>
#include "st_device.h"

export module device.gpio_init;

export {
enum class GPIOMode : unsigned int {
  INPUT = 0,
  OUTPUT = 1,
  ALTERNATE = 2,
  ANALOG = 3,
};

enum class GPIOOType : unsigned int {
  PUSH_PULL = 0,
  OPEN_DRAIN = 1,
};

enum class GPIOSpeed : unsigned int {
  LOW = 0,
  MEDIUM = 1,
  HIGH = 2,
  VERY_HIGH = 3,
};

enum class GPIOPullUpD : unsigned int {
  NO_PULL = 0,
  PULL_UP = 1,
  PULL_DOWN = 2,
};

enum class GPIOSet : unsigned int {
  NO_SET = 0,
  RESET = 1,
  SET = 2,
};

enum class GPIOLock : unsigned int {
  NO_LOCK = 0,
  LOCK = 1,
};

enum class GPIOAF : unsigned int {
  AF0 = 0,
  AF1 = 1,
  AF2 = 2,
  AF3 = 3,
  AF4 = 4,
  AF5 = 5,
  AF6 = 6,
  AF7 = 7,
  AF8 = 8,
  AF9 = 9,
  AF10 = 10,
  AF11 = 11,
  AF12 = 12,
  AF13 = 13,
  AF14 = 14,
  AF15 = 15,
};

struct GPIOInit {
  struct {
    const char* name;
    const char* net;
    const char* function;
    const char* pin;
    GPIOMode mode = GPIOMode::ANALOG;
    GPIOOType otype = GPIOOType::PUSH_PULL;
    GPIOSpeed speed = GPIOSpeed::LOW;
    GPIOPullUpD pullupd = GPIOPullUpD::NO_PULL;
    GPIOSet bsrr = GPIOSet::NO_SET;
    GPIOLock lck = GPIOLock::NO_LOCK;
    GPIOAF af = GPIOAF::AF0;
  } a[16], b[16], c[16], d[16], e[16], f[16], g[16], h[16];
};

// Reset values from the SVD: GPIOA MODER 0xABFFFFFF, OSPEEDR 0x0C000000,
// PUPDR 0x64000000; GPIOB MODER 0xFFFFFEBF; everything else analog, no pull.
// The SVD claims GPIOB PUPDR resets to 0, but PB4 is NJTRST and is pulled up
// at reset on every other STM32, so assume the same here.
inline constexpr GPIOInit get_default_gpio_init() {
    GPIOInit init {};
    init.a[13].mode = GPIOMode::ALTERNATE;
    init.a[14].mode = GPIOMode::ALTERNATE;
    init.a[15].mode = GPIOMode::ALTERNATE;
    init.b[3].mode = GPIOMode::ALTERNATE;
    init.b[4].mode = GPIOMode::ALTERNATE;
    init.a[13].speed = GPIOSpeed::VERY_HIGH;
    init.a[13].pullupd = GPIOPullUpD::PULL_UP;
    init.a[14].pullupd = GPIOPullUpD::PULL_DOWN;
    init.a[15].pullupd = GPIOPullUpD::PULL_UP;
    init.b[4].pullupd = GPIOPullUpD::PULL_UP;
    return init;
}

template <typename Regs>
concept GPIORegs = requires(Regs& regs) {
  { regs->GPIO_MODER } -> std::same_as<volatile uint32_t&>;
  { regs->GPIO_OTYPER } -> std::same_as<volatile uint32_t&>;
  { regs->GPIO_OSPEEDR } -> std::same_as<volatile uint32_t&>;
  { regs->GPIO_PUPDR } -> std::same_as<volatile uint32_t&>;
  { regs->GPIO_BSRR } -> std::same_as<volatile uint32_t&>;
  { regs->GPIO_LCKR } -> std::same_as<volatile uint32_t&>;
  { regs->GPIO_AFRL } -> std::same_as<volatile uint32_t&>;
  { regs->GPIO_AFRH } -> std::same_as<volatile uint32_t&>;
};

struct GPIORegsInit {
  struct Regs {
    uint32_t moder;
    uint32_t otyper;
    uint32_t ospeedr;
    uint32_t pupdr;
    uint32_t bsrr;
    uint32_t lckr;
    uint32_t afr[2];
  } a, b, c, d, e, f, g, h;
};

inline consteval const GPIORegsInit::Regs get_gpio_regs_init(const decltype(GPIOInit::a)& g) {
  GPIORegsInit::Regs init{};
  uint32_t moder = 0;
  for (int i = 0; i < 16; i++) {
    moder |= static_cast<uint32_t>(g[i].mode) << 2 * i;
  }
  init.moder = moder;

  uint32_t otyper = 0;
  for (int i = 0; i < 16; i++) {
    otyper |= static_cast<uint32_t>(g[i].otype) << i;
  }
  init.otyper = otyper;

  uint32_t ospeedr = 0;
  for (int i = 0; i < 16; i++) {
    ospeedr |= static_cast<uint32_t>(g[i].speed) << 2 * i;
  }
  init.ospeedr = ospeedr;

  uint32_t pupdr = 0;
  for (int i = 0; i < 16; i++) {
    pupdr |= static_cast<uint32_t>(g[i].pullupd) << 2 * i;
  }
  init.pupdr = pupdr;

  uint32_t af1 = 0;
  for (int i = 0; i < 8; i++) {
    af1 |= static_cast<uint32_t>(g[i].af) << 4 * i;
  }
  init.afr[0] = af1;

  uint32_t af2 = 0;
  for (int i = 0; i < 8; i++) {
    af2 |= static_cast<uint32_t>(g[i + 8].af) << 4 * i;
  }
  init.afr[1] = af2;
  uint32_t bsrr = 0;
  for (int i = 0; i < 16; i++) {
    if (g[i].bsrr == GPIOSet::SET) {
      bsrr |= 1 << i;
    } else if (g[i].bsrr == GPIOSet::RESET) {
      bsrr |= 1 << (16 + i);
    }
  }
  init.bsrr = bsrr;
  // todo lckr
  return init;
}

inline consteval const GPIORegsInit get_gpio_regs_init(const GPIOInit& g) {
  GPIORegsInit init{};
  init.a = get_gpio_regs_init(g.a);
  init.b = get_gpio_regs_init(g.b);
  init.c = get_gpio_regs_init(g.c);
  init.d = get_gpio_regs_init(g.d);
  init.e = get_gpio_regs_init(g.e);
  init.f = get_gpio_regs_init(g.f);
  init.g = get_gpio_regs_init(g.g);
  init.h = get_gpio_regs_init(g.h);
  return init;
}

template<const GPIORegsInit::Regs g, const GPIORegsInit::Regs g_default>
inline constexpr void init_gpio_regs(GPIORegs auto regs) {
  if constexpr (g.moder != g_default.moder) regs->GPIO_MODER = g.moder;
  if constexpr (g.otyper != g_default.otyper) regs->GPIO_OTYPER = g.otyper;
  if constexpr (g.ospeedr != g_default.ospeedr) regs->GPIO_OSPEEDR = g.ospeedr;
  if constexpr (g.pupdr != g_default.pupdr) regs->GPIO_PUPDR = g.pupdr;
  if constexpr (g.bsrr != g_default.bsrr) regs->GPIO_BSRR = g.bsrr;
  if constexpr (g.lckr != g_default.lckr) regs->GPIO_LCKR = g.lckr;
  if constexpr (g.afr[0] != g_default.afr[0]) regs->GPIO_AFRL = g.afr[0];
  if constexpr (g.afr[1] != g_default.afr[1]) regs->GPIO_AFRH = g.afr[1];
}

template<const GPIORegsInit g>
inline constexpr void init_gpio() {
  constexpr GPIORegsInit g_default = get_gpio_regs_init(get_default_gpio_init());
  init_gpio_regs<g.a , g_default.a>(GPIOA);
  init_gpio_regs<g.b , g_default.b>(GPIOB);
  init_gpio_regs<g.c , g_default.c>(GPIOC);
  init_gpio_regs<g.d , g_default.d>(GPIOD);
  init_gpio_regs<g.e , g_default.e>(GPIOE);
  init_gpio_regs<g.f , g_default.f>(GPIOF);
  init_gpio_regs<g.g , g_default.g>(GPIOG);
  init_gpio_regs<g.h , g_default.h>(GPIOH);
}

template<GPIOInit gpio_init>
inline constexpr void device_pin_config() { 
    init_gpio<get_gpio_regs_init(gpio_init)>();
}

// GPIOx by port letter, for config code that can't see the device macros
template<char port>
inline auto* gpio_port() {
    if constexpr (port == 'A') return GPIOA;
    else if constexpr (port == 'B') return GPIOB;
    else if constexpr (port == 'C') return GPIOC;
    else if constexpr (port == 'D') return GPIOD;
    else if constexpr (port == 'E') return GPIOE;
    else if constexpr (port == 'F') return GPIOF;
    else if constexpr (port == 'G') return GPIOG;
    else if constexpr (port == 'H') return GPIOH;
}

inline void gpio_toggle(auto* port, int pin) {
    port->GPIO_ODR ^= 1u << pin;
}
}
