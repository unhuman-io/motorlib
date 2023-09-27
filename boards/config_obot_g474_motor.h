#pragma once

#if defined(R3) || defined(R4) || defined(MR0) || defined(MR0P)
#ifndef BROKEN_MAX31875
#define HAS_MAX31875
#endif
#endif

#if defined(MR1)
#define HAS_MAX31889
#endif

#if defined(R4) || defined (MR0P) || defined (MR0) || defined(MR1)
#define HAS_BMI270
#endif

#if defined(MR0) || defined (MR0P) || defined (MR1)
#define HAS_BRIDGE_THERMISTORS
#endif

#if defined(MR1)
#define HAS_5V_SENSE
#define HAS_I5V_SENSE
#define HAS_I48V_SENSE
#endif