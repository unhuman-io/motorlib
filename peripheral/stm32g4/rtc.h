#pragma once
#include "stm32g4xx.h"

class RTClock {
 public:
    static void power_on_date_time(char t[18]) {
        t[0] = ((RTC->DR & RTC_DR_YT) >> RTC_DR_YT_Pos) + '0';
        t[1] = ((RTC->DR & RTC_DR_YU) >> RTC_DR_YU_Pos) + '0';
        t[2] = '/';
        t[3] = ((RTC->DR & RTC_DR_MT) >> RTC_DR_MT_Pos) + '0';
        t[4] = ((RTC->DR & RTC_DR_MU) >> RTC_DR_MU_Pos) + '0';
        t[5] = '/';
        t[6] = ((RTC->DR & RTC_DR_DT) >> RTC_DR_DT_Pos) + '0';
        t[7] = ((RTC->DR & RTC_DR_DU) >> RTC_DR_DU_Pos) + '0';
        t[8] = ' ';
        t[9] = ((RTC->TR & RTC_TR_HT) >> RTC_TR_HT_Pos) + '0';
        t[10] = ((RTC->TR & RTC_TR_HU) >> RTC_TR_HU_Pos) + '0';
        t[11] = ':';
        t[12] = ((RTC->TR & RTC_TR_MNT) >> RTC_TR_MNT_Pos) + '0';
        t[13] = ((RTC->TR & RTC_TR_MNU) >> RTC_TR_MNU_Pos) + '0';
        t[14] = ':';
        t[15] = ((RTC->TR & RTC_TR_ST) >> RTC_TR_ST_Pos) + '0';
        t[16] = ((RTC->TR & RTC_TR_SU) >> RTC_TR_SU_Pos) + '0';
        t[17] = 0;
    }
    static void power_on_time(char t[9]) {
        RTC->DR;
        t[0] = ((RTC->TR & RTC_TR_HT) >> RTC_TR_HT_Pos) + '0';
        t[1] = ((RTC->TR & RTC_TR_HU) >> RTC_TR_HU_Pos) + '0';
        t[2] = ':';
        t[3] = ((RTC->TR & RTC_TR_MNT) >> RTC_TR_MNT_Pos) + '0';
        t[4] = ((RTC->TR & RTC_TR_MNU) >> RTC_TR_MNU_Pos) + '0';
        t[5] = ':';
        t[6] = ((RTC->TR & RTC_TR_ST) >> RTC_TR_ST_Pos) + '0';
        t[7] = ((RTC->TR & RTC_TR_SU) >> RTC_TR_SU_Pos) + '0';
        t[8] = 0;
    }
};
