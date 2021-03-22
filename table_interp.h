#pragma once
#include <cstdint>

// interpolate in an table of piecewise cubic function coeffcients. x input 
// should be normalized 0 to 1. xp = xtable_pos(x), y = a+b*xp+c*xp^2+d*xp^3
template<typename TABLE_LENGTH>
class PChipTable {
 public:
    float table_interp(float x) {
        float table_x = x*TABLE_LENGTH;
        uint32_t i = table_x;
        float remainder_x = table_x - i;
        float dx = remainder_x*(1.0/TABLE_LENGTH);
        i &= TABLE_LENGTH - 1;
        float y = a_[i] + b_[i]*dx + c_[i]*dx*dx + d_[i]*dx*dx*dx;
        return y;
    }
 private:
    float table_[4][TABLE_LENGTH];
    float (&a_)[TABLE_LENGTH] = table_[0];
    float (&b_)[TABLE_LENGTH] = table_[1];
    float (&c_)[TABLE_LENGTH] = table_[2];
    float (&d_)[TABLE_LENGTH] = table_[3];
};
