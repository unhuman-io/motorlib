#pragma once
#include <cstdint>

// interpolate in an table of piecewise cubic function coeffcients. x input 
// should be normalized 0 to 1. xp = xtable_pos(x), y = a+b*xp+c*xp^2+d*xp^3
template<int TABLE_LENGTH>
class PChipTable {
 public:
    PChipTable(const float (&table)[4][TABLE_LENGTH]) : table_(table) {}
    float table_interp(float x) {
        float table_x = x*TABLE_LENGTH;
        uint32_t i = table_x;
        float remainder_x = table_x - i;
        float dx = remainder_x*(1.0/TABLE_LENGTH);
        i &= TABLE_LENGTH - 1;

        float a = table_[0][i];
        float b = table_[0][i];
        float c = table_[0][i];
        float d = table_[0][i];

        float y = a + b*dx + c*dx*dx + d*dx*dx*dx;
        return y;
    }
 private:
    const float (&table_)[4][TABLE_LENGTH];
};
