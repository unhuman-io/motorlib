#ifndef UNHUMAN_MOTORLIB_TABLE_INTERP_H_
#define UNHUMAN_MOTORLIB_TABLE_INTERP_H_

#include <cstdint>

// interpolate in an table of piecewise cubic function coeffcients. x input 
// should be normalized 0 to 1. xp = xtable_pos(x), y = a+b*xp+c*xp^2+d*xp^3
template<int TABLE_LENGTH>
class PChipTable {
 public:
    PChipTable(const float (&table)[TABLE_LENGTH][4]) : table_(table) {}
    float table_interp(float x) {
        float table_x = x*TABLE_LENGTH;
        int32_t i = table_x;
        float dx = table_x - i;
        //float dx = remainder_x*(1.0/TABLE_LENGTH);
        i &= TABLE_LENGTH - 1; // requirement for multiples of 2

        float a = table_[i][0];
        float b = table_[i][1];
        float c = table_[i][2];
        float d = table_[i][3];

        float y = a + b*dx + c*dx*dx + d*dx*dx*dx;
        return y;
    }
 private:
    const float (&table_)[TABLE_LENGTH][4];
};

#endif  // UNHUMAN_MOTORLIB_TABLE_INTERP_H_
