
#include <format>
#include <print>
#include <numbers>
#include <cmath>

struct {
    int num_encoder_poles = 1;
} startup_param_;

struct Status {
    float output_position;
    struct {
        struct {
            float position;
        } motor_position;
    } fast_loop;
};

float startup_motor_bias_ = 0.0f;

float output_position_to_motor_position(float output_position) {
    return output_position * 81;
}

float fun(const Status &status) {
    float round_by = 2*std::numbers::pi*(startup_param_.num_encoder_poles == 0 ? 1 : startup_param_.num_encoder_poles);
    float motor_bias_from_output = output_position_to_motor_position(status.output_position) 
        - (status.fast_loop.motor_position.position);
    float motor_bias_rounded = roundf(motor_bias_from_output/round_by)*(round_by) + startup_motor_bias_;
    return motor_bias_rounded;
}



int main() {
    Status status;
    status.output_position = -7.0f;
    
    for (int i = 0; i < 14*10; ++i) {
        status.output_position += 0.1f;
        for (int j = -100; j < 100*2; j++) {
            for (int k = -7; k <= 7; k++) {
                startup_motor_bias_ = k;
                float motor_position_raw = j;
                status.fast_loop.motor_position.position = motor_position_raw + startup_motor_bias_;
                float bias = fun(status);
                float position = motor_position_raw + bias;
                std::println("Output Position: {:3.2f}, Motor Position: {:6.2f}, Bias: {:7.2f}, Position after bias: {:6.2f}",
                    status.output_position, status.fast_loop.motor_position.position, bias, position);
                std::println("diff {}", position - output_position_to_motor_position(status.output_position));
                //std::println("diff2 {}", position)
                if (std::abs(position - output_position_to_motor_position(status.output_position)) > std::numbers::pi) {
                    std::println("Error: Position after bias does not match expected output position!");
                    exit(1);
                }
            }
        }
    }
    float result = fun(status);

    return result;
}