
test_control_fun: test_control_fun.cpp ../control_fun.h ../control_fun.cpp
	g++ test_control_fun.cpp ../control_fun.cpp -lm -o test_control_fun

test_logger: test_logger.cpp ../logger.h
	g++ test_logger.cpp -o test_logger

test_error_mask: test_error_mask.cpp ../motor_messages/motor_messages.h
	g++ test_error_mask.cpp -o test_error_mask

test_encoder_bias_from_output: test_encoder_bias_from_output.cpp ../motor_messages/motor_messages.h
	g++-14 -std=c++23 test_encoder_bias_from_output.cpp -o test_encoder_bias_from_output