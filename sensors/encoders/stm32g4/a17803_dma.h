
#include "../../../peripheral/stm32g4/sequenced_spidma.h"
#include "a17803.h"

#define A17803_DMA_SET_DEBUG_API(prefix, api, encoder) \
    api.add_api_variable(prefix "temp1", new const APIFloat(&encoder.temp_[0]));\
    api.add_api_variable(prefix "temp2", new const APIFloat(&encoder.temp_[1]));\
    api.add_api_variable(prefix "diag1", new const APIHex<uint16_t>(&encoder.diag_[0]));\
    api.add_api_variable(prefix "diag2", new const APIHex<uint16_t>(&encoder.diag_[1]));\


class A17803_DMA : public EncoderBase {
  public:
    A17803_DMA(SPIDMA &spidma1, SPIDMA &spidma2, void(*start_cs_trigger)(), void(*stop_cs_trigger)()) : a17803_{A17803(spidma1), A17803(spidma2)}, sequenced_spidma_{spidma1,
        *DMAMUX1_Channel0, *DMAMUX1_Channel1, 0,
        *DMA1_Channel3, *DMA1_Channel4, (uint32_t *) &GPIOA->BSRR,
        start_cs_trigger, stop_cs_trigger} {
        // read one behind
        sequenced_spidma_.init_sequence(0, 0, A17803::make_reg_message_consteval(A17803::PrimaryAddress::ANGLE));
        sequenced_spidma_.init_sequence(0, 1, A17803::make_reg_message_consteval(A17803::PrimaryAddress::ANGLE));
        sequenced_spidma_.init_sequence(0, 2, A17803::make_reg_message_consteval(A17803::PrimaryAddress::TEMPERATURE));
        sequenced_spidma_.init_sequence(1, 0, A17803::make_reg_message_consteval(A17803::PrimaryAddress::ANGLE));
        sequenced_spidma_.init_sequence(1, 1, A17803::make_reg_message_consteval(A17803::PrimaryAddress::ERROR));
        sequenced_spidma_.init_sequence(1, 2, A17803::make_reg_message_consteval(A17803::PrimaryAddress::ANGLE));
        sequenced_spidma_.init_sequence(2, 0, A17803::make_reg_message_consteval(A17803::PrimaryAddress::ANGLE));
        sequenced_spidma_.init_sequence(2, 1, A17803::make_reg_message_consteval(A17803::PrimaryAddress::ANGLE));
        sequenced_spidma_.init_sequence(2, 2, A17803::make_reg_message_consteval(A17803::PrimaryAddress::ERROR));
        sequenced_spidma_.init_sequence(3, 0, A17803::make_reg_message_consteval(A17803::PrimaryAddress::ANGLE));
        sequenced_spidma_.init_sequence(3, 1, A17803::make_reg_message_consteval(A17803::PrimaryAddress::TEMPERATURE));
        sequenced_spidma_.init_sequence(3, 2, A17803::make_reg_message_consteval(A17803::PrimaryAddress::ANGLE));
        sequenced_spidma_.init_gpio_cs_idle(GPIO_BSRR_BS_0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_3); // both CS high
        sequenced_spidma_.init_gpio_cs(0, 0, GPIO_BSRR_BR_4 | GPIO_BSRR_BR_0); // CS1 low
        sequenced_spidma_.init_gpio_cs(0, 1, GPIO_BSRR_BR_4 | GPIO_BSRR_BR_0); // CS1 low
        sequenced_spidma_.init_gpio_cs(0, 2, GPIO_BSRR_BR_3 | GPIO_BSRR_BR_0); // CS2 low
        sequenced_spidma_.init_gpio_cs(1, 0, GPIO_BSRR_BR_3 | GPIO_BSRR_BR_0); // CS2 low
        sequenced_spidma_.init_gpio_cs(1, 1, GPIO_BSRR_BR_4 | GPIO_BSRR_BR_0); // CS1 low
        sequenced_spidma_.init_gpio_cs(1, 2, GPIO_BSRR_BR_3 | GPIO_BSRR_BR_0); // CS2 low
        sequenced_spidma_.init_gpio_cs(2, 0, GPIO_BSRR_BR_4 | GPIO_BSRR_BR_0); // CS1 low
        sequenced_spidma_.init_gpio_cs(2, 1, GPIO_BSRR_BR_4 | GPIO_BSRR_BR_0); // CS1 low
        sequenced_spidma_.init_gpio_cs(2, 2, GPIO_BSRR_BR_3 | GPIO_BSRR_BR_0); // CS2 low
        sequenced_spidma_.init_gpio_cs(3, 0, GPIO_BSRR_BR_3 | GPIO_BSRR_BR_0); // CS2 low
        sequenced_spidma_.init_gpio_cs(3, 1, GPIO_BSRR_BR_4 | GPIO_BSRR_BR_0); // CS1 low
        sequenced_spidma_.init_gpio_cs(3, 2, GPIO_BSRR_BR_3 | GPIO_BSRR_BR_0); // CS2 low
        spidma1.pause_.start_callback_ = [this]{sequenced_spidma_.start_continuous_read();};
        spidma1.pause_.stop_callback_ = [this]{sequenced_spidma_.stop_continuous_read();};

        sequenced_spidma_.init();
        sequenced_spidma_.start_continuous_read();
    }

    int32_t read() {
        int index = sequenced_spidma_.current_sequence_index();
        a17803_[0].read_buf(sequenced_spidma_.get_data_buffer(index, 1));
        a17803_[1].read_buf(sequenced_spidma_.get_data_buffer(index, 2));
        switch (index) {
            case 0:
                temp_[0] = a17803_[0].get_temperature(sequenced_spidma_.get_data_buffer(index, 0));
                break;
            case 1:
                temp_[1] = a17803_[1].get_temperature(sequenced_spidma_.get_data_buffer(index, 0));
                break;
            case 2:
                diag_[0] = a17803_[0].get_diag(sequenced_spidma_.get_data_buffer(index, 0));
                break;
            case 3:
                diag_[1] = a17803_[1].get_diag(sequenced_spidma_.get_data_buffer(index, 0));
                break;
        }
        return a17803_[1].get_value();
    }

  //private:
    static constexpr int sequence_length = 4;
    static constexpr int reads_per_cycle = 3;
    A17803 a17803_[2];
    float temp_[2] = {};
    uint16_t diag_[2] = {};
    Sequenced_SPIDMA<sequence_length, reads_per_cycle, A17803::A17803_Message_Rev> sequenced_spidma_;

};
