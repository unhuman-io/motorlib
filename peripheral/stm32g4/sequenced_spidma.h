
#include <cstdint>

template<typename SPIDMA, int sequence_length, int reads_per_cycle, typename T>
class Sequenced_SPIDMA {
  public:
    Sequenced_SPIDMA(SPIDMA &spidma, DMAMUX_Channel_TypeDef &dmamux_tx_regs, DMAMUX_Channel_TypeDef &dmamux_rx_regs, int exti_num,
        DMA_Channel_TypeDef &gpio_cs_clear_dma, DMA_Channel_TypeDef &gpio_cs_set_dma, uint32_t *gpio_bsrr_reg,
        void(*start_cs_trigger)(), void(*stop_cs_trigger_and_wait_cs_high)()) :
        spidma_(spidma), dmamux_tx_regs_(dmamux_tx_regs), dmamux_rx_regs_(dmamux_rx_regs), exti_num_(exti_num),
        start_cs_trigger_(start_cs_trigger), stop_cs_trigger_and_wait_cs_high_(stop_cs_trigger_and_wait_cs_high) {
            gpio_cs_clear_dma.CMAR = (uint32_t) &gpio_cs_sequence_;
            gpio_cs_clear_dma.CPAR = (uint32_t) gpio_bsrr_reg;
            gpio_cs_clear_dma.CNDTR = sequence_length * reads_per_cycle;
            gpio_cs_clear_dma.CCR = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_CIRC | DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1;
            gpio_cs_set_dma.CMAR = (uint32_t) &gpio_cs_idle_state_;
            gpio_cs_set_dma.CPAR = (uint32_t) gpio_bsrr_reg;
            gpio_cs_set_dma.CNDTR = 1;
            gpio_cs_set_dma.CCR = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_CIRC | DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1;
    }

    void init() {
      for (int i=0; i<sequence_length; i++) {
        for (int j=0; j<reads_per_cycle; j++) {
          spidma_.readwrite((uint8_t *) &read_sequence_[i][j], (uint8_t *) &data_buffer_[i][j], sizeof(T));
        }
      }
      for (int j=0; j<reads_per_cycle; j++) {
          spidma_.readwrite((uint8_t *) &read_sequence_[0][j], (uint8_t *) &data_buffer_[0][j], sizeof(T));
      }
    }

    void start_continuous_read() {
      if (!stopped_) {
        dmamux_tx_regs_.CCR |= exti_num_ << DMAMUX_CxCR_SYNC_ID_Pos | (sizeof(T)-1) << DMAMUX_CxCR_NBREQ_Pos | 2 << DMAMUX_CxCR_SPOL_Pos | DMAMUX_CxCR_SE;
        dmamux_rx_regs_.CCR |= (sizeof(T)-1) << DMAMUX_CxCR_NBREQ_Pos | DMAMUX_CxCR_EGE;
        spidma_.start_continuous_readwrite((uint8_t *) &read_sequence_[0][0], (uint8_t *) &data_buffer_[0][0], sizeof(read_sequence_));
        // start automatic CS
        start_cs_trigger_();
      }
    }
    void stop_continuous_read() {
      // stop automatic CS
      stop_cs_trigger_and_wait_cs_high_();
      // wait for CS high
      spidma_.stop_continuous_readwrite();
      dmamux_tx_regs_.CCR &= DMAMUX_CxCR_DMAREQ_ID_Msk;
      dmamux_rx_regs_.CCR &= DMAMUX_CxCR_DMAREQ_ID_Msk;
    }

    void stop() {
      stopped_ = true;
      stop_continuous_read();
    }
    void init_sequence(int sequence_index, int read_index, const T command) {
        read_sequence_[sequence_index][read_index] = command;
    }
    // set gpio BSRR to clear in the read state
    void init_gpio_cs(int sequence_index, int read_index, uint32_t gpio_state) {
        // Initialize the GPIO CS state for the given sequence and read index
        gpio_cs_sequence_[sequence_index][read_index] = gpio_state;
    }
    // set gpio BSRR to set in the idle state, when rx complete is registered
    void init_gpio_cs_idle(uint32_t gpio_state) {
        gpio_cs_idle_state_ = gpio_state;
    }
    int current_sequence_index() const {
        uint32_t n = spidma_.tx_dma_.CNDTR / (reads_per_cycle * sizeof(T));
        return (n >= sequence_length) ? sequence_length - 1 : (sequence_length - 1 - n);
    }
    T get_data_buffer(int sequence_index, int read_index) {
        return data_buffer_[sequence_index][read_index];
    }

  private:
    SPIDMA &spidma_;
    DMAMUX_Channel_TypeDef &dmamux_tx_regs_, &dmamux_rx_regs_;
    int exti_num_;
    void(*start_cs_trigger_)();
    void(*stop_cs_trigger_and_wait_cs_high_)();
    bool stopped_ = false;
    T read_sequence_[sequence_length][reads_per_cycle];
    T data_buffer_[sequence_length][reads_per_cycle];
    uint32_t gpio_cs_sequence_[sequence_length][reads_per_cycle];
    uint32_t gpio_cs_idle_state_;
};
