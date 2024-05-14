#pragma once

#include "st_device.h"

 enum DMA_CHANNEL_INSTANCE {
        DMA1_CH1,
        DMA1_CH2,
        DMA1_CH3,
        DMA1_CH4,
        DMA1_CH5,
        DMA1_CH6,
        DMA1_CH7,
        DMA1_CH8,
        DMA2_CH1,
        DMA2_CH2,
        DMA2_CH3,
        DMA2_CH4,
        DMA2_CH5,
        DMA2_CH6,
        DMA2_CH7,
        DMA2_CH8,
        NUM_DMA_CHANNELS,
};
static constexpr DMA_Channel_TypeDef *dma_ch_regs[NUM_DMA_CHANNELS] = {
    DMA1_Channel1, DMA1_Channel2, DMA1_Channel3, DMA1_Channel4,
    DMA1_Channel5, DMA1_Channel6, DMA1_Channel7, DMA2_Channel8,
    DMA2_Channel1, DMA2_Channel2, DMA2_Channel3, DMA2_Channel4, 
    DMA2_Channel5, DMA2_Channel6, DMA2_Channel7, DMA2_Channel8,
};
